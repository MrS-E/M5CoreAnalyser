/**
 * @file tool_scope.c
 * @brief Core 1 high-rate ADC1 capture, PSRAM ring/double-buffer pipeline, trigger engine,
 *        speaker audio passthrough, recorder sink export, and LVGL 9 UI for `tool_scope`.
 */

#include "tool_scope.h"
#include "tool_scope_priv.h"

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "psram_buffer.h"
#include "recorder_sink.h"
#include "tool_core.h"

static const char *TAG = "tool_scope";

#define SCOPE_WORKER_PERIOD_MS      (25)
#define SCOPE_WORKER_STOP_WAIT_MS   (2000)
#define SCOPE_WORKER_STOP_RETRIES   (5)
#define SCOPE_SPK_CHUNK_SAMPLES     (256)
#define SCOPE_TRIG_STEP_MV          (150U)

static portMUX_TYPE s_scope_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_step_mutex_buf;
static SemaphoreHandle_t s_step_mutex = NULL;
static atomic_int s_step_mutex_state = 0;

static gpio_num_t s_selected_pin = BSP_PIN_ANALOG_IN_DEFAULT;
static scope_timebase_t s_timebase = SCOPE_TB_1MS_DIV;
static scope_vscale_t s_vscale = SCOPE_VSCALE_1000MV_DIV;
static scope_trig_mode_t s_trig_mode = SCOPE_TRIG_AUTO;
static uint16_t s_trig_level_mv = SCOPE_DEFAULT_TRIG_MV;
static bool s_running = true;
static bool s_tool_active = false;
static volatile bool s_stop_requested = false;
static volatile bool s_step_preempt = false;
static volatile uint32_t s_capture_seq = 0;

static scope_frame_t s_latest_frame = {0};
static psram_double_buf_t s_frame_dbuf = {0};
static uint16_t *s_raw_ring_buf = NULL;
static uint16_t *s_mv_ring_buf = NULL;
static uint16_t s_spk_chunk_buf[SCOPE_SPK_CHUNK_SAMPLES];
static tool_worker_t *s_worker = NULL;

static scope_sample_source_fn_t s_custom_source_fn = NULL;
static void *s_custom_source_ctx = NULL;

static void ensure_scope_step_mutex(void)
{
    int expected = 0;
    if (atomic_compare_exchange_strong(&s_step_mutex_state, &expected, 1)) {
        s_step_mutex = xSemaphoreCreateMutexStatic(&s_step_mutex_buf);
        atomic_store(&s_step_mutex_state, 2);
    } else {
        while (atomic_load(&s_step_mutex_state) != 2) {
            taskYIELD();
        }
    }
}

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_top_status = NULL;
static lv_obj_t *s_lbl_trig_badge = NULL;
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser_wave = NULL;
static lv_chart_series_t *s_ser_trig = NULL;
static lv_obj_t *s_lbl_metrics_1 = NULL;
static lv_obj_t *s_lbl_metrics_2 = NULL;
static lv_obj_t *s_btn_run_lbl = NULL;
static lv_obj_t *s_btn_tb_lbl = NULL;
static lv_obj_t *s_btn_vs_lbl = NULL;
static lv_obj_t *s_btn_trig_lbl = NULL;
static lv_obj_t *s_btn_pin_lbl = NULL;

static bool is_supported_analog_pin(gpio_num_t pin)
{
    return (pin == BSP_PIN_ANALOG_IN_DEFAULT) || (pin == BSP_PIN_ANALOG_IN_ALT);
}

esp_err_t tool_scope_set_pin(gpio_num_t pin)
{
    if (!is_supported_analog_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_scope_lock);
    gpio_num_t prev_pin = s_selected_pin;
    bool active = s_tool_active;
    portEXIT_CRITICAL(&s_scope_lock);

    if (pin == prev_pin) {
        return ESP_OK;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim(pin, SCOPE_TOOL_ID);
        if (err != ESP_OK) {
            return err;
        }
        (void)bsp_pin_release(prev_pin, SCOPE_TOOL_ID);
    }

    portENTER_CRITICAL(&s_scope_lock);
    s_selected_pin = pin;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

gpio_num_t tool_scope_get_pin(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    gpio_num_t pin = s_selected_pin;
    portEXIT_CRITICAL(&s_scope_lock);
    return pin;
}

esp_err_t tool_scope_set_timebase(scope_timebase_t tb)
{
    if (tb < 0 || tb >= SCOPE_TB_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scope_lock);
    if (s_timebase != tb) {
        s_timebase = tb;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

scope_timebase_t tool_scope_get_timebase(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    scope_timebase_t tb = s_timebase;
    portEXIT_CRITICAL(&s_scope_lock);
    return tb;
}

esp_err_t tool_scope_set_vscale(scope_vscale_t vs)
{
    if (vs < 0 || vs >= SCOPE_VSCALE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scope_lock);
    s_vscale = vs;
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

scope_vscale_t tool_scope_get_vscale(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    scope_vscale_t vs = s_vscale;
    portEXIT_CRITICAL(&s_scope_lock);
    return vs;
}

esp_err_t tool_scope_set_trig_mode(scope_trig_mode_t mode)
{
    if (mode < 0 || mode >= SCOPE_TRIG_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scope_lock);
    s_trig_mode = mode;
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

scope_trig_mode_t tool_scope_get_trig_mode(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    scope_trig_mode_t mode = s_trig_mode;
    portEXIT_CRITICAL(&s_scope_lock);
    return mode;
}

esp_err_t tool_scope_set_trig_level_mv(uint16_t level_mv)
{
    if (level_mv > 3300U) {
        level_mv = 3300U;
    }
    portENTER_CRITICAL(&s_scope_lock);
    s_trig_level_mv = level_mv;
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

uint16_t tool_scope_get_trig_level_mv(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    uint16_t lvl = s_trig_level_mv;
    portEXIT_CRITICAL(&s_scope_lock);
    return lvl;
}

esp_err_t tool_scope_set_running(bool running)
{
    portENTER_CRITICAL(&s_scope_lock);
    if (s_running != running) {
        s_running = running;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

bool tool_scope_is_running(void)
{
    portENTER_CRITICAL(&s_scope_lock);
    bool r = s_running;
    portEXIT_CRITICAL(&s_scope_lock);
    return r;
}

esp_err_t tool_scope_get_latest_frame(scope_frame_t *out_frame)
{
    if (out_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scope_lock);
    *out_frame = s_latest_frame;
    portEXIT_CRITICAL(&s_scope_lock);
    return ESP_OK;
}

void tool_scope_set_sample_source(scope_sample_source_fn_t fn, void *user_ctx)
{
    portENTER_CRITICAL(&s_scope_lock);
    s_custom_source_fn = fn;
    s_custom_source_ctx = user_ctx;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_scope_lock);
}

/**
 * @brief Compute capture size and target sample interval for the active timebase.
 */
static void get_capture_params(scope_timebase_t tb,
                               size_t *out_sample_count,
                               uint32_t *out_interval_ns,
                               uint32_t *out_decim_step)
{
    /* 150 samples on either side leave a full 300-point window around a valid trigger. */
    size_t needed = SCOPE_CHART_POINTS + (SCOPE_CHART_POINTS / 2U);
    if (needed > SCOPE_CAPTURE_BUF_SAMPLES) {
        needed = SCOPE_CAPTURE_BUF_SAMPLES;
    }

    *out_sample_count = needed;
    *out_interval_ns = scope_dsp_target_sample_interval_ns(tb);
    *out_decim_step = 1U;
}

static bool should_abort_capture(uint32_t start_seq, bool is_bg_worker)
{
    portENTER_CRITICAL(&s_scope_lock);
    bool abort = s_stop_requested || !s_running || (s_capture_seq != start_seq) ||
                 (is_bg_worker && s_step_preempt);
    portEXIT_CRITICAL(&s_scope_lock);
    return abort;
}

static bool wait_until_us(int64_t deadline_us, uint32_t start_seq, bool is_bg_worker)
{
    int64_t remaining_us = deadline_us - esp_timer_get_time();
    while (remaining_us >= 1500) {
        if (should_abort_capture(start_seq, is_bg_worker)) {
            return false;
        }
        uint32_t delay_ms = (uint32_t)((remaining_us - 500) / 1000);
        if (delay_ms > 5U) {
            delay_ms = 5U;
        }
        TickType_t sleep_ticks = pdMS_TO_TICKS(delay_ms);
        if (sleep_ticks == 0) {
            sleep_ticks = 1;
        }
        vTaskDelay(sleep_ticks);
        remaining_us = deadline_us - esp_timer_get_time();
    }

    while (remaining_us > 0) {
        if (should_abort_capture(start_seq, is_bg_worker)) {
            return false;
        }
        if (remaining_us > 60) {
            esp_rom_delay_us(50U);
        } else if (remaining_us > 4) {
            esp_rom_delay_us((uint32_t)(remaining_us - 2));
        }
        remaining_us = deadline_us - esp_timer_get_time();
    }
    return !should_abort_capture(start_seq, is_bg_worker);
}

static int64_t sample_deadline_us(int64_t first_sample_us,
                                  size_t sample_index,
                                  uint32_t interval_ns)
{
    uint64_t elapsed_ns = (uint64_t)sample_index * interval_ns;
    return first_sample_us + (int64_t)((elapsed_ns + 500ULL) / 1000ULL);
}

static esp_err_t capture_adc_buffer(gpio_num_t pin,
                                    scope_timebase_t tb,
                                    uint32_t start_seq,
                                    bool is_bg_worker,
                                    uint16_t *out_raw,
                                    uint16_t *out_mv,
                                    size_t *out_count,
                                    uint32_t *out_sample_rate_hz,
                                    uint32_t *out_decim_step,
                                    uint64_t *out_timestamp_us,
                                    bool *out_aborted)
{
    *out_aborted = false;

    scope_sample_source_fn_t custom_fn = NULL;
    void *custom_ctx = NULL;
    portENTER_CRITICAL(&s_scope_lock);
    custom_fn = s_custom_source_fn;
    custom_ctx = s_custom_source_ctx;
    portEXIT_CRITICAL(&s_scope_lock);

    size_t cap_count = 0;
    uint32_t interval_ns = 0;
    uint32_t decim_step = 1;
    get_capture_params(tb, &cap_count, &interval_ns, &decim_step);

    if (custom_fn != NULL) {
        uint32_t custom_fs = 0U;
        *out_timestamp_us = (uint64_t)esp_timer_get_time();
        esp_err_t err = custom_fn(pin, out_raw, out_mv, cap_count, &custom_fs, custom_ctx);
        *out_count = cap_count;
        *out_sample_rate_hz = custom_fs;
        *out_decim_step = decim_step;
        return err;
    }

    int64_t first_sample_us = 0;
    int64_t last_sample_us = 0;
    for (size_t i = 0; i < cap_count; ++i) {
        if (should_abort_capture(start_seq, is_bg_worker)) {
            *out_aborted = true;
            return ESP_ERR_INVALID_STATE;
        }
        if (i > 0U && interval_ns > 0U) {
            if (!wait_until_us(sample_deadline_us(first_sample_us, i, interval_ns),
                               start_seq,
                               is_bg_worker)) {
                *out_aborted = true;
                return ESP_ERR_INVALID_STATE;
            }
        }

        int raw = 0;
        int mv = 0;
        if (bsp_adc1_read_raw(pin, &raw) != ESP_OK) {
            raw = 0;
        }
        int64_t sample_time_us = esp_timer_get_time();
        if (raw < 0) {
            raw = 0;
        } else if (raw > 4095) {
            raw = 4095;
        }
        if (bsp_adc1_raw_to_mv(pin, raw, &mv) != ESP_OK) {
            mv = (raw * 3300 + 2047) / 4095;
        }
        if (mv < 0) {
            mv = 0;
        } else if (mv > 3600) {
            mv = 3600;
        }
        out_raw[i] = (uint16_t)raw;
        out_mv[i] = (uint16_t)mv;
        last_sample_us = sample_time_us;
        if (i == 0U) {
            first_sample_us = last_sample_us;
        }
    }
    int64_t elapsed_us = last_sample_us - first_sample_us;
    if (elapsed_us <= 0 || cap_count < 2U) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t rate_numerator = (uint64_t)(cap_count - 1U) * 1000000ULL;
    uint64_t rate_hz = (rate_numerator + (uint64_t)elapsed_us / 2U) / (uint64_t)elapsed_us;

    *out_count = cap_count;
    *out_sample_rate_hz = (rate_hz > UINT32_MAX) ? UINT32_MAX : (uint32_t)rate_hz;
    *out_decim_step = decim_step;
    *out_timestamp_us = (uint64_t)first_sample_us;
    return ESP_OK;
}

static esp_err_t scope_worker_step_locked(bool is_bg_worker)
{
    if (s_raw_ring_buf == NULL || s_mv_ring_buf == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_scope_lock);
    gpio_num_t pin = s_selected_pin;
    scope_timebase_t tb = s_timebase;
    bool running = s_running;
    bool stop_req = s_stop_requested;
    uint32_t start_seq = s_capture_seq;
    portEXIT_CRITICAL(&s_scope_lock);

    if (stop_req || !running) {
        return ESP_OK;
    }

    size_t captured = 0;
    uint32_t fs_hz = 0;
    uint32_t decim_step = 1;
    uint64_t capture_timestamp_us = 0;
    bool aborted = false;
    esp_err_t err = capture_adc_buffer(pin,
                                       tb,
                                       start_seq,
                                       is_bg_worker,
                                       s_raw_ring_buf,
                                       s_mv_ring_buf,
                                       &captured,
                                       &fs_hz,
                                       &decim_step,
                                       &capture_timestamp_us,
                                       &aborted);
    if (aborted) {
        return ESP_OK;
    }
    if (err != ESP_OK || captured < 2U) {
        return err;
    }

    portENTER_CRITICAL(&s_scope_lock);
    scope_vscale_t vs = s_vscale;
    scope_trig_mode_t tmode = s_trig_mode;
    uint16_t tlvl = s_trig_level_mv;
    portEXIT_CRITICAL(&s_scope_lock);

    if (bsp_spk_passthrough_is_enabled()) {
        uint32_t spk_rate_hz = bsp_spk_passthrough_get_sample_rate();
        if (spk_rate_hz == 0U) {
            spk_rate_hz = BSP_SPK_DEFAULT_RATE_HZ;
        }
        size_t spk_cnt = scope_dsp_resample_for_speaker(
            s_raw_ring_buf, captured, fs_hz, spk_rate_hz, s_spk_chunk_buf, SCOPE_SPK_CHUNK_SAMPLES);
        if (spk_cnt > 0U) {
            (void)bsp_spk_passthrough_push_adc12(s_spk_chunk_buf, spk_cnt);
        }
    }

    if (recorder_sink_is_recording()) {
        recorder_frame_t rec_frame = {
            .tool_id = SCOPE_TOOL_ID,
            .sample_rate_hz = fs_hz,
            .timestamp_us = capture_timestamp_us,
            .samples = s_raw_ring_buf,
            .sample_count = captured,
            .bytes_per_sample = sizeof(uint16_t),
        };
        (void)recorder_sink_push(&rec_frame);
    }

    scope_trig_result_t trig_res = {0};
    size_t pre_trig = (SCOPE_CHART_POINTS / 2U) * (size_t)decim_step;
    if (pre_trig * 2U >= captured) {
        pre_trig = captured / 4U;
    }
    (void)scope_dsp_find_trigger(
        s_mv_ring_buf, captured, tmode, tlvl, SCOPE_DEFAULT_HYST_MV, pre_trig, &trig_res);

    scope_metrics_t metrics = {0};
    (void)scope_dsp_compute_metrics(s_mv_ring_buf, captured, fs_hz, &metrics);

    /* In normal RISING/FALLING mode, only update the waveform trace if triggered (or on first frame) */
    bool should_update_wave = trig_res.triggered || trig_res.auto_fallback ||
                              (s_latest_frame.sample_rate_hz == 0U);

    scope_frame_t *wptr = (scope_frame_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        if (should_update_wave) {
            (void)scope_dsp_extract_window(s_mv_ring_buf,
                                           captured,
                                           trig_res.trigger_index,
                                           decim_step,
                                           wptr->wave_mv,
                                           SCOPE_CHART_POINTS);
        } else {
            portENTER_CRITICAL(&s_scope_lock);
            memcpy(wptr->wave_mv, s_latest_frame.wave_mv, sizeof(wptr->wave_mv));
            portEXIT_CRITICAL(&s_scope_lock);
        }
        wptr->metrics = metrics;
        wptr->trig = trig_res;
        wptr->timebase = tb;
        wptr->vscale = vs;
        wptr->trig_mode = tmode;
        wptr->trig_level_mv = tlvl;
        wptr->pin = pin;
        wptr->sample_rate_hz = fs_hz;
        wptr->viewport_us = scope_dsp_viewport_us(fs_hz, decim_step, SCOPE_CHART_POINTS);

        portENTER_CRITICAL(&s_scope_lock);
        s_latest_frame = *wptr;
        portEXIT_CRITICAL(&s_scope_lock);

        (void)psram_double_buf_swap(&s_frame_dbuf);
    }

    return ESP_OK;
}

static esp_err_t scope_worker_step(void *ctx)
{
    (void)ctx;
    ensure_scope_step_mutex();
    if (xSemaphoreTake(s_step_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_OK;
    }
    esp_err_t err = scope_worker_step_locked(true);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

esp_err_t tool_scope_run_single_step(void)
{
    ensure_scope_step_mutex();
    portENTER_CRITICAL(&s_scope_lock);
    s_step_preempt = true;
    portEXIT_CRITICAL(&s_scope_lock);

    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    portENTER_CRITICAL(&s_scope_lock);
    s_step_preempt = false;
    portEXIT_CRITICAL(&s_scope_lock);

    esp_err_t err = scope_worker_step_locked(false);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Controls & Chart Rendering (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_run_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_scope_set_running(!tool_scope_is_running());
}

static void on_btn_tb_clicked(lv_event_t *e)
{
    (void)e;
    scope_timebase_t next = (scope_timebase_t)((tool_scope_get_timebase() + 1) % SCOPE_TB_COUNT);
    (void)tool_scope_set_timebase(next);
}

static void on_btn_vs_clicked(lv_event_t *e)
{
    (void)e;
    scope_vscale_t next = (scope_vscale_t)((tool_scope_get_vscale() + 1) % SCOPE_VSCALE_COUNT);
    (void)tool_scope_set_vscale(next);
}

static void on_btn_trig_mode_clicked(lv_event_t *e)
{
    (void)e;
    scope_trig_mode_t next = (scope_trig_mode_t)((tool_scope_get_trig_mode() + 1) % SCOPE_TRIG_MODE_COUNT);
    (void)tool_scope_set_trig_mode(next);
}

static void on_btn_trig_lvl_clicked(lv_event_t *e)
{
    (void)e;
    uint16_t cur = tool_scope_get_trig_level_mv();
    uint16_t next = (cur + SCOPE_TRIG_STEP_MV > 3150U) ? 300U : (uint16_t)(cur + SCOPE_TRIG_STEP_MV);
    (void)tool_scope_set_trig_level_mv(next);
}

static void on_btn_pin_clicked(lv_event_t *e)
{
    (void)e;
    gpio_num_t cur = tool_scope_get_pin();
    gpio_num_t next = (cur == BSP_PIN_ANALOG_IN_DEFAULT)
                          ? BSP_PIN_ANALOG_IN_ALT
                          : BSP_PIN_ANALOG_IN_DEFAULT;
    (void)tool_scope_set_pin(next);
}

static void build_scope_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 2, 0);
    lv_obj_set_style_pad_row(parent, 2, 0);

    /* 1. Top Settings & Trigger Status Row (18 px) */
    lv_obj_t *top_row = lv_obj_create(parent);
    lv_obj_set_size(top_row, 314, 18);
    lv_obj_set_style_bg_color(top_row, lv_color_hex(0x16202A), 0);
    lv_obj_set_style_border_width(top_row, 0, 0);
    lv_obj_set_style_radius(top_row, 3, 0);
    lv_obj_set_style_pad_hor(top_row, 4, 0);
    lv_obj_set_style_pad_ver(top_row, 1, 0);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_top_status = lv_label_create(top_row);
    lv_label_set_text(s_lbl_top_status, "G36  1ms/d  1.0V/d  T:1.65V");
    lv_obj_set_style_text_font(s_lbl_top_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_top_status, lv_color_hex(0x4DD0E1), 0);

    s_lbl_trig_badge = lv_label_create(top_row);
    lv_label_set_text(s_lbl_trig_badge, "AUTO");
    lv_obj_set_style_text_font(s_lbl_trig_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_trig_badge, lv_color_hex(0x00E676), 0);

    /* 2. 300-Point Waveform Chart (314 x 114 px) */
    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 314, 114);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, SCOPE_CHART_POINTS);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 3300);
    lv_chart_set_div_line_count(s_chart, SCOPE_VERT_DIVS - 1, SCOPE_HORIZ_DIVS - 1);
    lv_obj_set_style_bg_color(s_chart, lv_color_hex(0x080D12), 0);
    lv_obj_set_style_border_color(s_chart, lv_color_hex(0x263238), 0);
    lv_obj_set_style_border_width(s_chart, 1, 0);
    lv_obj_set_style_radius(s_chart, 3, 0);
    lv_obj_set_style_pad_all(s_chart, 2, 0);
    lv_obj_set_style_size(s_chart, 0, 0, LV_PART_INDICATOR);

    s_ser_trig = lv_chart_add_series(s_chart, lv_color_hex(0xFF7043), LV_CHART_AXIS_PRIMARY_Y);
    s_ser_wave = lv_chart_add_series(s_chart, lv_color_hex(0x00E676), LV_CHART_AXIS_PRIMARY_Y);

    /* 3. Live Waveform Metrics Rows (2 x 16 px) */
    s_lbl_metrics_1 = lv_label_create(parent);
    lv_label_set_text(s_lbl_metrics_1, "Vpp:0mV  Avg:0.00V  RMS:0.00V");
    lv_obj_set_style_text_font(s_lbl_metrics_1, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_metrics_1, lv_color_hex(0xFFE082), 0);

    s_lbl_metrics_2 = lv_label_create(parent);
    lv_label_set_text(s_lbl_metrics_2, "Freq:0.0Hz  Duty:0%  Min:0.00V Max:0.00V");
    lv_obj_set_style_text_font(s_lbl_metrics_2, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_metrics_2, lv_color_hex(0xB0BEC5), 0);

    /* 4. Bottom Touch Controls Bar (28 px) */
    lv_obj_t *ctrl_row = lv_obj_create(parent);
    lv_obj_set_size(ctrl_row, 314, 28);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_style_pad_column(ctrl_row, 3, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_run = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_run, 48, 26);
    lv_obj_set_style_bg_color(btn_run, lv_color_hex(0x2E7D32), 0);
    lv_obj_add_event_cb(btn_run, on_btn_run_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_run_lbl = lv_label_create(btn_run);
    lv_label_set_text(s_btn_run_lbl, "RUN");
    lv_obj_set_style_text_font(s_btn_run_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_run_lbl);

    lv_obj_t *btn_tb = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_tb, 56, 26);
    lv_obj_set_style_bg_color(btn_tb, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_tb, on_btn_tb_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_tb_lbl = lv_label_create(btn_tb);
    lv_label_set_text(s_btn_tb_lbl, "1ms/d");
    lv_obj_set_style_text_font(s_btn_tb_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_tb_lbl);

    lv_obj_t *btn_vs = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_vs, 56, 26);
    lv_obj_set_style_bg_color(btn_vs, lv_color_hex(0x0277BD), 0);
    lv_obj_add_event_cb(btn_vs, on_btn_vs_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_vs_lbl = lv_label_create(btn_vs);
    lv_label_set_text(s_btn_vs_lbl, "1.0V/d");
    lv_obj_set_style_text_font(s_btn_vs_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_vs_lbl);

    lv_obj_t *btn_trig = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_trig, 48, 26);
    lv_obj_set_style_bg_color(btn_trig, lv_color_hex(0x4527A0), 0);
    lv_obj_add_event_cb(btn_trig, on_btn_trig_mode_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_trig_lbl = lv_label_create(btn_trig);
    lv_label_set_text(s_btn_trig_lbl, "AUTO");
    lv_obj_set_style_text_font(s_btn_trig_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_trig_lbl);

    lv_obj_t *btn_tlvl = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_tlvl, 46, 26);
    lv_obj_set_style_bg_color(btn_tlvl, lv_color_hex(0x6A1B9A), 0);
    lv_obj_add_event_cb(btn_tlvl, on_btn_trig_lvl_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_tlvl = lv_label_create(btn_tlvl);
    lv_label_set_text(lbl_tlvl, "TLVL");
    lv_obj_set_style_text_font(lbl_tlvl, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_tlvl);

    lv_obj_t *btn_pin = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pin, 44, 26);
    lv_obj_set_style_bg_color(btn_pin, lv_color_hex(0x37474F), 0);
    lv_obj_add_event_cb(btn_pin, on_btn_pin_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_pin_lbl = lv_label_create(btn_pin);
    lv_label_set_text(s_btn_pin_lbl, "G36");
    lv_obj_set_style_text_font(s_btn_pin_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_pin_lbl);
}

static void format_effective_timebase(const scope_frame_t *frame, char *out_text, size_t out_len)
{
    if (frame == NULL || out_text == NULL || out_len == 0U) {
        return;
    }

    uint32_t requested_us_div = scope_dsp_timebase_us_per_div(frame->timebase);
    uint32_t actual_us_div = (frame->viewport_us + (SCOPE_HORIZ_DIVS / 2U)) / SCOPE_HORIZ_DIVS;
    uint32_t tolerance_us = requested_us_div / 20U;
    if (actual_us_div > 0U &&
        (actual_us_div > requested_us_div ? actual_us_div - requested_us_div
                                         : requested_us_div - actual_us_div) <= tolerance_us) {
        (void)snprintf(out_text, out_len, "%s", scope_dsp_timebase_to_str(frame->timebase));
    } else if (actual_us_div < 1000U) {
        (void)snprintf(out_text, out_len, "~%luus/d", (unsigned long)actual_us_div);
    } else {
        uint32_t tenths_ms = (actual_us_div + 50U) / 100U;
        (void)snprintf(out_text,
                       out_len,
                       "~%lu.%lums/d",
                       (unsigned long)(tenths_ms / 10U),
                       (unsigned long)(tenths_ms % 10U));
    }
}

static void scope_on_ui_refresh(void *ctx)
{
    (void)ctx;
    scope_frame_t frame = {0};
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &frame, 1, &copied) != ESP_OK || copied == 0) {
        return;
    }

    bool running = tool_scope_is_running();
    gpio_num_t pin = tool_scope_get_pin();
    scope_vscale_t vs = tool_scope_get_vscale();
    scope_trig_mode_t tmode = tool_scope_get_trig_mode();
    uint16_t tlvl = tool_scope_get_trig_level_mv();
    char effective_tb[24];
    format_effective_timebase(&frame, effective_tb, sizeof(effective_tb));

    if (s_lbl_top_status != NULL) {
        lv_label_set_text_fmt(s_lbl_top_status,
                              "G%d %s %s T:%d.%02dV",
                              (int)pin,
                              effective_tb,
                              scope_dsp_vscale_to_str(vs),
                              (int)(tlvl / 1000U),
                              (int)((tlvl % 1000U) / 10U));
    }

    if (s_lbl_trig_badge != NULL) {
        if (!running) {
            lv_label_set_text(s_lbl_trig_badge, "STOP");
            lv_obj_set_style_text_color(s_lbl_trig_badge, lv_color_hex(0xFF5252), 0);
        } else if (frame.trig.triggered) {
            lv_label_set_text(s_lbl_trig_badge, "TRIG'D");
            lv_obj_set_style_text_color(s_lbl_trig_badge, lv_color_hex(0x00E676), 0);
        } else if (frame.trig.auto_fallback) {
            lv_label_set_text(s_lbl_trig_badge, "AUTO");
            lv_obj_set_style_text_color(s_lbl_trig_badge, lv_color_hex(0xFFD54F), 0);
        } else {
            lv_label_set_text(s_lbl_trig_badge, "WAIT");
            lv_obj_set_style_text_color(s_lbl_trig_badge, lv_color_hex(0x90A4AE), 0);
        }
    }

    if (s_chart != NULL && s_ser_wave != NULL && s_ser_trig != NULL) {
        uint16_t mv_per_div = scope_dsp_vscale_mv_per_div(vs);
        int32_t span_mv = (int32_t)mv_per_div * SCOPE_VERT_DIVS;
        int32_t y_max = (span_mv > 3300) ? 3300 : span_mv;
        lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, y_max);

        int32_t *wave_pts = lv_chart_get_y_array(s_chart, s_ser_wave);
        int32_t *trig_pts = lv_chart_get_y_array(s_chart, s_ser_trig);
        if (wave_pts != NULL && trig_pts != NULL) {
            for (size_t i = 0; i < SCOPE_CHART_POINTS; ++i) {
                wave_pts[i] = (int32_t)frame.wave_mv[i];
                trig_pts[i] = (int32_t)tlvl;
            }
            lv_chart_refresh(s_chart);
        }
    }

    int vpp_int = (int)lroundf(frame.metrics.vpp_mv);
    int vavg_int = (int)lroundf(frame.metrics.vavg_mv);
    int vrms_int = (int)lroundf(frame.metrics.vrms_mv);
    int vmin_int = (int)lroundf(frame.metrics.vmin_mv);
    int vmax_int = (int)lroundf(frame.metrics.vmax_mv);
    int freq_tenths = (int)lroundf(frame.metrics.freq_hz * 10.0f);
    int duty_int = (int)lroundf(frame.metrics.duty_pct);

    if (s_lbl_metrics_1 != NULL) {
        lv_label_set_text_fmt(s_lbl_metrics_1,
                              "Vpp:%dmV  Avg:%d.%02dV  RMS:%d.%02dV",
                              vpp_int,
                              vavg_int / 1000, abs((vavg_int % 1000) / 10),
                              vrms_int / 1000, abs((vrms_int % 1000) / 10));
    }
    if (s_lbl_metrics_2 != NULL) {
        lv_label_set_text_fmt(s_lbl_metrics_2,
                              "F:%d.%dHz D:%d%% Min:%d.%02dV Max:%d.%02dV",
                              freq_tenths / 10, abs(freq_tenths % 10),
                              duty_int,
                              vmin_int / 1000, abs((vmin_int % 1000) / 10),
                              vmax_int / 1000, abs((vmax_int % 1000) / 10));
    }
    if (s_btn_run_lbl != NULL) {
        lv_label_set_text(s_btn_run_lbl, running ? "RUN" : "STOP");
    }
    if (s_btn_tb_lbl != NULL) {
        lv_label_set_text(s_btn_tb_lbl, effective_tb);
    }
    if (s_btn_vs_lbl != NULL) {
        lv_label_set_text(s_btn_vs_lbl, scope_dsp_vscale_to_str(vs));
    }
    if (s_btn_trig_lbl != NULL) {
        lv_label_set_text(s_btn_trig_lbl, scope_dsp_trig_mode_to_str(tmode));
    }
    if (s_btn_pin_lbl != NULL) {
        lv_label_set_text(s_btn_pin_lbl, (pin == BSP_PIN_ANALOG_IN_ALT) ? "G35" : "G36");
    }
}

/* --------------------------------------------------------------------------
 * Tool Plugin Lifecycle Callbacks
 * -------------------------------------------------------------------------- */

static void free_scope_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    if (s_raw_ring_buf != NULL) {
        psram_buf_free(s_raw_ring_buf);
        s_raw_ring_buf = NULL;
    }
    if (s_mv_ring_buf != NULL) {
        psram_buf_free(s_mv_ring_buf);
        s_mv_ring_buf = NULL;
    }
}

static esp_err_t scope_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    gpio_num_t pin = tool_scope_get_pin();

    esp_err_t err = bsp_pin_claim(pin, SCOPE_TOOL_ID);
    if (err != ESP_OK) {
        return err;
    }

    err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(scope_frame_t));
    if (err != ESP_OK) {
        (void)bsp_pin_release(pin, SCOPE_TOOL_ID);
        return err;
    }

    s_raw_ring_buf = (uint16_t *)psram_buf_calloc(SCOPE_CAPTURE_BUF_SAMPLES, sizeof(uint16_t));
    s_mv_ring_buf = (uint16_t *)psram_buf_calloc(SCOPE_CAPTURE_BUF_SAMPLES, sizeof(uint16_t));
    if (s_raw_ring_buf == NULL || s_mv_ring_buf == NULL) {
        free_scope_buffers();
        (void)bsp_pin_release(pin, SCOPE_TOOL_ID);
        return ESP_ERR_NO_MEM;
    }

    ensure_scope_step_mutex();

    portENTER_CRITICAL(&s_scope_lock);
    s_running = true;
    s_stop_requested = false;
    s_step_preempt = false;
    s_capture_seq++;
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    s_tool_active = true;
    scope_sample_source_fn_t custom_fn = s_custom_source_fn;
    scope_timebase_t start_tb = s_timebase;
    portEXIT_CRITICAL(&s_scope_lock);

    if (parent_cont != NULL) {
        build_scope_ui(parent_cont);
    }

    /* Run an immediate initial capture step for fast timebases or injected sources
     * so a frame is ready before first UI refresh without blocking on slow sweeps. */
    if (custom_fn != NULL || start_tb <= SCOPE_TB_2MS_DIV) {
        (void)tool_scope_run_single_step();
    }

    tool_worker_config_t wcfg = {
        .name = "scope_c1",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = TOOL_WORKER_DEFAULT_CORE,
        .period_ms = SCOPE_WORKER_PERIOD_MS,
        .step_fn = scope_worker_step,
        .user_ctx = (void *)1,
    };
    err = tool_worker_start(&s_worker, &wcfg);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_scope_lock);
        s_tool_active = false;
        portEXIT_CRITICAL(&s_scope_lock);
        free_scope_buffers();
        (void)bsp_pin_release(pin, SCOPE_TOOL_ID);
        return err;
    }

    ESP_LOGI(TAG, "Oscilloscope started on GPIO%d", (int)pin);
    return ESP_OK;
}

static esp_err_t scope_on_stop(bool keep_background, void *ctx)
{
    (void)keep_background;
    (void)ctx;

    portENTER_CRITICAL(&s_scope_lock);
    s_stop_requested = true;
    s_capture_seq++;
    tool_worker_t *worker = s_worker;
    s_worker = NULL;
    portEXIT_CRITICAL(&s_scope_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < SCOPE_WORKER_STOP_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, SCOPE_WORKER_STOP_WAIT_MS);
        }
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop scope worker cleanly: %s", esp_err_to_name(stop_err));
            portENTER_CRITICAL(&s_scope_lock);
            s_worker = worker;
            portEXIT_CRITICAL(&s_scope_lock);
            return stop_err;
        }
    }

    ensure_scope_step_mutex();
    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    s_lbl_top_status = NULL;
    s_lbl_trig_badge = NULL;
    s_chart = NULL;
    s_ser_wave = NULL;
    s_ser_trig = NULL;
    s_lbl_metrics_1 = NULL;
    s_lbl_metrics_2 = NULL;
    s_btn_run_lbl = NULL;
    s_btn_tb_lbl = NULL;
    s_btn_vs_lbl = NULL;
    s_btn_trig_lbl = NULL;
    s_btn_pin_lbl = NULL;

    free_scope_buffers();

    portENTER_CRITICAL(&s_scope_lock);
    gpio_num_t pin = s_selected_pin;
    s_tool_active = false;
    portEXIT_CRITICAL(&s_scope_lock);

    (void)xSemaphoreGive(s_step_mutex);

    (void)bsp_pin_release(pin, SCOPE_TOOL_ID);
    return ESP_OK;
}

static esp_err_t scope_selftest(void)
{
    uint16_t wave[64];
    for (size_t i = 0; i < 64; ++i) {
        wave[i] = ((i % 16U) < 8U) ? 200U : 2800U;
    }

    scope_trig_result_t trig = {0};
    if (scope_dsp_find_trigger(wave, 64, SCOPE_TRIG_RISING, 1500U, 50U, 4U, &trig) != ESP_OK) {
        return ESP_FAIL;
    }
    if (!trig.triggered || trig.trigger_index != 8U) {
        return ESP_FAIL;
    }

    scope_metrics_t m = {0};
    if (scope_dsp_compute_metrics(wave, 64, 16000U, &m) != ESP_OK) {
        return ESP_FAIL;
    }
    if (fabsf(m.vpp_mv - 2600.0f) > 1.0f || fabsf(m.freq_hz - 1000.0f) > 10.0f ||
        fabsf(m.duty_pct - 50.0f) > 2.0f) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_scope_descriptor = {
    .id = SCOPE_TOOL_ID,
    .title = "Oscilloscope",
    .symbol = LV_SYMBOL_IMAGE,
    .category = TOOL_CATEGORY_ANALOG_IN,
    .supports_background = false,
    .supports_recording = true,
    .supports_spk_passthrough = true,
    .on_init = NULL,
    .on_start = scope_on_start,
    .on_stop = scope_on_stop,
    .on_ui_refresh = scope_on_ui_refresh,
    .is_running_bg = NULL,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_scope_get_descriptor(void)
{
    return &s_scope_descriptor;
}

esp_err_t tool_scope_register(void)
{
    esp_err_t err = tool_registry_register(&s_scope_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("scope_dsp", scope_selftest);
    }
    return err;
}
