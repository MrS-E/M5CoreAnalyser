/**
 * @file tool_voltmeter.c
 * @brief Core 1 ADC1 acquisition worker, NVS calibration persistence, PSRAM double-buffered
 *        frame pipeline, speaker passthrough, recorder sink export, and LVGL 9 UI for `tool_voltmeter`.
 */

#include "tool_voltmeter.h"
#include "tool_voltmeter_priv.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "psram_buffer.h"
#include "recorder_sink.h"
#include "tool_core.h"

static const char *TAG = "tool_voltmeter";

#define VOLTMETER_NVS_NAMESPACE     "volt_cal"
#define VOLTMETER_WORKER_PERIOD_MS  (25)
#define VOLTMETER_WORKER_STOP_MS    (1000)
#define VOLTMETER_WORKER_RETRIES    (3)

/**
 * @brief Frame payload transferred from Core 1 worker to Core 0 UI via `psram_double_buf_t`.
 */
typedef struct {
    voltmeter_metrics_t metrics;
    gpio_num_t pin;
    voltmeter_cal_t cal;
    bool hold;
} voltmeter_frame_t;

static portMUX_TYPE s_volt_lock = portMUX_INITIALIZER_UNLOCKED;
static gpio_num_t s_selected_pin = BSP_PIN_ANALOG_IN_DEFAULT;
static bool s_hold_enabled = false;
static bool s_reset_requested = false;
static bool s_tool_active = false;
static volatile bool s_stop_requested = false;
static bool s_nvs_ready = false;

static voltmeter_cal_t s_cal_g36 = {.slope = 1.0f, .offset_mv = 0.0f};
static voltmeter_cal_t s_cal_g35 = {.slope = 1.0f, .offset_mv = 0.0f};
static bool s_cal_cached_init = false;

static voltmeter_metrics_t s_latest_metrics = {0};
static psram_double_buf_t s_frame_dbuf = {0};
static voltmeter_avg_filter_t s_avg_filter = {0};
static uint16_t *s_raw_batch = NULL;
static float *s_mv_batch = NULL;
static tool_worker_t *s_worker = NULL;

static voltmeter_sample_source_fn_t s_custom_source_fn = NULL;
static void *s_custom_source_ctx = NULL;

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_pin_info = NULL;
static lv_obj_t *s_lbl_cal_info = NULL;
static lv_obj_t *s_lbl_vdc_main = NULL;
static lv_obj_t *s_lbl_vdc_mv = NULL;
static lv_obj_t *s_lbl_ac_rms = NULL;
static lv_obj_t *s_lbl_true_rms = NULL;
static lv_obj_t *s_bar_voltage = NULL;
static lv_obj_t *s_lbl_stats = NULL;
static lv_obj_t *s_btn_pin_lbl = NULL;
static lv_obj_t *s_btn_hold_lbl = NULL;
static lv_obj_t *s_btn_spk_lbl = NULL;

static bool is_supported_analog_pin(gpio_num_t pin)
{
    return (pin == BSP_PIN_ANALOG_IN_DEFAULT) || (pin == BSP_PIN_ANALOG_IN_ALT);
}

static bool is_valid_cal(const voltmeter_cal_t *cal)
{
    if (cal == NULL || !isfinite(cal->slope) || !isfinite(cal->offset_mv)) {
        return false;
    }
    return (cal->slope >= 0.5f && cal->slope <= 2.0f &&
            fabsf(cal->offset_mv) <= 1000.0f);
}

static const char *pin_nvs_key(gpio_num_t pin)
{
    return (pin == BSP_PIN_ANALOG_IN_ALT) ? "cal_g35" : "cal_g36";
}

static void ensure_cal_cache_init(void)
{
    portENTER_CRITICAL(&s_volt_lock);
    if (!s_cal_cached_init) {
        voltmeter_dsp_cal_init_default(&s_cal_g36);
        voltmeter_dsp_cal_init_default(&s_cal_g35);
        s_cal_cached_init = true;
    }
    portEXIT_CRITICAL(&s_volt_lock);
}

static void ensure_nvs_initialized(void)
{
    if (s_nvs_ready) {
        return;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err == ESP_OK) {
        s_nvs_ready = true;
    }
}

static voltmeter_cal_t get_cached_cal_for_pin(gpio_num_t pin)
{
    ensure_cal_cache_init();
    voltmeter_cal_t cal;
    portENTER_CRITICAL(&s_volt_lock);
    cal = (pin == BSP_PIN_ANALOG_IN_ALT) ? s_cal_g35 : s_cal_g36;
    portEXIT_CRITICAL(&s_volt_lock);
    return cal;
}

static void set_cached_cal_for_pin(gpio_num_t pin, const voltmeter_cal_t *cal)
{
    ensure_cal_cache_init();
    portENTER_CRITICAL(&s_volt_lock);
    if (pin == BSP_PIN_ANALOG_IN_ALT) {
        s_cal_g35 = *cal;
    } else {
        s_cal_g36 = *cal;
    }
    portEXIT_CRITICAL(&s_volt_lock);
}

esp_err_t tool_voltmeter_save_cal(gpio_num_t pin, const voltmeter_cal_t *cal)
{
    if (!is_supported_analog_pin(pin) || !is_valid_cal(cal)) {
        return ESP_ERR_INVALID_ARG;
    }

    set_cached_cal_for_pin(pin, cal);

    ensure_nvs_initialized();
    if (s_nvs_ready) {
        nvs_handle_t handle = 0;
        if (nvs_open(VOLTMETER_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            (void)nvs_set_blob(handle, pin_nvs_key(pin), cal, sizeof(voltmeter_cal_t));
            (void)nvs_commit(handle);
            nvs_close(handle);
        }
    }
    return ESP_OK;
}

esp_err_t tool_voltmeter_load_cal(gpio_num_t pin, voltmeter_cal_t *out_cal)
{
    if (!is_supported_analog_pin(pin) || out_cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ensure_cal_cache_init();
    ensure_nvs_initialized();

    if (s_nvs_ready) {
        nvs_handle_t handle = 0;
        if (nvs_open(VOLTMETER_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            voltmeter_cal_t stored = {0};
            size_t len = sizeof(stored);
            esp_err_t err = nvs_get_blob(handle, pin_nvs_key(pin), &stored, &len);
            nvs_close(handle);
            if (err == ESP_OK && len == sizeof(stored) && is_valid_cal(&stored)) {
                set_cached_cal_for_pin(pin, &stored);
                *out_cal = stored;
                return ESP_OK;
            }
        }
    }

    *out_cal = get_cached_cal_for_pin(pin);
    return ESP_OK;
}

esp_err_t tool_voltmeter_reset_cal(gpio_num_t pin)
{
    if (!is_supported_analog_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    voltmeter_cal_t unity;
    voltmeter_dsp_cal_init_default(&unity);
    set_cached_cal_for_pin(pin, &unity);

    ensure_nvs_initialized();
    if (s_nvs_ready) {
        nvs_handle_t handle = 0;
        if (nvs_open(VOLTMETER_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            (void)nvs_erase_key(handle, pin_nvs_key(pin));
            (void)nvs_commit(handle);
            nvs_close(handle);
        }
    }
    return ESP_OK;
}

esp_err_t tool_voltmeter_set_pin(gpio_num_t pin)
{
    if (!is_supported_analog_pin(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_volt_lock);
    gpio_num_t prev_pin = s_selected_pin;
    bool active = s_tool_active;
    portEXIT_CRITICAL(&s_volt_lock);

    if (pin == prev_pin) {
        return ESP_OK;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim(pin, VOLTMETER_TOOL_ID);
        if (err != ESP_OK) {
            return err;
        }
        (void)bsp_pin_release(prev_pin, VOLTMETER_TOOL_ID);
    }

    portENTER_CRITICAL(&s_volt_lock);
    s_selected_pin = pin;
    s_reset_requested = true;
    portEXIT_CRITICAL(&s_volt_lock);
    return ESP_OK;
}

gpio_num_t tool_voltmeter_get_pin(void)
{
    portENTER_CRITICAL(&s_volt_lock);
    gpio_num_t pin = s_selected_pin;
    portEXIT_CRITICAL(&s_volt_lock);
    return pin;
}

esp_err_t tool_voltmeter_set_hold(bool hold)
{
    portENTER_CRITICAL(&s_volt_lock);
    s_hold_enabled = hold;
    portEXIT_CRITICAL(&s_volt_lock);
    return ESP_OK;
}

bool tool_voltmeter_is_hold(void)
{
    portENTER_CRITICAL(&s_volt_lock);
    bool hold = s_hold_enabled;
    portEXIT_CRITICAL(&s_volt_lock);
    return hold;
}

esp_err_t tool_voltmeter_reset_stats(void)
{
    portENTER_CRITICAL(&s_volt_lock);
    s_reset_requested = true;
    if (s_avg_filter.window_mv != NULL) {
        voltmeter_dsp_avg_reset(&s_avg_filter);
    }
    s_latest_metrics.vmin_mv = 0.0f;
    s_latest_metrics.vmax_mv = 0.0f;
    s_latest_metrics.vavg_mv = 0.0f;
    s_latest_metrics.sample_count = 0;
    portEXIT_CRITICAL(&s_volt_lock);
    return ESP_OK;
}

esp_err_t tool_voltmeter_get_latest_metrics(voltmeter_metrics_t *out_metrics)
{
    if (out_metrics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_volt_lock);
    *out_metrics = s_latest_metrics;
    portEXIT_CRITICAL(&s_volt_lock);
    return ESP_OK;
}

void tool_voltmeter_set_sample_source(voltmeter_sample_source_fn_t fn, void *user_ctx)
{
    portENTER_CRITICAL(&s_volt_lock);
    s_custom_source_fn = fn;
    s_custom_source_ctx = user_ctx;
    portEXIT_CRITICAL(&s_volt_lock);
}

static esp_err_t read_adc_batch(gpio_num_t pin,
                               uint16_t *out_raw,
                               float *out_mv,
                               size_t count,
                               uint32_t *out_sample_rate_hz,
                               uint64_t *out_timestamp_us)
{
    if (out_sample_rate_hz == NULL || out_timestamp_us == NULL || count == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    voltmeter_sample_source_fn_t custom_fn = NULL;
    void *custom_ctx = NULL;
    portENTER_CRITICAL(&s_volt_lock);
    custom_fn = s_custom_source_fn;
    custom_ctx = s_custom_source_ctx;
    portEXIT_CRITICAL(&s_volt_lock);

    int64_t batch_start_us = esp_timer_get_time();
    *out_timestamp_us = (uint64_t)batch_start_us;
    *out_sample_rate_hz = 0U;
    if (custom_fn != NULL) {
        return custom_fn(pin, out_raw, out_mv, count, out_sample_rate_hz, custom_ctx);
    }

    int64_t last_sample_us = 0;
    for (size_t i = 0; i < count; ++i) {
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
        out_raw[i] = (uint16_t)raw;
        out_mv[i] = (float)mv;
        last_sample_us = sample_time_us;
        if (i == 0U) {
            batch_start_us = sample_time_us;
            *out_timestamp_us = (uint64_t)sample_time_us;
        }
    }

    int64_t elapsed_us = last_sample_us - batch_start_us;
    if (count > 1U && elapsed_us > 0) {
        uint64_t rate_numerator = (uint64_t)(count - 1U) * 1000000ULL;
        uint64_t rate_hz = (rate_numerator + (uint64_t)elapsed_us / 2U) / (uint64_t)elapsed_us;
        *out_sample_rate_hz = (rate_hz > UINT32_MAX) ? UINT32_MAX : (uint32_t)rate_hz;
    }
    return ESP_OK;
}

static esp_err_t voltmeter_worker_step(void *ctx)
{
    (void)ctx;
    if (s_raw_batch == NULL || s_mv_batch == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_volt_lock);
    gpio_num_t pin = s_selected_pin;
    bool hold = s_hold_enabled;
    bool stop_req = s_stop_requested;
    bool do_reset = s_reset_requested;
    s_reset_requested = false;
    voltmeter_cal_t cal = (pin == BSP_PIN_ANALOG_IN_ALT) ? s_cal_g35 : s_cal_g36;
    portEXIT_CRITICAL(&s_volt_lock);

    if (stop_req) {
        return ESP_OK;
    }

    if (do_reset && s_avg_filter.window_mv != NULL) {
        voltmeter_dsp_avg_reset(&s_avg_filter);
    }

    const size_t batch_count = VOLTMETER_DEFAULT_BATCH_SAMPLES;
    uint32_t batch_sample_rate_hz = 0U;
    uint64_t batch_timestamp_us = 0U;
    esp_err_t err = read_adc_batch(pin,
                                   s_raw_batch,
                                   s_mv_batch,
                                   batch_count,
                                   &batch_sample_rate_hz,
                                   &batch_timestamp_us);
    if (err != ESP_OK) {
        return err;
    }

    if (bsp_spk_passthrough_is_enabled()) {
        (void)bsp_spk_passthrough_push_adc12(s_raw_batch, batch_count);
    }

    if (recorder_sink_is_recording()) {
        recorder_frame_t rec_frame = {
            .tool_id = VOLTMETER_TOOL_ID,
            .sample_rate_hz = batch_sample_rate_hz,
            .timestamp_us = batch_timestamp_us,
            .samples = s_raw_batch,
            .sample_count = batch_count,
            .bytes_per_sample = sizeof(uint16_t),
        };
        (void)recorder_sink_push(&rec_frame);
    }

    if (!hold) {
        voltmeter_metrics_t metrics = {0};
        err = voltmeter_dsp_process_batch(
            s_raw_batch, s_mv_batch, batch_count, &cal, &s_avg_filter, &metrics);
        if (err == ESP_OK) {
            portENTER_CRITICAL(&s_volt_lock);
            s_latest_metrics = metrics;
            portEXIT_CRITICAL(&s_volt_lock);

            voltmeter_frame_t *wptr = (voltmeter_frame_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
            if (wptr != NULL) {
                wptr->metrics = metrics;
                wptr->pin = pin;
                wptr->cal = cal;
                wptr->hold = hold;
                (void)psram_double_buf_swap(&s_frame_dbuf);
            }
        }
    }

    return ESP_OK;
}

esp_err_t tool_voltmeter_run_single_step(void)
{
    return voltmeter_worker_step(NULL);
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Event Callbacks & Rendering (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_pin_clicked(lv_event_t *e)
{
    (void)e;
    gpio_num_t cur = tool_voltmeter_get_pin();
    gpio_num_t next = (cur == BSP_PIN_ANALOG_IN_DEFAULT)
                          ? BSP_PIN_ANALOG_IN_ALT
                          : BSP_PIN_ANALOG_IN_DEFAULT;
    (void)tool_voltmeter_set_pin(next);
}

static void on_btn_hold_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_voltmeter_set_hold(!tool_voltmeter_is_hold());
}

static void on_btn_reset_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_voltmeter_reset_stats();
}

static void on_btn_cal_reset_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_voltmeter_reset_cal(tool_voltmeter_get_pin());
}

static void on_btn_spk_clicked(lv_event_t *e)
{
    (void)e;
    if (!bsp_spk_passthrough_is_enabled()) {
        (void)bsp_spk_passthrough_enable(true, VOLTMETER_DEFAULT_SAMPLE_RATE_HZ);
        (void)bsp_spk_passthrough_set_mute(false);
    } else {
        (void)bsp_spk_passthrough_enable(false, 0);
    }
}

static void build_voltmeter_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 4, 0);
    lv_obj_set_style_pad_row(parent, 4, 0);

    /* 1. Top Info Bar (Pin & Calibration Info) */
    lv_obj_t *top_row = lv_obj_create(parent);
    lv_obj_set_size(top_row, 310, 22);
    lv_obj_set_style_bg_color(top_row, lv_color_hex(0x16202A), 0);
    lv_obj_set_style_border_width(top_row, 0, 0);
    lv_obj_set_style_radius(top_row, 4, 0);
    lv_obj_set_style_pad_hor(top_row, 6, 0);
    lv_obj_set_style_pad_ver(top_row, 2, 0);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_pin_info = lv_label_create(top_row);
    lv_label_set_text(s_lbl_pin_info, "IN: GPIO36 (ADC1_CH0)");
    lv_obj_set_style_text_font(s_lbl_pin_info, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_pin_info, lv_color_hex(0x4DD0E1), 0);

    s_lbl_cal_info = lv_label_create(top_row);
    lv_label_set_text(s_lbl_cal_info, "CAL 1.00x +0mV");
    lv_obj_set_style_text_font(s_lbl_cal_info, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_cal_info, lv_color_hex(0xB0BEC5), 0);

    /* 2. Primary Digital Readout Card */
    lv_obj_t *readout_card = lv_obj_create(parent);
    lv_obj_set_size(readout_card, 310, 86);
    lv_obj_set_style_bg_color(readout_card, lv_color_hex(0x101B24), 0);
    lv_obj_set_style_border_color(readout_card, lv_color_hex(0x00897B), 0);
    lv_obj_set_style_border_width(readout_card, 1, 0);
    lv_obj_set_style_radius(readout_card, 6, 0);
    lv_obj_set_style_pad_all(readout_card, 6, 0);
    lv_obj_set_flex_flow(readout_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(readout_card, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *main_row = lv_obj_create(readout_card);
    lv_obj_set_size(main_row, 294, 42);
    lv_obj_set_style_bg_opa(main_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(main_row, 0, 0);
    lv_obj_set_style_pad_all(main_row, 0, 0);
    lv_obj_set_flex_flow(main_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(main_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_vdc_main = lv_label_create(main_row);
    lv_label_set_text(s_lbl_vdc_main, "0.000 V");
    lv_obj_set_style_text_font(s_lbl_vdc_main, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_lbl_vdc_main, lv_color_hex(0x00E676), 0);

    s_lbl_vdc_mv = lv_label_create(main_row);
    lv_label_set_text(s_lbl_vdc_mv, "0.0 mV DC");
    lv_obj_set_style_text_font(s_lbl_vdc_mv, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lbl_vdc_mv, lv_color_hex(0x80CBC4), 0);

    lv_obj_t *rms_row = lv_obj_create(readout_card);
    lv_obj_set_size(rms_row, 294, 22);
    lv_obj_set_style_bg_opa(rms_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(rms_row, 0, 0);
    lv_obj_set_style_pad_all(rms_row, 0, 0);
    lv_obj_set_flex_flow(rms_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(rms_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_ac_rms = lv_label_create(rms_row);
    lv_label_set_text(s_lbl_ac_rms, "AC RMS: 0.0 mV");
    lv_obj_set_style_text_font(s_lbl_ac_rms, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_ac_rms, lv_color_hex(0xFFD54F), 0);

    s_lbl_true_rms = lv_label_create(rms_row);
    lv_label_set_text(s_lbl_true_rms, "True RMS: 0.000 V");
    lv_obj_set_style_text_font(s_lbl_true_rms, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_true_rms, lv_color_hex(0xCFD8DC), 0);

    /* 3. 0..3300 mV Range Bar */
    s_bar_voltage = lv_bar_create(parent);
    lv_obj_set_size(s_bar_voltage, 310, 14);
    lv_bar_set_range(s_bar_voltage, 0, 3300);
    lv_bar_set_value(s_bar_voltage, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_bar_voltage, lv_color_hex(0x263238), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar_voltage, lv_color_hex(0x00E676), LV_PART_INDICATOR);

    /* 4. Session Min / Max / Avg / Vpp Row */
    s_lbl_stats = lv_label_create(parent);
    lv_label_set_text(s_lbl_stats, "Min: 0.00V  Max: 0.00V  Avg: 0.00V  Vpp: 0mV");
    lv_obj_set_style_text_font(s_lbl_stats, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_stats, lv_color_hex(0xB0BEC5), 0);

    /* 5. Bottom Touch Control Buttons */
    lv_obj_t *ctrl_row = lv_obj_create(parent);
    lv_obj_set_size(ctrl_row, 310, 34);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_style_pad_column(ctrl_row, 4, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_pin = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pin, 58, 30);
    lv_obj_set_style_bg_color(btn_pin, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_pin, on_btn_pin_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_pin_lbl = lv_label_create(btn_pin);
    lv_label_set_text(s_btn_pin_lbl, "G36");
    lv_obj_set_style_text_font(s_btn_pin_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_pin_lbl);

    lv_obj_t *btn_hold = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_hold, 58, 30);
    lv_obj_set_style_bg_color(btn_hold, lv_color_hex(0x37474F), 0);
    lv_obj_add_event_cb(btn_hold, on_btn_hold_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_hold_lbl = lv_label_create(btn_hold);
    lv_label_set_text(s_btn_hold_lbl, "LIVE");
    lv_obj_set_style_text_font(s_btn_hold_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_hold_lbl);

    lv_obj_t *btn_rst = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_rst, 58, 30);
    lv_obj_set_style_bg_color(btn_rst, lv_color_hex(0x455A64), 0);
    lv_obj_add_event_cb(btn_rst, on_btn_reset_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_rst = lv_label_create(btn_rst);
    lv_label_set_text(lbl_rst, "RST");
    lv_obj_set_style_text_font(lbl_rst, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_rst);

    lv_obj_t *btn_cal = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_cal, 60, 30);
    lv_obj_set_style_bg_color(btn_cal, lv_color_hex(0x546E7A), 0);
    lv_obj_add_event_cb(btn_cal, on_btn_cal_reset_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_cal = lv_label_create(btn_cal);
    lv_label_set_text(lbl_cal, "CAL 1x");
    lv_obj_set_style_text_font(lbl_cal, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_cal);

    lv_obj_t *btn_spk = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_spk, 58, 30);
    lv_obj_set_style_bg_color(btn_spk, lv_color_hex(0x283593), 0);
    lv_obj_add_event_cb(btn_spk, on_btn_spk_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_spk_lbl = lv_label_create(btn_spk);
    lv_label_set_text(s_btn_spk_lbl, "SPK");
    lv_obj_set_style_text_font(s_btn_spk_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_spk_lbl);
}

static void voltmeter_on_ui_refresh(void *ctx)
{
    (void)ctx;
    voltmeter_frame_t frame = {0};
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &frame, 1, &copied) != ESP_OK || copied == 0) {
        return;
    }

    gpio_num_t cur_pin = tool_voltmeter_get_pin();
    bool hold = tool_voltmeter_is_hold();
    voltmeter_cal_t cal = get_cached_cal_for_pin(cur_pin);

    if (s_lbl_pin_info != NULL) {
        lv_label_set_text_fmt(s_lbl_pin_info,
                              "IN: GPIO%d (%s)%s",
                              (int)cur_pin,
                              (cur_pin == BSP_PIN_ANALOG_IN_ALT) ? "ADC1_CH7" : "ADC1_CH0",
                              hold ? " [HOLD]" : "");
    }
    if (s_lbl_cal_info != NULL) {
        int slope_milli = (int)lroundf(cal.slope * 1000.0f);
        int offset_int = (int)lroundf(cal.offset_mv);
        lv_label_set_text_fmt(s_lbl_cal_info,
                              "CAL %d.%03dx %+dmV",
                              slope_milli / 1000,
                              abs(slope_milli % 1000),
                              offset_int);
    }

    int vdc_mv_int = (int)lroundf(frame.metrics.vdc_mv);
    int vrms_ac_tenths = (int)lroundf(frame.metrics.vrms_ac_mv * 10.0f);
    int vrms_true_mv_int = (int)lroundf(frame.metrics.vrms_true_mv);
    int vmin_mv_int = (int)lroundf(frame.metrics.vmin_mv);
    int vmax_mv_int = (int)lroundf(frame.metrics.vmax_mv);
    int vavg_mv_int = (int)lroundf(frame.metrics.vavg_mv);
    int vpp_mv_int = (int)lroundf(frame.metrics.vpp_mv);

    if (s_lbl_vdc_main != NULL) {
        lv_label_set_text_fmt(s_lbl_vdc_main, "%d.%03d V", vdc_mv_int / 1000, abs(vdc_mv_int % 1000));
    }
    if (s_lbl_vdc_mv != NULL) {
        lv_label_set_text_fmt(s_lbl_vdc_mv, "%d mV DC", vdc_mv_int);
    }
    if (s_lbl_ac_rms != NULL) {
        lv_label_set_text_fmt(s_lbl_ac_rms,
                              "AC RMS: %d.%d mV",
                              vrms_ac_tenths / 10,
                              abs(vrms_ac_tenths % 10));
    }
    if (s_lbl_true_rms != NULL) {
        lv_label_set_text_fmt(s_lbl_true_rms,
                              "True RMS: %d.%03d V",
                              vrms_true_mv_int / 1000,
                              abs(vrms_true_mv_int % 1000));
    }
    if (s_bar_voltage != NULL) {
        int clamped_bar = (vdc_mv_int < 0) ? 0 : ((vdc_mv_int > 3300) ? 3300 : vdc_mv_int);
        lv_bar_set_value(s_bar_voltage, clamped_bar, LV_ANIM_OFF);
    }
    if (s_lbl_stats != NULL) {
        lv_label_set_text_fmt(s_lbl_stats,
                              "Min:%d.%02dV Max:%d.%02dV Avg:%d.%02dV Vpp:%dmV",
                              vmin_mv_int / 1000, abs((vmin_mv_int % 1000) / 10),
                              vmax_mv_int / 1000, abs((vmax_mv_int % 1000) / 10),
                              vavg_mv_int / 1000, abs((vavg_mv_int % 1000) / 10),
                              vpp_mv_int);
    }
    if (s_btn_pin_lbl != NULL) {
        lv_label_set_text(s_btn_pin_lbl, (cur_pin == BSP_PIN_ANALOG_IN_ALT) ? "G35" : "G36");
    }
    if (s_btn_hold_lbl != NULL) {
        lv_label_set_text(s_btn_hold_lbl, hold ? "HOLD" : "LIVE");
    }
    if (s_btn_spk_lbl != NULL) {
        bool spk_on = bsp_spk_passthrough_is_enabled() && !bsp_spk_passthrough_is_muted();
        lv_label_set_text(s_btn_spk_lbl, spk_on ? "SPK1" : "SPK0");
    }
}

/* --------------------------------------------------------------------------
 * Tool Plugin Lifecycle Callbacks
 * -------------------------------------------------------------------------- */

static esp_err_t voltmeter_on_init(void *ctx)
{
    (void)ctx;
    ensure_cal_cache_init();
    voltmeter_cal_t cal36, cal35;
    (void)tool_voltmeter_load_cal(BSP_PIN_ANALOG_IN_DEFAULT, &cal36);
    (void)tool_voltmeter_load_cal(BSP_PIN_ANALOG_IN_ALT, &cal35);
    return ESP_OK;
}

static void free_voltmeter_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    voltmeter_dsp_avg_free(&s_avg_filter);
    if (s_raw_batch != NULL) {
        psram_buf_free(s_raw_batch);
        s_raw_batch = NULL;
    }
    if (s_mv_batch != NULL) {
        psram_buf_free(s_mv_batch);
        s_mv_batch = NULL;
    }
}

static esp_err_t voltmeter_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    gpio_num_t pin = tool_voltmeter_get_pin();

    esp_err_t err = bsp_pin_claim(pin, VOLTMETER_TOOL_ID);
    if (err != ESP_OK) {
        return err;
    }

    err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(voltmeter_frame_t));
    if (err != ESP_OK) {
        (void)bsp_pin_release(pin, VOLTMETER_TOOL_ID);
        return err;
    }

    err = voltmeter_dsp_avg_init(&s_avg_filter, VOLTMETER_DEFAULT_MOVING_AVG_LEN);
    if (err != ESP_OK) {
        free_voltmeter_buffers();
        (void)bsp_pin_release(pin, VOLTMETER_TOOL_ID);
        return err;
    }

    s_raw_batch = (uint16_t *)psram_buf_calloc(VOLTMETER_MAX_BATCH_SAMPLES, sizeof(uint16_t));
    s_mv_batch = (float *)psram_buf_calloc(VOLTMETER_MAX_BATCH_SAMPLES, sizeof(float));
    if (s_raw_batch == NULL || s_mv_batch == NULL) {
        free_voltmeter_buffers();
        (void)bsp_pin_release(pin, VOLTMETER_TOOL_ID);
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_volt_lock);
    s_hold_enabled = false;
    s_reset_requested = false;
    s_stop_requested = false;
    memset(&s_latest_metrics, 0, sizeof(s_latest_metrics));
    s_tool_active = true;
    portEXIT_CRITICAL(&s_volt_lock);

    if (parent_cont != NULL) {
        build_voltmeter_ui(parent_cont);
    }

    /* Execute an immediate initial sample step so metrics are populated before first UI tick */
    (void)voltmeter_worker_step(NULL);

    tool_worker_config_t wcfg = {
        .name = "volt_c1",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = TOOL_WORKER_DEFAULT_CORE,
        .period_ms = VOLTMETER_WORKER_PERIOD_MS,
        .step_fn = voltmeter_worker_step,
        .user_ctx = NULL,
    };
    err = tool_worker_start(&s_worker, &wcfg);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_volt_lock);
        s_tool_active = false;
        portEXIT_CRITICAL(&s_volt_lock);
        free_voltmeter_buffers();
        (void)bsp_pin_release(pin, VOLTMETER_TOOL_ID);
        return err;
    }

    ESP_LOGI(TAG, "Voltmeter started on GPIO%d", (int)pin);
    return ESP_OK;
}

static esp_err_t voltmeter_on_stop(bool keep_background, void *ctx)
{
    (void)keep_background;
    (void)ctx;

    portENTER_CRITICAL(&s_volt_lock);
    s_stop_requested = true;
    tool_worker_t *worker = s_worker;
    portEXIT_CRITICAL(&s_volt_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < VOLTMETER_WORKER_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, VOLTMETER_WORKER_STOP_MS);
        }
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop voltmeter worker cleanly: %s", esp_err_to_name(stop_err));
            return stop_err;
        }
        portENTER_CRITICAL(&s_volt_lock);
        s_worker = NULL;
        portEXIT_CRITICAL(&s_volt_lock);
    }

    s_lbl_pin_info = NULL;
    s_lbl_cal_info = NULL;
    s_lbl_vdc_main = NULL;
    s_lbl_vdc_mv = NULL;
    s_lbl_ac_rms = NULL;
    s_lbl_true_rms = NULL;
    s_bar_voltage = NULL;
    s_lbl_stats = NULL;
    s_btn_pin_lbl = NULL;
    s_btn_hold_lbl = NULL;
    s_btn_spk_lbl = NULL;

    free_voltmeter_buffers();

    portENTER_CRITICAL(&s_volt_lock);
    gpio_num_t pin = s_selected_pin;
    s_tool_active = false;
    portEXIT_CRITICAL(&s_volt_lock);

    (void)bsp_pin_release(pin, VOLTMETER_TOOL_ID);
    return ESP_OK;
}

static esp_err_t voltmeter_selftest(void)
{
    voltmeter_cal_t cal = {0};
    if (voltmeter_dsp_calc_two_point_cal(500.0f, 510.0f, 2500.0f, 2510.0f, &cal) != ESP_OK) {
        return ESP_FAIL;
    }
    if (fabsf(cal.slope - 1.0f) > 0.001f || fabsf(cal.offset_mv - 10.0f) > 0.1f) {
        return ESP_FAIL;
    }

    float batch[8] = {100.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f, 3200.0f};
    float median = 0.0f;
    float trimmed = 0.0f;
    if (voltmeter_dsp_median_filter_mv(batch, 8, &median, &trimmed) != ESP_OK) {
        return ESP_FAIL;
    }
    if (fabsf(median - 1500.0f) > 0.01f || fabsf(trimmed - 1500.0f) > 0.01f) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_voltmeter_descriptor = {
    .id = VOLTMETER_TOOL_ID,
    .title = "Voltmeter",
    .symbol = LV_SYMBOL_CHARGE,
    .category = TOOL_CATEGORY_ANALOG_IN,
    .supports_background = false,
    .supports_recording = true,
    .supports_spk_passthrough = true,
    .on_init = voltmeter_on_init,
    .on_start = voltmeter_on_start,
    .on_stop = voltmeter_on_stop,
    .on_ui_refresh = voltmeter_on_ui_refresh,
    .is_running_bg = NULL,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_voltmeter_get_descriptor(void)
{
    return &s_voltmeter_descriptor;
}

esp_err_t tool_voltmeter_register(void)
{
    esp_err_t err = tool_registry_register(&s_voltmeter_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("voltmeter_dsp", voltmeter_selftest);
    }
    return err;
}
