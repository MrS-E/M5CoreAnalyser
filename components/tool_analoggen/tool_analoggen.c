/**
 * @file tool_analoggen.c
 * @brief 8-Bit DAC Analog Generator instrument (`tool_analoggen`) using PSRAM/SRAM 256-entry
 *        lookup tables, Core 1 task-driven `dac_oneshot` output, speaker passthrough monitoring
 *        with foreground ownership coordination, and LVGL 9 UI.
 */

#include "tool_analoggen.h"
#include "tool_analoggen_priv.h"

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "psram_buffer.h"
#include "tool_core.h"

static const char *TAG = "tool_analoggen";

#define ANALOGGEN_WORKER_PERIOD_MS       (16U)
#define ANALOGGEN_WORKER_STOP_WAIT_MS    (1000U)
#define ANALOGGEN_WORKER_STOP_RETRIES    (4)
#define ANALOGGEN_FG_MICROBATCH_SAMPLES  (8U)
#define ANALOGGEN_BG_MICROBATCH_SAMPLES  (4U)
#define ANALOGGEN_SAMPLE_STEP_US         (125U)

static const uint32_t s_freq_presets_hz[] = {
    10U,
    50U,
    100U,
    200U,
    440U,
    500U,
    1000U,
};
#define ANALOGGEN_FREQ_PRESET_COUNT (sizeof(s_freq_presets_hz) / sizeof(s_freq_presets_hz[0]))

static const uint16_t s_amp_presets_mv[] = {
    500U,
    1000U,
    2000U,
    3000U,
    3300U,
};
#define ANALOGGEN_AMP_PRESET_COUNT (sizeof(s_amp_presets_mv) / sizeof(s_amp_presets_mv[0]))

static const uint16_t s_offset_presets_mv[] = {
    500U,
    1000U,
    1650U,
    2200U,
    2800U,
};
#define ANALOGGEN_OFFSET_PRESET_COUNT (sizeof(s_offset_presets_mv) / sizeof(s_offset_presets_mv[0]))

static portMUX_TYPE s_analog_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_hw_mutex_buf;
static SemaphoreHandle_t s_hw_mutex = NULL;
static atomic_int s_hw_mutex_state = 0;

static analoggen_config_t s_cfg = {
    .pin = ANALOGGEN_DEFAULT_PIN,
    .waveform = ANALOGGEN_WAVE_SINE,
    .freq_hz = ANALOGGEN_DEFAULT_FREQ_HZ,
    .amplitude_mv = ANALOGGEN_DEFAULT_AMP_MV,
    .offset_mv = ANALOGGEN_DEFAULT_OFFSET_MV,
    .stair_steps = ANALOGGEN_DEFAULT_STAIR_STEPS,
    .output_enabled = true,
    .keep_in_bg = true,
};

static analoggen_status_t s_status = {0};
static bool s_running_fg = false;
static bool s_running_bg = false;
static volatile bool s_stop_requested = false;
static esp_err_t s_forced_hw_err = ESP_OK;

static dac_oneshot_handle_t s_dac_handle = NULL;
static gpio_num_t s_hw_pin = GPIO_NUM_NC;
static uint8_t *s_psram_lut = NULL;
static uint8_t s_sram_lut[ANALOGGEN_LUT_SIZE];
static uint8_t s_spk_audio_buf[ANALOGGEN_SPK_CHUNK_SAMPLES];
static uint32_t s_spk_phase_q16 = 0U;
static bool s_lut_initialized = false;
static psram_double_buf_t s_frame_dbuf = {.spinlock = portMUX_INITIALIZER_UNLOCKED};
static tool_worker_t *s_worker = NULL;

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_top_status = NULL;
static lv_obj_t *s_lbl_badge = NULL;
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser_wave = NULL;
static lv_obj_t *s_lbl_primary = NULL;
static lv_obj_t *s_lbl_secondary = NULL;
static lv_obj_t *s_btn_out_lbl = NULL;
static lv_obj_t *s_btn_wave_lbl = NULL;
static lv_obj_t *s_btn_freq_lbl = NULL;
static lv_obj_t *s_btn_amp_lbl = NULL;
static lv_obj_t *s_btn_ofs_lbl = NULL;
static lv_obj_t *s_btn_pin_lbl = NULL;
static lv_obj_t *s_btn_bg_lbl = NULL;

static void ensure_analog_hw_mutex(void)
{
    int expected = 0;
    if (atomic_compare_exchange_strong(&s_hw_mutex_state, &expected, 1)) {
        s_hw_mutex = xSemaphoreCreateMutexStatic(&s_hw_mutex_buf);
        atomic_store(&s_hw_mutex_state, 2);
    } else {
        while (atomic_load(&s_hw_mutex_state) != 2) {
            taskYIELD();
        }
    }
}

static void hw_lock(void)
{
    ensure_analog_hw_mutex();
    (void)xSemaphoreTake(s_hw_mutex, portMAX_DELAY);
}

static void hw_unlock(void)
{
    (void)xSemaphoreGive(s_hw_mutex);
}

bool tool_analoggen_is_pin_supported(gpio_num_t pin)
{
    return (pin == ANALOGGEN_DEFAULT_PIN) || (pin == ANALOGGEN_ALT_PIN);
}

static dac_channel_t pin_to_dac_channel(gpio_num_t pin)
{
    return (pin == ANALOGGEN_ALT_PIN) ? DAC_CHAN_1 : DAC_CHAN_0;
}

static void rebuild_lut_locked(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    analoggen_config_t cfg = s_cfg;
    portEXIT_CRITICAL(&s_analog_lock);

    analoggen_lut_stats_t stats = {0};
    (void)analoggen_lut_build(&cfg, s_sram_lut, ANALOGGEN_LUT_SIZE, &stats);
    if (s_psram_lut != NULL) {
        memcpy(s_psram_lut, s_sram_lut, ANALOGGEN_LUT_SIZE);
    }

    portENTER_CRITICAL(&s_analog_lock);
    s_lut_initialized = true;
    s_status.cfg = cfg;
    s_status.lut_stats = stats;
    s_status.psram_lut_allocated = (s_psram_lut != NULL);
    portEXIT_CRITICAL(&s_analog_lock);
}

static void teardown_dac_locked(void)
{
    if (s_dac_handle != NULL) {
        (void)dac_oneshot_output_voltage(s_dac_handle, 0U);
        (void)dac_oneshot_del_channel(s_dac_handle);
        s_dac_handle = NULL;
    }
    s_hw_pin = GPIO_NUM_NC;
}

bool tool_analoggen_owns_speaker(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    bool fg = s_running_fg;
    bool bg = s_running_bg;
    bool out_en = s_cfg.output_enabled;
    portEXIT_CRITICAL(&s_analog_lock);

    if (!out_en || (!fg && !bg)) {
        return false;
    }
    if (fg) {
        return true;
    }

    /* Running in background: yield speaker if a foreground input tool supports speaker audio */
    const analyser_tool_t *active = tool_core_get_active_tool();
    if (active != NULL &&
        strcmp(active->id, ANALOGGEN_TOOL_ID) != 0 &&
        active->supports_spk_passthrough) {
        return false;
    }
    return true;
}

static void publish_status_locked(void)
{
    bool owns_spk = tool_analoggen_owns_speaker();
    bool spk_en = bsp_spk_passthrough_is_enabled() && !bsp_spk_passthrough_is_muted();
    const analyser_tool_t *fg_tool = tool_core_get_active_tool();
    bool other_fg_active = (fg_tool != NULL && strcmp(fg_tool->id, ANALOGGEN_TOOL_ID) != 0);

    portENTER_CRITICAL(&s_analog_lock);
    bool bg_with_fg = s_running_bg && other_fg_active;
    s_status.cfg = s_cfg;
    s_status.running_fg = s_running_fg;
    s_status.running_bg = s_running_bg;
    s_status.hw_active = (s_dac_handle != NULL) && s_cfg.output_enabled;
    s_status.psram_lut_allocated = (s_psram_lut != NULL);
    s_status.target_update_rate_hz = bg_with_fg ? ANALOGGEN_BG_UPDATE_RATE_HZ : ANALOGGEN_FG_UPDATE_RATE_HZ;
    s_status.spk_yielded_to_fg = s_running_bg && !owns_spk && s_cfg.output_enabled;
    s_status.spk_streaming = spk_en && owns_spk && s_cfg.output_enabled;
    analoggen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_analog_lock);

    analoggen_status_t *wptr = (analoggen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }
}

static esp_err_t apply_dac_config_locked(void)
{
    rebuild_lut_locked();

    portENTER_CRITICAL(&s_analog_lock);
    analoggen_config_t cfg = s_cfg;
    bool active = s_running_fg || s_running_bg;
    esp_err_t injected_err = s_forced_hw_err;
    s_forced_hw_err = ESP_OK;
    portEXIT_CRITICAL(&s_analog_lock);

    if (!active) {
        publish_status_locked();
        return ESP_OK;
    }

    if (injected_err != ESP_OK) {
        teardown_dac_locked();
        portENTER_CRITICAL(&s_analog_lock);
        s_status.last_error = injected_err;
        portEXIT_CRITICAL(&s_analog_lock);
        publish_status_locked();
        return injected_err;
    }

    if (s_dac_handle != NULL && s_hw_pin != cfg.pin) {
        teardown_dac_locked();
    }

    if (s_dac_handle == NULL) {
        dac_oneshot_config_t dac_cfg = {
            .chan_id = pin_to_dac_channel(cfg.pin),
        };
        esp_err_t err = dac_oneshot_new_channel(&dac_cfg, &s_dac_handle);
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_analog_lock);
            s_status.last_error = err;
            portEXIT_CRITICAL(&s_analog_lock);
            publish_status_locked();
            return err;
        }
        s_hw_pin = cfg.pin;
    }

    uint8_t init_code = 0U;
    if (cfg.output_enabled) {
        size_t idx = (cfg.waveform == ANALOGGEN_WAVE_DC)
                         ? 0U
                         : analoggen_calc_phase_index(
                               (uint64_t)esp_timer_get_time(), cfg.freq_hz, ANALOGGEN_LUT_SIZE);
        init_code = s_sram_lut[idx];
    }
    esp_err_t err = dac_oneshot_output_voltage(s_dac_handle, init_code);

    portENTER_CRITICAL(&s_analog_lock);
    s_status.last_dac_code = init_code;
    s_status.dac_updates_total++;
    s_status.last_error = err;
    portEXIT_CRITICAL(&s_analog_lock);

    publish_status_locked();
    return err;
}

/* --------------------------------------------------------------------------
 * Public Configuration Setters & Getters
 * -------------------------------------------------------------------------- */

esp_err_t tool_analoggen_set_pin(gpio_num_t pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bsp_pin_is_protected(pin)) {
        return ESP_ERR_NOT_ALLOWED;
    }
    if (bsp_pin_is_input_only(pin)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!tool_analoggen_is_pin_supported(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    gpio_num_t prev_pin = s_cfg.pin;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    if (pin == prev_pin) {
        hw_unlock();
        return ESP_OK;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim_ex(pin, ANALOGGEN_TOOL_ID, true);
        if (err != ESP_OK) {
            hw_unlock();
            return err;
        }
        teardown_dac_locked();
        (void)bsp_pin_release(prev_pin, ANALOGGEN_TOOL_ID);
    }

    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.pin = pin;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t apply_err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return apply_err;
}

gpio_num_t tool_analoggen_get_pin(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    gpio_num_t pin = s_cfg.pin;
    portEXIT_CRITICAL(&s_analog_lock);
    return pin;
}

esp_err_t tool_analoggen_set_waveform(analoggen_wave_t waveform)
{
    if (waveform < 0 || waveform >= ANALOGGEN_WAVE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.waveform = waveform;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

analoggen_wave_t tool_analoggen_get_waveform(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    analoggen_wave_t w = s_cfg.waveform;
    portEXIT_CRITICAL(&s_analog_lock);
    return w;
}

esp_err_t tool_analoggen_set_freq_hz(uint32_t freq_hz)
{
    if (freq_hz < ANALOGGEN_MIN_FREQ_HZ || freq_hz > ANALOGGEN_MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.freq_hz = freq_hz;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

uint32_t tool_analoggen_get_freq_hz(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    uint32_t f = s_cfg.freq_hz;
    portEXIT_CRITICAL(&s_analog_lock);
    return f;
}

esp_err_t tool_analoggen_set_amplitude_mv(uint16_t amplitude_mv)
{
    if (amplitude_mv > ANALOGGEN_MAX_MV) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.amplitude_mv = amplitude_mv;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

uint16_t tool_analoggen_get_amplitude_mv(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    uint16_t amp = s_cfg.amplitude_mv;
    portEXIT_CRITICAL(&s_analog_lock);
    return amp;
}

esp_err_t tool_analoggen_set_offset_mv(uint16_t offset_mv)
{
    if (offset_mv > ANALOGGEN_MAX_MV) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.offset_mv = offset_mv;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

uint16_t tool_analoggen_get_offset_mv(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    uint16_t ofs = s_cfg.offset_mv;
    portEXIT_CRITICAL(&s_analog_lock);
    return ofs;
}

esp_err_t tool_analoggen_set_stair_steps(uint8_t steps)
{
    if (steps < ANALOGGEN_MIN_STAIR_STEPS || steps > ANALOGGEN_MAX_STAIR_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.stair_steps = steps;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

uint8_t tool_analoggen_get_stair_steps(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    uint8_t s = s_cfg.stair_steps;
    portEXIT_CRITICAL(&s_analog_lock);
    return s;
}

esp_err_t tool_analoggen_set_output_enabled(bool enabled)
{
    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.output_enabled = enabled;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = active ? apply_dac_config_locked() : ESP_OK;
    if (!active) {
        rebuild_lut_locked();
    }
    hw_unlock();
    return err;
}

bool tool_analoggen_is_output_enabled(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    bool en = s_cfg.output_enabled;
    portEXIT_CRITICAL(&s_analog_lock);
    return en;
}

esp_err_t tool_analoggen_set_keep_in_bg(bool keep_in_bg)
{
    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.keep_in_bg = keep_in_bg;
    portEXIT_CRITICAL(&s_analog_lock);
    publish_status_locked();
    hw_unlock();
    return ESP_OK;
}

bool tool_analoggen_get_keep_in_bg(void)
{
    portENTER_CRITICAL(&s_analog_lock);
    bool keep = s_cfg.keep_in_bg;
    portEXIT_CRITICAL(&s_analog_lock);
    return keep;
}

size_t tool_analoggen_copy_lut(uint8_t *out_lut, size_t max_len)
{
    if (out_lut == NULL || max_len == 0U) {
        return 0U;
    }

    hw_lock();
    if (!s_lut_initialized) {
        rebuild_lut_locked();
    }
    size_t copy_len = (max_len < ANALOGGEN_LUT_SIZE) ? max_len : ANALOGGEN_LUT_SIZE;
    const uint8_t *src = (s_psram_lut != NULL) ? s_psram_lut : s_sram_lut;
    memcpy(out_lut, src, copy_len);
    hw_unlock();
    return copy_len;
}

esp_err_t tool_analoggen_get_status(analoggen_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    bool owns_spk = tool_analoggen_owns_speaker();
    bool spk_en = bsp_spk_passthrough_is_enabled() && !bsp_spk_passthrough_is_muted();
    const analyser_tool_t *fg_tool = tool_core_get_active_tool();
    bool other_fg_active = (fg_tool != NULL && strcmp(fg_tool->id, ANALOGGEN_TOOL_ID) != 0);

    portENTER_CRITICAL(&s_analog_lock);
    bool bg_with_fg = s_running_bg && other_fg_active;
    s_status.cfg = s_cfg;
    s_status.running_fg = s_running_fg;
    s_status.running_bg = s_running_bg;
    s_status.target_update_rate_hz = bg_with_fg ? ANALOGGEN_BG_UPDATE_RATE_HZ : ANALOGGEN_FG_UPDATE_RATE_HZ;
    s_status.spk_yielded_to_fg = s_running_bg && !owns_spk && s_cfg.output_enabled;
    s_status.spk_streaming = spk_en && owns_spk && s_cfg.output_enabled;
    *out_status = s_status;
    portEXIT_CRITICAL(&s_analog_lock);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Core 1 Task-Driven DAC Oneshot + Speaker Monitor Worker
 * -------------------------------------------------------------------------- */

static esp_err_t execute_dac_and_speaker_step_locked(bool is_bg_worker)
{
    portENTER_CRITICAL(&s_analog_lock);
    analoggen_config_t cfg = s_cfg;
    bool stop_req = s_stop_requested;
    bool bg_mode = s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    if (stop_req) {
        return ESP_OK;
    }

    if (s_dac_handle != NULL && cfg.output_enabled) {
        if (cfg.waveform == ANALOGGEN_WAVE_DC) {
            uint8_t code = s_sram_lut[0];
            (void)dac_oneshot_output_voltage(s_dac_handle, code);
            portENTER_CRITICAL(&s_analog_lock);
            s_status.last_dac_code = code;
            s_status.dac_updates_total++;
            portEXIT_CRITICAL(&s_analog_lock);
        } else {
            size_t batch = (bg_mode || !is_bg_worker)
                               ? ANALOGGEN_BG_MICROBATCH_SAMPLES
                               : ANALOGGEN_FG_MICROBATCH_SAMPLES;
            uint8_t last_code = 0U;
            uint32_t updates = 0U;
            for (size_t i = 0; i < batch; ++i) {
                if (s_stop_requested) {
                    break;
                }
                uint64_t now_us = (uint64_t)esp_timer_get_time() + (uint64_t)(i * ANALOGGEN_SAMPLE_STEP_US);
                size_t lut_idx = analoggen_calc_phase_index(now_us, cfg.freq_hz, ANALOGGEN_LUT_SIZE);
                last_code = s_sram_lut[lut_idx];
                (void)dac_oneshot_output_voltage(s_dac_handle, last_code);
                updates++;
                if (is_bg_worker && i + 1U < batch) {
                    esp_rom_delay_us(20U);
                }
            }
            portENTER_CRITICAL(&s_analog_lock);
            s_status.last_dac_code = last_code;
            s_status.dac_updates_total += updates;
            portEXIT_CRITICAL(&s_analog_lock);
        }
    }

    /* Stream LUT audio to I2S1 speaker passthrough ONLY when analoggen owns speaker feed */
    if (cfg.output_enabled && bsp_spk_passthrough_is_enabled() && tool_analoggen_owns_speaker()) {
        uint32_t spk_rate = bsp_spk_passthrough_get_sample_rate();
        if (spk_rate == 0U) {
            spk_rate = BSP_SPK_DEFAULT_RATE_HZ;
        }
        uint32_t wave_f = (cfg.waveform == ANALOGGEN_WAVE_DC) ? 0U : cfg.freq_hz;
        const uint8_t *lut_src = (s_psram_lut != NULL) ? s_psram_lut : s_sram_lut;
        if (analoggen_resample_lut_for_speaker(lut_src,
                                               ANALOGGEN_LUT_SIZE,
                                               wave_f,
                                               spk_rate,
                                               &s_spk_phase_q16,
                                               s_spk_audio_buf,
                                               ANALOGGEN_SPK_CHUNK_SAMPLES) == ESP_OK) {
            (void)bsp_spk_passthrough_push_u8(s_spk_audio_buf, ANALOGGEN_SPK_CHUNK_SAMPLES);
        }
    }

    publish_status_locked();
    return ESP_OK;
}

static esp_err_t analoggen_worker_step(void *ctx)
{
    (void)ctx;
    if (s_stop_requested) {
        return ESP_OK;
    }
    ensure_analog_hw_mutex();
    if (xSemaphoreTake(s_hw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_OK;
    }
    esp_err_t err = execute_dac_and_speaker_step_locked(true);
    (void)xSemaphoreGive(s_hw_mutex);
    return err;
}

esp_err_t tool_analoggen_run_single_step(void)
{
    hw_lock();
    esp_err_t err = execute_dac_and_speaker_step_locked(false);
    hw_unlock();
    return err;
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Controls & Chart Rendering (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_out_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_analoggen_set_output_enabled(!tool_analoggen_is_output_enabled());
}

static void on_btn_wave_clicked(lv_event_t *e)
{
    (void)e;
    analoggen_wave_t next = (analoggen_wave_t)((tool_analoggen_get_waveform() + 1) % ANALOGGEN_WAVE_COUNT);
    (void)tool_analoggen_set_waveform(next);
}

static void on_btn_freq_clicked(lv_event_t *e)
{
    (void)e;
    if (tool_analoggen_get_waveform() == ANALOGGEN_WAVE_STAIRCASE) {
        uint8_t cur_steps = tool_analoggen_get_stair_steps();
        uint8_t next_steps = (cur_steps >= 16U) ? 4U : (uint8_t)(cur_steps * 2U);
        (void)tool_analoggen_set_stair_steps(next_steps);
    }

    uint32_t cur_f = tool_analoggen_get_freq_hz();
    size_t next_idx = 0U;
    for (size_t i = 0; i < ANALOGGEN_FREQ_PRESET_COUNT; ++i) {
        if (s_freq_presets_hz[i] > cur_f) {
            next_idx = i;
            break;
        }
    }
    (void)tool_analoggen_set_freq_hz(s_freq_presets_hz[next_idx]);
}

static void on_btn_amp_clicked(lv_event_t *e)
{
    (void)e;
    uint16_t cur = tool_analoggen_get_amplitude_mv();
    size_t next_idx = 0U;
    for (size_t i = 0; i < ANALOGGEN_AMP_PRESET_COUNT; ++i) {
        if (s_amp_presets_mv[i] > cur) {
            next_idx = i;
            break;
        }
    }
    (void)tool_analoggen_set_amplitude_mv(s_amp_presets_mv[next_idx]);
}

static void on_btn_ofs_clicked(lv_event_t *e)
{
    (void)e;
    uint16_t cur = tool_analoggen_get_offset_mv();
    size_t next_idx = 0U;
    for (size_t i = 0; i < ANALOGGEN_OFFSET_PRESET_COUNT; ++i) {
        if (s_offset_presets_mv[i] > cur) {
            next_idx = i;
            break;
        }
    }
    (void)tool_analoggen_set_offset_mv(s_offset_presets_mv[next_idx]);
}

static void on_btn_pin_clicked(lv_event_t *e)
{
    (void)e;
    gpio_num_t cur = tool_analoggen_get_pin();
    gpio_num_t next = (cur == ANALOGGEN_DEFAULT_PIN) ? ANALOGGEN_ALT_PIN : ANALOGGEN_DEFAULT_PIN;
    (void)tool_analoggen_set_pin(next);
}

static void on_btn_bg_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_analoggen_set_keep_in_bg(!tool_analoggen_get_keep_in_bg());
}

static void build_analoggen_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 2, 0);
    lv_obj_set_style_pad_row(parent, 2, 0);

    /* 1. Top Status & Mode Badge Bar (18 px) */
    lv_obj_t *top_row = lv_obj_create(parent);
    lv_obj_set_size(top_row, 314, 18);
    lv_obj_set_style_bg_color(top_row, lv_color_hex(0x1E152A), 0);
    lv_obj_set_style_border_width(top_row, 0, 0);
    lv_obj_set_style_radius(top_row, 3, 0);
    lv_obj_set_style_pad_hor(top_row, 4, 0);
    lv_obj_set_style_pad_ver(top_row, 1, 0);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_top_status = lv_label_create(top_row);
    lv_label_set_text(s_lbl_top_status, "DAC1 G25  SINE  200Hz (Task)");
    lv_obj_set_style_text_font(s_lbl_top_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_top_status, lv_color_hex(0xCE93D8), 0);

    s_lbl_badge = lv_label_create(top_row);
    lv_label_set_text(s_lbl_badge, "BG:ON [RUN]");
    lv_obj_set_style_text_font(s_lbl_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_badge, lv_color_hex(0x00E676), 0);

    /* 2. 2-Cycle LUT Preview Chart (314 x 96 px) */
    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 314, 96);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, ANALOGGEN_CHART_POINTS);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 3300);
    lv_chart_set_div_line_count(s_chart, 3, 5);
    lv_obj_set_style_bg_color(s_chart, lv_color_hex(0x090D14), 0);
    lv_obj_set_style_border_color(s_chart, lv_color_hex(0x6A1B9A), 0);
    lv_obj_set_style_border_width(s_chart, 1, 0);
    lv_obj_set_style_radius(s_chart, 3, 0);
    lv_obj_set_style_pad_all(s_chart, 2, 0);
    lv_obj_set_style_size(s_chart, 0, 0, LV_PART_INDICATOR);

    s_ser_wave = lv_chart_add_series(s_chart, lv_color_hex(0xFFAB40), LV_CHART_AXIS_PRIMARY_Y);

    /* 3. Primary & Secondary Telemetry Rows (2 x 18 px) */
    s_lbl_primary = lv_label_create(parent);
    lv_label_set_text(s_lbl_primary, "200 Hz  Vpp: 2.00V  Ofs: 1.65V");
    lv_obj_set_style_text_font(s_lbl_primary, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lbl_primary, lv_color_hex(0xFFD54F), 0);

    s_lbl_secondary = lv_label_create(parent);
    lv_label_set_text(s_lbl_secondary, "LUT: 50..205 (0.65..2.65V)  SPK:OFF  BG:Keep");
    lv_obj_set_style_text_font(s_lbl_secondary, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_secondary, lv_color_hex(0xCFD8DC), 0);

    /* 4. Bottom Touch Controls Bar (28 px, 7 compact buttons) */
    lv_obj_t *ctrl_row = lv_obj_create(parent);
    lv_obj_set_size(ctrl_row, 314, 28);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_style_pad_column(ctrl_row, 2, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_out = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_out, 38, 26);
    lv_obj_set_style_bg_color(btn_out, lv_color_hex(0x2E7D32), 0);
    lv_obj_add_event_cb(btn_out, on_btn_out_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_out_lbl = lv_label_create(btn_out);
    lv_label_set_text(s_btn_out_lbl, "ON");
    lv_obj_set_style_text_font(s_btn_out_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_out_lbl);

    lv_obj_t *btn_wave = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_wave, 50, 26);
    lv_obj_set_style_bg_color(btn_wave, lv_color_hex(0x4527A0), 0);
    lv_obj_add_event_cb(btn_wave, on_btn_wave_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_wave_lbl = lv_label_create(btn_wave);
    lv_label_set_text(s_btn_wave_lbl, "SINE");
    lv_obj_set_style_text_font(s_btn_wave_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_wave_lbl);

    lv_obj_t *btn_freq = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_freq, 44, 26);
    lv_obj_set_style_bg_color(btn_freq, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_freq, on_btn_freq_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_freq_lbl = lv_label_create(btn_freq);
    lv_label_set_text(s_btn_freq_lbl, "FREQ");
    lv_obj_set_style_text_font(s_btn_freq_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_freq_lbl);

    lv_obj_t *btn_amp = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_amp, 42, 26);
    lv_obj_set_style_bg_color(btn_amp, lv_color_hex(0x0277BD), 0);
    lv_obj_add_event_cb(btn_amp, on_btn_amp_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_amp_lbl = lv_label_create(btn_amp);
    lv_label_set_text(s_btn_amp_lbl, "AMP");
    lv_obj_set_style_text_font(s_btn_amp_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_amp_lbl);

    lv_obj_t *btn_ofs = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_ofs, 42, 26);
    lv_obj_set_style_bg_color(btn_ofs, lv_color_hex(0x558B2F), 0);
    lv_obj_add_event_cb(btn_ofs, on_btn_ofs_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_ofs_lbl = lv_label_create(btn_ofs);
    lv_label_set_text(s_btn_ofs_lbl, "OFS");
    lv_obj_set_style_text_font(s_btn_ofs_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_ofs_lbl);

    lv_obj_t *btn_pin = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pin, 40, 26);
    lv_obj_set_style_bg_color(btn_pin, lv_color_hex(0x37474F), 0);
    lv_obj_add_event_cb(btn_pin, on_btn_pin_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_pin_lbl = lv_label_create(btn_pin);
    lv_label_set_text(s_btn_pin_lbl, "G25");
    lv_obj_set_style_text_font(s_btn_pin_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_pin_lbl);

    lv_obj_t *btn_bg = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_bg, 42, 26);
    lv_obj_set_style_bg_color(btn_bg, lv_color_hex(0xE65100), 0);
    lv_obj_add_event_cb(btn_bg, on_btn_bg_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_bg_lbl = lv_label_create(btn_bg);
    lv_label_set_text(s_btn_bg_lbl, "BG:1");
    lv_obj_set_style_text_font(s_btn_bg_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_bg_lbl);
}

static void analoggen_on_ui_refresh(void *ctx)
{
    (void)ctx;
    static analoggen_status_t st;
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &st, 1, &copied) != ESP_OK || copied == 0) {
        (void)tool_analoggen_get_status(&st);
    }

    if (s_lbl_top_status != NULL) {
        const char *dac_name = (st.cfg.pin == ANALOGGEN_ALT_PIN) ? "DAC2 G26" : "DAC1 G25";
        lv_label_set_text_fmt(s_lbl_top_status,
                              "%s  %s  %luHz (Task %lukS/s)",
                              dac_name,
                              analoggen_wave_to_str(st.cfg.waveform),
                              (unsigned long)st.cfg.freq_hz,
                              (unsigned long)(st.target_update_rate_hz / 1000U));
    }

    if (s_lbl_badge != NULL) {
        if (st.last_error != ESP_OK) {
            lv_label_set_text(s_lbl_badge, "[HW ERR]");
            lv_obj_set_style_text_color(s_lbl_badge, lv_color_hex(0xFF5252), 0);
        } else if (st.lut_stats.clipped) {
            lv_label_set_text(s_lbl_badge, "[CLIP]");
            lv_obj_set_style_text_color(s_lbl_badge, lv_color_hex(0xFFAB00), 0);
        } else {
            lv_label_set_text_fmt(s_lbl_badge,
                                  "BG:%s [%s]",
                                  st.cfg.keep_in_bg ? "ON" : "OFF",
                                  st.cfg.output_enabled ? "RUN" : "IDLE");
            lv_obj_set_style_text_color(s_lbl_badge,
                                        lv_color_hex(st.cfg.output_enabled ? 0x00E676 : 0xFFD54F),
                                        0);
        }
    }

    if (s_chart != NULL && s_ser_wave != NULL) {
        int32_t *pts = lv_chart_get_y_array(s_chart, s_ser_wave);
        if (pts != NULL) {
            for (size_t i = 0; i < ANALOGGEN_CHART_POINTS; ++i) {
                if (!st.cfg.output_enabled) {
                    pts[i] = 0;
                } else {
                    /* Render 2 complete periods of the 256-entry LUT across 128 chart points */
                    size_t lut_idx = (i * 4U) % ANALOGGEN_LUT_SIZE;
                    pts[i] = (int32_t)analoggen_dac_code_to_mv(s_sram_lut[lut_idx]);
                }
            }
            lv_chart_refresh(s_chart);
        }
    }

    if (s_lbl_primary != NULL) {
        if (st.cfg.waveform == ANALOGGEN_WAVE_DC) {
            lv_label_set_text_fmt(s_lbl_primary,
                                  "DC Output: %u.%02u V  (DAC Code %u)",
                                  (unsigned)(st.lut_stats.mean_mv / 1000U),
                                  (unsigned)((st.lut_stats.mean_mv % 1000U) / 10U),
                                  (unsigned)st.lut_stats.min_code);
        } else {
            lv_label_set_text_fmt(s_lbl_primary,
                                  "%lu Hz  Vpp:%u.%02uV  Ofs:%u.%02uV%s",
                                  (unsigned long)st.cfg.freq_hz,
                                  (unsigned)(st.lut_stats.vpp_mv / 1000U),
                                  (unsigned)((st.lut_stats.vpp_mv % 1000U) / 10U),
                                  (unsigned)(st.cfg.offset_mv / 1000U),
                                  (unsigned)((st.cfg.offset_mv % 1000U) / 10U),
                                  st.lut_stats.clipped ? " [CLIP]" : "");
        }
    }

    if (s_lbl_secondary != NULL) {
        const char *spk_txt = st.spk_streaming ? "SPK:ON" : (st.spk_yielded_to_fg ? "SPK:FG" : "SPK:OFF");
        lv_label_set_text_fmt(s_lbl_secondary,
                              "LUT:%u..%u (%u.%02u..%u.%02uV)  %s  BG:%s",
                              (unsigned)st.lut_stats.min_code,
                              (unsigned)st.lut_stats.max_code,
                              (unsigned)(st.lut_stats.min_mv / 1000U),
                              (unsigned)((st.lut_stats.min_mv % 1000U) / 10U),
                              (unsigned)(st.lut_stats.max_mv / 1000U),
                              (unsigned)((st.lut_stats.max_mv % 1000U) / 10U),
                              spk_txt,
                              st.cfg.keep_in_bg ? "Keep" : "Stop");
    }

    if (s_btn_out_lbl != NULL) {
        lv_label_set_text(s_btn_out_lbl, st.cfg.output_enabled ? "ON" : "OFF");
    }
    if (s_btn_wave_lbl != NULL) {
        lv_label_set_text(s_btn_wave_lbl, analoggen_wave_to_str(st.cfg.waveform));
    }
    if (s_btn_pin_lbl != NULL) {
        lv_label_set_text(s_btn_pin_lbl, (st.cfg.pin == ANALOGGEN_ALT_PIN) ? "G26" : "G25");
    }
    if (s_btn_bg_lbl != NULL) {
        lv_label_set_text(s_btn_bg_lbl, st.cfg.keep_in_bg ? "BG:1" : "BG:0");
    }
}

/* --------------------------------------------------------------------------
 * Tool Plugin Lifecycle Callbacks
 * -------------------------------------------------------------------------- */

static void clear_ui_pointers(void)
{
    s_lbl_top_status = NULL;
    s_lbl_badge = NULL;
    s_chart = NULL;
    s_ser_wave = NULL;
    s_lbl_primary = NULL;
    s_lbl_secondary = NULL;
    s_btn_out_lbl = NULL;
    s_btn_wave_lbl = NULL;
    s_btn_freq_lbl = NULL;
    s_btn_amp_lbl = NULL;
    s_btn_ofs_lbl = NULL;
    s_btn_pin_lbl = NULL;
    s_btn_bg_lbl = NULL;
}

static void free_analoggen_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    if (s_psram_lut != NULL) {
        psram_buf_free(s_psram_lut);
        s_psram_lut = NULL;
    }
}

static bool analoggen_is_running_bg(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_analog_lock);
    bool bg = s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);
    return bg;
}

static esp_err_t analoggen_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    hw_lock();

    portENTER_CRITICAL(&s_analog_lock);
    gpio_num_t pin = s_cfg.pin;
    bool was_bg = s_running_bg;
    portEXIT_CRITICAL(&s_analog_lock);

    esp_err_t err = bsp_pin_claim_ex(pin, ANALOGGEN_TOOL_ID, true);
    if (err != ESP_OK) {
        hw_unlock();
        return err;
    }

    if (s_frame_dbuf.buf_a == NULL) {
        err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(analoggen_status_t));
        if (err != ESP_OK) {
            if (!was_bg) {
                (void)bsp_pin_release(pin, ANALOGGEN_TOOL_ID);
            }
            hw_unlock();
            return err;
        }
    }

    if (s_psram_lut == NULL) {
        s_psram_lut = (uint8_t *)psram_buf_calloc(ANALOGGEN_LUT_SIZE, sizeof(uint8_t));
        if (s_psram_lut == NULL) {
            if (!was_bg) {
                free_analoggen_buffers();
                (void)bsp_pin_release(pin, ANALOGGEN_TOOL_ID);
            }
            hw_unlock();
            return ESP_ERR_NO_MEM;
        }
    }

    portENTER_CRITICAL(&s_analog_lock);
    s_running_fg = true;
    s_running_bg = false;
    s_stop_requested = false;
    portEXIT_CRITICAL(&s_analog_lock);

    if (!was_bg || s_dac_handle == NULL) {
        err = apply_dac_config_locked();
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_analog_lock);
            s_running_fg = false;
            portEXIT_CRITICAL(&s_analog_lock);
            free_analoggen_buffers();
            (void)bsp_pin_release(pin, ANALOGGEN_TOOL_ID);
            hw_unlock();
            return err;
        }
    } else {
        publish_status_locked();
    }

    if (parent_cont != NULL) {
        build_analoggen_ui(parent_cont);
    }

    if (s_worker == NULL) {
        tool_worker_config_t wcfg = {
            .name = "analoggen_c1",
            .stack_size_bytes = 4096,
            .priority = 5,
            .core_id = TOOL_WORKER_DEFAULT_CORE,
            .period_ms = ANALOGGEN_WORKER_PERIOD_MS,
            .step_fn = analoggen_worker_step,
            .user_ctx = NULL,
        };
        err = tool_worker_start(&s_worker, &wcfg);
        if (err != ESP_OK) {
            teardown_dac_locked();
            portENTER_CRITICAL(&s_analog_lock);
            s_running_fg = false;
            portEXIT_CRITICAL(&s_analog_lock);
            free_analoggen_buffers();
            (void)bsp_pin_release(pin, ANALOGGEN_TOOL_ID);
            hw_unlock();
            return err;
        }
    }

    hw_unlock();
    if (parent_cont != NULL) {
        analoggen_on_ui_refresh(NULL);
    }
    ESP_LOGI(TAG, "Analog Generator started on GPIO%d (%s, %lu Hz)",
             (int)pin,
             analoggen_wave_to_str(tool_analoggen_get_waveform()),
             (unsigned long)tool_analoggen_get_freq_hz());
    return ESP_OK;
}

static esp_err_t analoggen_on_stop(bool keep_background, void *ctx)
{
    (void)ctx;
    clear_ui_pointers();

    portENTER_CRITICAL(&s_analog_lock);
    bool retain_bg = keep_background && s_cfg.keep_in_bg && s_cfg.output_enabled;
    if (retain_bg) {
        s_running_fg = false;
        s_running_bg = true;
        s_status.running_fg = false;
        s_status.running_bg = true;
        portEXIT_CRITICAL(&s_analog_lock);
        ESP_LOGI(TAG, "Analog Generator retained in background on GPIO%d", (int)s_cfg.pin);
        return ESP_OK;
    }

    s_stop_requested = true;
    tool_worker_t *worker = s_worker;
    s_worker = NULL;
    portEXIT_CRITICAL(&s_analog_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < ANALOGGEN_WORKER_STOP_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, ANALOGGEN_WORKER_STOP_WAIT_MS);
        }
        if (stop_err != ESP_OK) {
            portENTER_CRITICAL(&s_analog_lock);
            s_worker = worker;
            portEXIT_CRITICAL(&s_analog_lock);
            return stop_err;
        }
    }

    hw_lock();
    teardown_dac_locked();
    free_analoggen_buffers();

    portENTER_CRITICAL(&s_analog_lock);
    bool was_active = s_running_fg || s_running_bg;
    gpio_num_t pin = s_cfg.pin;
    s_running_fg = false;
    s_running_bg = false;
    s_status.running_fg = false;
    s_status.running_bg = false;
    s_status.hw_active = false;
    s_status.spk_streaming = false;
    s_status.spk_yielded_to_fg = false;
    s_status.psram_lut_allocated = false;
    portEXIT_CRITICAL(&s_analog_lock);
    hw_unlock();

    if (was_active) {
        (void)bsp_pin_release(pin, ANALOGGEN_TOOL_ID);
        ESP_LOGI(TAG, "Analog Generator stopped and released GPIO%d", (int)pin);
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Private Test Seams & Self-Test Registration
 * -------------------------------------------------------------------------- */

void tool_analoggen_reset_for_test(void)
{
    (void)analoggen_on_stop(false, NULL);
    hw_lock();
    portENTER_CRITICAL(&s_analog_lock);
    s_cfg.pin = ANALOGGEN_DEFAULT_PIN;
    s_cfg.waveform = ANALOGGEN_WAVE_SINE;
    s_cfg.freq_hz = ANALOGGEN_DEFAULT_FREQ_HZ;
    s_cfg.amplitude_mv = ANALOGGEN_DEFAULT_AMP_MV;
    s_cfg.offset_mv = ANALOGGEN_DEFAULT_OFFSET_MV;
    s_cfg.stair_steps = ANALOGGEN_DEFAULT_STAIR_STEPS;
    s_cfg.output_enabled = true;
    s_cfg.keep_in_bg = true;
    memset(&s_status, 0, sizeof(s_status));
    s_forced_hw_err = ESP_OK;
    s_spk_phase_q16 = 0U;
    s_lut_initialized = false;
    portEXIT_CRITICAL(&s_analog_lock);
    rebuild_lut_locked();
    hw_unlock();
}

void tool_analoggen_force_hw_error_for_test(esp_err_t err)
{
    portENTER_CRITICAL(&s_analog_lock);
    s_forced_hw_err = err;
    portEXIT_CRITICAL(&s_analog_lock);
}

static esp_err_t analoggen_selftest(void)
{
    uint8_t lut[ANALOGGEN_LUT_SIZE] = {0};
    analoggen_lut_stats_t st = {0};
    analoggen_config_t cfg = {
        .pin = ANALOGGEN_DEFAULT_PIN,
        .waveform = ANALOGGEN_WAVE_SINE,
        .freq_hz = 200U,
        .amplitude_mv = 2000U,
        .offset_mv = 1650U,
        .stair_steps = 8U,
        .output_enabled = true,
        .keep_in_bg = true,
    };

    if (analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &st) != ESP_OK) {
        return ESP_FAIL;
    }
    if (st.clipped || st.min_code >= st.max_code || abs((int)st.mean_mv - 1650) > 25) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_analoggen_descriptor = {
    .id = ANALOGGEN_TOOL_ID,
    .title = "Analog Generator",
    .symbol = LV_SYMBOL_AUDIO,
    .category = TOOL_CATEGORY_GENERATOR,
    .supports_background = true,
    .supports_recording = false,
    .supports_spk_passthrough = true,
    .on_init = NULL,
    .on_start = analoggen_on_start,
    .on_stop = analoggen_on_stop,
    .on_ui_refresh = analoggen_on_ui_refresh,
    .is_running_bg = analoggen_is_running_bg,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_analoggen_get_descriptor(void)
{
    return &s_analoggen_descriptor;
}

esp_err_t tool_analoggen_register(void)
{
    esp_err_t err = tool_registry_register(&s_analoggen_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("analoggen_lut", analoggen_selftest);
    }
    return err;
}
