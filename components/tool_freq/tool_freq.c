/**
 * @file tool_freq.c
 * @brief Core 1 Frequency Meter worker (RMT RX + PCNT + GPTimer), internal SRAM ISR bounce
 *        buffers, PSRAM staging and double-buffer handoff, recorder sink integration, and
 *        LVGL 9 UI for `tool_freq`.
 */

#include "tool_freq.h"
#include "tool_freq_priv.h"

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "psram_buffer.h"
#include "recorder_sink.h"
#include "tool_core.h"

static const char *TAG = "tool_freq";

#define FREQ_WORKER_PERIOD_MS       (20)
#define FREQ_WORKER_STOP_WAIT_MS    (2000)
#define FREQ_WORKER_STOP_RETRIES    (5)

static const gpio_num_t s_supported_pins[] = {
    BSP_PIN_ANALOG_IN_DEFAULT, /* GPIO36 (Port B / M5-Bus default) */
    BSP_PIN_ANALOG_IN_ALT,     /* GPIO35 (M5-Bus) */
    BSP_PIN_DAC_OUT_CH2,       /* GPIO26 (Port B) */
    BSP_PIN_PORT_C_RXD2,       /* GPIO13 (Port C) */
    BSP_PIN_PORT_C_TXD2,       /* GPIO14 (Port C) */
    BSP_PIN_PORT_A_SDA,        /* GPIO32 (Port A) */
    BSP_PIN_PORT_A_SCL,        /* GPIO33 (Port A) */
    BSP_PIN_MBUS_GPIO27,       /* GPIO27 (M5-Bus) */
    BSP_PIN_MBUS_GPIO19,       /* GPIO19 (M5-Bus) */
};
#define FREQ_SUPPORTED_PIN_COUNT (sizeof(s_supported_pins) / sizeof(s_supported_pins[0]))

static portMUX_TYPE s_freq_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_step_mutex_buf;
static SemaphoreHandle_t s_step_mutex = NULL;
static atomic_int s_step_mutex_state = 0;

static gpio_num_t s_selected_pin = BSP_PIN_ANALOG_IN_DEFAULT;
static freq_mode_t s_mode = FREQ_MODE_AUTO;
static freq_gate_window_t s_gate_window = FREQ_GATE_100MS;
static bool s_running = true;
static bool s_tool_active = false;
static bool s_reset_requested = false;
static volatile bool s_stop_requested = false;
static volatile bool s_step_preempt = false;
static volatile uint32_t s_capture_seq = 0;

/* Internal SRAM ISR bounce buffers & state (never in PSRAM) */
static DRAM_ATTR rmt_symbol_word_t s_rmt_bounce_buf[FREQ_RMT_BOUNCE_SYMBOLS];
static DRAM_ATTR volatile size_t s_rmt_isr_symbol_count = 0;
static DRAM_ATTR volatile bool s_rmt_isr_done = false;
static DRAM_ATTR volatile int64_t s_pcnt_overflow_accum = 0;
static DRAM_ATTR volatile bool s_gate_alarm_fired = false;

/* Hardware peripheral handles */
static gpio_num_t s_hw_configured_pin = GPIO_NUM_NC;
static pcnt_unit_handle_t s_pcnt_unit = NULL;
static pcnt_channel_handle_t s_pcnt_chan = NULL;
static rmt_channel_handle_t s_rmt_chan = NULL;
static bool s_rmt_enabled = false;
static gptimer_handle_t s_gptimer = NULL;
static bool s_gptimer_enabled = false;

/* PSRAM buffers & state */
static rmt_symbol_word_t *s_psram_symbols = NULL;
static freq_session_tracker_t s_tracker = {0};
static psram_double_buf_t s_frame_dbuf = {0};
static freq_reading_t s_latest_reading = {0};
static freq_method_t s_prev_method = FREQ_METHOD_NONE;
static tool_worker_t *s_worker = NULL;

static freq_capture_source_fn_t s_custom_source_fn = NULL;
static void *s_custom_source_ctx = NULL;

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_top_status = NULL;
static lv_obj_t *s_lbl_method_badge = NULL;
static lv_obj_t *s_lbl_freq_main = NULL;
static lv_obj_t *s_lbl_rpm = NULL;
static lv_obj_t *s_lbl_period_pw = NULL;
static lv_obj_t *s_lbl_duty_val = NULL;
static lv_obj_t *s_bar_duty = NULL;
static lv_obj_t *s_lbl_stats = NULL;
static lv_obj_t *s_btn_run_lbl = NULL;
static lv_obj_t *s_btn_pin_lbl = NULL;
static lv_obj_t *s_btn_mode_lbl = NULL;
static lv_obj_t *s_btn_gate_lbl = NULL;

static void ensure_freq_step_mutex(void)
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

bool tool_freq_is_pin_supported(gpio_num_t pin)
{
    for (size_t i = 0; i < FREQ_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == pin) {
            return true;
        }
    }
    return false;
}

esp_err_t tool_freq_set_pin(gpio_num_t pin)
{
    if (!tool_freq_is_pin_supported(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_freq_lock);
    gpio_num_t prev_pin = s_selected_pin;
    bool active = s_tool_active;
    portEXIT_CRITICAL(&s_freq_lock);

    if (pin == prev_pin) {
        return ESP_OK;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim(pin, FREQ_TOOL_ID);
        if (err != ESP_OK) {
            return err;
        }
        (void)bsp_pin_release(prev_pin, FREQ_TOOL_ID);
    }

    portENTER_CRITICAL(&s_freq_lock);
    s_selected_pin = pin;
    s_reset_requested = true;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

gpio_num_t tool_freq_get_pin(void)
{
    portENTER_CRITICAL(&s_freq_lock);
    gpio_num_t pin = s_selected_pin;
    portEXIT_CRITICAL(&s_freq_lock);
    return pin;
}

esp_err_t tool_freq_set_mode(freq_mode_t mode)
{
    if (mode < 0 || mode >= FREQ_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_freq_lock);
    if (s_mode != mode) {
        s_mode = mode;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

freq_mode_t tool_freq_get_mode(void)
{
    portENTER_CRITICAL(&s_freq_lock);
    freq_mode_t mode = s_mode;
    portEXIT_CRITICAL(&s_freq_lock);
    return mode;
}

esp_err_t tool_freq_set_gate_window(freq_gate_window_t gate)
{
    if (gate < 0 || gate >= FREQ_GATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_freq_lock);
    if (s_gate_window != gate) {
        s_gate_window = gate;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

freq_gate_window_t tool_freq_get_gate_window(void)
{
    portENTER_CRITICAL(&s_freq_lock);
    freq_gate_window_t gate = s_gate_window;
    portEXIT_CRITICAL(&s_freq_lock);
    return gate;
}

esp_err_t tool_freq_set_running(bool running)
{
    portENTER_CRITICAL(&s_freq_lock);
    if (s_running != running) {
        s_running = running;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

bool tool_freq_is_running(void)
{
    portENTER_CRITICAL(&s_freq_lock);
    bool r = s_running;
    portEXIT_CRITICAL(&s_freq_lock);
    return r;
}

esp_err_t tool_freq_reset_stats(void)
{
    portENTER_CRITICAL(&s_freq_lock);
    s_reset_requested = true;
    s_latest_reading.fmin_hz = 0.0f;
    s_latest_reading.fmax_hz = 0.0f;
    s_latest_reading.favg_hz = 0.0f;
    s_latest_reading.sample_Windows = 0U;
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

esp_err_t tool_freq_get_latest_reading(freq_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_freq_lock);
    *out_reading = s_latest_reading;
    portEXIT_CRITICAL(&s_freq_lock);
    return ESP_OK;
}

void tool_freq_set_capture_source(freq_capture_source_fn_t fn, void *user_ctx)
{
    portENTER_CRITICAL(&s_freq_lock);
    s_custom_source_fn = fn;
    s_custom_source_ctx = user_ctx;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_freq_lock);
}

/* --------------------------------------------------------------------------
 * Hardware ISR Callbacks (Internal SRAM Only)
 * -------------------------------------------------------------------------- */

static bool IRAM_ATTR on_rmt_rx_done_isr(rmt_channel_handle_t channel,
                                         const rmt_rx_done_event_data_t *edata,
                                         void *user_data)
{
    (void)channel;
    (void)user_data;
    if (edata != NULL) {
        size_t n = edata->num_symbols;
        if (n > FREQ_RMT_BOUNCE_SYMBOLS) {
            n = FREQ_RMT_BOUNCE_SYMBOLS;
        }
        s_rmt_isr_symbol_count = n;
    } else {
        s_rmt_isr_symbol_count = 0U;
    }
    s_rmt_isr_done = true;
    return false;
}

static bool IRAM_ATTR on_pcnt_watch_reach_isr(pcnt_unit_handle_t unit,
                                              const pcnt_watch_event_data_t *edata,
                                              void *user_ctx)
{
    (void)unit;
    (void)user_ctx;
    if (edata != NULL) {
        s_pcnt_overflow_accum += (int64_t)edata->watch_point_value;
    }
    return false;
}

static bool IRAM_ATTR on_gate_timer_alarm_isr(gptimer_handle_t timer,
                                              const gptimer_alarm_event_data_t *edata,
                                              void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;
    s_gate_alarm_fired = true;
    return false;
}

/* --------------------------------------------------------------------------
 * Hardware Peripheral Setup & Teardown (RMT RX + PCNT + GPTimer)
 * -------------------------------------------------------------------------- */

static void teardown_freq_peripherals(void)
{
    if (s_gptimer != NULL) {
        if (s_gptimer_enabled) {
            (void)gptimer_stop(s_gptimer);
            (void)gptimer_disable(s_gptimer);
            s_gptimer_enabled = false;
        }
        (void)gptimer_del_timer(s_gptimer);
        s_gptimer = NULL;
    }

    if (s_rmt_chan != NULL) {
        if (s_rmt_enabled) {
            (void)rmt_disable(s_rmt_chan);
            s_rmt_enabled = false;
        }
        (void)rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
    }

    if (s_pcnt_chan != NULL) {
        (void)pcnt_del_channel(s_pcnt_chan);
        s_pcnt_chan = NULL;
    }
    if (s_pcnt_unit != NULL) {
        (void)pcnt_unit_stop(s_pcnt_unit);
        (void)pcnt_unit_disable(s_pcnt_unit);
        (void)pcnt_del_unit(s_pcnt_unit);
        s_pcnt_unit = NULL;
    }

    s_hw_configured_pin = GPIO_NUM_NC;
}

static esp_err_t ensure_freq_peripherals(gpio_num_t pin)
{
    if (s_hw_configured_pin == pin && s_pcnt_unit != NULL && s_gptimer != NULL) {
        return ESP_OK;
    }

    teardown_freq_peripherals();

    /* 1. Configure GPTimer at 1 MHz resolution for gate window timing */
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = FREQ_GPTIMER_RESOLUTION_HZ,
    };
    if (gptimer_new_timer(&tcfg, &s_gptimer) == ESP_OK && s_gptimer != NULL) {
        gptimer_event_callbacks_t tcbs = {
            .on_alarm = on_gate_timer_alarm_isr,
        };
        (void)gptimer_register_event_callbacks(s_gptimer, &tcbs, NULL);
        if (gptimer_enable(s_gptimer) == ESP_OK) {
            s_gptimer_enabled = true;
        }
    }

    /* 2. Configure PCNT unit & channel on `pin` (both rising & falling edges -> 2x resolution) */
    pcnt_unit_config_t ucfg = {
        .low_limit = FREQ_PCNT_LOW_LIMIT,
        .high_limit = FREQ_PCNT_HIGH_LIMIT,
    };
    if (pcnt_new_unit(&ucfg, &s_pcnt_unit) == ESP_OK && s_pcnt_unit != NULL) {
        pcnt_glitch_filter_config_t fcfg = {
            .max_glitch_ns = FREQ_DEFAULT_GLITCH_NS,
        };
        (void)pcnt_unit_set_glitch_filter(s_pcnt_unit, &fcfg);
        (void)pcnt_unit_add_watch_point(s_pcnt_unit, FREQ_PCNT_HIGH_LIMIT);
        pcnt_event_callbacks_t pcbs = {
            .on_reach = on_pcnt_watch_reach_isr,
        };
        (void)pcnt_unit_register_event_callbacks(s_pcnt_unit, &pcbs, NULL);

        pcnt_chan_config_t ccfg = {
            .edge_gpio_num = pin,
            .level_gpio_num = -1,
        };
        if (pcnt_new_channel(s_pcnt_unit, &ccfg, &s_pcnt_chan) == ESP_OK && s_pcnt_chan != NULL) {
            (void)pcnt_channel_set_edge_action(
                s_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        }
        (void)pcnt_unit_enable(s_pcnt_unit);
    }

    /* 3. Configure RMT RX channel on `pin` at 1 MHz (1 us tick) resolution */
    rmt_rx_channel_config_t rcfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = FREQ_RMT_RESOLUTION_HZ,
        .mem_block_symbols = FREQ_RMT_BOUNCE_SYMBOLS,
    };
    if (rmt_new_rx_channel(&rcfg, &s_rmt_chan) == ESP_OK && s_rmt_chan != NULL) {
        rmt_rx_event_callbacks_t rcbs = {
            .on_recv_done = on_rmt_rx_done_isr,
        };
        (void)rmt_rx_register_event_callbacks(s_rmt_chan, &rcbs, NULL);
        if (rmt_enable(s_rmt_chan) == ESP_OK) {
            s_rmt_enabled = true;
        }
    }

    s_hw_configured_pin = pin;
    return ESP_OK;
}

static bool should_abort_capture(uint32_t start_seq, bool is_bg_worker)
{
    portENTER_CRITICAL(&s_freq_lock);
    bool abort = s_stop_requested || !s_running || (s_capture_seq != start_seq) ||
                 (is_bg_worker && s_step_preempt);
    portEXIT_CRITICAL(&s_freq_lock);
    return abort;
}

static bool wait_gate_or_abort(int64_t deadline_us, uint32_t start_seq, bool is_bg_worker)
{
    int64_t remaining_us = deadline_us - esp_timer_get_time();
    while (remaining_us > 0 && !s_gate_alarm_fired) {
        if (should_abort_capture(start_seq, is_bg_worker)) {
            return false;
        }
        if (remaining_us >= 2000) {
            uint32_t delay_ms = (uint32_t)((remaining_us - 1000) / 1000);
            if (delay_ms > 5U) {
                delay_ms = 5U;
            }
            TickType_t ticks = pdMS_TO_TICKS(delay_ms);
            if (ticks == 0) {
                ticks = 1;
            }
            vTaskDelay(ticks);
        } else {
            taskYIELD();
        }
        remaining_us = deadline_us - esp_timer_get_time();
    }
    return !should_abort_capture(start_seq, is_bg_worker);
}

static esp_err_t capture_freq_window(gpio_num_t pin,
                                     uint32_t gate_window_us,
                                     uint32_t start_seq,
                                     bool is_bg_worker,
                                     rmt_symbol_word_t *out_psram_symbols,
                                     size_t max_symbols,
                                     size_t *out_num_symbols,
                                     uint64_t *out_pcnt_pulses,
                                     uint64_t *out_actual_gate_us,
                                     freq_pcnt_edge_mode_t *out_edge_mode,
                                     uint64_t *out_timestamp_us,
                                     bool *out_aborted)
{
    *out_aborted = false;
    *out_num_symbols = 0U;
    *out_pcnt_pulses = 0U;
    *out_actual_gate_us = gate_window_us;
    *out_edge_mode = FREQ_PCNT_EDGES_BOTH;
    *out_timestamp_us = (uint64_t)esp_timer_get_time();

    freq_capture_source_fn_t custom_fn = NULL;
    void *custom_ctx = NULL;
    portENTER_CRITICAL(&s_freq_lock);
    custom_fn = s_custom_source_fn;
    custom_ctx = s_custom_source_ctx;
    portEXIT_CRITICAL(&s_freq_lock);

    if (custom_fn != NULL) {
        return custom_fn(pin,
                         gate_window_us,
                         out_psram_symbols,
                         max_symbols,
                         out_num_symbols,
                         out_pcnt_pulses,
                         out_actual_gate_us,
                         out_edge_mode,
                         custom_ctx);
    }

    (void)ensure_freq_peripherals(pin);

    s_rmt_isr_symbol_count = 0U;
    s_rmt_isr_done = false;
    s_pcnt_overflow_accum = 0;
    s_gate_alarm_fired = false;

    /* Start RMT RX capture into internal SRAM bounce buffer */
    if (s_rmt_chan != NULL && s_rmt_enabled) {
        rmt_receive_config_t rx_cfg = {
            .signal_range_min_ns = FREQ_DEFAULT_GLITCH_NS,
            .signal_range_max_ns = 20000000U, /* 20 ms idle threshold */
        };
        (void)rmt_receive(
            s_rmt_chan, s_rmt_bounce_buf, sizeof(s_rmt_bounce_buf), &rx_cfg);
    }

    /* Reset and start PCNT hardware pulse counter + GPTimer gate window */
    if (s_pcnt_unit != NULL) {
        (void)pcnt_unit_stop(s_pcnt_unit);
        (void)pcnt_unit_clear_count(s_pcnt_unit);
    }
    if (s_gptimer != NULL && s_gptimer_enabled) {
        (void)gptimer_stop(s_gptimer);
        (void)gptimer_set_raw_count(s_gptimer, 0ULL);
        gptimer_alarm_config_t acfg = {
            .alarm_count = gate_window_us,
            .reload_count = 0ULL,
        };
        (void)gptimer_set_alarm_action(s_gptimer, &acfg);
        (void)gptimer_start(s_gptimer);
    }

    int64_t start_us = esp_timer_get_time();
    *out_timestamp_us = (uint64_t)start_us;
    if (s_pcnt_unit != NULL) {
        (void)pcnt_unit_start(s_pcnt_unit);
    }

    bool ok = wait_gate_or_abort(start_us + (int64_t)gate_window_us, start_seq, is_bg_worker);

    if (s_pcnt_unit != NULL) {
        (void)pcnt_unit_stop(s_pcnt_unit);
    }
    int64_t end_us = esp_timer_get_time();
    if (s_gptimer != NULL && s_gptimer_enabled) {
        (void)gptimer_stop(s_gptimer);
    }

    /* Reset RMT RX channel if still waiting for edges so next receive is clean */
    if (s_rmt_chan != NULL && s_rmt_enabled && !s_rmt_isr_done) {
        (void)rmt_disable(s_rmt_chan);
        (void)rmt_enable(s_rmt_chan);
    }

    if (!ok) {
        *out_aborted = true;
        return ESP_ERR_INVALID_STATE;
    }

    int unit_count = 0;
    if (s_pcnt_unit != NULL) {
        (void)pcnt_unit_get_count(s_pcnt_unit, &unit_count);
    }
    int64_t total_edges = s_pcnt_overflow_accum + (int64_t)unit_count;
    if (total_edges < 0) {
        total_edges = 0;
    }
    *out_pcnt_pulses = (uint64_t)total_edges;

    int64_t elapsed_us = end_us - start_us;
    if (elapsed_us <= 0) {
        elapsed_us = (int64_t)gate_window_us;
    }
    *out_actual_gate_us = (uint64_t)elapsed_us;

    /* Copy RMT symbols from internal SRAM bounce buffer into PSRAM staging buffer */
    size_t rmt_cnt = s_rmt_isr_symbol_count;
    if (rmt_cnt > max_symbols) {
        rmt_cnt = max_symbols;
    }
    if (rmt_cnt > 0U && out_psram_symbols != NULL) {
        memcpy(out_psram_symbols, (const void *)s_rmt_bounce_buf, rmt_cnt * sizeof(rmt_symbol_word_t));
        *out_num_symbols = rmt_cnt;
    }

    return ESP_OK;
}

static esp_err_t freq_worker_step_locked(bool is_bg_worker)
{
    if (s_psram_symbols == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_freq_lock);
    gpio_num_t pin = s_selected_pin;
    freq_mode_t mode = s_mode;
    freq_gate_window_t gate_win = s_gate_window;
    bool running = s_running;
    bool stop_req = s_stop_requested;
    bool do_reset = s_reset_requested;
    s_reset_requested = false;
    uint32_t start_seq = s_capture_seq;
    freq_method_t prev_method = s_prev_method;
    portEXIT_CRITICAL(&s_freq_lock);

    if (do_reset) {
        freq_calc_tracker_reset(&s_tracker);
    }

    if (stop_req || !running) {
        return ESP_OK;
    }

    uint32_t gate_us = freq_calc_gate_window_us(gate_win);
    if (gate_us == 0U) {
        gate_us = 100000U;
    }

    size_t num_symbols = 0U;
    uint64_t pcnt_pulses = 0U;
    uint64_t actual_gate_us = gate_us;
    freq_pcnt_edge_mode_t edge_mode = FREQ_PCNT_EDGES_BOTH;
    uint64_t capture_ts_us = 0U;
    bool aborted = false;

    esp_err_t err = capture_freq_window(pin,
                                        gate_us,
                                        start_seq,
                                        is_bg_worker,
                                        s_psram_symbols,
                                        FREQ_PSRAM_SYMBOL_BUF_CAP,
                                        &num_symbols,
                                        &pcnt_pulses,
                                        &actual_gate_us,
                                        &edge_mode,
                                        &capture_ts_us,
                                        &aborted);
    if (aborted) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    freq_period_result_t period_res = {0};
    (void)freq_calc_from_rmt_symbols(
        s_psram_symbols, num_symbols, FREQ_RMT_RESOLUTION_HZ, FREQ_DEFAULT_GLITCH_NS, &period_res);

    freq_gate_result_t gate_res = {0};
    (void)freq_calc_from_gate_counts(pcnt_pulses, actual_gate_us, edge_mode, &gate_res);

    freq_reading_t reading = {0};
    (void)freq_calc_combine_reading(mode, prev_method, &period_res, &gate_res, &s_tracker, &reading);

    if (recorder_sink_is_recording()) {
        uint32_t update_rate_hz = (actual_gate_us > 0U && actual_gate_us <= 1000000ULL)
                                      ? (uint32_t)((1000000ULL + actual_gate_us / 2ULL) / actual_gate_us)
                                      : 1U;
        if (num_symbols > 0U) {
            recorder_frame_t rec_frame = {
                .tool_id = FREQ_TOOL_ID,
                .sample_rate_hz = FREQ_RMT_RESOLUTION_HZ,
                .timestamp_us = capture_ts_us,
                .samples = s_psram_symbols,
                .sample_count = num_symbols,
                .bytes_per_sample = sizeof(rmt_symbol_word_t),
            };
            (void)recorder_sink_push(&rec_frame);
        } else {
            recorder_frame_t rec_frame = {
                .tool_id = FREQ_TOOL_ID,
                .sample_rate_hz = update_rate_hz,
                .timestamp_us = capture_ts_us,
                .samples = &reading,
                .sample_count = 1U,
                .bytes_per_sample = sizeof(freq_reading_t),
            };
            (void)recorder_sink_push(&rec_frame);
        }
    }

    portENTER_CRITICAL(&s_freq_lock);
    s_prev_method = reading.active_method;
    s_latest_reading = reading;
    portEXIT_CRITICAL(&s_freq_lock);

    freq_frame_t *wptr = (freq_frame_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        wptr->reading = reading;
        wptr->pin = pin;
        wptr->mode = mode;
        wptr->gate_window = gate_win;
        wptr->running = running;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }

    return ESP_OK;
}

static esp_err_t freq_worker_step(void *ctx)
{
    (void)ctx;
    ensure_freq_step_mutex();
    if (xSemaphoreTake(s_step_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_OK;
    }
    esp_err_t err = freq_worker_step_locked(true);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

esp_err_t tool_freq_run_single_step(void)
{
    ensure_freq_step_mutex();
    portENTER_CRITICAL(&s_freq_lock);
    s_step_preempt = true;
    portEXIT_CRITICAL(&s_freq_lock);

    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    portENTER_CRITICAL(&s_freq_lock);
    s_step_preempt = false;
    portEXIT_CRITICAL(&s_freq_lock);

    esp_err_t err = freq_worker_step_locked(false);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Event Callbacks & Rendering (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_run_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_freq_set_running(!tool_freq_is_running());
}

static void on_btn_pin_clicked(lv_event_t *e)
{
    (void)e;
    gpio_num_t cur = tool_freq_get_pin();
    size_t idx = 0U;
    for (size_t i = 0; i < FREQ_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == cur) {
            idx = i;
            break;
        }
    }
    for (size_t step = 1U; step <= FREQ_SUPPORTED_PIN_COUNT; ++step) {
        gpio_num_t cand = s_supported_pins[(idx + step) % FREQ_SUPPORTED_PIN_COUNT];
        if (tool_freq_set_pin(cand) == ESP_OK) {
            break;
        }
    }
}

static void on_btn_mode_clicked(lv_event_t *e)
{
    (void)e;
    freq_mode_t next = (freq_mode_t)((tool_freq_get_mode() + 1) % FREQ_MODE_COUNT);
    (void)tool_freq_set_mode(next);
}

static void on_btn_gate_clicked(lv_event_t *e)
{
    (void)e;
    freq_gate_window_t next = (freq_gate_window_t)((tool_freq_get_gate_window() + 1) % FREQ_GATE_COUNT);
    (void)tool_freq_set_gate_window(next);
}

static void on_btn_rst_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_freq_reset_stats();
}

static void build_freq_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 3, 0);
    lv_obj_set_style_pad_row(parent, 3, 0);

    /* 1. Top Status Bar (20 px) */
    lv_obj_t *top_row = lv_obj_create(parent);
    lv_obj_set_size(top_row, 312, 20);
    lv_obj_set_style_bg_color(top_row, lv_color_hex(0x16202A), 0);
    lv_obj_set_style_border_width(top_row, 0, 0);
    lv_obj_set_style_radius(top_row, 4, 0);
    lv_obj_set_style_pad_hor(top_row, 6, 0);
    lv_obj_set_style_pad_ver(top_row, 1, 0);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_top_status = lv_label_create(top_row);
    lv_label_set_text(s_lbl_top_status, "IN:GPIO36  Gate:100ms");
    lv_obj_set_style_text_font(s_lbl_top_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_top_status, lv_color_hex(0x4DD0E1), 0);

    s_lbl_method_badge = lv_label_create(top_row);
    lv_label_set_text(s_lbl_method_badge, "AUTO [NO SIG]");
    lv_obj_set_style_text_font(s_lbl_method_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_method_badge, lv_color_hex(0xFFD54F), 0);

    /* 2. Primary Frequency, Period, Pulse Width, Duty & RPM Card (112 px) */
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, 312, 112);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0E171F), 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0x00897B), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 6, 0);
    lv_obj_set_style_pad_all(card, 6, 0);
    lv_obj_set_style_pad_row(card, 3, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *freq_row = lv_obj_create(card);
    lv_obj_set_size(freq_row, 296, 36);
    lv_obj_set_style_bg_opa(freq_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(freq_row, 0, 0);
    lv_obj_set_style_pad_all(freq_row, 0, 0);
    lv_obj_set_flex_flow(freq_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(freq_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_freq_main = lv_label_create(freq_row);
    lv_label_set_text(s_lbl_freq_main, "0.00 Hz");
    lv_obj_set_style_text_font(s_lbl_freq_main, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_lbl_freq_main, lv_color_hex(0x00E676), 0);

    s_lbl_rpm = lv_label_create(freq_row);
    lv_label_set_text(s_lbl_rpm, "0 RPM");
    lv_obj_set_style_text_font(s_lbl_rpm, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lbl_rpm, lv_color_hex(0x80CBC4), 0);

    s_lbl_period_pw = lv_label_create(card);
    lv_label_set_text(s_lbl_period_pw, "T: 0.0 us   PW+: 0.0 us   PW-: 0.0 us");
    lv_obj_set_style_text_font(s_lbl_period_pw, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_period_pw, lv_color_hex(0xFFE082), 0);

    lv_obj_t *duty_row = lv_obj_create(card);
    lv_obj_set_size(duty_row, 296, 22);
    lv_obj_set_style_bg_opa(duty_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(duty_row, 0, 0);
    lv_obj_set_style_pad_all(duty_row, 0, 0);
    lv_obj_set_flex_flow(duty_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(duty_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_duty_val = lv_label_create(duty_row);
    lv_label_set_text(s_lbl_duty_val, "Duty: 0.0%");
    lv_obj_set_style_text_font(s_lbl_duty_val, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_duty_val, lv_color_hex(0xCFD8DC), 0);

    s_bar_duty = lv_bar_create(duty_row);
    lv_obj_set_size(s_bar_duty, 190, 12);
    lv_bar_set_range(s_bar_duty, 0, 100);
    lv_bar_set_value(s_bar_duty, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_bar_duty, lv_color_hex(0x263238), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar_duty, lv_color_hex(0x29B6F6), LV_PART_INDICATOR);

    /* 3. Session Min / Max / Avg & Edge Count Row (18 px) */
    s_lbl_stats = lv_label_create(parent);
    lv_label_set_text(s_lbl_stats, "Min:0.00Hz Max:0.00Hz Avg:0.00Hz Edges:0");
    lv_obj_set_style_text_font(s_lbl_stats, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_stats, lv_color_hex(0xB0BEC5), 0);

    /* 4. Bottom Touch Controls Row (30 px) */
    lv_obj_t *ctrl_row = lv_obj_create(parent);
    lv_obj_set_size(ctrl_row, 312, 30);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_style_pad_column(ctrl_row, 4, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_run = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_run, 56, 28);
    lv_obj_set_style_bg_color(btn_run, lv_color_hex(0x2E7D32), 0);
    lv_obj_add_event_cb(btn_run, on_btn_run_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_run_lbl = lv_label_create(btn_run);
    lv_label_set_text(s_btn_run_lbl, "RUN");
    lv_obj_set_style_text_font(s_btn_run_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_run_lbl);

    lv_obj_t *btn_pin = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pin, 56, 28);
    lv_obj_set_style_bg_color(btn_pin, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_pin, on_btn_pin_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_pin_lbl = lv_label_create(btn_pin);
    lv_label_set_text(s_btn_pin_lbl, "G36");
    lv_obj_set_style_text_font(s_btn_pin_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_pin_lbl);

    lv_obj_t *btn_mode = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_mode, 62, 28);
    lv_obj_set_style_bg_color(btn_mode, lv_color_hex(0x0277BD), 0);
    lv_obj_add_event_cb(btn_mode, on_btn_mode_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_mode_lbl = lv_label_create(btn_mode);
    lv_label_set_text(s_btn_mode_lbl, "AUTO");
    lv_obj_set_style_text_font(s_btn_mode_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_mode_lbl);

    lv_obj_t *btn_gate = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_gate, 64, 28);
    lv_obj_set_style_bg_color(btn_gate, lv_color_hex(0x4527A0), 0);
    lv_obj_add_event_cb(btn_gate, on_btn_gate_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_gate_lbl = lv_label_create(btn_gate);
    lv_label_set_text(s_btn_gate_lbl, "100ms");
    lv_obj_set_style_text_font(s_btn_gate_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_gate_lbl);

    lv_obj_t *btn_rst = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_rst, 54, 28);
    lv_obj_set_style_bg_color(btn_rst, lv_color_hex(0x455A64), 0);
    lv_obj_add_event_cb(btn_rst, on_btn_rst_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_rst = lv_label_create(btn_rst);
    lv_label_set_text(lbl_rst, "RST");
    lv_obj_set_style_text_font(lbl_rst, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_rst);
}

static void freq_on_ui_refresh(void *ctx)
{
    (void)ctx;
    freq_frame_t frame = {0};
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &frame, 1, &copied) != ESP_OK || copied == 0) {
        return;
    }

    gpio_num_t pin = tool_freq_get_pin();
    freq_mode_t mode = tool_freq_get_mode();
    freq_gate_window_t gate = tool_freq_get_gate_window();
    bool running = tool_freq_is_running();

    if (s_lbl_top_status != NULL) {
        lv_label_set_text_fmt(s_lbl_top_status,
                              "IN:GPIO%d  Gate:%s",
                              (int)pin,
                              freq_calc_gate_window_to_str(gate));
    }

    if (s_lbl_method_badge != NULL) {
        if (!running) {
            lv_label_set_text(s_lbl_method_badge, "HOLD");
            lv_obj_set_style_text_color(s_lbl_method_badge, lv_color_hex(0xFF5252), 0);
        } else if (!frame.reading.signal_detected) {
            lv_label_set_text_fmt(s_lbl_method_badge, "%s [NO SIG]", freq_calc_mode_to_str(mode));
            lv_obj_set_style_text_color(s_lbl_method_badge, lv_color_hex(0xFFD54F), 0);
        } else if (frame.reading.active_method == FREQ_METHOD_PERIOD) {
            lv_label_set_text_fmt(s_lbl_method_badge, "%s [RMT 1/T]", freq_calc_mode_to_str(mode));
            lv_obj_set_style_text_color(s_lbl_method_badge, lv_color_hex(0x00E676), 0);
        } else {
            lv_label_set_text_fmt(s_lbl_method_badge, "%s [PCNT GATE]", freq_calc_mode_to_str(mode));
            lv_obj_set_style_text_color(s_lbl_method_badge, lv_color_hex(0x29B6F6), 0);
        }
    }

    if (s_lbl_freq_main != NULL) {
        char fbuf[32];
        freq_calc_format_freq(frame.reading.freq_hz, fbuf, sizeof(fbuf));
        lv_label_set_text(s_lbl_freq_main, fbuf);
    }

    if (s_lbl_rpm != NULL) {
        long rpm_val = lroundf(frame.reading.rpm);
        lv_label_set_text_fmt(s_lbl_rpm, "%ld RPM", rpm_val);
    }

    if (s_lbl_period_pw != NULL) {
        long t_tenths = lroundf(frame.reading.period_us * 10.0f);
        long pw_tenths = lroundf(frame.reading.pulse_width_us * 10.0f);
        long lw_tenths = lroundf(frame.reading.low_width_us * 10.0f);
        lv_label_set_text_fmt(s_lbl_period_pw,
                              "T:%ld.%ldus  PW+:%ld.%ldus  PW-:%ld.%ldus",
                              t_tenths / 10L, labs(t_tenths % 10L),
                              pw_tenths / 10L, labs(pw_tenths % 10L),
                              lw_tenths / 10L, labs(lw_tenths % 10L));
    }

    int duty_tenths = (int)lroundf(frame.reading.duty_pct * 10.0f);
    if (s_lbl_duty_val != NULL) {
        lv_label_set_text_fmt(
            s_lbl_duty_val, "Duty: %d.%d%%", duty_tenths / 10, abs(duty_tenths % 10));
    }
    if (s_bar_duty != NULL) {
        int duty_clamped = duty_tenths / 10;
        if (duty_clamped < 0) {
            duty_clamped = 0;
        } else if (duty_clamped > 100) {
            duty_clamped = 100;
        }
        lv_bar_set_value(s_bar_duty, duty_clamped, LV_ANIM_OFF);
    }

    if (s_lbl_stats != NULL) {
        char fmin_s[20], fmax_s[20], favg_s[20];
        freq_calc_format_freq(frame.reading.fmin_hz, fmin_s, sizeof(fmin_s));
        freq_calc_format_freq(frame.reading.fmax_hz, fmax_s, sizeof(fmax_s));
        freq_calc_format_freq(frame.reading.favg_hz, favg_s, sizeof(favg_s));
        lv_label_set_text_fmt(s_lbl_stats,
                              "Min:%s Max:%s Avg:%s N:%lu",
                              fmin_s,
                              fmax_s,
                              favg_s,
                              (unsigned long)frame.reading.edge_count);
    }

    if (s_btn_run_lbl != NULL) {
        lv_label_set_text(s_btn_run_lbl, running ? "RUN" : "HOLD");
    }
    if (s_btn_pin_lbl != NULL) {
        lv_label_set_text_fmt(s_btn_pin_lbl, "G%d", (int)pin);
    }
    if (s_btn_mode_lbl != NULL) {
        lv_label_set_text(s_btn_mode_lbl, freq_calc_mode_to_str(mode));
    }
    if (s_btn_gate_lbl != NULL) {
        lv_label_set_text(s_btn_gate_lbl, freq_calc_gate_window_to_str(gate));
    }
}

/* --------------------------------------------------------------------------
 * Tool Plugin Lifecycle Callbacks
 * -------------------------------------------------------------------------- */

static void free_freq_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    freq_calc_tracker_free(&s_tracker);
    if (s_psram_symbols != NULL) {
        psram_buf_free(s_psram_symbols);
        s_psram_symbols = NULL;
    }
}

static esp_err_t freq_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    gpio_num_t pin = tool_freq_get_pin();

    esp_err_t err = bsp_pin_claim(pin, FREQ_TOOL_ID);
    if (err != ESP_OK) {
        return err;
    }

    err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(freq_frame_t));
    if (err != ESP_OK) {
        (void)bsp_pin_release(pin, FREQ_TOOL_ID);
        return err;
    }

    err = freq_calc_tracker_init(&s_tracker, FREQ_SESSION_AVG_WINDOW);
    if (err != ESP_OK) {
        free_freq_buffers();
        (void)bsp_pin_release(pin, FREQ_TOOL_ID);
        return err;
    }

    s_psram_symbols = (rmt_symbol_word_t *)psram_buf_calloc(
        FREQ_PSRAM_SYMBOL_BUF_CAP, sizeof(rmt_symbol_word_t));
    if (s_psram_symbols == NULL) {
        free_freq_buffers();
        (void)bsp_pin_release(pin, FREQ_TOOL_ID);
        return ESP_ERR_NO_MEM;
    }

    ensure_freq_step_mutex();

    portENTER_CRITICAL(&s_freq_lock);
    s_running = true;
    s_stop_requested = false;
    s_step_preempt = false;
    s_reset_requested = false;
    s_capture_seq++;
    s_prev_method = FREQ_METHOD_NONE;
    memset(&s_latest_reading, 0, sizeof(s_latest_reading));
    s_tool_active = true;
    freq_capture_source_fn_t custom_fn = s_custom_source_fn;
    freq_gate_window_t start_gate = s_gate_window;
    portEXIT_CRITICAL(&s_freq_lock);

    if (parent_cont != NULL) {
        build_freq_ui(parent_cont);
    }

    /* Run an immediate initial step when a custom source or fast 20ms gate is configured */
    if (custom_fn != NULL || start_gate == FREQ_GATE_20MS) {
        (void)tool_freq_run_single_step();
    }

    tool_worker_config_t wcfg = {
        .name = "freq_c1",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = TOOL_WORKER_DEFAULT_CORE,
        .period_ms = FREQ_WORKER_PERIOD_MS,
        .step_fn = freq_worker_step,
        .user_ctx = NULL,
    };
    err = tool_worker_start(&s_worker, &wcfg);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_freq_lock);
        s_tool_active = false;
        portEXIT_CRITICAL(&s_freq_lock);
        teardown_freq_peripherals();
        free_freq_buffers();
        (void)bsp_pin_release(pin, FREQ_TOOL_ID);
        return err;
    }

    ESP_LOGI(TAG, "Frequency Meter started on GPIO%d", (int)pin);
    return ESP_OK;
}

static esp_err_t freq_on_stop(bool keep_background, void *ctx)
{
    (void)keep_background;
    (void)ctx;

    portENTER_CRITICAL(&s_freq_lock);
    s_stop_requested = true;
    s_capture_seq++;
    tool_worker_t *worker = s_worker;
    s_worker = NULL;
    portEXIT_CRITICAL(&s_freq_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < FREQ_WORKER_STOP_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, FREQ_WORKER_STOP_WAIT_MS);
        }
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop freq worker cleanly: %s", esp_err_to_name(stop_err));
            portENTER_CRITICAL(&s_freq_lock);
            s_worker = worker;
            portEXIT_CRITICAL(&s_freq_lock);
            return stop_err;
        }
    }

    ensure_freq_step_mutex();
    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    s_lbl_top_status = NULL;
    s_lbl_method_badge = NULL;
    s_lbl_freq_main = NULL;
    s_lbl_rpm = NULL;
    s_lbl_period_pw = NULL;
    s_lbl_duty_val = NULL;
    s_bar_duty = NULL;
    s_lbl_stats = NULL;
    s_btn_run_lbl = NULL;
    s_btn_pin_lbl = NULL;
    s_btn_mode_lbl = NULL;
    s_btn_gate_lbl = NULL;

    teardown_freq_peripherals();
    free_freq_buffers();

    portENTER_CRITICAL(&s_freq_lock);
    gpio_num_t pin = s_selected_pin;
    s_tool_active = false;
    portEXIT_CRITICAL(&s_freq_lock);

    (void)xSemaphoreGive(s_step_mutex);

    (void)bsp_pin_release(pin, FREQ_TOOL_ID);
    return ESP_OK;
}

static esp_err_t freq_selftest(void)
{
    /* Verify 1 kHz 25% duty cycle RMT symbol period calculation (250 us high, 750 us low) */
    rmt_symbol_word_t sym = {
        .duration0 = 250U,
        .level0 = 1U,
        .duration1 = 750U,
        .level1 = 0U,
    };
    freq_period_result_t pres = {0};
    if (freq_calc_from_rmt_symbols(&sym, 1U, FREQ_RMT_RESOLUTION_HZ, FREQ_DEFAULT_GLITCH_NS, &pres) != ESP_OK) {
        return ESP_FAIL;
    }
    if (!pres.valid || fabsf(pres.freq_hz - 1000.0f) > 1.0f || fabsf(pres.duty_pct - 25.0f) > 0.5f ||
        fabsf(pres.rpm - 60000.0f) > 10.0f) {
        return ESP_FAIL;
    }

    /* Verify 250 kHz PCNT gate counting (50,000 both-edges in 100,000 us -> 250,000 Hz) */
    freq_gate_result_t gres = {0};
    if (freq_calc_from_gate_counts(50000ULL, 100000ULL, FREQ_PCNT_EDGES_BOTH, &gres) != ESP_OK) {
        return ESP_FAIL;
    }
    if (!gres.valid || fabsf(gres.freq_hz - 250000.0f) > 1.0f) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_freq_descriptor = {
    .id = FREQ_TOOL_ID,
    .title = "Freq Meter",
    .symbol = LV_SYMBOL_LOOP,
    .category = TOOL_CATEGORY_DIGITAL_IN,
    .supports_background = false,
    .supports_recording = true,
    .supports_spk_passthrough = false,
    .on_init = NULL,
    .on_start = freq_on_start,
    .on_stop = freq_on_stop,
    .on_ui_refresh = freq_on_ui_refresh,
    .is_running_bg = NULL,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_freq_get_descriptor(void)
{
    return &s_freq_descriptor;
}

esp_err_t tool_freq_register(void)
{
    esp_err_t err = tool_registry_register(&s_freq_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("freq_calc", freq_selftest);
    }
    return err;
}
