/**
 * @file tool_logic.c
 * @brief Core 1 Multi-Channel Logic Analyzer worker (GPTimer ISR + GPIO.in/in1 bounce buffer
 *        and RMT RX edge-timestamp capture), 64 KB PSRAM staging, UART/I²C/SPI/PWM decoding,
 *        recorder sink export, and LVGL 9 multi-lane waveform + modal UI for `tool_logic`.
 */

#include "tool_logic.h"
#include "tool_logic_priv.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "psram_buffer.h"
#include "recorder_sink.h"
#include "soc/gpio_struct.h"
#include "tool_core.h"

static const char *TAG = "tool_logic";

#define LOGIC_WORKER_PERIOD_MS      (25)
#define LOGIC_WORKER_STOP_WAIT_MS   (2000)
#define LOGIC_WORKER_STOP_RETRIES   (5)
#define LOGIC_PAN_STEP_SAMPLES      (30U)

static const gpio_num_t s_supported_pins[] = {
    BSP_PIN_ANALOG_IN_DEFAULT, /* GPIO36 */
    BSP_PIN_ANALOG_IN_ALT,     /* GPIO35 */
    BSP_PIN_DAC_OUT_CH2,       /* GPIO26 */
    BSP_PIN_PORT_C_RXD2,       /* GPIO13 */
    BSP_PIN_PORT_C_TXD2,       /* GPIO14 */
    BSP_PIN_PORT_A_SDA,        /* GPIO32 */
    BSP_PIN_PORT_A_SCL,        /* GPIO33 */
    BSP_PIN_MBUS_GPIO27,       /* GPIO27 */
    BSP_PIN_MBUS_GPIO19,       /* GPIO19 */
};
#define LOGIC_SUPPORTED_PIN_COUNT (sizeof(s_supported_pins) / sizeof(s_supported_pins[0]))

static const uint32_t s_uart_baud_presets[] = {9600U, 19200U, 38400U, 57600U, 115200U};
#define LOGIC_UART_BAUD_COUNT (sizeof(s_uart_baud_presets) / sizeof(s_uart_baud_presets[0]))

static portMUX_TYPE s_logic_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_step_mutex_buf;
static SemaphoreHandle_t s_step_mutex = NULL;
static atomic_int s_step_mutex_state = 0;

static gpio_num_t s_channel_pins[LOGIC_CHANNEL_COUNT] = {
    BSP_PIN_PORT_C_RXD2, /* CH0: GPIO13 (UART RX / I2C SDA / SPI CS / PWM) */
    BSP_PIN_PORT_C_TXD2, /* CH1: GPIO14 (I2C SCL / SPI SCK) */
    BSP_PIN_PORT_A_SDA,  /* CH2: GPIO32 (SPI MOSI) */
    BSP_PIN_PORT_A_SCL,  /* CH3: GPIO33 (SPI MISO) */
};
static logic_capture_mode_t s_cap_mode = LOGIC_CAP_MODE_SAMPLED;
static logic_sample_rate_t s_sample_rate = LOGIC_RATE_100KHZ;
static logic_protocol_t s_protocol = LOGIC_PROTO_UART;
static uint32_t s_uart_baud = 9600U;
static logic_trig_mode_t s_trig_mode = LOGIC_TRIG_AUTO;
static logic_zoom_t s_zoom = LOGIC_ZOOM_1X;
static uint16_t s_pan_offset = 0U;
static bool s_running = true;
static bool s_tool_active = false;
static volatile bool s_stop_requested = false;
static volatile bool s_step_preempt = false;
static volatile uint32_t s_capture_seq = 0;

/* Internal SRAM ISR bounce buffers & state (never in PSRAM) */
static DRAM_ATTR uint8_t s_isr_bounce_buf[LOGIC_ISR_BOUNCE_SAMPLES];
static DRAM_ATTR volatile size_t s_isr_target_samples = LOGIC_DEFAULT_CAPTURE_SAMPLES;
static DRAM_ATTR volatile size_t s_isr_write_idx = 0;
static DRAM_ATTR volatile size_t s_isr_read_idx = 0;
static DRAM_ATTR volatile uint32_t s_isr_overrun_count = 0;
static DRAM_ATTR volatile uint32_t s_isr_alarm_count = 0;
static DRAM_ATTR volatile uint64_t s_isr_interval_us = 10ULL;
static DRAM_ATTR gptimer_alarm_config_t s_isr_alarm_cfg = {
    .reload_count = 0ULL,
    .alarm_count = 10ULL,
    .flags.auto_reload_on_alarm = true,
};
static DRAM_ATTR volatile int64_t s_isr_first_sample_us = 0;
static DRAM_ATTR volatile int64_t s_isr_last_sample_us = 0;
static DRAM_ATTR volatile bool s_isr_capture_done = false;
static DRAM_ATTR gpio_num_t s_isr_pins[LOGIC_CHANNEL_COUNT] = {
    BSP_PIN_PORT_C_RXD2,
    BSP_PIN_PORT_C_TXD2,
    BSP_PIN_PORT_A_SDA,
    BSP_PIN_PORT_A_SCL,
};
static DRAM_ATTR logic_isr_sample_fn_t s_test_isr_sample_fn = NULL;
static DRAM_ATTR void *s_test_isr_sample_ctx = NULL;
static DRAM_ATTR volatile bool s_force_gptimer_timeout = false;

static DRAM_ATTR rmt_symbol_word_t s_rmt_bounce_buf[LOGIC_RMT_BOUNCE_SYMBOLS];
static DRAM_ATTR volatile size_t s_rmt_isr_symbol_count = 0;
static DRAM_ATTR volatile bool s_rmt_isr_done = false;

/* Internal SRAM GPIO edge bounce buffer for continuous signals in RMT_EDGE mode */
static DRAM_ATTR logic_edge_sample_t s_edge_bounce_buf[LOGIC_EDGE_BOUNCE_MAX];
static DRAM_ATTR volatile size_t s_edge_bounce_count = 0;
static DRAM_ATTR volatile uint32_t s_edge_isr_invocations = 0;
static DRAM_ATTR volatile bool s_edge_isr_capped = false;
static DRAM_ATTR volatile uint8_t s_edge_last_level = 0xFFU;
static DRAM_ATTR volatile uint32_t s_edge_first_us = 0;
static DRAM_ATTR volatile uint32_t s_edge_last_us = 0;
static DRAM_ATTR volatile gpio_num_t s_edge_isr_pin = GPIO_NUM_NC;

/* Hardware peripheral handles & diagnostics */
static gptimer_handle_t s_gptimer = NULL;
static bool s_gptimer_enabled = false;
static rmt_channel_handle_t s_rmt_chan = NULL;
static bool s_rmt_enabled = false;
static gpio_num_t s_rmt_configured_pin = GPIO_NUM_NC;
static gpio_num_t s_edge_registered_pin = GPIO_NUM_NC;
static uint32_t s_edge_timeout_us = LOGIC_EDGE_DEFAULT_TIMEOUT_US;
static logic_window_stats_t s_last_win_stats = {0};
static volatile uint32_t s_worker_stack_hwm = 0U;

/* 64 KB PSRAM capture buffer & frame double buffer */
static uint8_t *s_psram_capture_buf = NULL;
static rmt_symbol_word_t *s_psram_rmt_buf = NULL;
static psram_double_buf_t s_frame_dbuf = {0};
static logic_frame_t s_latest_frame = {0};
static tool_worker_t *s_worker = NULL;

static logic_capture_source_fn_t s_custom_source_fn = NULL;
static void *s_custom_source_ctx = NULL;

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_top_status = NULL;
static lv_obj_t *s_lbl_zoom_badge = NULL;
static lv_obj_t *s_lbl_ch_pins[LOGIC_CHANNEL_COUNT] = {NULL};
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser_ch[LOGIC_CHANNEL_COUNT] = {NULL};
static lv_obj_t *s_lbl_decode_summary = NULL;
static lv_obj_t *s_lbl_decode_rows = NULL;
static lv_obj_t *s_btn_run_lbl = NULL;
static lv_obj_t *s_btn_proto_lbl = NULL;
static lv_obj_t *s_btn_zoom_lbl = NULL;
static lv_obj_t *s_modal_cfg = NULL;
static lv_obj_t *s_modal_mode_lbl = NULL;
static lv_obj_t *s_modal_rate_lbl = NULL;
static lv_obj_t *s_modal_baud_lbl = NULL;
static lv_obj_t *s_modal_ch_lbl[LOGIC_CHANNEL_COUNT] = {NULL};

static void ensure_logic_step_mutex(void)
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

bool tool_logic_is_pin_supported(gpio_num_t pin)
{
    for (size_t i = 0; i < LOGIC_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == pin) {
            return true;
        }
    }
    return false;
}

esp_err_t tool_logic_set_channel_pin(uint8_t channel, gpio_num_t pin)
{
    if (channel >= LOGIC_CHANNEL_COUNT || !tool_logic_is_pin_supported(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_logic_lock);
    gpio_num_t prev_pin = s_channel_pins[channel];
    bool active = s_tool_active;
    bool duplicate = false;
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        if (c != channel && s_channel_pins[c] == pin) {
            duplicate = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_logic_lock);

    if (pin == prev_pin) {
        return ESP_OK;
    }
    if (duplicate) {
        return ESP_ERR_INVALID_ARG;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim(pin, LOGIC_TOOL_ID);
        if (err != ESP_OK) {
            return err;
        }
        (void)bsp_pin_release(prev_pin, LOGIC_TOOL_ID);
    }

    portENTER_CRITICAL(&s_logic_lock);
    s_channel_pins[channel] = pin;
    s_isr_pins[channel] = pin;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

gpio_num_t tool_logic_get_channel_pin(uint8_t channel)
{
    if (channel >= LOGIC_CHANNEL_COUNT) {
        return GPIO_NUM_NC;
    }
    portENTER_CRITICAL(&s_logic_lock);
    gpio_num_t pin = s_channel_pins[channel];
    portEXIT_CRITICAL(&s_logic_lock);
    return pin;
}

esp_err_t tool_logic_set_capture_mode(logic_capture_mode_t mode)
{
    if (mode < 0 || mode >= LOGIC_CAP_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    if (s_cap_mode != mode) {
        s_cap_mode = mode;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

logic_capture_mode_t tool_logic_get_capture_mode(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    logic_capture_mode_t mode = s_cap_mode;
    portEXIT_CRITICAL(&s_logic_lock);
    return mode;
}

esp_err_t tool_logic_set_sample_rate(logic_sample_rate_t rate)
{
    if (rate < 0 || rate >= LOGIC_RATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    if (s_sample_rate != rate) {
        s_sample_rate = rate;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

logic_sample_rate_t tool_logic_get_sample_rate(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    logic_sample_rate_t rate = s_sample_rate;
    portEXIT_CRITICAL(&s_logic_lock);
    return rate;
}

esp_err_t tool_logic_set_protocol(logic_protocol_t proto)
{
    if (proto < 0 || proto >= LOGIC_PROTO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    s_protocol = proto;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

logic_protocol_t tool_logic_get_protocol(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    logic_protocol_t proto = s_protocol;
    portEXIT_CRITICAL(&s_logic_lock);
    return proto;
}

esp_err_t tool_logic_set_uart_baud(uint32_t baud_rate)
{
    if (baud_rate < 300U || baud_rate > 2000000U) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    s_uart_baud = baud_rate;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

uint32_t tool_logic_get_uart_baud(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    uint32_t baud = s_uart_baud;
    portEXIT_CRITICAL(&s_logic_lock);
    return baud;
}

esp_err_t tool_logic_set_zoom(logic_zoom_t zoom)
{
    if (zoom < 0 || zoom >= LOGIC_ZOOM_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    s_zoom = zoom;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

logic_zoom_t tool_logic_get_zoom(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    logic_zoom_t z = s_zoom;
    portEXIT_CRITICAL(&s_logic_lock);
    return z;
}

esp_err_t tool_logic_set_pan_offset(uint16_t offset_samples)
{
    if (offset_samples >= LOGIC_DEFAULT_CAPTURE_SAMPLES) {
        offset_samples = (uint16_t)(LOGIC_DEFAULT_CAPTURE_SAMPLES - 1U);
    }
    portENTER_CRITICAL(&s_logic_lock);
    s_pan_offset = offset_samples;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

uint16_t tool_logic_get_pan_offset(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    uint16_t off = s_pan_offset;
    portEXIT_CRITICAL(&s_logic_lock);
    return off;
}

esp_err_t tool_logic_set_running(bool running)
{
    portENTER_CRITICAL(&s_logic_lock);
    if (s_running != running) {
        s_running = running;
        s_capture_seq++;
    }
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

bool tool_logic_is_running(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    bool r = s_running;
    portEXIT_CRITICAL(&s_logic_lock);
    return r;
}

esp_err_t tool_logic_get_latest_frame(logic_frame_t *out_frame)
{
    if (out_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    *out_frame = s_latest_frame;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

void tool_logic_set_capture_source(logic_capture_source_fn_t fn, void *user_ctx)
{
    portENTER_CRITICAL(&s_logic_lock);
    s_custom_source_fn = fn;
    s_custom_source_ctx = user_ctx;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_logic_lock);
}

esp_err_t tool_logic_set_edge_timeout_us(uint32_t timeout_us)
{
    if (timeout_us < LOGIC_EDGE_MIN_TIMEOUT_US || timeout_us > LOGIC_EDGE_MAX_TIMEOUT_US) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_logic_lock);
    s_edge_timeout_us = timeout_us;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_logic_lock);
    return ESP_OK;
}

uint32_t tool_logic_get_edge_timeout_us(void)
{
    portENTER_CRITICAL(&s_logic_lock);
    uint32_t t = s_edge_timeout_us;
    portEXIT_CRITICAL(&s_logic_lock);
    return t;
}

void tool_logic_set_isr_sample_cb_for_test(logic_isr_sample_fn_t fn, void *user_ctx)
{
    portENTER_CRITICAL(&s_logic_lock);
    s_test_isr_sample_fn = fn;
    s_test_isr_sample_ctx = user_ctx;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_logic_lock);
}

void tool_logic_force_gptimer_timeout_for_test(bool force_timeout)
{
    portENTER_CRITICAL(&s_logic_lock);
    s_force_gptimer_timeout = force_timeout;
    s_capture_seq++;
    portEXIT_CRITICAL(&s_logic_lock);
}

void tool_logic_get_last_window_stats(logic_window_stats_t *out_stats)
{
    if (out_stats == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_logic_lock);
    *out_stats = s_last_win_stats;
    portEXIT_CRITICAL(&s_logic_lock);
}

uint32_t tool_logic_get_worker_stack_hwm(void)
{
    return s_worker_stack_hwm;
}

/* --------------------------------------------------------------------------
 * Fast GPIO Register Packing & Hardware ISR Callbacks (Internal SRAM Only)
 * -------------------------------------------------------------------------- */

static inline uint8_t IRAM_ATTR pack_gpio_sample_fast(const gpio_num_t pins[LOGIC_CHANNEL_COUNT])
{
    uint32_t in_lo = GPIO.in;
    uint32_t in_hi = GPIO.in1.val;
    uint8_t packed = 0U;

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        uint32_t p = (uint32_t)pins[c];
        uint32_t bit = (p < 32U) ? ((in_lo >> p) & 1U) : ((in_hi >> (p - 32U)) & 1U);
        packed |= (uint8_t)(bit << c);
    }
    return packed;
}

static bool IRAM_ATTR on_logic_gptimer_alarm_isr(gptimer_handle_t timer,
                                                 const gptimer_alarm_event_data_t *edata,
                                                 void *user_ctx)
{
    (void)user_ctx;
    size_t idx = s_isr_write_idx;
    size_t target = s_isr_target_samples;
    if (idx < target) {
        s_isr_alarm_count++;
        int64_t now_us = esp_timer_get_time();
        if (idx == 0U) {
            s_isr_first_sample_us = now_us;
        }
        logic_isr_sample_fn_t sample_fn = s_test_isr_sample_fn;
        uint8_t sample = (sample_fn != NULL)
                             ? sample_fn(idx, now_us, s_test_isr_sample_ctx)
                             : pack_gpio_sample_fast((const gpio_num_t *)s_isr_pins);
        if ((idx - s_isr_read_idx) < LOGIC_ISR_BOUNCE_SAMPLES) {
            s_isr_bounce_buf[idx & (LOGIC_ISR_BOUNCE_SAMPLES - 1U)] = sample;
        } else {
            s_isr_overrun_count++;
        }
        idx++;
        s_isr_write_idx = idx;
        if (idx >= target) {
            s_isr_last_sample_us = now_us;
            s_isr_capture_done = true;
            (void)gptimer_stop(timer);
        } else {
            /* With `auto_reload_on_alarm = true`, `gptimer_default_isr` has already reloaded
             * the counter to 0 and re-enabled the alarm at `s_isr_interval_us` with zero
             * cumulative drift whenever `cur_cnt < s_isr_interval_us`. Only if ISR entry/host
             * latency already exceeded `s_isr_interval_us` (where QEMU's `esp32_timg` deletes
             * the timer on `alarm_val - value <= 0`) do we reset the counter and re-arm. */
            uint64_t cur_cnt = 0ULL;
            (void)gptimer_get_raw_count(timer, &cur_cnt);
            if ((edata != NULL && edata->count_value >= s_isr_interval_us) ||
                cur_cnt >= s_isr_interval_us) {
                (void)gptimer_set_raw_count(timer, 0ULL);
            }
        }
    } else {
        s_isr_capture_done = true;
        (void)gptimer_stop(timer);
    }
    return false;
}

static inline void IRAM_ATTR record_logic_edge_iram(uint32_t now_us, uint8_t level)
{
    uint32_t inv = s_edge_isr_invocations + 1U;
    s_edge_isr_invocations = inv;

    if (s_edge_isr_capped || inv > LOGIC_EDGE_ISR_MAX_INVOCATIONS) {
        s_edge_isr_capped = true;
        gpio_num_t pin = s_edge_isr_pin;
        if (pin >= 0) {
            (void)gpio_intr_disable(pin);
        }
        return;
    }

    uint8_t lvl = level & 1U;
    size_t idx = s_edge_bounce_count;
    if (idx > 0U && lvl == s_edge_last_level) {
        if (inv >= LOGIC_EDGE_ISR_MAX_INVOCATIONS) {
            s_edge_isr_capped = true;
            gpio_num_t pin = s_edge_isr_pin;
            if (pin >= 0) {
                (void)gpio_intr_disable(pin);
            }
        }
        return;
    }

    if (idx < LOGIC_EDGE_BOUNCE_MAX) {
        s_edge_bounce_buf[idx].timestamp_us = now_us;
        s_edge_bounce_buf[idx].level = lvl;
        s_edge_last_level = lvl;
        if (idx == 0U) {
            s_edge_first_us = now_us;
        }
        s_edge_last_us = now_us;
        idx++;
        s_edge_bounce_count = idx;
    }

    if (idx >= LOGIC_EDGE_BOUNCE_MAX || inv >= LOGIC_EDGE_ISR_MAX_INVOCATIONS) {
        s_edge_isr_capped = true;
        gpio_num_t pin = s_edge_isr_pin;
        if (pin >= 0) {
            (void)gpio_intr_disable(pin);
        }
    }
}

static void IRAM_ATTR on_logic_gpio_edge_isr(void *arg)
{
    (void)arg;
    gpio_num_t pin = s_edge_isr_pin;
    if (pin < 0) {
        return;
    }
    uint32_t p = (uint32_t)pin;
    uint8_t lvl = (p < 32U) ? (uint8_t)((GPIO.in >> p) & 1U)
                            : (uint8_t)((GPIO.in1.val >> (p - 32U)) & 1U);
    record_logic_edge_iram((uint32_t)esp_timer_get_time(), lvl);
}

void tool_logic_record_edge_isr_for_test(uint32_t timestamp_us, uint8_t level)
{
    record_logic_edge_iram(timestamp_us, level);
}

static bool IRAM_ATTR on_logic_rmt_done_isr(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_data)
{
    (void)channel;
    (void)user_data;
    if (edata != NULL) {
        size_t n = edata->num_symbols;
        if (n > LOGIC_RMT_BOUNCE_SYMBOLS) {
            n = LOGIC_RMT_BOUNCE_SYMBOLS;
        }
        s_rmt_isr_symbol_count = n;
    } else {
        s_rmt_isr_symbol_count = 0U;
    }
    s_rmt_isr_done = true;
    return false;
}

/* --------------------------------------------------------------------------
 * Hardware Peripheral Setup & Teardown (GPTimer + RMT RX + GPIO Edge ISR)
 * -------------------------------------------------------------------------- */

static void teardown_logic_peripherals(void)
{
    if (s_edge_registered_pin != GPIO_NUM_NC) {
        (void)gpio_intr_disable(s_edge_registered_pin);
        (void)gpio_set_intr_type(s_edge_registered_pin, GPIO_INTR_DISABLE);
        (void)gpio_isr_handler_remove(s_edge_registered_pin);
        s_edge_registered_pin = GPIO_NUM_NC;
    }
    s_edge_isr_pin = GPIO_NUM_NC;

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
    s_rmt_configured_pin = GPIO_NUM_NC;
}

static esp_err_t ensure_logic_gptimer(void)
{
    if (s_gptimer != NULL && s_gptimer_enabled) {
        return ESP_OK;
    }
    if (s_gptimer != NULL) {
        (void)gptimer_del_timer(s_gptimer);
        s_gptimer = NULL;
    }

    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000U,
    };
    esp_err_t err = gptimer_new_timer(&tcfg, &s_gptimer);
    if (err != ESP_OK || s_gptimer == NULL) {
        s_gptimer = NULL;
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_logic_gptimer_alarm_isr,
    };
    err = gptimer_register_event_callbacks(s_gptimer, &cbs, NULL);
    if (err != ESP_OK) {
        (void)gptimer_del_timer(s_gptimer);
        s_gptimer = NULL;
        return err;
    }

    err = gptimer_enable(s_gptimer);
    if (err != ESP_OK) {
        (void)gptimer_del_timer(s_gptimer);
        s_gptimer = NULL;
        return err;
    }

    s_gptimer_enabled = true;
    return ESP_OK;
}

static esp_err_t ensure_logic_rmt(gpio_num_t ch0_pin)
{
    if (s_rmt_chan != NULL && s_rmt_enabled && s_rmt_configured_pin == ch0_pin) {
        return ESP_OK;
    }
    if (s_rmt_chan != NULL) {
        if (s_rmt_enabled) {
            (void)rmt_disable(s_rmt_chan);
            s_rmt_enabled = false;
        }
        (void)rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
        s_rmt_configured_pin = GPIO_NUM_NC;
    }

    rmt_rx_channel_config_t rcfg = {
        .gpio_num = ch0_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LOGIC_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64U,
    };
    esp_err_t err = rmt_new_rx_channel(&rcfg, &s_rmt_chan);
    if (err != ESP_OK || s_rmt_chan == NULL) {
        s_rmt_chan = NULL;
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = on_logic_rmt_done_isr,
    };
    err = rmt_rx_register_event_callbacks(s_rmt_chan, &cbs, NULL);
    if (err != ESP_OK) {
        (void)rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
        return err;
    }

    err = rmt_enable(s_rmt_chan);
    if (err != ESP_OK) {
        (void)rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
        return err;
    }

    s_rmt_enabled = true;
    s_rmt_configured_pin = ch0_pin;
    return ESP_OK;
}

static esp_err_t ensure_logic_edge_isr(gpio_num_t ch0_pin)
{
    esp_err_t isr_srv_err = gpio_install_isr_service(0);
    if (isr_srv_err != ESP_OK && isr_srv_err != ESP_ERR_INVALID_STATE) {
        return isr_srv_err;
    }

    if (s_edge_registered_pin != ch0_pin) {
        if (s_edge_registered_pin != GPIO_NUM_NC) {
            (void)gpio_intr_disable(s_edge_registered_pin);
            (void)gpio_set_intr_type(s_edge_registered_pin, GPIO_INTR_DISABLE);
            (void)gpio_isr_handler_remove(s_edge_registered_pin);
            s_edge_registered_pin = GPIO_NUM_NC;
        }
        esp_err_t err = gpio_set_intr_type(ch0_pin, GPIO_INTR_ANYEDGE);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_isr_handler_add(ch0_pin, on_logic_gpio_edge_isr, NULL);
        if (err != ESP_OK) {
            return err;
        }
        s_edge_registered_pin = ch0_pin;
    }
    return ESP_OK;
}

static bool should_abort_capture(uint32_t start_seq, bool is_bg_worker)
{
    portENTER_CRITICAL(&s_logic_lock);
    bool abort = s_stop_requested || !s_running || (s_capture_seq != start_seq) ||
                 (is_bg_worker && s_step_preempt);
    portEXIT_CRITICAL(&s_logic_lock);
    return abort;
}

static uint32_t count_buffer_transitions(const uint8_t *samples, size_t count)
{
    if (samples == NULL || count < 2U) {
        return 0U;
    }
    uint32_t edges = 0U;
    for (size_t i = 1U; i < count; ++i) {
        if ((samples[i] & 0x0FU) != (samples[i - 1U] & 0x0FU)) {
            edges++;
        }
    }
    return edges;
}

static esp_err_t capture_logic_buffer(logic_capture_mode_t cap_mode,
                                      const gpio_num_t pins[LOGIC_CHANNEL_COUNT],
                                      logic_sample_rate_t rate_enum,
                                      uint32_t requested_rate_hz,
                                      uint32_t start_seq,
                                      bool is_bg_worker,
                                      uint8_t *out_psram_samples,
                                      size_t target_samples,
                                      size_t *out_captured_count,
                                      uint32_t *out_actual_rate_hz,
                                      uint32_t *out_effective_rate_hz,
                                      uint32_t *out_elapsed_us,
                                      uint32_t *out_edge_count,
                                      bool *out_rate_capped,
                                      bool *out_no_capture,
                                      uint64_t *out_timestamp_us,
                                      bool *out_aborted)
{
    *out_aborted = false;
    *out_captured_count = 0U;
    *out_actual_rate_hz = 0U;
    *out_elapsed_us = 0U;
    *out_edge_count = 0U;
    *out_no_capture = false;
    *out_timestamp_us = (uint64_t)esp_timer_get_time();

    uint32_t effective_rate_hz = logic_effective_sample_rate_hz(cap_mode, rate_enum);
    if (effective_rate_hz == 0U) {
        effective_rate_hz = LOGIC_SAMPLED_MAX_RATE_HZ;
    }
    *out_effective_rate_hz = effective_rate_hz;
    *out_rate_capped = (requested_rate_hz > effective_rate_hz);

    logic_capture_source_fn_t custom_fn = NULL;
    void *custom_ctx = NULL;
    uint32_t edge_timeout_us = LOGIC_EDGE_DEFAULT_TIMEOUT_US;
    bool force_timeout = false;
    portENTER_CRITICAL(&s_logic_lock);
    custom_fn = s_custom_source_fn;
    custom_ctx = s_custom_source_ctx;
    edge_timeout_us = s_edge_timeout_us;
    force_timeout = s_force_gptimer_timeout;
    portEXIT_CRITICAL(&s_logic_lock);

    if (custom_fn != NULL) {
        esp_err_t cerr = custom_fn(cap_mode,
                                   pins,
                                   requested_rate_hz,
                                   out_psram_samples,
                                   target_samples,
                                   out_captured_count,
                                   out_actual_rate_hz,
                                   custom_ctx);
        if (cerr == ESP_OK) {
            *out_edge_count = count_buffer_transitions(out_psram_samples, *out_captured_count);
            if (cap_mode == LOGIC_CAP_MODE_RMT_EDGE && *out_captured_count == 0U) {
                *out_no_capture = true;
                *out_actual_rate_hz = 0U;
            }
        }
        return cerr;
    }

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        (void)gpio_set_direction(pins[c], GPIO_MODE_INPUT);
    }

    if (cap_mode == LOGIC_CAP_MODE_RMT_EDGE) {
        esp_err_t rmt_err = ensure_logic_rmt(pins[0]);
        if (rmt_err != ESP_OK) {
            return rmt_err;
        }
        esp_err_t edge_err = ensure_logic_edge_isr(pins[0]);
        if (edge_err != ESP_OK) {
            return edge_err;
        }

        s_rmt_isr_symbol_count = 0U;
        s_rmt_isr_done = false;
        s_edge_bounce_count = 0U;
        s_edge_isr_invocations = 0U;
        s_edge_isr_capped = false;
        s_edge_last_level = 0xFFU;
        s_edge_first_us = 0U;
        s_edge_last_us = 0U;
        s_edge_isr_pin = pins[0];

        int64_t edge_start_us = esp_timer_get_time();
        *out_timestamp_us = (uint64_t)edge_start_us;
        (void)gpio_intr_enable(pins[0]);

        rmt_receive_config_t rx_cfg = {
            .signal_range_min_ns = 500U,
            .signal_range_max_ns = 2500000U, /* 2.5 ms idle threshold for burst frames */
        };
        (void)rmt_receive(s_rmt_chan, s_rmt_bounce_buf, sizeof(s_rmt_bounce_buf), &rx_cfg);

        uint64_t target_span_us = ((uint64_t)target_samples * 1000000ULL) / (uint64_t)effective_rate_hz;
        int64_t rmt_deadline_us = edge_start_us + (int64_t)edge_timeout_us;

        while (esp_timer_get_time() < rmt_deadline_us) {
            if (s_rmt_isr_done && s_rmt_isr_symbol_count > 0U) {
                break;
            }
            if (s_edge_isr_capped) {
                break;
            }
            size_t e_cnt = s_edge_bounce_count;
            if (e_cnt >= 4U && (uint64_t)(s_edge_last_us - s_edge_first_us) >= target_span_us) {
                break;
            }
            if (should_abort_capture(start_seq, is_bg_worker)) {
                (void)gpio_intr_disable(pins[0]);
                if (s_rmt_chan != NULL && s_rmt_enabled) {
                    (void)rmt_disable(s_rmt_chan);
                    (void)rmt_enable(s_rmt_chan);
                }
                *out_aborted = true;
                return ESP_ERR_INVALID_STATE;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        (void)gpio_intr_disable(pins[0]);
        if (s_rmt_chan != NULL && s_rmt_enabled && !s_rmt_isr_done) {
            (void)rmt_disable(s_rmt_chan);
            (void)rmt_enable(s_rmt_chan);
        }

        uint32_t elapsed_us = (uint32_t)(esp_timer_get_time() - edge_start_us);
        *out_elapsed_us = elapsed_us;

        size_t rmt_cnt = s_rmt_isr_symbol_count;
        size_t edge_cnt = s_edge_bounce_count;
        size_t sym_cnt = 0U;
        size_t expanded = 0U;
        memset(out_psram_samples, 0, target_samples);

        if (rmt_cnt > 0U && s_psram_rmt_buf != NULL) {
            memcpy(s_psram_rmt_buf,
                   (const void *)s_rmt_bounce_buf,
                   rmt_cnt * sizeof(rmt_symbol_word_t));
            sym_cnt = rmt_cnt;
            (void)logic_expand_rmt_to_samples(s_psram_rmt_buf,
                                              sym_cnt,
                                              LOGIC_RMT_RESOLUTION_HZ,
                                              effective_rate_hz,
                                              0U,
                                              out_psram_samples,
                                              target_samples,
                                              &expanded);
        } else if (edge_cnt >= 2U && s_psram_rmt_buf != NULL) {
            /* Continuous signal fallback: convert DRAM edge timestamps to RMT symbols in PSRAM */
            (void)logic_edge_timestamps_to_rmt_symbols((const logic_edge_sample_t *)s_edge_bounce_buf,
                                                       edge_cnt,
                                                       s_psram_rmt_buf,
                                                       LOGIC_RMT_BOUNCE_SYMBOLS,
                                                       &sym_cnt);
            if (sym_cnt > 0U) {
                (void)logic_expand_rmt_to_samples(s_psram_rmt_buf,
                                                  sym_cnt,
                                                  LOGIC_RMT_RESOLUTION_HZ,
                                                  effective_rate_hz,
                                                  0U,
                                                  out_psram_samples,
                                                  target_samples,
                                                  &expanded);
            }
        }

        portENTER_CRITICAL(&s_logic_lock);
        s_last_win_stats.gptimer_isr_count = 0U;
        s_last_win_stats.bounce_overruns = 0U;
        s_last_win_stats.edge_isr_invocations = s_edge_isr_invocations;
        s_last_win_stats.edge_stored_count = (uint32_t)edge_cnt;
        s_last_win_stats.edge_isr_capped = s_edge_isr_capped;
        s_last_win_stats.rmt_symbol_count = (uint32_t)sym_cnt;
        s_last_win_stats.capture_elapsed_us = elapsed_us;
        portEXIT_CRITICAL(&s_logic_lock);

        if (expanded > 0U) {
            *out_captured_count = expanded;
            *out_actual_rate_hz = effective_rate_hz;
            *out_edge_count = (edge_cnt > 0U) ? (uint32_t)edge_cnt
                                              : count_buffer_transitions(out_psram_samples, expanded);
            *out_no_capture = false;
        } else {
            /* Distinct no-capture state: do NOT fabricate flat-line samples */
            *out_captured_count = 0U;
            *out_actual_rate_hz = 0U;
            *out_edge_count = (uint32_t)edge_cnt;
            *out_no_capture = true;
        }
        return ESP_OK;
    }

    /* LOGIC_CAP_MODE_SAMPLED:
     * Genuine hardware GPTimer auto-reload ISR sampling into internal SRAM ring bounce buffer
     * (`s_isr_bounce_buf`) at sustainable rates (<= 100 kHz / >= 10 us period), drained by the
     * Core 1 worker into the 64 KB PSRAM capture buffer (`out_psram_samples`). */
    if (s_edge_registered_pin != GPIO_NUM_NC) {
        (void)gpio_intr_disable(s_edge_registered_pin);
    }

    esp_err_t err = ensure_logic_gptimer();
    if (err != ESP_OK) {
        return err;
    }

    uint32_t interval_us = 1000000U / effective_rate_hz;
    if (interval_us < (1000000U / LOGIC_SAMPLED_MAX_RATE_HZ)) {
        interval_us = (1000000U / LOGIC_SAMPLED_MAX_RATE_HZ);
    }

    (void)gptimer_stop(s_gptimer);
    s_isr_target_samples = target_samples;
    s_isr_write_idx = 0U;
    s_isr_read_idx = 0U;
    s_isr_overrun_count = 0U;
    s_isr_alarm_count = 0U;
    s_isr_interval_us = (uint64_t)interval_us;
    s_isr_alarm_cfg.reload_count = 0ULL;
    s_isr_alarm_cfg.alarm_count = (uint64_t)interval_us;
    s_isr_alarm_cfg.flags.auto_reload_on_alarm = true;
    s_isr_first_sample_us = 0;
    s_isr_last_sample_us = 0;
    s_isr_capture_done = false;
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        s_isr_pins[c] = pins[c];
    }

    err = gptimer_set_raw_count(s_gptimer, 0ULL);
    if (err == ESP_OK) {
        err = gptimer_set_alarm_action(s_gptimer, &s_isr_alarm_cfg);
    }
    int64_t capture_start_us = esp_timer_get_time();
    *out_timestamp_us = (uint64_t)capture_start_us;
    if (err == ESP_OK && !force_timeout) {
        err = gptimer_start(s_gptimer);
    }
    if (err != ESP_OK) {
        (void)gptimer_stop(s_gptimer);
        return err;
    }

    int64_t expected_us = (int64_t)target_samples * (int64_t)interval_us;
    int64_t timeout_us = force_timeout ? 20000 : (expected_us * 4 + 100000);
    int64_t deadline_us = capture_start_us + timeout_us;
    size_t read_idx = 0U;

    while (!s_isr_capture_done) {
        size_t write_snap = s_isr_write_idx;
        while (read_idx < write_snap && read_idx < target_samples) {
            out_psram_samples[read_idx] =
                s_isr_bounce_buf[read_idx & (LOGIC_ISR_BOUNCE_SAMPLES - 1U)];
            read_idx++;
        }
        s_isr_read_idx = read_idx;

        if (s_isr_capture_done) {
            break;
        }
        if (should_abort_capture(start_seq, is_bg_worker)) {
            (void)gptimer_stop(s_gptimer);
            *out_aborted = true;
            return ESP_ERR_INVALID_STATE;
        }
        if (esp_timer_get_time() >= deadline_us) {
            (void)gptimer_stop(s_gptimer);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    (void)gptimer_stop(s_gptimer);

    size_t final_write = s_isr_write_idx;
    if (final_write > target_samples) {
        final_write = target_samples;
    }
    while (read_idx < final_write) {
        out_psram_samples[read_idx] =
            s_isr_bounce_buf[read_idx & (LOGIC_ISR_BOUNCE_SAMPLES - 1U)];
        read_idx++;
    }
    s_isr_read_idx = read_idx;

    int64_t span_us = s_isr_last_sample_us - s_isr_first_sample_us;
    if (span_us <= 0) {
        span_us = esp_timer_get_time() - capture_start_us;
    }
    uint32_t elapsed_u32 = (span_us > 0) ? (uint32_t)span_us : 0U;
    *out_elapsed_us = elapsed_u32;

    if (span_us > 0 && final_write > 1U) {
        uint64_t num = (uint64_t)(final_write - 1U) * 1000000ULL;
        uint64_t measured_hz = (num + (uint64_t)span_us / 2ULL) / (uint64_t)span_us;
        *out_actual_rate_hz = (measured_hz > UINT32_MAX) ? UINT32_MAX : (uint32_t)measured_hz;
    } else {
        *out_actual_rate_hz = 0U;
    }
    *out_captured_count = final_write;
    *out_edge_count = count_buffer_transitions(out_psram_samples, final_write);

    portENTER_CRITICAL(&s_logic_lock);
    s_last_win_stats.gptimer_isr_count = s_isr_alarm_count;
    s_last_win_stats.bounce_overruns = s_isr_overrun_count;
    s_last_win_stats.edge_isr_invocations = 0U;
    s_last_win_stats.edge_stored_count = 0U;
    s_last_win_stats.edge_isr_capped = false;
    s_last_win_stats.rmt_symbol_count = 0U;
    s_last_win_stats.capture_elapsed_us = elapsed_u32;
    portEXIT_CRITICAL(&s_logic_lock);

    return ESP_OK;
}

static esp_err_t logic_worker_step_locked(bool is_bg_worker)
{
    if (is_bg_worker) {
        uint32_t hwm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
        if (s_worker_stack_hwm == 0U || hwm < s_worker_stack_hwm) {
            s_worker_stack_hwm = hwm;
        }
    }

    if (s_psram_capture_buf == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_num_t pins[LOGIC_CHANNEL_COUNT];
    portENTER_CRITICAL(&s_logic_lock);
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        pins[c] = s_channel_pins[c];
    }
    logic_capture_mode_t cap_mode = s_cap_mode;
    logic_sample_rate_t rate_enum = s_sample_rate;
    logic_protocol_t proto = s_protocol;
    uint32_t uart_baud = s_uart_baud;
    logic_trig_mode_t trig_mode = s_trig_mode;
    logic_zoom_t zoom = s_zoom;
    uint16_t pan_off = s_pan_offset;
    bool running = s_running;
    bool stop_req = s_stop_requested;
    uint32_t start_seq = s_capture_seq;
    portEXIT_CRITICAL(&s_logic_lock);

    if (stop_req) {
        return ESP_OK;
    }

    uint32_t req_rate_hz = logic_sample_rate_to_hz(rate_enum);
    if (req_rate_hz == 0U) {
        req_rate_hz = 100000U;
    }
    uint32_t eff_rate_hz = logic_effective_sample_rate_hz(cap_mode, rate_enum);
    if (eff_rate_hz == 0U) {
        eff_rate_hz = LOGIC_SAMPLED_MAX_RATE_HZ;
    }

    size_t captured = 0U;
    uint32_t actual_rate_hz = 0U;
    uint32_t elapsed_us = 0U;
    uint32_t edge_count = 0U;
    bool rate_capped = (req_rate_hz > eff_rate_hz);
    bool no_capture = false;
    esp_err_t hw_err = ESP_OK;
    uint64_t capture_ts_us = 0U;

    if (running) {
        bool aborted = false;
        hw_err = capture_logic_buffer(cap_mode,
                                      pins,
                                      rate_enum,
                                      req_rate_hz,
                                      start_seq,
                                      is_bg_worker,
                                      s_psram_capture_buf,
                                      LOGIC_DEFAULT_CAPTURE_SAMPLES,
                                      &captured,
                                      &actual_rate_hz,
                                      &eff_rate_hz,
                                      &elapsed_us,
                                      &edge_count,
                                      &rate_capped,
                                      &no_capture,
                                      &capture_ts_us,
                                      &aborted);
        if (aborted) {
            return ESP_OK;
        }
        if (hw_err == ESP_OK && !no_capture && captured > 0U && actual_rate_hz > 0U &&
            recorder_sink_is_recording()) {
            recorder_frame_t rec_frame = {
                .tool_id = LOGIC_TOOL_ID,
                .sample_rate_hz = actual_rate_hz,
                .timestamp_us = capture_ts_us,
                .samples = s_psram_capture_buf,
                .sample_count = captured,
                .bytes_per_sample = sizeof(uint8_t),
            };
            (void)recorder_sink_push(&rec_frame);
        }
    } else {
        portENTER_CRITICAL(&s_logic_lock);
        captured = s_latest_frame.total_captured;
        actual_rate_hz = s_latest_frame.sample_rate_hz;
        eff_rate_hz = s_latest_frame.effective_rate_hz;
        elapsed_us = s_latest_frame.capture_elapsed_us;
        edge_count = s_latest_frame.edge_count;
        rate_capped = s_latest_frame.rate_capped;
        no_capture = s_latest_frame.no_capture;
        hw_err = s_latest_frame.hw_error;
        portEXIT_CRITICAL(&s_logic_lock);
    }

    /* Re-read latest UI settings (zoom, pan, protocol, baud) before building frame */
    portENTER_CRITICAL(&s_logic_lock);
    proto = s_protocol;
    uart_baud = s_uart_baud;
    zoom = s_zoom;
    pan_off = s_pan_offset;
    portEXIT_CRITICAL(&s_logic_lock);

    logic_frame_t *wptr = (logic_frame_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        memset(wptr, 0, sizeof(*wptr));
        wptr->total_captured = captured;
        wptr->sample_rate_hz = actual_rate_hz;
        wptr->requested_rate_hz = req_rate_hz;
        wptr->effective_rate_hz = eff_rate_hz;
        wptr->capture_elapsed_us = elapsed_us;
        wptr->edge_count = edge_count;
        wptr->rate_capped = rate_capped;
        wptr->no_capture = no_capture;
        wptr->hw_error = hw_err;
        wptr->cap_mode = cap_mode;
        wptr->protocol = proto;
        for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
            wptr->channel_pins[c] = pins[c];
        }
        wptr->zoom = zoom;
        wptr->pan_offset = pan_off;
        wptr->decoded.protocol = proto;

        if (hw_err != ESP_OK) {
            (void)snprintf(wptr->decoded.summary,
                           sizeof(wptr->decoded.summary),
                           "HW Error: %s",
                           esp_err_to_name(hw_err));
        } else if (no_capture || captured == 0U) {
            wptr->no_capture = true;
            (void)snprintf(wptr->decoded.summary,
                           sizeof(wptr->decoded.summary),
                           "RMT EDGE: No transitions captured");
        } else {
            bool triggered = false;
            size_t trig_idx = 0U;
            (void)logic_find_digital_trigger(
                s_psram_capture_buf, captured, 0U, trig_mode, &triggered, &trig_idx);
            wptr->trigger_index = trig_idx;
            wptr->triggered = triggered;

            (void)logic_extract_viewport(
                s_psram_capture_buf, captured, zoom, pan_off, wptr->view_samples, LOGIC_VIEW_POINTS);
            (void)logic_decode_run_protocol(
                s_psram_capture_buf, captured, actual_rate_hz, proto, uart_baud, &wptr->decoded);
        }

        portENTER_CRITICAL(&s_logic_lock);
        s_latest_frame = *wptr;
        portEXIT_CRITICAL(&s_logic_lock);

        (void)psram_double_buf_swap(&s_frame_dbuf);
    }

    return hw_err;
}

static esp_err_t logic_worker_step(void *ctx)
{
    (void)ctx;
    ensure_logic_step_mutex();
    if (xSemaphoreTake(s_step_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_OK;
    }
    esp_err_t err = logic_worker_step_locked(true);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

esp_err_t tool_logic_run_single_step(void)
{
    ensure_logic_step_mutex();
    portENTER_CRITICAL(&s_logic_lock);
    s_step_preempt = true;
    portEXIT_CRITICAL(&s_logic_lock);

    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    portENTER_CRITICAL(&s_logic_lock);
    s_step_preempt = false;
    portEXIT_CRITICAL(&s_logic_lock);

    esp_err_t err = logic_worker_step_locked(false);
    (void)xSemaphoreGive(s_step_mutex);
    return err;
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Controls, Multi-Lane Tracks & Protocol/Pin Modal (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_run_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_logic_set_running(!tool_logic_is_running());
}

static void on_btn_proto_clicked(lv_event_t *e)
{
    (void)e;
    logic_protocol_t next = (logic_protocol_t)((tool_logic_get_protocol() + 1) % LOGIC_PROTO_COUNT);
    (void)tool_logic_set_protocol(next);
}

static void on_btn_zoom_clicked(lv_event_t *e)
{
    (void)e;
    logic_zoom_t next = (logic_zoom_t)((tool_logic_get_zoom() + 1) % LOGIC_ZOOM_COUNT);
    (void)tool_logic_set_zoom(next);
}

static void on_btn_pan_left_clicked(lv_event_t *e)
{
    (void)e;
    uint16_t cur = tool_logic_get_pan_offset();
    uint16_t next = (cur > LOGIC_PAN_STEP_SAMPLES) ? (uint16_t)(cur - LOGIC_PAN_STEP_SAMPLES) : 0U;
    (void)tool_logic_set_pan_offset(next);
}

static void on_btn_pan_right_clicked(lv_event_t *e)
{
    (void)e;
    uint16_t cur = tool_logic_get_pan_offset();
    uint16_t max_off = (uint16_t)(LOGIC_DEFAULT_CAPTURE_SAMPLES - LOGIC_VIEW_POINTS);
    uint16_t next = (uint16_t)(cur + LOGIC_PAN_STEP_SAMPLES);
    if (next > max_off) {
        next = max_off;
    }
    (void)tool_logic_set_pan_offset(next);
}

static void on_btn_cfg_clicked(lv_event_t *e)
{
    (void)e;
    if (s_modal_cfg != NULL) {
        bool hidden = lv_obj_has_flag(s_modal_cfg, LV_OBJ_FLAG_HIDDEN);
        if (hidden) {
            lv_obj_remove_flag(s_modal_cfg, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_modal_cfg, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void on_modal_mode_clicked(lv_event_t *e)
{
    (void)e;
    logic_capture_mode_t next = (logic_capture_mode_t)((tool_logic_get_capture_mode() + 1) % LOGIC_CAP_MODE_COUNT);
    (void)tool_logic_set_capture_mode(next);
}

static void on_modal_rate_clicked(lv_event_t *e)
{
    (void)e;
    logic_capture_mode_t mode = tool_logic_get_capture_mode();
    uint32_t max_rates = (mode == LOGIC_CAP_MODE_SAMPLED)
                             ? ((uint32_t)LOGIC_SAMPLED_MAX_RATE + 1U)
                             : (uint32_t)LOGIC_RATE_COUNT;
    logic_sample_rate_t next = (logic_sample_rate_t)(((uint32_t)tool_logic_get_sample_rate() + 1U) % max_rates);
    (void)tool_logic_set_sample_rate(next);
}

static void on_modal_baud_clicked(lv_event_t *e)
{
    (void)e;
    uint32_t cur = tool_logic_get_uart_baud();
    size_t idx = 0U;
    for (size_t i = 0; i < LOGIC_UART_BAUD_COUNT; ++i) {
        if (s_uart_baud_presets[i] == cur) {
            idx = i;
            break;
        }
    }
    uint32_t next = s_uart_baud_presets[(idx + 1U) % LOGIC_UART_BAUD_COUNT];
    (void)tool_logic_set_uart_baud(next);
}

static void on_modal_ch_pin_clicked(lv_event_t *e)
{
    uint8_t ch = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (ch >= LOGIC_CHANNEL_COUNT) {
        return;
    }
    gpio_num_t cur = tool_logic_get_channel_pin(ch);
    size_t idx = 0U;
    for (size_t i = 0; i < LOGIC_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == cur) {
            idx = i;
            break;
        }
    }
    for (size_t step = 1U; step <= LOGIC_SUPPORTED_PIN_COUNT; ++step) {
        gpio_num_t cand = s_supported_pins[(idx + step) % LOGIC_SUPPORTED_PIN_COUNT];
        if (tool_logic_set_channel_pin(ch, cand) == ESP_OK) {
            break;
        }
    }
}

static void build_logic_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 2, 0);
    lv_obj_set_style_pad_row(parent, 2, 0);

    /* 1. Top Status Bar (18 px) */
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
    lv_label_set_text(s_lbl_top_status, "SAMP 100kHz  UART 9600");
    lv_obj_set_style_text_font(s_lbl_top_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_top_status, lv_color_hex(0x4DD0E1), 0);

    s_lbl_zoom_badge = lv_label_create(top_row);
    lv_label_set_text(s_lbl_zoom_badge, "Z:1x +0");
    lv_obj_set_style_text_font(s_lbl_zoom_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_zoom_badge, lv_color_hex(0x00E676), 0);

    /* 2. Multi-Lane Digital Waveform Tracks Area (82 px) */
    lv_obj_t *wave_row = lv_obj_create(parent);
    lv_obj_set_size(wave_row, 314, 82);
    lv_obj_set_style_bg_opa(wave_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wave_row, 0, 0);
    lv_obj_set_style_pad_all(wave_row, 0, 0);
    lv_obj_set_style_pad_column(wave_row, 2, 0);
    lv_obj_set_flex_flow(wave_row, LV_FLEX_FLOW_ROW);

    lv_obj_t *pin_col = lv_obj_create(wave_row);
    lv_obj_set_size(pin_col, 52, 82);
    lv_obj_set_style_bg_color(pin_col, lv_color_hex(0x101820), 0);
    lv_obj_set_style_border_width(pin_col, 0, 0);
    lv_obj_set_style_pad_all(pin_col, 2, 0);
    lv_obj_set_flex_flow(pin_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(pin_col, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    static const uint32_t s_ch_colors[LOGIC_CHANNEL_COUNT] = {
        0x00E5FF, /* CH0: Cyan */
        0xFFD54F, /* CH1: Yellow */
        0x00E676, /* CH2: Green */
        0xFF8A65, /* CH3: Orange */
    };
    /* Display CH3 at top down to CH0 at bottom */
    for (int lane = (int)LOGIC_CHANNEL_COUNT - 1; lane >= 0; --lane) {
        s_lbl_ch_pins[lane] = lv_label_create(pin_col);
        lv_label_set_text_fmt(s_lbl_ch_pins[lane], "C%d:G%d", lane, (int)s_channel_pins[lane]);
        lv_obj_set_style_text_font(s_lbl_ch_pins[lane], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_lbl_ch_pins[lane], lv_color_hex(s_ch_colors[lane]), 0);
    }

    s_chart = lv_chart_create(wave_row);
    lv_obj_set_size(s_chart, 258, 82);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, LOGIC_VIEW_POINTS);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 400);
    lv_chart_set_div_line_count(s_chart, 3, 5);
    lv_obj_set_style_bg_color(s_chart, lv_color_hex(0x080D12), 0);
    lv_obj_set_style_border_color(s_chart, lv_color_hex(0x263238), 0);
    lv_obj_set_style_border_width(s_chart, 1, 0);
    lv_obj_set_style_radius(s_chart, 3, 0);
    lv_obj_set_style_pad_all(s_chart, 1, 0);
    lv_obj_set_style_size(s_chart, 0, 0, LV_PART_INDICATOR);

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        s_ser_ch[c] = lv_chart_add_series(
            s_chart, lv_color_hex(s_ch_colors[c]), LV_CHART_AXIS_PRIMARY_Y);
    }

    /* 3. Decoded Packet Table Panel (72 px) */
    lv_obj_t *dec_panel = lv_obj_create(parent);
    lv_obj_set_size(dec_panel, 314, 72);
    lv_obj_set_style_bg_color(dec_panel, lv_color_hex(0x0E161E), 0);
    lv_obj_set_style_border_color(dec_panel, lv_color_hex(0x37474F), 0);
    lv_obj_set_style_border_width(dec_panel, 1, 0);
    lv_obj_set_style_radius(dec_panel, 3, 0);
    lv_obj_set_style_pad_all(dec_panel, 3, 0);
    lv_obj_set_style_pad_row(dec_panel, 2, 0);
    lv_obj_set_flex_flow(dec_panel, LV_FLEX_FLOW_COLUMN);

    s_lbl_decode_summary = lv_label_create(dec_panel);
    lv_label_set_text(s_lbl_decode_summary, "UART 9600 8N1: 0B");
    lv_obj_set_style_text_font(s_lbl_decode_summary, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_decode_summary, lv_color_hex(0xFFE082), 0);

    s_lbl_decode_rows = lv_label_create(dec_panel);
    lv_label_set_text(s_lbl_decode_rows, "Waiting for protocol frames...");
    lv_obj_set_style_text_font(s_lbl_decode_rows, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_decode_rows, lv_color_hex(0xCFD8DC), 0);

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

    lv_obj_t *btn_proto = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_proto, 54, 26);
    lv_obj_set_style_bg_color(btn_proto, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_proto, on_btn_proto_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_proto_lbl = lv_label_create(btn_proto);
    lv_label_set_text(s_btn_proto_lbl, "UART");
    lv_obj_set_style_text_font(s_btn_proto_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_proto_lbl);

    lv_obj_t *btn_zoom = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_zoom, 48, 26);
    lv_obj_set_style_bg_color(btn_zoom, lv_color_hex(0x0277BD), 0);
    lv_obj_add_event_cb(btn_zoom, on_btn_zoom_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_zoom_lbl = lv_label_create(btn_zoom);
    lv_label_set_text(s_btn_zoom_lbl, "1x");
    lv_obj_set_style_text_font(s_btn_zoom_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_zoom_lbl);

    lv_obj_t *btn_pan_l = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pan_l, 42, 26);
    lv_obj_set_style_bg_color(btn_pan_l, lv_color_hex(0x455A64), 0);
    lv_obj_add_event_cb(btn_pan_l, on_btn_pan_left_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_pan_l = lv_label_create(btn_pan_l);
    lv_label_set_text(lbl_pan_l, "<");
    lv_obj_set_style_text_font(lbl_pan_l, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_pan_l);

    lv_obj_t *btn_pan_r = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pan_r, 42, 26);
    lv_obj_set_style_bg_color(btn_pan_r, lv_color_hex(0x455A64), 0);
    lv_obj_add_event_cb(btn_pan_r, on_btn_pan_right_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_pan_r = lv_label_create(btn_pan_r);
    lv_label_set_text(lbl_pan_r, ">");
    lv_obj_set_style_text_font(lbl_pan_r, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_pan_r);

    lv_obj_t *btn_cfg = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_cfg, 54, 26);
    lv_obj_set_style_bg_color(btn_cfg, lv_color_hex(0x6A1B9A), 0);
    lv_obj_add_event_cb(btn_cfg, on_btn_cfg_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_cfg = lv_label_create(btn_cfg);
    lv_label_set_text(lbl_cfg, "PINS");
    lv_obj_set_style_text_font(lbl_cfg, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_cfg);

    /* 5. Floating Protocol & Pin Mapping Modal (hidden by default) */
    s_modal_cfg = lv_obj_create(parent);
    lv_obj_add_flag(s_modal_cfg, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(s_modal_cfg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(s_modal_cfg, 296, 174);
    lv_obj_align(s_modal_cfg, LV_ALIGN_CENTER, 0, -8);
    lv_obj_set_style_bg_color(s_modal_cfg, lv_color_hex(0x16222E), 0);
    lv_obj_set_style_border_color(s_modal_cfg, lv_color_hex(0x4DD0E1), 0);
    lv_obj_set_style_border_width(s_modal_cfg, 2, 0);
    lv_obj_set_style_radius(s_modal_cfg, 6, 0);
    lv_obj_set_style_pad_all(s_modal_cfg, 6, 0);
    lv_obj_set_style_pad_row(s_modal_cfg, 4, 0);
    lv_obj_set_flex_flow(s_modal_cfg, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *m_title = lv_label_create(s_modal_cfg);
    lv_label_set_text(m_title, "Capture & Pin Mapping Config");
    lv_obj_set_style_text_font(m_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(m_title, lv_color_hex(0x4DD0E1), 0);

    lv_obj_t *m_row1 = lv_obj_create(s_modal_cfg);
    lv_obj_set_size(m_row1, 278, 30);
    lv_obj_set_style_bg_opa(m_row1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(m_row1, 0, 0);
    lv_obj_set_style_pad_all(m_row1, 0, 0);
    lv_obj_set_flex_flow(m_row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(m_row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *b_mode = lv_button_create(m_row1);
    lv_obj_set_size(b_mode, 88, 26);
    lv_obj_add_event_cb(b_mode, on_modal_mode_clicked, LV_EVENT_CLICKED, NULL);
    s_modal_mode_lbl = lv_label_create(b_mode);
    lv_label_set_text(s_modal_mode_lbl, "SAMPLED");
    lv_obj_set_style_text_font(s_modal_mode_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_modal_mode_lbl);

    lv_obj_t *b_rate = lv_button_create(m_row1);
    lv_obj_set_size(b_rate, 88, 26);
    lv_obj_add_event_cb(b_rate, on_modal_rate_clicked, LV_EVENT_CLICKED, NULL);
    s_modal_rate_lbl = lv_label_create(b_rate);
    lv_label_set_text(s_modal_rate_lbl, "100kHz");
    lv_obj_set_style_text_font(s_modal_rate_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_modal_rate_lbl);

    lv_obj_t *b_baud = lv_button_create(m_row1);
    lv_obj_set_size(b_baud, 88, 26);
    lv_obj_add_event_cb(b_baud, on_modal_baud_clicked, LV_EVENT_CLICKED, NULL);
    s_modal_baud_lbl = lv_label_create(b_baud);
    lv_label_set_text(s_modal_baud_lbl, "9600bd");
    lv_obj_set_style_text_font(s_modal_baud_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_modal_baud_lbl);

    lv_obj_t *m_row2 = lv_obj_create(s_modal_cfg);
    lv_obj_set_size(m_row2, 278, 30);
    lv_obj_set_style_bg_opa(m_row2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(m_row2, 0, 0);
    lv_obj_set_style_pad_all(m_row2, 0, 0);
    lv_obj_set_flex_flow(m_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(m_row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        lv_obj_t *b_ch = lv_button_create(m_row2);
        lv_obj_set_size(b_ch, 64, 26);
        lv_obj_add_event_cb(b_ch, on_modal_ch_pin_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)c);
        s_modal_ch_lbl[c] = lv_label_create(b_ch);
        lv_label_set_text_fmt(s_modal_ch_lbl[c], "C%u:G%d", (unsigned)c, (int)s_channel_pins[c]);
        lv_obj_set_style_text_font(s_modal_ch_lbl[c], &lv_font_montserrat_12, 0);
        lv_obj_center(s_modal_ch_lbl[c]);
    }

    lv_obj_t *b_close = lv_button_create(s_modal_cfg);
    lv_obj_set_size(b_close, 96, 26);
    lv_obj_set_style_bg_color(b_close, lv_color_hex(0x37474F), 0);
    lv_obj_add_event_cb(b_close, on_btn_cfg_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_close = lv_label_create(b_close);
    lv_label_set_text(lbl_close, "CLOSE");
    lv_obj_set_style_text_font(lbl_close, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_close);
}

static void logic_on_ui_refresh(void *ctx)
{
    (void)ctx;
    static logic_frame_t s_ui_frame;
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &s_ui_frame, 1, &copied) != ESP_OK || copied == 0) {
        return;
    }

    bool running = tool_logic_is_running();
    logic_capture_mode_t cap_mode = tool_logic_get_capture_mode();
    logic_sample_rate_t rate_enum = tool_logic_get_sample_rate();
    logic_protocol_t proto = tool_logic_get_protocol();
    uint32_t baud = tool_logic_get_uart_baud();
    logic_zoom_t zoom = tool_logic_get_zoom();
    uint16_t pan_off = tool_logic_get_pan_offset();

    if (s_lbl_top_status != NULL) {
        const char *mode_str = (cap_mode == LOGIC_CAP_MODE_RMT_EDGE) ? "RMT" : "SAMP";
        const char *hold_str = running ? "" : " [HOLD]";
        if (s_ui_frame.hw_error != ESP_OK) {
            lv_label_set_text_fmt(s_lbl_top_status,
                                  "%s %s %s [HW ERR]%s",
                                  mode_str,
                                  logic_sample_rate_to_str(rate_enum),
                                  logic_protocol_to_str(proto),
                                  hold_str);
        } else if (s_ui_frame.no_capture) {
            lv_label_set_text_fmt(s_lbl_top_status,
                                  "%s %s %s [NO CAP]%s",
                                  mode_str,
                                  logic_sample_rate_to_str(rate_enum),
                                  logic_protocol_to_str(proto),
                                  hold_str);
        } else if (s_ui_frame.rate_capped) {
            unsigned long act_khz = (unsigned long)((s_ui_frame.sample_rate_hz + 500U) / 1000U);
            lv_label_set_text_fmt(s_lbl_top_status,
                                  "%s %luk[CAP] %s%s",
                                  mode_str,
                                  act_khz,
                                  logic_protocol_to_str(proto),
                                  hold_str);
        } else if (s_ui_frame.sample_rate_hz > 0U && cap_mode == LOGIC_CAP_MODE_SAMPLED) {
            unsigned long act_khz = (unsigned long)((s_ui_frame.sample_rate_hz + 500U) / 1000U);
            lv_label_set_text_fmt(s_lbl_top_status,
                                  "%s %s(%luk) %s%s",
                                  mode_str,
                                  logic_sample_rate_to_str(rate_enum),
                                  act_khz,
                                  logic_protocol_to_str(proto),
                                  hold_str);
        } else {
            lv_label_set_text_fmt(s_lbl_top_status,
                                  "%s %s %s%s",
                                  mode_str,
                                  logic_sample_rate_to_str(rate_enum),
                                  logic_protocol_to_str(proto),
                                  hold_str);
        }
    }

    if (s_lbl_zoom_badge != NULL) {
        lv_label_set_text_fmt(s_lbl_zoom_badge,
                              "Z:%lux +%u",
                              (unsigned long)logic_zoom_factor(zoom),
                              (unsigned)pan_off);
    }

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        gpio_num_t pin = tool_logic_get_channel_pin(c);
        if (s_lbl_ch_pins[c] != NULL) {
            lv_label_set_text_fmt(s_lbl_ch_pins[c], "C%u:G%d", (unsigned)c, (int)pin);
        }
        if (s_modal_ch_lbl[c] != NULL) {
            lv_label_set_text_fmt(s_modal_ch_lbl[c], "C%u:G%d", (unsigned)c, (int)pin);
        }
    }

    if (s_chart != NULL) {
        bool hide_waveform = (s_ui_frame.hw_error != ESP_OK) ||
                             s_ui_frame.no_capture ||
                             (s_ui_frame.total_captured == 0U);
        uint32_t zf = logic_zoom_factor(zoom);
        for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
            if (s_ser_ch[c] == NULL) {
                continue;
            }
            int32_t *pts = lv_chart_get_y_array(s_chart, s_ser_ch[c]);
            if (pts != NULL) {
                if (hide_waveform) {
                    for (size_t i = 0; i < LOGIC_VIEW_POINTS; ++i) {
                        pts[i] = LV_CHART_POINT_NONE;
                    }
                } else {
                    int32_t base_y = (int32_t)c * 100 + 15;
                    int32_t high_y = base_y + 65;
                    uint8_t mask = (uint8_t)(1U << c);
                    for (size_t i = 0; i < LOGIC_VIEW_POINTS; ++i) {
                        size_t src_idx = (size_t)pan_off + (i / (size_t)zf);
                        if (src_idx >= s_ui_frame.total_captured) {
                            pts[i] = LV_CHART_POINT_NONE;
                        } else {
                            pts[i] = ((s_ui_frame.view_samples[i] & mask) != 0U) ? high_y : base_y;
                        }
                    }
                }
            }
        }
        lv_chart_refresh(s_chart);
    }

    if (s_lbl_decode_summary != NULL) {
        lv_label_set_text(s_lbl_decode_summary, s_ui_frame.decoded.summary);
    }

    if (s_lbl_decode_rows != NULL) {
        if (s_ui_frame.hw_error != ESP_OK) {
            lv_label_set_text(s_lbl_decode_rows, "Capture aborted due to hardware/timer error");
        } else if (s_ui_frame.no_capture) {
            lv_label_set_text(s_lbl_decode_rows, "No edge transitions captured (idle line)");
        } else if (s_ui_frame.decoded.row_count == 0U) {
            lv_label_set_text(s_lbl_decode_rows, "No decoded frames in buffer");
        } else {
            char table_buf[220];
            size_t pos = 0U;
            size_t max_show = (s_ui_frame.decoded.row_count > 3U) ? 3U : s_ui_frame.decoded.row_count;
            table_buf[0] = '\0';
            for (size_t i = 0; i < max_show; ++i) {
                int n = snprintf(table_buf + pos,
                                 sizeof(table_buf) - pos,
                                 "%s%s",
                                 (i > 0U) ? "\n" : "",
                                 s_ui_frame.decoded.rows[i].text);
                if (n < 0 || (size_t)n >= sizeof(table_buf) - pos) {
                    break;
                }
                pos += (size_t)n;
            }
            lv_label_set_text(s_lbl_decode_rows, table_buf);
        }
    }

    if (s_btn_run_lbl != NULL) {
        lv_label_set_text(s_btn_run_lbl, running ? "RUN" : "HOLD");
    }
    if (s_btn_proto_lbl != NULL) {
        lv_label_set_text(s_btn_proto_lbl, logic_protocol_to_str(proto));
    }
    if (s_btn_zoom_lbl != NULL) {
        lv_label_set_text_fmt(s_btn_zoom_lbl, "%lux", (unsigned long)logic_zoom_factor(zoom));
    }
    if (s_modal_mode_lbl != NULL) {
        lv_label_set_text(s_modal_mode_lbl,
                          (cap_mode == LOGIC_CAP_MODE_RMT_EDGE) ? "RMT EDGE" : "SAMPLED");
    }
    if (s_modal_rate_lbl != NULL) {
        if (cap_mode == LOGIC_CAP_MODE_SAMPLED && rate_enum > LOGIC_SAMPLED_MAX_RATE) {
            lv_label_set_text(s_modal_rate_lbl, "100k(CAP)");
        } else {
            lv_label_set_text(s_modal_rate_lbl, logic_sample_rate_to_str(rate_enum));
        }
    }
    if (s_modal_baud_lbl != NULL) {
        lv_label_set_text_fmt(s_modal_baud_lbl, "%lubd", (unsigned long)baud);
    }
}

/* --------------------------------------------------------------------------
 * Tool Plugin Lifecycle Callbacks
 * -------------------------------------------------------------------------- */

static esp_err_t claim_all_channel_pins(const gpio_num_t pins[LOGIC_CHANNEL_COUNT])
{
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        esp_err_t err = bsp_pin_claim(pins[c], LOGIC_TOOL_ID);
        if (err != ESP_OK) {
            for (uint8_t prev = 0U; prev < c; ++prev) {
                (void)bsp_pin_release(pins[prev], LOGIC_TOOL_ID);
            }
            return err;
        }
    }
    return ESP_OK;
}

static void release_all_channel_pins(void)
{
    (void)bsp_pin_release_all_for_owner(LOGIC_TOOL_ID);
}

static void free_logic_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    if (s_psram_capture_buf != NULL) {
        psram_buf_free(s_psram_capture_buf);
        s_psram_capture_buf = NULL;
    }
    if (s_psram_rmt_buf != NULL) {
        psram_buf_free(s_psram_rmt_buf);
        s_psram_rmt_buf = NULL;
    }
}

static esp_err_t logic_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    gpio_num_t pins[LOGIC_CHANNEL_COUNT];
    portENTER_CRITICAL(&s_logic_lock);
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        pins[c] = s_channel_pins[c];
        s_isr_pins[c] = s_channel_pins[c];
    }
    portEXIT_CRITICAL(&s_logic_lock);

    esp_err_t err = claim_all_channel_pins(pins);
    if (err != ESP_OK) {
        return err;
    }

    err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(logic_frame_t));
    if (err != ESP_OK) {
        release_all_channel_pins();
        return err;
    }

    s_psram_capture_buf = (uint8_t *)psram_buf_calloc(LOGIC_PSRAM_BUFFER_BYTES, sizeof(uint8_t));
    s_psram_rmt_buf = (rmt_symbol_word_t *)psram_buf_calloc(
        LOGIC_RMT_BOUNCE_SYMBOLS, sizeof(rmt_symbol_word_t));
    if (s_psram_capture_buf == NULL || s_psram_rmt_buf == NULL) {
        free_logic_buffers();
        release_all_channel_pins();
        return ESP_ERR_NO_MEM;
    }

    ensure_logic_step_mutex();

    portENTER_CRITICAL(&s_logic_lock);
    s_running = true;
    s_stop_requested = false;
    s_step_preempt = false;
    s_capture_seq++;
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    s_tool_active = true;
    logic_capture_source_fn_t custom_fn = s_custom_source_fn;
    logic_capture_mode_t start_mode = s_cap_mode;
    logic_sample_rate_t start_rate = s_sample_rate;
    portEXIT_CRITICAL(&s_logic_lock);

    if (parent_cont != NULL) {
        build_logic_ui(parent_cont);
    }

    if (custom_fn != NULL ||
        (start_mode == LOGIC_CAP_MODE_SAMPLED && start_rate >= LOGIC_RATE_50KHZ)) {
        (void)tool_logic_run_single_step();
    }

    tool_worker_config_t wcfg = {
        .name = "logic_c1",
        .stack_size_bytes = 6144,
        .priority = 5,
        .core_id = TOOL_WORKER_DEFAULT_CORE,
        .period_ms = LOGIC_WORKER_PERIOD_MS,
        .step_fn = logic_worker_step,
        .user_ctx = NULL,
    };
    err = tool_worker_start(&s_worker, &wcfg);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_logic_lock);
        s_tool_active = false;
        portEXIT_CRITICAL(&s_logic_lock);
        teardown_logic_peripherals();
        free_logic_buffers();
        release_all_channel_pins();
        return err;
    }

    ESP_LOGI(TAG,
             "Logic Analyzer started on CH0..3 = GPIO%d/%d/%d/%d",
             (int)pins[0],
             (int)pins[1],
             (int)pins[2],
             (int)pins[3]);
    return ESP_OK;
}

static esp_err_t logic_on_stop(bool keep_background, void *ctx)
{
    (void)keep_background;
    (void)ctx;

    portENTER_CRITICAL(&s_logic_lock);
    s_stop_requested = true;
    s_capture_seq++;
    tool_worker_t *worker = s_worker;
    s_worker = NULL;
    portEXIT_CRITICAL(&s_logic_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < LOGIC_WORKER_STOP_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, LOGIC_WORKER_STOP_WAIT_MS);
        }
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop logic worker cleanly: %s", esp_err_to_name(stop_err));
            portENTER_CRITICAL(&s_logic_lock);
            s_worker = worker;
            portEXIT_CRITICAL(&s_logic_lock);
            return stop_err;
        }
    }

    ensure_logic_step_mutex();
    (void)xSemaphoreTake(s_step_mutex, portMAX_DELAY);

    s_lbl_top_status = NULL;
    s_lbl_zoom_badge = NULL;
    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        s_lbl_ch_pins[c] = NULL;
        s_ser_ch[c] = NULL;
        s_modal_ch_lbl[c] = NULL;
    }
    s_chart = NULL;
    s_lbl_decode_summary = NULL;
    s_lbl_decode_rows = NULL;
    s_btn_run_lbl = NULL;
    s_btn_proto_lbl = NULL;
    s_btn_zoom_lbl = NULL;
    s_modal_cfg = NULL;
    s_modal_mode_lbl = NULL;
    s_modal_rate_lbl = NULL;
    s_modal_baud_lbl = NULL;

    teardown_logic_peripherals();
    free_logic_buffers();

    portENTER_CRITICAL(&s_logic_lock);
    s_tool_active = false;
    portEXIT_CRITICAL(&s_logic_lock);

    (void)xSemaphoreGive(s_step_mutex);

    release_all_channel_pins();
    return ESP_OK;
}

static esp_err_t logic_selftest(void)
{
    /* Synthesize a 1 kHz 25% duty cycle PWM wave on CH0 at 32 kHz (32 samples/period, 8 high) */
    uint8_t buf[128];
    for (size_t i = 0; i < sizeof(buf); ++i) {
        buf[i] = ((i % 32U) < 8U) ? 0x01U : 0x00U;
    }

    logic_pwm_result_t pwm = {0};
    if (logic_decode_pwm(buf, sizeof(buf), 32000U, 0U, &pwm) != ESP_OK) {
        return ESP_FAIL;
    }
    if (!pwm.valid || pwm.complete_cycles < 2U) {
        return ESP_FAIL;
    }
    if (pwm.freq_hz < 990.0f || pwm.freq_hz > 1010.0f ||
        pwm.duty_pct < 24.0f || pwm.duty_pct > 26.0f) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_logic_descriptor = {
    .id = LOGIC_TOOL_ID,
    .title = "Logic Analyzer",
    .symbol = LV_SYMBOL_LIST,
    .category = TOOL_CATEGORY_DIGITAL_IN,
    .supports_background = false,
    .supports_recording = true,
    .supports_spk_passthrough = false,
    .on_init = NULL,
    .on_start = logic_on_start,
    .on_stop = logic_on_stop,
    .on_ui_refresh = logic_on_ui_refresh,
    .is_running_bg = NULL,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_logic_get_descriptor(void)
{
    return &s_logic_descriptor;
}

esp_err_t tool_logic_register(void)
{
    esp_err_t err = tool_registry_register(&s_logic_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("logic_decoders", logic_selftest);
    }
    return err;
}
