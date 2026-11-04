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
static DRAM_ATTR volatile size_t s_isr_write_idx = 0;
static DRAM_ATTR volatile bool s_isr_capture_done = false;
static DRAM_ATTR gpio_num_t s_isr_pins[LOGIC_CHANNEL_COUNT] = {
    BSP_PIN_PORT_C_RXD2,
    BSP_PIN_PORT_C_TXD2,
    BSP_PIN_PORT_A_SDA,
    BSP_PIN_PORT_A_SCL,
};

static DRAM_ATTR rmt_symbol_word_t s_rmt_bounce_buf[LOGIC_RMT_BOUNCE_SYMBOLS];
static DRAM_ATTR volatile size_t s_rmt_isr_symbol_count = 0;
static DRAM_ATTR volatile bool s_rmt_isr_done = false;

/* Hardware peripheral handles */
static gptimer_handle_t s_gptimer = NULL;
static bool s_gptimer_enabled = false;
static rmt_channel_handle_t s_rmt_chan = NULL;
static bool s_rmt_enabled = false;
static gpio_num_t s_rmt_configured_pin = GPIO_NUM_NC;

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
    (void)edata;
    (void)user_ctx;
    size_t idx = s_isr_write_idx;
    if (idx < LOGIC_ISR_BOUNCE_SAMPLES) {
        s_isr_bounce_buf[idx] = pack_gpio_sample_fast((const gpio_num_t *)s_isr_pins);
        idx++;
        s_isr_write_idx = idx;
        if (idx >= LOGIC_ISR_BOUNCE_SAMPLES) {
            s_isr_capture_done = true;
            (void)gptimer_stop(timer);
        }
    }
    return false;
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
 * Hardware Peripheral Setup & Teardown (GPTimer + RMT RX)
 * -------------------------------------------------------------------------- */

static void teardown_logic_peripherals(void)
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
    s_rmt_configured_pin = GPIO_NUM_NC;
}

static void ensure_logic_gptimer(void)
{
    if (s_gptimer != NULL) {
        return;
    }
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000U,
    };
    if (gptimer_new_timer(&tcfg, &s_gptimer) == ESP_OK && s_gptimer != NULL) {
        gptimer_event_callbacks_t cbs = {
            .on_alarm = on_logic_gptimer_alarm_isr,
        };
        (void)gptimer_register_event_callbacks(s_gptimer, &cbs, NULL);
        if (gptimer_enable(s_gptimer) == ESP_OK) {
            s_gptimer_enabled = true;
        }
    }
}

static void ensure_logic_rmt(gpio_num_t ch0_pin)
{
    if (s_rmt_chan != NULL && s_rmt_configured_pin == ch0_pin) {
        return;
    }
    if (s_rmt_chan != NULL) {
        if (s_rmt_enabled) {
            (void)rmt_disable(s_rmt_chan);
            s_rmt_enabled = false;
        }
        (void)rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
    }

    rmt_rx_channel_config_t rcfg = {
        .gpio_num = ch0_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LOGIC_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64U,
    };
    if (rmt_new_rx_channel(&rcfg, &s_rmt_chan) == ESP_OK && s_rmt_chan != NULL) {
        rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = on_logic_rmt_done_isr,
        };
        (void)rmt_rx_register_event_callbacks(s_rmt_chan, &cbs, NULL);
        if (rmt_enable(s_rmt_chan) == ESP_OK) {
            s_rmt_enabled = true;
        }
        s_rmt_configured_pin = ch0_pin;
    }
}

static bool should_abort_capture(uint32_t start_seq, bool is_bg_worker)
{
    portENTER_CRITICAL(&s_logic_lock);
    bool abort = s_stop_requested || !s_running || (s_capture_seq != start_seq) ||
                 (is_bg_worker && s_step_preempt);
    portEXIT_CRITICAL(&s_logic_lock);
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
        TickType_t ticks = pdMS_TO_TICKS(delay_ms);
        if (ticks == 0) {
            ticks = 1;
        }
        vTaskDelay(ticks);
        remaining_us = deadline_us - esp_timer_get_time();
    }

    while (remaining_us > 0) {
        if (should_abort_capture(start_seq, is_bg_worker)) {
            return false;
        }
        if (remaining_us > 50) {
            esp_rom_delay_us(40U);
        } else if (remaining_us > 4) {
            esp_rom_delay_us((uint32_t)(remaining_us - 2));
        }
        remaining_us = deadline_us - esp_timer_get_time();
    }
    return !should_abort_capture(start_seq, is_bg_worker);
}

static esp_err_t capture_logic_buffer(logic_capture_mode_t cap_mode,
                                      const gpio_num_t pins[LOGIC_CHANNEL_COUNT],
                                      uint32_t requested_rate_hz,
                                      uint32_t start_seq,
                                      bool is_bg_worker,
                                      uint8_t *out_psram_samples,
                                      size_t target_samples,
                                      size_t *out_captured_count,
                                      uint32_t *out_actual_rate_hz,
                                      uint64_t *out_timestamp_us,
                                      bool *out_aborted)
{
    *out_aborted = false;
    *out_captured_count = 0U;
    *out_actual_rate_hz = requested_rate_hz;
    *out_timestamp_us = (uint64_t)esp_timer_get_time();

    logic_capture_source_fn_t custom_fn = NULL;
    void *custom_ctx = NULL;
    portENTER_CRITICAL(&s_logic_lock);
    custom_fn = s_custom_source_fn;
    custom_ctx = s_custom_source_ctx;
    portEXIT_CRITICAL(&s_logic_lock);

    if (custom_fn != NULL) {
        return custom_fn(cap_mode,
                         pins,
                         requested_rate_hz,
                         out_psram_samples,
                         target_samples,
                         out_captured_count,
                         out_actual_rate_hz,
                         custom_ctx);
    }

    for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
        (void)gpio_set_direction(pins[c], GPIO_MODE_INPUT);
    }

    if (cap_mode == LOGIC_CAP_MODE_RMT_EDGE) {
        ensure_logic_rmt(pins[0]);
        s_rmt_isr_symbol_count = 0U;
        s_rmt_isr_done = false;

        if (s_rmt_chan != NULL && s_rmt_enabled) {
            rmt_receive_config_t rx_cfg = {
                .signal_range_min_ns = 500U,
                .signal_range_max_ns = 15000000U, /* 15 ms idle threshold */
            };
            (void)rmt_receive(
                s_rmt_chan, s_rmt_bounce_buf, sizeof(s_rmt_bounce_buf), &rx_cfg);
        }

        int64_t rmt_deadline_us = esp_timer_get_time() + 18000;
        while (!s_rmt_isr_done && esp_timer_get_time() < rmt_deadline_us) {
            if (should_abort_capture(start_seq, is_bg_worker)) {
                if (s_rmt_chan != NULL && s_rmt_enabled) {
                    (void)rmt_disable(s_rmt_chan);
                    (void)rmt_enable(s_rmt_chan);
                }
                *out_aborted = true;
                return ESP_ERR_INVALID_STATE;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        if (s_rmt_chan != NULL && s_rmt_enabled && !s_rmt_isr_done) {
            (void)rmt_disable(s_rmt_chan);
            (void)rmt_enable(s_rmt_chan);
        }

        size_t rmt_cnt = s_rmt_isr_symbol_count;
        memset(out_psram_samples, 0, target_samples);
        if (rmt_cnt > 0U && s_psram_rmt_buf != NULL) {
            memcpy(s_psram_rmt_buf,
                   (const void *)s_rmt_bounce_buf,
                   rmt_cnt * sizeof(rmt_symbol_word_t));
            size_t expanded = 0U;
            (void)logic_expand_rmt_to_samples(s_psram_rmt_buf,
                                              rmt_cnt,
                                              LOGIC_RMT_RESOLUTION_HZ,
                                              requested_rate_hz,
                                              0U,
                                              out_psram_samples,
                                              target_samples,
                                              &expanded);
        } else {
            /* Populate static pin levels when no RMT transitions arrived */
            uint8_t idle_mask = pack_gpio_sample_fast(pins);
            memset(out_psram_samples, idle_mask, target_samples);
        }
        *out_captured_count = target_samples;
        *out_actual_rate_hz = requested_rate_hz;
        return ESP_OK;
    }

    /* LOGIC_CAP_MODE_SAMPLED:
     * Sample in internal SRAM bounce chunks (`s_isr_bounce_buf`) using GPTimer / deadline
     * pacing and copy batches into the 64 KB PSRAM buffer (`out_psram_samples`). */
    ensure_logic_gptimer();
    uint32_t interval_us = (requested_rate_hz > 0U && requested_rate_hz <= 1000000U)
                               ? (1000000U / requested_rate_hz)
                               : 1U;
    if (interval_us == 0U) {
        interval_us = 1U;
    }

    size_t written_total = 0U;
    int64_t capture_start_us = esp_timer_get_time();
    *out_timestamp_us = (uint64_t)capture_start_us;

    while (written_total < target_samples) {
        size_t chunk_len = target_samples - written_total;
        if (chunk_len > LOGIC_ISR_BOUNCE_SAMPLES) {
            chunk_len = LOGIC_ISR_BOUNCE_SAMPLES;
        }

        for (size_t i = 0; i < chunk_len; ++i) {
            size_t global_idx = written_total + i;
            if ((i & 15U) == 0U && should_abort_capture(start_seq, is_bg_worker)) {
                *out_aborted = true;
                return ESP_ERR_INVALID_STATE;
            }
            if (global_idx > 0U && interval_us >= 5U) {
                int64_t deadline = capture_start_us + (int64_t)global_idx * (int64_t)interval_us;
                if (!wait_until_us(deadline, start_seq, is_bg_worker)) {
                    *out_aborted = true;
                    return ESP_ERR_INVALID_STATE;
                }
            }
            s_isr_bounce_buf[i] = pack_gpio_sample_fast(pins);
        }

        /* Stage internal SRAM bounce chunk into PSRAM capture buffer */
        memcpy(&out_psram_samples[written_total], (const void *)s_isr_bounce_buf, chunk_len);
        written_total += chunk_len;
    }

    int64_t elapsed_us = esp_timer_get_time() - capture_start_us;
    if (elapsed_us > 0 && written_total > 1U && interval_us >= 5U) {
        uint64_t num = (uint64_t)(written_total - 1U) * 1000000ULL;
        uint64_t measured_hz = (num + (uint64_t)elapsed_us / 2ULL) / (uint64_t)elapsed_us;
        *out_actual_rate_hz = (measured_hz > UINT32_MAX) ? UINT32_MAX : (uint32_t)measured_hz;
    } else {
        *out_actual_rate_hz = requested_rate_hz;
    }
    *out_captured_count = written_total;
    return ESP_OK;
}

static esp_err_t logic_worker_step_locked(bool is_bg_worker)
{
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

    size_t captured = LOGIC_DEFAULT_CAPTURE_SAMPLES;
    uint32_t actual_rate_hz = req_rate_hz;
    uint64_t capture_ts_us = 0U;

    if (running) {
        bool aborted = false;
        esp_err_t err = capture_logic_buffer(cap_mode,
                                             pins,
                                             req_rate_hz,
                                             start_seq,
                                             is_bg_worker,
                                             s_psram_capture_buf,
                                             LOGIC_DEFAULT_CAPTURE_SAMPLES,
                                             &captured,
                                             &actual_rate_hz,
                                             &capture_ts_us,
                                             &aborted);
        if (aborted) {
            return ESP_OK;
        }
        if (err != ESP_OK || captured == 0U) {
            return err;
        }

        if (recorder_sink_is_recording()) {
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
        if (s_latest_frame.total_captured > 0U) {
            captured = s_latest_frame.total_captured;
            actual_rate_hz = s_latest_frame.sample_rate_hz;
        }
        portEXIT_CRITICAL(&s_logic_lock);
    }

    /* Re-read latest UI settings (zoom, pan, protocol, baud) before building frame */
    portENTER_CRITICAL(&s_logic_lock);
    proto = s_protocol;
    uart_baud = s_uart_baud;
    zoom = s_zoom;
    pan_off = s_pan_offset;
    portEXIT_CRITICAL(&s_logic_lock);

    bool triggered = false;
    size_t trig_idx = 0U;
    (void)logic_find_digital_trigger(
        s_psram_capture_buf, captured, 0U, trig_mode, &triggered, &trig_idx);

    logic_frame_t *wptr = (logic_frame_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        memset(wptr, 0, sizeof(*wptr));
        (void)logic_extract_viewport(
            s_psram_capture_buf, captured, zoom, pan_off, wptr->view_samples, LOGIC_VIEW_POINTS);
        wptr->total_captured = captured;
        wptr->trigger_index = trig_idx;
        wptr->triggered = triggered;
        wptr->sample_rate_hz = actual_rate_hz;
        wptr->cap_mode = cap_mode;
        wptr->protocol = proto;
        for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
            wptr->channel_pins[c] = pins[c];
        }
        wptr->zoom = zoom;
        wptr->pan_offset = pan_off;

        (void)logic_decode_run_protocol(
            s_psram_capture_buf, captured, actual_rate_hz, proto, uart_baud, &wptr->decoded);

        portENTER_CRITICAL(&s_logic_lock);
        s_latest_frame = *wptr;
        portEXIT_CRITICAL(&s_logic_lock);

        (void)psram_double_buf_swap(&s_frame_dbuf);
    }

    return ESP_OK;
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
    logic_sample_rate_t next = (logic_sample_rate_t)((tool_logic_get_sample_rate() + 1) % LOGIC_RATE_COUNT);
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
        lv_label_set_text_fmt(s_lbl_top_status,
                              "%s %s %s%s",
                              (cap_mode == LOGIC_CAP_MODE_RMT_EDGE) ? "RMT" : "SAMP",
                              logic_sample_rate_to_str(rate_enum),
                              logic_protocol_to_str(proto),
                              running ? "" : " [HOLD]");
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
        for (uint8_t c = 0U; c < LOGIC_CHANNEL_COUNT; ++c) {
            if (s_ser_ch[c] == NULL) {
                continue;
            }
            int32_t *pts = lv_chart_get_y_array(s_chart, s_ser_ch[c]);
            if (pts != NULL) {
                int32_t base_y = (int32_t)c * 100 + 15;
                int32_t high_y = base_y + 65;
                uint8_t mask = (uint8_t)(1U << c);
                for (size_t i = 0; i < LOGIC_VIEW_POINTS; ++i) {
                    pts[i] = ((s_ui_frame.view_samples[i] & mask) != 0U) ? high_y : base_y;
                }
            }
        }
        lv_chart_refresh(s_chart);
    }

    if (s_lbl_decode_summary != NULL) {
        lv_label_set_text(s_lbl_decode_summary, s_ui_frame.decoded.summary);
    }

    if (s_lbl_decode_rows != NULL) {
        if (s_ui_frame.decoded.row_count == 0U) {
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
        lv_label_set_text(s_modal_rate_lbl, logic_sample_rate_to_str(rate_enum));
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
