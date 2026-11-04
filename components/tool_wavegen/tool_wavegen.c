/**
 * @file tool_wavegen.c
 * @brief Digital Wave Generator instrument (`tool_wavegen`) driving ESP32 LEDC, MCPWM, and RMT TX
 *        peripherals with background concurrency, GPIO reservation, PSRAM symbol staging,
 *        and LVGL 9 touch UI.
 */

#include "tool_wavegen.h"
#include "tool_wavegen_priv.h"

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "psram_buffer.h"
#include "tool_core.h"

static const char *TAG = "tool_wavegen";

#define WAVEGEN_WORKER_PERIOD_MS       (20U)
#define WAVEGEN_WORKER_STOP_WAIT_MS    (1000U)
#define WAVEGEN_WORKER_STOP_RETRIES    (4)
#define WAVEGEN_LEDC_SPEED_MODE        (LEDC_HIGH_SPEED_MODE)
#define WAVEGEN_LEDC_TIMER_NUM         (LEDC_TIMER_0)
#define WAVEGEN_LEDC_CHANNEL_NUM       (LEDC_CHANNEL_0)
#define WAVEGEN_MCPWM_GROUP_ID         (0)

static const gpio_num_t s_supported_pins[] = {
    BSP_PIN_PORT_C_TXD2,  /* GPIO14 (Port C TXD2 - Default) */
    BSP_PIN_PORT_C_RXD2,  /* GPIO13 (Port C RXD2) */
    BSP_PIN_DAC_OUT_CH2,  /* GPIO26 (Port B / M5-Bus) */
    BSP_PIN_PORT_A_SDA,   /* GPIO32 (Port A SDA) */
    BSP_PIN_PORT_A_SCL,   /* GPIO33 (Port A SCL) */
    BSP_PIN_MBUS_GPIO27,  /* GPIO27 (M5-Bus) */
    BSP_PIN_MBUS_GPIO19,  /* GPIO19 (M5-Bus) */
    BSP_PIN_DAC_OUT_CH1,  /* GPIO25 (M5-Bus) */
};
#define WAVEGEN_SUPPORTED_PIN_COUNT (sizeof(s_supported_pins) / sizeof(s_supported_pins[0]))

static const uint32_t s_freq_presets_hz[] = {
    10U,
    50U,
    100U,
    500U,
    1000U,
    5000U,
    10000U,
    50000U,
    100000U,
    250000U,
    1000000U,
};
#define WAVEGEN_FREQ_PRESET_COUNT (sizeof(s_freq_presets_hz) / sizeof(s_freq_presets_hz[0]))

static portMUX_TYPE s_wave_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_hw_mutex_buf;
static SemaphoreHandle_t s_hw_mutex = NULL;
static atomic_int s_hw_mutex_state = 0;

static wavegen_config_t s_cfg = {
    .pin = WAVEGEN_DEFAULT_PIN,
    .mode = WAVEGEN_MODE_SQUARE,
    .freq_hz = WAVEGEN_DEFAULT_FREQ_HZ,
    .duty_pct = WAVEGEN_DEFAULT_DUTY_PCT,
    .servo_pulse_us = WAVEGEN_SERVO_DEFAULT_US,
    .burst_high_us = WAVEGEN_DEFAULT_BURST_HIGH_US,
    .burst_low_us = WAVEGEN_DEFAULT_BURST_LOW_US,
    .burst_pulse_count = WAVEGEN_DEFAULT_BURST_PULSES,
    .burst_period_ms = WAVEGEN_DEFAULT_BURST_REP_MS,
    .output_enabled = true,
    .keep_in_bg = true,
};

static wavegen_status_t s_status = {0};
static bool s_running_fg = false;
static bool s_running_bg = false;
static volatile bool s_stop_requested = false;
static esp_err_t s_forced_hw_err = ESP_OK;

/* Active peripheral state (protected by s_hw_mutex) */
typedef enum {
    WAVEGEN_HW_NONE = 0,
    WAVEGEN_HW_LEDC,
    WAVEGEN_HW_MCPWM,
    WAVEGEN_HW_RMT,
} wavegen_hw_type_t;

static wavegen_hw_type_t s_active_hw = WAVEGEN_HW_NONE;
static gpio_num_t s_hw_pin = GPIO_NUM_NC;
static mcpwm_timer_handle_t s_mcpwm_timer = NULL;
static mcpwm_oper_handle_t s_mcpwm_oper = NULL;
static mcpwm_cmpr_handle_t s_mcpwm_cmpr = NULL;
static mcpwm_gen_handle_t s_mcpwm_gen = NULL;
static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_rmt_encoder = NULL;
static bool s_rmt_enabled = false;
static int64_t s_last_burst_tx_us = 0;

/* Internal SRAM bounce buffer for RMT TX (DMA/ISR-safe) & PSRAM symbol staging */
static DRAM_ATTR rmt_symbol_word_t s_rmt_bounce_symbols[WAVEGEN_MAX_BURST_PULSES];
static size_t s_rmt_bounce_count = 0;
static rmt_symbol_word_t *s_psram_symbols = NULL;
static psram_double_buf_t s_frame_dbuf = {.spinlock = portMUX_INITIALIZER_UNLOCKED};
static tool_worker_t *s_worker = NULL;

/* LVGL 9 UI Widgets (Core 0) */
static lv_obj_t *s_lbl_top_status = NULL;
static lv_obj_t *s_lbl_bg_badge = NULL;
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser_wave = NULL;
static lv_obj_t *s_lbl_primary = NULL;
static lv_obj_t *s_lbl_secondary = NULL;
static lv_obj_t *s_btn_out_lbl = NULL;
static lv_obj_t *s_btn_mode_lbl = NULL;
static lv_obj_t *s_btn_freq_lbl = NULL;
static lv_obj_t *s_btn_duty_lbl = NULL;
static lv_obj_t *s_btn_pin_lbl = NULL;
static lv_obj_t *s_btn_bg_lbl = NULL;

static void ensure_wave_hw_mutex(void)
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
    ensure_wave_hw_mutex();
    (void)xSemaphoreTake(s_hw_mutex, portMAX_DELAY);
}

static void hw_unlock(void)
{
    (void)xSemaphoreGive(s_hw_mutex);
}

bool tool_wavegen_is_pin_supported(gpio_num_t pin)
{
    if (bsp_pin_is_protected(pin) || bsp_pin_is_input_only(pin)) {
        return false;
    }
    for (size_t i = 0; i < WAVEGEN_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == pin) {
            return true;
        }
    }
    return false;
}

static void teardown_active_hw_locked(void)
{
    if (s_active_hw == WAVEGEN_HW_LEDC) {
        (void)ledc_stop(WAVEGEN_LEDC_SPEED_MODE, WAVEGEN_LEDC_CHANNEL_NUM, 0U);
        ledc_channel_config_t ch_decfg = {
            .speed_mode = WAVEGEN_LEDC_SPEED_MODE,
            .channel = WAVEGEN_LEDC_CHANNEL_NUM,
            .deconfigure = true,
        };
        (void)ledc_channel_config(&ch_decfg);
        (void)ledc_timer_pause(WAVEGEN_LEDC_SPEED_MODE, WAVEGEN_LEDC_TIMER_NUM);
        ledc_timer_config_t tm_decfg = {
            .speed_mode = WAVEGEN_LEDC_SPEED_MODE,
            .timer_num = WAVEGEN_LEDC_TIMER_NUM,
            .deconfigure = true,
        };
        (void)ledc_timer_config(&tm_decfg);
    } else if (s_active_hw == WAVEGEN_HW_MCPWM) {
        if (s_mcpwm_timer != NULL) {
            (void)mcpwm_timer_start_stop(s_mcpwm_timer, MCPWM_TIMER_STOP_EMPTY);
            (void)mcpwm_timer_disable(s_mcpwm_timer);
        }
        if (s_mcpwm_gen != NULL) {
            (void)mcpwm_del_generator(s_mcpwm_gen);
            s_mcpwm_gen = NULL;
        }
        if (s_mcpwm_cmpr != NULL) {
            (void)mcpwm_del_comparator(s_mcpwm_cmpr);
            s_mcpwm_cmpr = NULL;
        }
        if (s_mcpwm_oper != NULL) {
            (void)mcpwm_del_operator(s_mcpwm_oper);
            s_mcpwm_oper = NULL;
        }
        if (s_mcpwm_timer != NULL) {
            (void)mcpwm_del_timer(s_mcpwm_timer);
            s_mcpwm_timer = NULL;
        }
    } else if (s_active_hw == WAVEGEN_HW_RMT) {
        if (s_rmt_chan != NULL) {
            if (s_rmt_enabled) {
                (void)rmt_disable(s_rmt_chan);
                s_rmt_enabled = false;
            }
            (void)rmt_del_channel(s_rmt_chan);
            s_rmt_chan = NULL;
        }
        if (s_rmt_encoder != NULL) {
            (void)rmt_del_encoder(s_rmt_encoder);
            s_rmt_encoder = NULL;
        }
    }

    if (s_hw_pin != GPIO_NUM_NC) {
        (void)gpio_reset_pin(s_hw_pin);
    }
    s_active_hw = WAVEGEN_HW_NONE;
    s_hw_pin = GPIO_NUM_NC;
}

static esp_err_t transmit_rmt_frame_locked(void)
{
    if (s_active_hw != WAVEGEN_HW_RMT || s_rmt_chan == NULL ||
        s_rmt_encoder == NULL || s_rmt_bounce_count == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stage from PSRAM symbol buffer (when allocated) into internal DRAM bounce buffer */
    if (s_psram_symbols != NULL) {
        memcpy(s_rmt_bounce_symbols,
               s_psram_symbols,
               s_rmt_bounce_count * sizeof(rmt_symbol_word_t));
    }

    /* On real ESP32 silicon, transmit symbols via RMT TX hardware and wait for completion
     * so the channel returns to RMT_FSM_ENABLE before any future rmt_disable().
     * In QEMU (where RMT 0x3ff56000 is un-emulated and rmt_disable() spins on TX_DONE if
     * rmt_transmit placed the channel in RMT_FSM_RUN), we record the staged frame directly. */
    if (!bsp_adc1_uses_mock_fallback()) {
        rmt_transmit_config_t tx_cfg = {
            .loop_count = 0,
            .flags = {
                .eot_level = 0U,
                .queue_nonblocking = 1U,
            },
        };
        esp_err_t err = rmt_transmit(s_rmt_chan,
                                     s_rmt_encoder,
                                     s_rmt_bounce_symbols,
                                     s_rmt_bounce_count * sizeof(rmt_symbol_word_t),
                                     &tx_cfg);
        if (err != ESP_OK) {
            return err;
        }
        (void)rmt_tx_wait_all_done(s_rmt_chan, 50);
    }

    portENTER_CRITICAL(&s_wave_lock);
    s_status.burst_frames_sent++;
    portEXIT_CRITICAL(&s_wave_lock);
    s_last_burst_tx_us = esp_timer_get_time();
    return ESP_OK;
}

static void publish_status_snapshot_locked(const wavegen_config_t *cfg,
                                           uint32_t actual_freq_hz,
                                           float actual_duty_pct,
                                           uint8_t res_bits,
                                           uint32_t period_ticks,
                                           uint32_t compare_ticks,
                                           size_t rmt_syms,
                                           bool hw_active,
                                           esp_err_t last_err)
{
    portENTER_CRITICAL(&s_wave_lock);
    s_status.cfg = *cfg;
    s_status.actual_freq_hz = actual_freq_hz;
    s_status.actual_duty_pct = actual_duty_pct;
    s_status.resolution_bits = res_bits;
    s_status.period_ticks = period_ticks;
    s_status.compare_ticks = compare_ticks;
    s_status.rmt_symbol_count = rmt_syms;
    s_status.running_fg = s_running_fg;
    s_status.running_bg = s_running_bg;
    s_status.hw_active = hw_active;
    s_status.last_error = last_err;
    wavegen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_wave_lock);

    wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }
}

static esp_err_t apply_hw_config_locked(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    wavegen_config_t cfg = s_cfg;
    bool is_active = s_running_fg || s_running_bg;
    esp_err_t injected_err = s_forced_hw_err;
    s_forced_hw_err = ESP_OK;
    portEXIT_CRITICAL(&s_wave_lock);

    if (!is_active) {
        publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, ESP_OK);
        return ESP_OK;
    }

    teardown_active_hw_locked();

    if (injected_err != ESP_OK) {
        publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, injected_err);
        return injected_err;
    }

    if (!cfg.output_enabled) {
        (void)gpio_reset_pin(cfg.pin);
        (void)gpio_set_direction(cfg.pin, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(cfg.pin, 0);
        publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, ESP_OK);
        return ESP_OK;
    }

    if (cfg.mode == WAVEGEN_MODE_SQUARE || cfg.mode == WAVEGEN_MODE_PWM_LEDC) {
        float target_duty = (cfg.mode == WAVEGEN_MODE_SQUARE) ? 50.0f : cfg.duty_pct;
        wavegen_ledc_params_t lp = {0};
        esp_err_t err = wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, cfg.freq_hz, target_duty, &lp);
        if (err != ESP_OK) {
            publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
            return err;
        }

        ledc_timer_config_t tm_cfg = {
            .speed_mode = WAVEGEN_LEDC_SPEED_MODE,
            .duty_resolution = (ledc_timer_bit_t)lp.duty_resolution_bits,
            .timer_num = WAVEGEN_LEDC_TIMER_NUM,
            .freq_hz = cfg.freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        err = ledc_timer_config(&tm_cfg);
        if (err != ESP_OK) {
            publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
            return err;
        }

        ledc_channel_config_t ch_cfg = {
            .gpio_num = (int)cfg.pin,
            .speed_mode = WAVEGEN_LEDC_SPEED_MODE,
            .channel = WAVEGEN_LEDC_CHANNEL_NUM,
            .timer_sel = WAVEGEN_LEDC_TIMER_NUM,
            .duty = lp.duty_ticks,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        };
        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK) {
            (void)ledc_timer_pause(WAVEGEN_LEDC_SPEED_MODE, WAVEGEN_LEDC_TIMER_NUM);
            tm_cfg.deconfigure = true;
            (void)ledc_timer_config(&tm_cfg);
            publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
            return err;
        }

        s_active_hw = WAVEGEN_HW_LEDC;
        s_hw_pin = cfg.pin;
        publish_status_snapshot_locked(&cfg,
                                       lp.actual_freq_hz,
                                       lp.actual_duty_pct,
                                       lp.duty_resolution_bits,
                                       lp.max_duty_ticks,
                                       lp.duty_ticks,
                                       0U,
                                       true,
                                       ESP_OK);
        return ESP_OK;
    }

    if (cfg.mode == WAVEGEN_MODE_PWM_MCPWM) {
        uint32_t eff_freq = cfg.freq_hz;
        if (eff_freq < WAVEGEN_MCPWM_MIN_FREQ_HZ) {
            eff_freq = WAVEGEN_MCPWM_MIN_FREQ_HZ;
        } else if (eff_freq > WAVEGEN_MCPWM_MAX_FREQ_HZ) {
            eff_freq = WAVEGEN_MCPWM_MAX_FREQ_HZ;
        }

        wavegen_mcpwm_params_t mp = {0};
        esp_err_t err = wavegen_calc_mcpwm_params(eff_freq, cfg.duty_pct, &mp);
        if (err != ESP_OK) {
            publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
            return err;
        }

        s_active_hw = WAVEGEN_HW_MCPWM;
        s_hw_pin = cfg.pin;

        /* On real ESP32 silicon, program MCPWM0 (0x3ff5e000). In QEMU (where 0x3ff5e000 is
         * unmapped in the PIF memory map and triggers LoadStorePIFAddrError), configure the
         * GPIO output direction while publishing the solved MCPWM timer/comparator parameters. */
        if (!bsp_adc1_uses_mock_fallback()) {
            mcpwm_timer_config_t timer_cfg = {
                .group_id = WAVEGEN_MCPWM_GROUP_ID,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = mp.resolution_hz,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = mp.period_ticks,
            };
            err = mcpwm_new_timer(&timer_cfg, &s_mcpwm_timer);
            if (err == ESP_OK) {
                mcpwm_operator_config_t oper_cfg = {
                    .group_id = WAVEGEN_MCPWM_GROUP_ID,
                };
                err = mcpwm_new_operator(&oper_cfg, &s_mcpwm_oper);
            }
            if (err == ESP_OK) {
                err = mcpwm_operator_connect_timer(s_mcpwm_oper, s_mcpwm_timer);
            }
            if (err == ESP_OK) {
                mcpwm_comparator_config_t cmpr_cfg = {
                    .flags.update_cmp_on_tez = 1U,
                };
                err = mcpwm_new_comparator(s_mcpwm_oper, &cmpr_cfg, &s_mcpwm_cmpr);
            }
            if (err == ESP_OK) {
                err = mcpwm_comparator_set_compare_value(s_mcpwm_cmpr, mp.compare_ticks);
            }
            if (err == ESP_OK) {
                mcpwm_generator_config_t gen_cfg = {
                    .gen_gpio_num = (int)cfg.pin,
                };
                err = mcpwm_new_generator(s_mcpwm_oper, &gen_cfg, &s_mcpwm_gen);
            }
            if (err == ESP_OK) {
                if (mp.compare_ticks == 0U) {
                    err = mcpwm_generator_set_force_level(s_mcpwm_gen, 0, true);
                } else if (mp.compare_ticks >= mp.period_ticks) {
                    err = mcpwm_generator_set_force_level(s_mcpwm_gen, 1, true);
                } else {
                    (void)mcpwm_generator_set_force_level(s_mcpwm_gen, -1, true);
                    err = mcpwm_generator_set_action_on_timer_event(
                        s_mcpwm_gen,
                        MCPWM_GEN_TIMER_EVENT_ACTION(
                            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
                    if (err == ESP_OK) {
                        err = mcpwm_generator_set_action_on_compare_event(
                            s_mcpwm_gen,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, s_mcpwm_cmpr, MCPWM_GEN_ACTION_LOW));
                    }
                }
            }
            if (err == ESP_OK) {
                err = mcpwm_timer_enable(s_mcpwm_timer);
            }
            if (err == ESP_OK) {
                err = mcpwm_timer_start_stop(s_mcpwm_timer, MCPWM_TIMER_START_NO_STOP);
            }
        } else {
            (void)gpio_reset_pin(cfg.pin);
            err = gpio_set_direction(cfg.pin, GPIO_MODE_OUTPUT);
        }

        if (err != ESP_OK) {
            teardown_active_hw_locked();
            publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
            return err;
        }

        publish_status_snapshot_locked(&cfg,
                                       mp.actual_freq_hz,
                                       mp.actual_duty_pct,
                                       16U,
                                       mp.period_ticks,
                                       mp.compare_ticks,
                                       0U,
                                       true,
                                       ESP_OK);
        return ESP_OK;
    }

    /* RMT TX Modes: WAVEGEN_MODE_SERVO and WAVEGEN_MODE_RMT_BURST */
    rmt_symbol_word_t *stage_buf = (s_psram_symbols != NULL) ? s_psram_symbols : s_rmt_bounce_symbols;
    size_t sym_count = 0U;
    esp_err_t err = ESP_OK;
    uint32_t eff_freq_hz = 0U;
    float eff_duty_pct = 0.0f;

    if (cfg.mode == WAVEGEN_MODE_SERVO) {
        err = wavegen_calc_servo_rmt_symbol(cfg.servo_pulse_us,
                                            WAVEGEN_SERVO_PERIOD_US,
                                            stage_buf,
                                            WAVEGEN_MAX_BURST_PULSES,
                                            &sym_count);
        eff_freq_hz = WAVEGEN_SERVO_FREQ_HZ;
        eff_duty_pct = ((float)cfg.servo_pulse_us * 100.0f) / (float)WAVEGEN_SERVO_PERIOD_US;
    } else {
        err = wavegen_calc_burst_rmt_symbols(cfg.burst_high_us,
                                             cfg.burst_low_us,
                                             cfg.burst_pulse_count,
                                             stage_buf,
                                             WAVEGEN_MAX_BURST_PULSES,
                                             &sym_count);
        uint32_t pulse_period_us = (uint32_t)cfg.burst_high_us + (uint32_t)cfg.burst_low_us;
        eff_freq_hz = (pulse_period_us > 0U) ? (1000000U / pulse_period_us) : 0U;
        eff_duty_pct = (pulse_period_us > 0U)
                           ? (((float)cfg.burst_high_us * 100.0f) / (float)pulse_period_us)
                           : 0.0f;
    }

    if (err != ESP_OK || sym_count == 0U) {
        publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false,
                                       (err != ESP_OK) ? err : ESP_ERR_INVALID_ARG);
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_ARG;
    }

    memcpy(s_rmt_bounce_symbols, stage_buf, sym_count * sizeof(rmt_symbol_word_t));
    s_rmt_bounce_count = sym_count;

    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num = cfg.pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WAVEGEN_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        .flags = {
            .invert_out = 0U,
            .with_dma = 0U,
        },
    };
    s_active_hw = WAVEGEN_HW_RMT;
    s_hw_pin = cfg.pin;
    s_rmt_enabled = false;

    err = rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_chan);
    if (err == ESP_OK) {
        rmt_copy_encoder_config_t enc_cfg = {};
        err = rmt_new_copy_encoder(&enc_cfg, &s_rmt_encoder);
    }
    if (err == ESP_OK) {
        err = rmt_enable(s_rmt_chan);
        if (err == ESP_OK) {
            s_rmt_enabled = true;
        }
    }
    if (err == ESP_OK) {
        err = transmit_rmt_frame_locked();
    }

    if (err != ESP_OK) {
        teardown_active_hw_locked();
        publish_status_snapshot_locked(&cfg, 0U, 0.0f, 0U, 0U, 0U, 0U, false, err);
        return err;
    }

    publish_status_snapshot_locked(&cfg,
                                   eff_freq_hz,
                                   eff_duty_pct,
                                   15U,
                                   0U,
                                   0U,
                                   sym_count,
                                   true,
                                   ESP_OK);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public Runtime Setters & Getters
 * -------------------------------------------------------------------------- */

esp_err_t tool_wavegen_set_pin(gpio_num_t pin)
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
    if (!tool_wavegen_is_pin_supported(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    gpio_num_t prev_pin = s_cfg.pin;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    if (pin == prev_pin) {
        hw_unlock();
        return ESP_OK;
    }

    if (active) {
        esp_err_t err = bsp_pin_claim_ex(pin, WAVEGEN_TOOL_ID, true);
        if (err != ESP_OK) {
            hw_unlock();
            return err;
        }
        teardown_active_hw_locked();
        (void)bsp_pin_release(prev_pin, WAVEGEN_TOOL_ID);
    }

    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.pin = pin;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t apply_err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return apply_err;
}

gpio_num_t tool_wavegen_get_pin(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    gpio_num_t pin = s_cfg.pin;
    portEXIT_CRITICAL(&s_wave_lock);
    return pin;
}

esp_err_t tool_wavegen_set_mode(wavegen_mode_t mode)
{
    if (mode < 0 || mode >= WAVEGEN_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.mode = mode;
    if (mode == WAVEGEN_MODE_PWM_MCPWM) {
        if (s_cfg.freq_hz < WAVEGEN_MCPWM_MIN_FREQ_HZ) {
            s_cfg.freq_hz = WAVEGEN_MCPWM_MIN_FREQ_HZ;
        } else if (s_cfg.freq_hz > WAVEGEN_MCPWM_MAX_FREQ_HZ) {
            s_cfg.freq_hz = WAVEGEN_MCPWM_MAX_FREQ_HZ;
        }
    }
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

wavegen_mode_t tool_wavegen_get_mode(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    wavegen_mode_t mode = s_cfg.mode;
    portEXIT_CRITICAL(&s_wave_lock);
    return mode;
}

esp_err_t tool_wavegen_set_freq_hz(uint32_t freq_hz)
{
    portENTER_CRITICAL(&s_wave_lock);
    wavegen_mode_t mode = s_cfg.mode;
    portEXIT_CRITICAL(&s_wave_lock);

    uint32_t min_f = (mode == WAVEGEN_MODE_PWM_MCPWM) ? WAVEGEN_MCPWM_MIN_FREQ_HZ : WAVEGEN_LEDC_MIN_FREQ_HZ;
    uint32_t max_f = (mode == WAVEGEN_MODE_PWM_MCPWM) ? WAVEGEN_MCPWM_MAX_FREQ_HZ : WAVEGEN_LEDC_MAX_FREQ_HZ;
    if (freq_hz < min_f || freq_hz > max_f) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.freq_hz = freq_hz;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

uint32_t tool_wavegen_get_freq_hz(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    uint32_t f = s_cfg.freq_hz;
    portEXIT_CRITICAL(&s_wave_lock);
    return f;
}

esp_err_t tool_wavegen_set_duty_pct(float duty_pct)
{
    if (!isfinite(duty_pct) || duty_pct < 0.0f || duty_pct > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.duty_pct = duty_pct;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

float tool_wavegen_get_duty_pct(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    float d = s_cfg.duty_pct;
    portEXIT_CRITICAL(&s_wave_lock);
    return d;
}

esp_err_t tool_wavegen_set_servo_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < WAVEGEN_SERVO_MIN_US || pulse_us > WAVEGEN_SERVO_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.servo_pulse_us = pulse_us;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

uint16_t tool_wavegen_get_servo_pulse_us(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    uint16_t us = s_cfg.servo_pulse_us;
    portEXIT_CRITICAL(&s_wave_lock);
    return us;
}

esp_err_t tool_wavegen_set_burst_config(uint16_t high_us,
                                        uint16_t low_us,
                                        uint16_t pulse_count,
                                        uint16_t period_ms)
{
    if (high_us == 0U || high_us > WAVEGEN_RMT_MAX_DURATION_US ||
        low_us == 0U || low_us > WAVEGEN_RMT_MAX_DURATION_US ||
        pulse_count == 0U || pulse_count > WAVEGEN_MAX_BURST_PULSES ||
        period_ms < 5U || period_ms > 1000U) {
        return ESP_ERR_INVALID_ARG;
    }

    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.burst_high_us = high_us;
    s_cfg.burst_low_us = low_us;
    s_cfg.burst_pulse_count = pulse_count;
    s_cfg.burst_period_ms = period_ms;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

esp_err_t tool_wavegen_trigger_burst(void)
{
    hw_lock();
    esp_err_t err = transmit_rmt_frame_locked();
    portENTER_CRITICAL(&s_wave_lock);
    wavegen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_wave_lock);
    wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }
    hw_unlock();
    return err;
}

esp_err_t tool_wavegen_set_output_enabled(bool enabled)
{
    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.output_enabled = enabled;
    bool active = s_running_fg || s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = active ? apply_hw_config_locked() : ESP_OK;
    hw_unlock();
    return err;
}

bool tool_wavegen_is_output_enabled(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    bool en = s_cfg.output_enabled;
    portEXIT_CRITICAL(&s_wave_lock);
    return en;
}

esp_err_t tool_wavegen_set_keep_in_bg(bool keep_in_bg)
{
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.keep_in_bg = keep_in_bg;
    s_status.cfg.keep_in_bg = keep_in_bg;
    wavegen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_wave_lock);

    wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }
    return ESP_OK;
}

bool tool_wavegen_get_keep_in_bg(void)
{
    portENTER_CRITICAL(&s_wave_lock);
    bool keep = s_cfg.keep_in_bg;
    portEXIT_CRITICAL(&s_wave_lock);
    return keep;
}

esp_err_t tool_wavegen_get_status(wavegen_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_wave_lock);
    *out_status = s_status;
    portEXIT_CRITICAL(&s_wave_lock);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Core 1 Periodic RMT Frame / Status Worker
 * -------------------------------------------------------------------------- */

static esp_err_t wavegen_worker_step(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_wave_lock);
    bool stop_req = s_stop_requested;
    wavegen_config_t cfg = s_cfg;
    portEXIT_CRITICAL(&s_wave_lock);

    if (stop_req) {
        return ESP_OK;
    }

    if (xSemaphoreTake(s_hw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_OK;
    }

    if (cfg.output_enabled && s_active_hw == WAVEGEN_HW_RMT) {
        int64_t now_us = esp_timer_get_time();
        int64_t interval_us = (cfg.mode == WAVEGEN_MODE_SERVO)
                                  ? (int64_t)WAVEGEN_SERVO_PERIOD_US
                                  : ((int64_t)cfg.burst_period_ms * 1000LL);
        if (now_us - s_last_burst_tx_us >= interval_us) {
            (void)transmit_rmt_frame_locked();
        }
    }

    portENTER_CRITICAL(&s_wave_lock);
    s_status.running_fg = s_running_fg;
    s_status.running_bg = s_running_bg;
    wavegen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_wave_lock);

    wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }

    (void)xSemaphoreGive(s_hw_mutex);
    return ESP_OK;
}

esp_err_t tool_wavegen_run_single_step(void)
{
    ensure_wave_hw_mutex();
    hw_lock();
    if (s_active_hw == WAVEGEN_HW_RMT && s_cfg.output_enabled) {
        (void)transmit_rmt_frame_locked();
    }
    portENTER_CRITICAL(&s_wave_lock);
    s_status.running_fg = s_running_fg;
    s_status.running_bg = s_running_bg;
    wavegen_status_t snap = s_status;
    portEXIT_CRITICAL(&s_wave_lock);

    wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
    if (wptr != NULL) {
        *wptr = snap;
        (void)psram_double_buf_swap(&s_frame_dbuf);
    }
    hw_unlock();
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * LVGL 9 UI Controls & Waveform Preview (Core 0)
 * -------------------------------------------------------------------------- */

static void on_btn_out_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_wavegen_set_output_enabled(!tool_wavegen_is_output_enabled());
}

static void on_btn_mode_clicked(lv_event_t *e)
{
    (void)e;
    wavegen_mode_t next = (wavegen_mode_t)((tool_wavegen_get_mode() + 1) % WAVEGEN_MODE_COUNT);
    (void)tool_wavegen_set_mode(next);
}

static void on_btn_freq_clicked(lv_event_t *e)
{
    (void)e;
    wavegen_mode_t mode = tool_wavegen_get_mode();
    if (mode == WAVEGEN_MODE_SERVO) {
        uint16_t cur = tool_wavegen_get_servo_pulse_us();
        uint16_t next = (cur + 250U > WAVEGEN_SERVO_MAX_US) ? WAVEGEN_SERVO_MIN_US : (uint16_t)(cur + 250U);
        (void)tool_wavegen_set_servo_pulse_us(next);
        return;
    }
    if (mode == WAVEGEN_MODE_RMT_BURST) {
        wavegen_status_t st = {0};
        (void)tool_wavegen_get_status(&st);
        uint16_t next_cnt = (st.cfg.burst_pulse_count >= 16U) ? 2U : (uint16_t)(st.cfg.burst_pulse_count * 2U);
        (void)tool_wavegen_set_burst_config(
            st.cfg.burst_high_us, st.cfg.burst_low_us, next_cnt, st.cfg.burst_period_ms);
        return;
    }

    uint32_t cur_f = tool_wavegen_get_freq_hz();
    size_t next_idx = 0U;
    for (size_t i = 0; i < WAVEGEN_FREQ_PRESET_COUNT; ++i) {
        if (s_freq_presets_hz[i] > cur_f) {
            next_idx = i;
            break;
        }
    }
    uint32_t next_f = s_freq_presets_hz[next_idx];
    if (mode == WAVEGEN_MODE_PWM_MCPWM && next_f > WAVEGEN_MCPWM_MAX_FREQ_HZ) {
        next_f = WAVEGEN_MCPWM_MIN_FREQ_HZ;
    }
    (void)tool_wavegen_set_freq_hz(next_f);
}

static void on_btn_duty_clicked(lv_event_t *e)
{
    (void)e;
    wavegen_mode_t mode = tool_wavegen_get_mode();
    if (mode == WAVEGEN_MODE_RMT_BURST) {
        (void)tool_wavegen_trigger_burst();
        return;
    }
    if (mode == WAVEGEN_MODE_SERVO) {
        uint16_t cur = tool_wavegen_get_servo_pulse_us();
        uint16_t next = (cur == WAVEGEN_SERVO_DEFAULT_US) ? WAVEGEN_SERVO_MAX_US : WAVEGEN_SERVO_DEFAULT_US;
        (void)tool_wavegen_set_servo_pulse_us(next);
        return;
    }

    float cur_d = tool_wavegen_get_duty_pct();
    float next_d = cur_d + 12.5f;
    if (next_d > 90.0f) {
        next_d = 12.5f;
    }
    (void)tool_wavegen_set_duty_pct(next_d);
}

static void on_btn_pin_clicked(lv_event_t *e)
{
    (void)e;
    gpio_num_t cur = tool_wavegen_get_pin();
    size_t cur_idx = 0U;
    for (size_t i = 0; i < WAVEGEN_SUPPORTED_PIN_COUNT; ++i) {
        if (s_supported_pins[i] == cur) {
            cur_idx = i;
            break;
        }
    }
    for (size_t step = 1U; step <= WAVEGEN_SUPPORTED_PIN_COUNT; ++step) {
        gpio_num_t cand = s_supported_pins[(cur_idx + step) % WAVEGEN_SUPPORTED_PIN_COUNT];
        if (tool_wavegen_set_pin(cand) == ESP_OK) {
            break;
        }
    }
}

static void on_btn_bg_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_wavegen_set_keep_in_bg(!tool_wavegen_get_keep_in_bg());
}

static void build_wavegen_ui(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 2, 0);
    lv_obj_set_style_pad_row(parent, 2, 0);

    /* 1. Top Status & Background Mode Bar (18 px) */
    lv_obj_t *top_row = lv_obj_create(parent);
    lv_obj_set_size(top_row, 314, 18);
    lv_obj_set_style_bg_color(top_row, lv_color_hex(0x1A162B), 0);
    lv_obj_set_style_border_width(top_row, 0, 0);
    lv_obj_set_style_radius(top_row, 3, 0);
    lv_obj_set_style_pad_hor(top_row, 4, 0);
    lv_obj_set_style_pad_ver(top_row, 1, 0);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_lbl_top_status = lv_label_create(top_row);
    lv_label_set_text(s_lbl_top_status, "OUT: GPIO14  LEDC  SQUARE");
    lv_obj_set_style_text_font(s_lbl_top_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_top_status, lv_color_hex(0xB39DDB), 0);

    s_lbl_bg_badge = lv_label_create(top_row);
    lv_label_set_text(s_lbl_bg_badge, "BG:ON [RUN]");
    lv_obj_set_style_text_font(s_lbl_bg_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_bg_badge, lv_color_hex(0x00E676), 0);

    /* 2. Digital Waveform Preview Chart (314 x 96 px) */
    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 314, 96);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, WAVEGEN_CHART_POINTS);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, -200, 3600);
    lv_chart_set_div_line_count(s_chart, 3, 5);
    lv_obj_set_style_bg_color(s_chart, lv_color_hex(0x0A0E14), 0);
    lv_obj_set_style_border_color(s_chart, lv_color_hex(0x4527A0), 0);
    lv_obj_set_style_border_width(s_chart, 1, 0);
    lv_obj_set_style_radius(s_chart, 3, 0);
    lv_obj_set_style_pad_all(s_chart, 2, 0);
    lv_obj_set_style_size(s_chart, 0, 0, LV_PART_INDICATOR);

    s_ser_wave = lv_chart_add_series(s_chart, lv_color_hex(0x00E5FF), LV_CHART_AXIS_PRIMARY_Y);

    /* 3. Primary & Secondary Telemetry Readouts (2 x 18 px) */
    s_lbl_primary = lv_label_create(parent);
    lv_label_set_text(s_lbl_primary, "1000 Hz  Duty: 50.0%  (16-bit LEDC)");
    lv_obj_set_style_text_font(s_lbl_primary, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lbl_primary, lv_color_hex(0x00E676), 0);

    s_lbl_secondary = lv_label_create(parent);
    lv_label_set_text(s_lbl_secondary, "Pin: GPIO14 (Port C)  Keep in Background: ON");
    lv_obj_set_style_text_font(s_lbl_secondary, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_secondary, lv_color_hex(0xCFD8DC), 0);

    /* 4. Bottom Touch Controls Bar (28 px, 6 buttons) */
    lv_obj_t *ctrl_row = lv_obj_create(parent);
    lv_obj_set_size(ctrl_row, 314, 28);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_style_pad_column(ctrl_row, 3, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn_out = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_out, 46, 26);
    lv_obj_set_style_bg_color(btn_out, lv_color_hex(0x2E7D32), 0);
    lv_obj_add_event_cb(btn_out, on_btn_out_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_out_lbl = lv_label_create(btn_out);
    lv_label_set_text(s_btn_out_lbl, "ON");
    lv_obj_set_style_text_font(s_btn_out_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_out_lbl);

    lv_obj_t *btn_mode = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_mode, 56, 26);
    lv_obj_set_style_bg_color(btn_mode, lv_color_hex(0x4527A0), 0);
    lv_obj_add_event_cb(btn_mode, on_btn_mode_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_mode_lbl = lv_label_create(btn_mode);
    lv_label_set_text(s_btn_mode_lbl, "MODE");
    lv_obj_set_style_text_font(s_btn_mode_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_mode_lbl);

    lv_obj_t *btn_freq = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_freq, 54, 26);
    lv_obj_set_style_bg_color(btn_freq, lv_color_hex(0x00695C), 0);
    lv_obj_add_event_cb(btn_freq, on_btn_freq_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_freq_lbl = lv_label_create(btn_freq);
    lv_label_set_text(s_btn_freq_lbl, "FREQ");
    lv_obj_set_style_text_font(s_btn_freq_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_freq_lbl);

    lv_obj_t *btn_duty = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_duty, 52, 26);
    lv_obj_set_style_bg_color(btn_duty, lv_color_hex(0x0277BD), 0);
    lv_obj_add_event_cb(btn_duty, on_btn_duty_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_duty_lbl = lv_label_create(btn_duty);
    lv_label_set_text(s_btn_duty_lbl, "DUTY");
    lv_obj_set_style_text_font(s_btn_duty_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_duty_lbl);

    lv_obj_t *btn_pin = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_pin, 44, 26);
    lv_obj_set_style_bg_color(btn_pin, lv_color_hex(0x37474F), 0);
    lv_obj_add_event_cb(btn_pin, on_btn_pin_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_pin_lbl = lv_label_create(btn_pin);
    lv_label_set_text(s_btn_pin_lbl, "G14");
    lv_obj_set_style_text_font(s_btn_pin_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_pin_lbl);

    lv_obj_t *btn_bg = lv_button_create(ctrl_row);
    lv_obj_set_size(btn_bg, 46, 26);
    lv_obj_set_style_bg_color(btn_bg, lv_color_hex(0xE65100), 0);
    lv_obj_add_event_cb(btn_bg, on_btn_bg_clicked, LV_EVENT_CLICKED, NULL);
    s_btn_bg_lbl = lv_label_create(btn_bg);
    lv_label_set_text(s_btn_bg_lbl, "BG:1");
    lv_obj_set_style_text_font(s_btn_bg_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_btn_bg_lbl);
}

static void wavegen_on_ui_refresh(void *ctx)
{
    (void)ctx;
    static wavegen_status_t st;
    size_t copied = 0;
    if (psram_double_buf_read_copy(&s_frame_dbuf, &st, 1, &copied) != ESP_OK || copied == 0) {
        (void)tool_wavegen_get_status(&st);
    }

    if (s_lbl_top_status != NULL) {
        lv_label_set_text_fmt(s_lbl_top_status,
                              "OUT: GPIO%d  %s  %s",
                              (int)st.cfg.pin,
                              wavegen_periph_to_str(st.cfg.mode),
                              wavegen_mode_to_str(st.cfg.mode));
    }

    if (s_lbl_bg_badge != NULL) {
        if (st.last_error != ESP_OK) {
            lv_label_set_text(s_lbl_bg_badge, "[HW ERR]");
            lv_obj_set_style_text_color(s_lbl_bg_badge, lv_color_hex(0xFF5252), 0);
        } else {
            lv_label_set_text_fmt(s_lbl_bg_badge,
                                  "BG:%s [%s]",
                                  st.cfg.keep_in_bg ? "ON" : "OFF",
                                  st.cfg.output_enabled ? "RUN" : "IDLE");
            lv_obj_set_style_text_color(s_lbl_bg_badge,
                                        lv_color_hex(st.cfg.output_enabled ? 0x00E676 : 0xFFD54F),
                                        0);
        }
    }

    if (s_chart != NULL && s_ser_wave != NULL) {
        int32_t *pts = lv_chart_get_y_array(s_chart, s_ser_wave);
        if (pts != NULL) {
            (void)wavegen_build_preview_points(&st.cfg, pts, WAVEGEN_CHART_POINTS);
            lv_chart_refresh(s_chart);
        }
    }

    int duty_tenths = (int)lroundf(st.actual_duty_pct * 10.0f);
    if (s_lbl_primary != NULL) {
        if (st.cfg.mode == WAVEGEN_MODE_SERVO) {
            int angle_int = (int)lroundf(wavegen_calc_servo_us_to_angle(st.cfg.servo_pulse_us));
            lv_label_set_text_fmt(s_lbl_primary,
                                  "Servo: %u us (%d deg) @ 50 Hz",
                                  (unsigned)st.cfg.servo_pulse_us,
                                  angle_int);
        } else if (st.cfg.mode == WAVEGEN_MODE_RMT_BURST) {
            lv_label_set_text_fmt(s_lbl_primary,
                                  "Burst: %ux (%u/%u us) every %u ms",
                                  (unsigned)st.cfg.burst_pulse_count,
                                  (unsigned)st.cfg.burst_high_us,
                                  (unsigned)st.cfg.burst_low_us,
                                  (unsigned)st.cfg.burst_period_ms);
        } else {
            lv_label_set_text_fmt(s_lbl_primary,
                                  "%lu Hz  Duty: %d.%d%%  (%u-bit %s)",
                                  (unsigned long)st.actual_freq_hz,
                                  duty_tenths / 10,
                                  abs(duty_tenths % 10),
                                  (unsigned)st.resolution_bits,
                                  wavegen_periph_to_str(st.cfg.mode));
        }
    }

    if (s_lbl_secondary != NULL) {
        if (st.cfg.mode == WAVEGEN_MODE_RMT_BURST || st.cfg.mode == WAVEGEN_MODE_SERVO) {
            lv_label_set_text_fmt(s_lbl_secondary,
                                  "GPIO%d  Syms:%u  Sent:#%lu  BG:%s",
                                  (int)st.cfg.pin,
                                  (unsigned)st.rmt_symbol_count,
                                  (unsigned long)st.burst_frames_sent,
                                  st.cfg.keep_in_bg ? "Keep" : "Stop");
        } else {
            lv_label_set_text_fmt(s_lbl_secondary,
                                  "Target:%lu Hz  Ticks:%lu/%lu  BG:%s",
                                  (unsigned long)st.cfg.freq_hz,
                                  (unsigned long)st.compare_ticks,
                                  (unsigned long)st.period_ticks,
                                  st.cfg.keep_in_bg ? "Keep" : "Stop");
        }
    }

    if (s_btn_out_lbl != NULL) {
        lv_label_set_text(s_btn_out_lbl, st.cfg.output_enabled ? "ON" : "OFF");
    }
    if (s_btn_mode_lbl != NULL) {
        lv_label_set_text(s_btn_mode_lbl, wavegen_mode_to_str(st.cfg.mode));
    }
    if (s_btn_freq_lbl != NULL) {
        if (st.cfg.mode == WAVEGEN_MODE_SERVO) {
            lv_label_set_text(s_btn_freq_lbl, "PW+");
        } else if (st.cfg.mode == WAVEGEN_MODE_RMT_BURST) {
            lv_label_set_text(s_btn_freq_lbl, "N+");
        } else {
            lv_label_set_text(s_btn_freq_lbl, "FREQ");
        }
    }
    if (s_btn_duty_lbl != NULL) {
        if (st.cfg.mode == WAVEGEN_MODE_RMT_BURST) {
            lv_label_set_text(s_btn_duty_lbl, "TRIG");
        } else {
            lv_label_set_text(s_btn_duty_lbl, "DUTY");
        }
    }
    if (s_btn_pin_lbl != NULL) {
        lv_label_set_text_fmt(s_btn_pin_lbl, "G%d", (int)st.cfg.pin);
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
    s_lbl_bg_badge = NULL;
    s_chart = NULL;
    s_ser_wave = NULL;
    s_lbl_primary = NULL;
    s_lbl_secondary = NULL;
    s_btn_out_lbl = NULL;
    s_btn_mode_lbl = NULL;
    s_btn_freq_lbl = NULL;
    s_btn_duty_lbl = NULL;
    s_btn_pin_lbl = NULL;
    s_btn_bg_lbl = NULL;
}

static void free_wavegen_buffers(void)
{
    psram_double_buf_free(&s_frame_dbuf);
    if (s_psram_symbols != NULL) {
        psram_buf_free(s_psram_symbols);
        s_psram_symbols = NULL;
    }
}

static bool wavegen_is_running_bg(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_wave_lock);
    bool bg = s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);
    return bg;
}

static esp_err_t wavegen_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)ctx;
    hw_lock();

    portENTER_CRITICAL(&s_wave_lock);
    gpio_num_t pin = s_cfg.pin;
    bool was_bg = s_running_bg;
    portEXIT_CRITICAL(&s_wave_lock);

    esp_err_t err = bsp_pin_claim_ex(pin, WAVEGEN_TOOL_ID, true);
    if (err != ESP_OK) {
        hw_unlock();
        return err;
    }

    if (s_frame_dbuf.buf_a == NULL) {
        err = psram_double_buf_alloc(&s_frame_dbuf, 1, sizeof(wavegen_status_t));
        if (err != ESP_OK) {
            if (!was_bg) {
                (void)bsp_pin_release(pin, WAVEGEN_TOOL_ID);
            }
            hw_unlock();
            return err;
        }
    }

    if (s_psram_symbols == NULL) {
        s_psram_symbols = (rmt_symbol_word_t *)psram_buf_calloc(
            WAVEGEN_MAX_BURST_PULSES, sizeof(rmt_symbol_word_t));
        if (s_psram_symbols == NULL) {
            if (!was_bg) {
                free_wavegen_buffers();
                (void)bsp_pin_release(pin, WAVEGEN_TOOL_ID);
            }
            hw_unlock();
            return ESP_ERR_NO_MEM;
        }
    }

    portENTER_CRITICAL(&s_wave_lock);
    s_running_fg = true;
    s_running_bg = false;
    s_stop_requested = false;
    portEXIT_CRITICAL(&s_wave_lock);

    if (!was_bg || s_active_hw == WAVEGEN_HW_NONE) {
        err = apply_hw_config_locked();
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_wave_lock);
            s_running_fg = false;
            portEXIT_CRITICAL(&s_wave_lock);
            free_wavegen_buffers();
            (void)bsp_pin_release(pin, WAVEGEN_TOOL_ID);
            hw_unlock();
            return err;
        }
    } else {
        /* Refresh status snapshot when promoting from background to foreground */
        portENTER_CRITICAL(&s_wave_lock);
        s_status.running_fg = true;
        s_status.running_bg = false;
        wavegen_status_t snap = s_status;
        portEXIT_CRITICAL(&s_wave_lock);
        wavegen_status_t *wptr = (wavegen_status_t *)psram_double_buf_get_write_ptr(&s_frame_dbuf);
        if (wptr != NULL) {
            *wptr = snap;
            (void)psram_double_buf_swap(&s_frame_dbuf);
        }
    }

    if (parent_cont != NULL) {
        build_wavegen_ui(parent_cont);
    }

    if (s_worker == NULL) {
        tool_worker_config_t wcfg = {
            .name = "wavegen_c1",
            .stack_size_bytes = 4096,
            .priority = 5,
            .core_id = TOOL_WORKER_DEFAULT_CORE,
            .period_ms = WAVEGEN_WORKER_PERIOD_MS,
            .step_fn = wavegen_worker_step,
            .user_ctx = NULL,
        };
        err = tool_worker_start(&s_worker, &wcfg);
        if (err != ESP_OK) {
            teardown_active_hw_locked();
            portENTER_CRITICAL(&s_wave_lock);
            s_running_fg = false;
            portEXIT_CRITICAL(&s_wave_lock);
            free_wavegen_buffers();
            (void)bsp_pin_release(pin, WAVEGEN_TOOL_ID);
            hw_unlock();
            return err;
        }
    }

    hw_unlock();
    if (parent_cont != NULL) {
        wavegen_on_ui_refresh(NULL);
    }
    ESP_LOGI(TAG, "Wave Generator started on GPIO%d (mode=%s)",
             (int)pin, wavegen_mode_to_str(tool_wavegen_get_mode()));
    return ESP_OK;
}

static esp_err_t wavegen_on_stop(bool keep_background, void *ctx)
{
    (void)ctx;
    clear_ui_pointers();

    portENTER_CRITICAL(&s_wave_lock);
    bool retain_bg = keep_background && s_cfg.keep_in_bg && s_cfg.output_enabled;
    if (retain_bg) {
        s_running_fg = false;
        s_running_bg = true;
        s_status.running_fg = false;
        s_status.running_bg = true;
        portEXIT_CRITICAL(&s_wave_lock);
        ESP_LOGI(TAG, "Wave Generator retained in background on GPIO%d", (int)s_cfg.pin);
        return ESP_OK;
    }

    s_stop_requested = true;
    tool_worker_t *worker = s_worker;
    s_worker = NULL;
    portEXIT_CRITICAL(&s_wave_lock);

    if (worker != NULL) {
        esp_err_t stop_err = ESP_ERR_TIMEOUT;
        for (int attempt = 0; attempt < WAVEGEN_WORKER_STOP_RETRIES && stop_err == ESP_ERR_TIMEOUT; ++attempt) {
            stop_err = tool_worker_stop(worker, WAVEGEN_WORKER_STOP_WAIT_MS);
        }
        if (stop_err != ESP_OK) {
            portENTER_CRITICAL(&s_wave_lock);
            s_worker = worker;
            portEXIT_CRITICAL(&s_wave_lock);
            return stop_err;
        }
    }

    hw_lock();
    teardown_active_hw_locked();
    free_wavegen_buffers();

    portENTER_CRITICAL(&s_wave_lock);
    bool was_active = s_running_fg || s_running_bg;
    gpio_num_t pin = s_cfg.pin;
    s_running_fg = false;
    s_running_bg = false;
    s_status.running_fg = false;
    s_status.running_bg = false;
    s_status.hw_active = false;
    portEXIT_CRITICAL(&s_wave_lock);
    hw_unlock();

    if (was_active) {
        (void)bsp_pin_release(pin, WAVEGEN_TOOL_ID);
        ESP_LOGI(TAG, "Wave Generator stopped and released GPIO%d", (int)pin);
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Private Test Seams & Self-Test Registration
 * -------------------------------------------------------------------------- */

void tool_wavegen_reset_for_test(void)
{
    (void)wavegen_on_stop(false, NULL);
    hw_lock();
    portENTER_CRITICAL(&s_wave_lock);
    s_cfg.pin = WAVEGEN_DEFAULT_PIN;
    s_cfg.mode = WAVEGEN_MODE_SQUARE;
    s_cfg.freq_hz = WAVEGEN_DEFAULT_FREQ_HZ;
    s_cfg.duty_pct = WAVEGEN_DEFAULT_DUTY_PCT;
    s_cfg.servo_pulse_us = WAVEGEN_SERVO_DEFAULT_US;
    s_cfg.burst_high_us = WAVEGEN_DEFAULT_BURST_HIGH_US;
    s_cfg.burst_low_us = WAVEGEN_DEFAULT_BURST_LOW_US;
    s_cfg.burst_pulse_count = WAVEGEN_DEFAULT_BURST_PULSES;
    s_cfg.burst_period_ms = WAVEGEN_DEFAULT_BURST_REP_MS;
    s_cfg.output_enabled = true;
    s_cfg.keep_in_bg = true;
    memset(&s_status, 0, sizeof(s_status));
    s_status.cfg = s_cfg;
    s_forced_hw_err = ESP_OK;
    s_rmt_bounce_count = 0U;
    memset(s_rmt_bounce_symbols, 0, sizeof(s_rmt_bounce_symbols));
    portEXIT_CRITICAL(&s_wave_lock);
    hw_unlock();
}

void tool_wavegen_force_hw_error_for_test(esp_err_t err)
{
    portENTER_CRITICAL(&s_wave_lock);
    s_forced_hw_err = err;
    portEXIT_CRITICAL(&s_wave_lock);
}

size_t tool_wavegen_copy_last_rmt_symbols_for_test(rmt_symbol_word_t *out_symbols,
                                                   size_t max_symbols)
{
    hw_lock();
    size_t count = s_rmt_bounce_count;
    if (out_symbols != NULL && max_symbols > 0U) {
        if (count > max_symbols) {
            count = max_symbols;
        }
        memcpy(out_symbols, s_rmt_bounce_symbols, count * sizeof(rmt_symbol_word_t));
    }
    hw_unlock();
    return count;
}

static esp_err_t wavegen_selftest(void)
{
    wavegen_ledc_params_t lp = {0};
    if (wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 1000U, 25.0f, &lp) != ESP_OK) {
        return ESP_FAIL;
    }
    if (abs((int)lp.actual_freq_hz - 1000) > 2 || fabsf(lp.actual_duty_pct - 25.0f) > 0.2f) {
        return ESP_FAIL;
    }

    wavegen_mcpwm_params_t mp = {0};
    if (wavegen_calc_mcpwm_params(2000U, 50.0f, &mp) != ESP_OK) {
        return ESP_FAIL;
    }
    if (mp.actual_freq_hz != 2000U || mp.compare_ticks * 2U != mp.period_ticks) {
        return ESP_FAIL;
    }

    rmt_symbol_word_t syms[4] = {0};
    size_t sym_cnt = 0;
    if (wavegen_calc_servo_rmt_symbol(1500U, 20000U, syms, 4, &sym_cnt) != ESP_OK ||
        sym_cnt != 1U || syms[0].duration0 != 1500U || syms[0].duration1 != 18500U) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const analyser_tool_t s_wavegen_descriptor = {
    .id = WAVEGEN_TOOL_ID,
    .title = "Wave Generator",
    .symbol = LV_SYMBOL_REFRESH,
    .category = TOOL_CATEGORY_GENERATOR,
    .supports_background = true,
    .supports_recording = false,
    .supports_spk_passthrough = false,
    .on_init = NULL,
    .on_start = wavegen_on_start,
    .on_stop = wavegen_on_stop,
    .on_ui_refresh = wavegen_on_ui_refresh,
    .is_running_bg = wavegen_is_running_bg,
    .user_ctx = NULL,
};

const analyser_tool_t *tool_wavegen_get_descriptor(void)
{
    return &s_wavegen_descriptor;
}

esp_err_t tool_wavegen_register(void)
{
    esp_err_t err = tool_registry_register(&s_wavegen_descriptor);
    if (err == ESP_OK) {
        (void)tool_core_register_selftest("wavegen_calc", wavegen_selftest);
    }
    return err;
}
