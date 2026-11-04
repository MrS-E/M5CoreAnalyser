/**
 * @file test_tool_freq.c
 * @brief Unity unit & integration tests for `tool_freq`: RMT RX period/duty calculation,
 *        PCNT gate-time pulse counting, 5 kHz crossover hysteresis, zero-edge timeout,
 *        PSRAM session tracking, recorder sink export, and cooperative gate cancellation.
 */

#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_freq.h"
#include "tool_freq_priv.h"
#include "ui_shell.h"
#include "unity.h"

typedef struct {
    rmt_symbol_word_t symbols[64];
    size_t num_symbols;
    uint64_t pcnt_pulses;
    uint64_t actual_gate_us;
    freq_pcnt_edge_mode_t edge_mode;
} synth_freq_ctx_t;

typedef struct {
    gpio_num_t pin;
    uint32_t tick_idx;
    uint8_t cur_level;
} live_pwm_gen_ctx_t;

static void live_pwm_timer_cb(void *arg)
{
    live_pwm_gen_ctx_t *g = (live_pwm_gen_ctx_t *)arg;
    /* 500 us timer tick, 8 ticks per period = 4000 us (250 Hz), 2 ticks HIGH (1000 us = 25% duty) */
    uint32_t phase = g->tick_idx % 8U;
    uint8_t next_level = (phase < 2U) ? 1U : 0U;
    if (g->tick_idx == 0U || next_level != g->cur_level) {
        g->cur_level = next_level;
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        tool_freq_record_edge_isr_for_test(g->pin, now_us, next_level);
    }
    g->tick_idx++;
}

static esp_err_t synth_freq_source_cb(gpio_num_t pin,
                                      uint32_t gate_window_us,
                                      rmt_symbol_word_t *out_symbols,
                                      size_t max_symbols,
                                      size_t *out_num_symbols,
                                      uint64_t *out_pcnt_pulses,
                                      uint64_t *out_actual_gate_us,
                                      freq_pcnt_edge_mode_t *out_edge_mode,
                                      void *user_ctx)
{
    (void)pin;
    const synth_freq_ctx_t *ctx = (const synth_freq_ctx_t *)user_ctx;
    size_t n = (ctx->num_symbols > max_symbols) ? max_symbols : ctx->num_symbols;
    if (n > 0U && out_symbols != NULL) {
        memcpy(out_symbols, ctx->symbols, n * sizeof(rmt_symbol_word_t));
    }
    *out_num_symbols = n;
    *out_pcnt_pulses = ctx->pcnt_pulses;
    *out_actual_gate_us = (ctx->actual_gate_us > 0U) ? ctx->actual_gate_us : gate_window_us;
    *out_edge_mode = ctx->edge_mode;
    return ESP_OK;
}

TEST_CASE("given RMT RX symbols with servo PWM and glitch pulses when period is calculated then frequency duty pulse width and RPM are exact", "[tool_freq][period]")
{
    /* 1. 50 Hz servo signal (1500 us high, 18500 us low -> T = 20,000 us, f = 50.0 Hz, Duty = 7.5%, RPM = 3000) */
    rmt_symbol_word_t servo_syms[3];
    for (size_t i = 0; i < 3; ++i) {
        servo_syms[i].duration0 = 1500U;
        servo_syms[i].level0 = 1U;
        servo_syms[i].duration1 = 18500U;
        servo_syms[i].level1 = 0U;
    }

    freq_period_result_t res = {0};
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_rmt_symbols(servo_syms, 3, 1000000U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_EQUAL_UINT32(3U, res.cycles_measured);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 20000.0f, res.period_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1500.0f, res.pulse_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 18500.0f, res.low_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 7.5f, res.duty_pct);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 3000.0f, res.rpm);

    /* 2. Glitch filtering & same-level coalescing at 10 MHz (100 ns/tick):
     *    High 100 us (1000 ticks), glitch low 200 ns (2 ticks < 1000 ns), High 150 us (1500 ticks) -> coalesced 250 us high
     *    Low 750 us (7500 ticks) -> T = 1000 us (1 kHz), Duty = 25% */
    rmt_symbol_word_t glitch_syms[2] = {
        {.duration0 = 1000U, .level0 = 1U, .duration1 = 2U, .level1 = 0U},
        {.duration0 = 1500U, .level0 = 1U, .duration1 = 7500U, .level1 = 0U},
    };
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_rmt_symbols(glitch_syms, 2, 10000000U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 250.0f, res.pulse_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 750.0f, res.low_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 25.0f, res.duty_pct);

    /* 3. Zero-edge / idle timeout symbols -> valid == false, 0.0 Hz */
    rmt_symbol_word_t empty_sym = {.val = 0U};
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_rmt_symbols(&empty_sym, 1, 1000000U, 1000U, &res));
    TEST_ASSERT_FALSE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, res.freq_hz);

    /* 4. Continuous GPIO edge ISR timestamps: 1 kHz 25% duty (250 us high, 750 us low) across 4 cycles (9 edges)
     *    plus a 0 us glitch edge pair coalesced in a single stack-free pass */
    const freq_edge_stamp_t cont_edges[11] = {
        {.timestamp_us = 10000ULL, .level = 1U},
        {.timestamp_us = 10100ULL, .level = 0U}, /* 0 us glitch low (< 1 us) */
        {.timestamp_us = 10100ULL, .level = 1U}, /* coalesced back into high pulse -> 250 us total high */
        {.timestamp_us = 10250ULL, .level = 0U},
        {.timestamp_us = 11000ULL, .level = 1U},
        {.timestamp_us = 11250ULL, .level = 0U},
        {.timestamp_us = 12000ULL, .level = 1U},
        {.timestamp_us = 12250ULL, .level = 0U},
        {.timestamp_us = 13000ULL, .level = 1U},
        {.timestamp_us = 13250ULL, .level = 0U},
        {.timestamp_us = 14000ULL, .level = 1U},
    };
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_edge_timestamps(cont_edges, 11U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_EQUAL_UINT32(4U, res.cycles_measured);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, res.period_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 250.0f, res.pulse_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 750.0f, res.low_width_us);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 25.0f, res.duty_pct);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 60000.0f, res.rpm);

    /* 5. Fewer than 3 edges cannot bound a complete period -> valid == false */
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_edge_timestamps(cont_edges, 2U, 1000U, &res));
    TEST_ASSERT_FALSE(res.valid);
}

TEST_CASE("given PCNT gate pulse counts across 20ms 100ms and 1000ms windows when calculated then high frequency and zero-edge timeout are accurate", "[tool_freq][gate]")
{
    freq_gate_result_t gres = {0};

    /* 0. Verify default PCNT hardware glitch filter (25 ns = 2 APB cycles) permits multi-MHz (up to 20 MHz @ 50% duty) */
    TEST_ASSERT_EQUAL_UINT32(FREQ_PCNT_DEFAULT_GLITCH_NS, tool_freq_get_pcnt_glitch_ns());
    TEST_ASSERT_GREATER_OR_EQUAL(10000000.0f, freq_calc_pcnt_max_freq_hz(tool_freq_get_pcnt_glitch_ns()));
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 20000000.0f, freq_calc_pcnt_max_freq_hz(FREQ_PCNT_DEFAULT_GLITCH_NS));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pcnt_glitch_ns(0U));
    TEST_ASSERT_EQUAL_UINT32(0U, tool_freq_get_pcnt_glitch_ns());
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 40000000.0f, freq_calc_pcnt_max_freq_hz(0U));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_freq_set_pcnt_glitch_ns(FREQ_PCNT_MAX_GLITCH_NS + 1U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pcnt_glitch_ns(FREQ_PCNT_DEFAULT_GLITCH_NS));

    /* 1. 4.000 MHz signal over 100 ms gate (800,000 rising+falling edges in 100,000 us) */
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_gate_counts(800000ULL, 100000ULL, FREQ_PCNT_EDGES_BOTH, &gres));
    TEST_ASSERT_TRUE(gres.valid);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 4000000.0f, gres.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, gres.period_us);

    /* 2. 12.345 kHz signal over 1000 ms gate (12,345 rising edges in 1,000,000 us) */
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_gate_counts(12345ULL, 1000000ULL, FREQ_PCNT_EDGES_RISING_ONLY, &gres));
    TEST_ASSERT_TRUE(gres.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 12345.0f, gres.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 740700.0f, gres.rpm);

    /* 3. Zero-edge timeout -> valid == false, 0.0 Hz */
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_from_gate_counts(0ULL, 100000ULL, FREQ_PCNT_EDGES_BOTH, &gres));
    TEST_ASSERT_FALSE(gres.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gres.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gres.period_us);

    /* 4. Invalid gate_duration_us == 0 -> ESP_ERR_INVALID_ARG */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      freq_calc_from_gate_counts(100ULL, 0ULL, FREQ_PCNT_EDGES_BOTH, &gres));
}

TEST_CASE("given frequencies sweeping across 4.5kHz to 5.5kHz when AUTO mode selects method then hysteresis prevents chatter and tracker preserves stats on timeout", "[tool_freq][crossover]")
{
    freq_session_tracker_t tracker = {0};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_tracker_init(&tracker, 8U));

    freq_period_result_t pres = {.valid = true, .freq_hz = 1000.0f, .period_us = 1000.0f,
                                 .pulse_width_us = 300.0f, .low_width_us = 700.0f,
                                 .duty_pct = 30.0f, .rpm = 60000.0f, .cycles_measured = 10U};
    freq_gate_result_t gres = {.valid = true, .freq_hz = 1000.0f, .period_us = 1000.0f,
                               .rpm = 60000.0f, .pulse_count = 200ULL, .gate_duration_us = 100000ULL};
    freq_reading_t reading = {0};

    /* 1. At 1.0 kHz and 5.2 kHz (below 5.5 kHz upper threshold), stays in PERIOD */
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, FREQ_METHOD_NONE, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, reading.duty_pct);

    pres.freq_hz = 5200.0f;
    gres.freq_hz = 5200.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);

    /* 2. At 6.0 kHz (> 5.5 kHz), crosses over to GATE while retaining RMT duty cycle */
    pres.freq_hz = 6000.0f;
    pres.duty_pct = 40.0f;
    gres.freq_hz = 6000.0f;
    gres.period_us = 1000000.0f / 6000.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_GATE, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 6000.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, reading.duty_pct);

    /* 3. Dropping to 4.8 kHz (inside 4.5..5.5 kHz hysteresis band) remains in GATE */
    pres.freq_hz = 4800.0f;
    gres.freq_hz = 4800.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_GATE, reading.active_method);

    /* 4. Dropping to 4.2 kHz (< 4.5 kHz lower threshold) switches back to PERIOD */
    pres.freq_hz = 4200.0f;
    gres.freq_hz = 4200.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_TRUE(reading.duty_valid);

    /* 4b. When only gate counts exist (pres.valid == false), duty_valid is false and duty/pulse widths are NaN (never fabricated 50%) */
    pres.valid = false;
    gres.valid = true;
    gres.freq_hz = 250000.0f;
    gres.period_us = 4.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_EQUAL(FREQ_METHOD_GATE, reading.active_method);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_TRUE(isnan(reading.duty_pct));
    TEST_ASSERT_TRUE(isnan(reading.pulse_width_us));
    TEST_ASSERT_TRUE(isnan(reading.low_width_us));

    /* 5. Zero-edge timeout zeroes live reading while preserving session min/max/avg */
    pres.valid = false;
    gres.valid = false;
    TEST_ASSERT_EQUAL(ESP_OK,
                      freq_calc_combine_reading(FREQ_MODE_AUTO, reading.active_method, &pres, &gres, &tracker, &reading));
    TEST_ASSERT_FALSE(reading.signal_detected);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_TRUE(isnan(reading.duty_pct));
    TEST_ASSERT_EQUAL(FREQ_METHOD_NONE, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, reading.fmin_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 250000.0f, reading.fmax_hz);

    /* 6. Auto-ranging formatter */
    char buf[32];
    freq_calc_format_freq(0.0f, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("0.00 Hz", buf);
    freq_calc_format_freq(440.25f, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("440.25 Hz", buf);
    freq_calc_format_freq(12500.0f, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("12.500 kHz", buf);
    freq_calc_format_freq(2500000.0f, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("2.500 MHz", buf);

    freq_calc_tracker_free(&tracker);
}

TEST_CASE("given registered tool_freq when started configured and stepped then UI pin ownership and recorder export work", "[tool_freq][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_mode(FREQ_MODE_AUTO));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_gate_window(FREQ_GATE_20MS));

    synth_freq_ctx_t fctx = {
        .num_symbols = 2U,
        .pcnt_pulses = 40ULL,
        .actual_gate_us = 20000ULL,
        .edge_mode = FREQ_PCNT_EDGES_BOTH,
    };
    /* 1 kHz 25% duty (250 us high, 750 us low) across 64 symbols to exercise full bounce buffer in worker */
    fctx.num_symbols = 64U;
    for (size_t i = 0; i < 64U; ++i) {
        fctx.symbols[i].duration0 = 250U;
        fctx.symbols[i].level0 = 1U;
        fctx.symbols[i].duration1 = 750U;
        fctx.symbols[i].level1 = 0U;
    }
    tool_freq_set_capture_source(synth_freq_source_cb, &fctx);

    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_register());
    size_t passed = 0, failed = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_run_selftests(&passed, &failed));
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)failed);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, FREQ_TOOL_ID));

    /* Let the `freq_c1` worker task itself execute steps and verify stack high-water mark */
    for (int wait_iter = 0; wait_iter < 20 && tool_freq_get_worker_completed_steps() < 2U; ++wait_iter) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, tool_freq_get_worker_completed_steps());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3000U, tool_freq_get_worker_stack_hwm_bytes());

    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_run_single_step());
    ui_shell_step();

    freq_reading_t reading = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_TRUE(reading.signal_detected);
    TEST_ASSERT_TRUE(reading.duty_valid);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 25.0f, reading.duty_pct);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, rstats.frames_pushed);
    TEST_ASSERT_EQUAL_STRING(FREQ_TOOL_ID, rstats.last_tool_id);

    /* Switch to high-frequency 250 kHz PCNT gate signal (10,000 both-edges in 20,000 us) with no period edges */
    fctx.num_symbols = 0U;
    fctx.pcnt_pulses = 10000ULL;
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_run_single_step());
    ui_shell_step();
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_TRUE(reading.signal_detected);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_TRUE(isnan(reading.duty_pct));
    TEST_ASSERT_TRUE(isnan(reading.pulse_width_us));
    TEST_ASSERT_TRUE(isnan(reading.low_width_us));
    TEST_ASSERT_EQUAL(FREQ_METHOD_GATE, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 250000.0f, reading.freq_hz);

    /* Pin switch to GPIO13 and conflict rejection against a background generator on GPIO14 */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(BSP_PIN_PORT_C_TXD2, "wavegen"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_freq_set_pin(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, FREQ_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_PORT_C_RXD2));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_RXD2, FREQ_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, FREQ_TOOL_ID));
    (void)bsp_pin_release(BSP_PIN_PORT_C_TXD2, "wavegen");

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_RXD2, FREQ_TOOL_ID));

    tool_freq_set_capture_source(NULL, NULL);
    (void)tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT);
}

TEST_CASE("given real hardware capture path without source injection when continuous signal error injection and 1000ms gate stop run then worker captures accurately and cancels cleanly", "[tool_freq][hardware][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_freq_set_capture_source(NULL, NULL);
    tool_freq_set_driver_error_for_test(ESP_OK);

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_mode(FREQ_MODE_PERIOD_RMT));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_gate_window(FREQ_GATE_20MS));
    /* Keep the no-signal edge wait short here (default is 2 s) */
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_edge_acq_max_us(100000U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_register());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_run_single_step());

    freq_reading_t reading = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_FALSE(reading.signal_detected);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_EQUAL(ESP_OK, reading.hw_error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, reading.freq_hz);

    /* 1. Drive a continuous 250 Hz 25%-duty signal (1000 us high, 3000 us low) via esp_timer while `freq_c1` worker runs live */
    live_pwm_gen_ctx_t pwm_ctx = {
        .pin = BSP_PIN_PORT_C_TXD2,
        .tick_idx = 0U,
        .cur_level = 0U,
    };
    const esp_timer_create_args_t targs = {
        .callback = live_pwm_timer_cb,
        .arg = &pwm_ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test_pwm_250hz",
    };
    esp_timer_handle_t pwm_timer = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&targs, &pwm_timer));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_periodic(pwm_timer, 500ULL));

    /* Wait for `freq_c1` worker itself to complete at least 2 live capture windows on the continuous signal */
    uint32_t base_steps = tool_freq_get_worker_completed_steps();
    bool got_live_reading = false;
    for (int i = 0; i < 30; ++i) {
        vTaskDelay(pdMS_TO_TICKS(15));
        (void)tool_freq_get_latest_reading(&reading);
        if (tool_freq_get_worker_completed_steps() >= base_steps + 2U && reading.signal_detected) {
            got_live_reading = true;
            break;
        }
    }
    (void)esp_timer_stop(pwm_timer);
    (void)esp_timer_delete(pwm_timer);

    ui_shell_step();
    TEST_ASSERT_TRUE(got_live_reading);
    TEST_ASSERT_TRUE(reading.signal_detected);
    TEST_ASSERT_TRUE(reading.duty_valid);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 250.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(200.0f, 4000.0f, reading.period_us);
    TEST_ASSERT_FLOAT_WITHIN(150.0f, 1000.0f, reading.pulse_width_us);
    TEST_ASSERT_FLOAT_WITHIN(200.0f, 3000.0f, reading.low_width_us);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 25.0f, reading.duty_pct);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3000U, tool_freq_get_worker_stack_hwm_bytes());

    /* 2. Verify peripheral driver error propagation and recovery */
    tool_freq_set_driver_error_for_test(ESP_ERR_NOT_FOUND);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, tool_freq_run_single_step());
    ui_shell_step();
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_FALSE(reading.signal_detected);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, reading.hw_error);

    tool_freq_set_driver_error_for_test(ESP_OK);
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_EQUAL(ESP_OK, reading.hw_error);

    /* 3. Switch to 1000 ms gate window, let worker enter gate wait, then stop immediately */
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_mode(FREQ_MODE_AUTO));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_gate_window(FREQ_GATE_1000MS));
    vTaskDelay(pdMS_TO_TICKS(25));

    int64_t stop_start_us = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    int64_t stop_elapsed_us = esp_timer_get_time() - stop_start_us;
    TEST_ASSERT_LESS_THAN_INT32(250000, (int32_t)stop_elapsed_us);
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));

    (void)tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT);
    (void)tool_freq_set_gate_window(FREQ_GATE_100MS);
    (void)tool_freq_set_edge_acq_max_us(FREQ_EDGE_ACQ_DEFAULT_MAX_US);
}

TEST_CASE("given gate counts of varying size when cross-check tolerance and verdict are evaluated then tolerance scales with 1/N and low counts are inconclusive", "[tool_freq][cross_check]")
{
    /* tolerance = max(5%, 3/N) */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, freq_calc_cross_check_tolerance(0ULL));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, freq_calc_cross_check_tolerance(1000ULL));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, freq_calc_cross_check_tolerance(60ULL));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.15f, freq_calc_cross_check_tolerance(20ULL));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.30f, freq_calc_cross_check_tolerance(10ULL));

    freq_period_result_t p = {.valid = true, .freq_hz = 100.0f};
    freq_gate_result_t g = {.valid = true, .freq_hz = 100.0f, .pulse_count = 200ULL};
    float rel = -1.0f;

    /* Equal -> OK with zero difference */
    TEST_ASSERT_EQUAL(FREQ_XCHECK_OK, freq_calc_cross_check(&p, &g, &rel));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, rel);

    /* N=200: 3.8% is inside 5%, 9.1% is outside */
    g.freq_hz = 104.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_OK, freq_calc_cross_check(&p, &g, &rel));
    g.freq_hz = 110.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_MISMATCH, freq_calc_cross_check(&p, &g, &rel));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0909f, rel);

    /* N=10: +-1 count is +-10%, so 100 vs 120 Hz (16.7%) is a correct reading, 100 vs 150 (33%) is not */
    g.pulse_count = 10ULL;
    g.freq_hz = 120.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_OK, freq_calc_cross_check(&p, &g, &rel));
    g.freq_hz = 150.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_MISMATCH, freq_calc_cross_check(&p, &g, &rel));

    /* N below the statistical minimum -> skipped even when frequencies differ wildly */
    g.pulse_count = FREQ_XCHECK_MIN_COUNTS - 1U;
    g.freq_hz = 1000.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(&p, &g, &rel));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, rel);

    /* Invalid inputs -> skipped (never a 100% false mismatch) */
    g.pulse_count = 200ULL;
    g.freq_hz = 100.0f;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(NULL, &g, &rel));
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(&p, NULL, &rel));
    p.valid = false;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(&p, &g, NULL));
    p.valid = true;
    g.valid = false;
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(&p, &g, NULL));
    g.valid = true;
    p.freq_hz = 0.0f; /* gate-only style result: period frequency 0 */
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, freq_calc_cross_check(&p, &g, NULL));
}

TEST_CASE("given edge stream with two lost edges when combined with gate counts then gate frequency replaces the wrong 1/T value and duty is unknown", "[tool_freq][cross_check]")
{
    /* 1 kHz 50% duty: edge i at i*500 us, level 1 on even i. */
    freq_edge_stamp_t good[12];
    for (size_t i = 0; i < 12U; ++i) {
        good[i].timestamp_us = (uint64_t)i * 500ULL;
        good[i].level = (uint8_t)((i % 2U) == 0U ? 1U : 0U);
    }
    /* Same stream with edges 4 and 5 lost (merged 1500 us low pulse) */
    freq_edge_stamp_t lost[10];
    size_t n = 0;
    for (size_t i = 0; i < 12U; ++i) {
        if (i == 4U || i == 5U) {
            continue;
        }
        lost[n++] = good[i];
    }
    TEST_ASSERT_EQUAL_UINT32(10U, (uint32_t)n);

    freq_gate_result_t gres = {0};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_gate_counts(200ULL, 100000ULL, FREQ_PCNT_EDGES_BOTH, &gres));
    TEST_ASSERT_TRUE(gres.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, gres.freq_hz);

    freq_period_result_t pres = {0};
    freq_reading_t reading = {0};

    /* No lost edges: estimators agree, duty stays valid and the reading is reliable */
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(good, 12U, 1000U, &pres));
    TEST_ASSERT_TRUE(pres.valid);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, pres.freq_hz);
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_combine_reading(FREQ_MODE_AUTO, FREQ_METHOD_NONE, &pres, &gres, NULL, &reading));
    TEST_ASSERT_EQUAL(FREQ_XCHECK_OK, reading.xcheck);
    TEST_ASSERT_FALSE(reading.unreliable);
    TEST_ASSERT_TRUE(reading.duty_valid);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, reading.duty_pct);

    /* Lost edges: 1/T reads ~800 Hz, gate says 1000 Hz (20% off, tolerance 5%) -> gate wins */
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(lost, n, 1000U, &pres));
    TEST_ASSERT_TRUE(pres.valid);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 800.0f, pres.freq_hz);
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_combine_reading(FREQ_MODE_AUTO, FREQ_METHOD_NONE, &pres, &gres, NULL, &reading));
    TEST_ASSERT_EQUAL(FREQ_XCHECK_MISMATCH, reading.xcheck);
    TEST_ASSERT_TRUE(reading.unreliable);
    TEST_ASSERT_TRUE(reading.signal_detected);
    TEST_ASSERT_EQUAL(FREQ_METHOD_GATE, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, reading.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, reading.period_us);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 60000.0f, reading.rpm);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_TRUE(isnan(reading.duty_pct));
    TEST_ASSERT_TRUE(isnan(reading.pulse_width_us));
    TEST_ASSERT_TRUE(isnan(reading.low_width_us));

    /* Forced 1/T mode keeps the user's method but is explicitly flagged unreliable with unknown duty */
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_combine_reading(FREQ_MODE_PERIOD_RMT, FREQ_METHOD_NONE, &pres, &gres, NULL, &reading));
    TEST_ASSERT_TRUE(reading.unreliable);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 800.0f, reading.freq_hz);
    TEST_ASSERT_FALSE(reading.duty_valid);
    TEST_ASSERT_TRUE(isnan(reading.duty_pct));

    /* Low-count boundary: 5 Hz over a 1 s gate gives N=10 edges; a +-1 count quantised gate (5.5 Hz)
     * against a correct 5.0 Hz period stays reliable (tolerance 30%), and a missing gate result is skipped */
    freq_period_result_t p5 = {.valid = true, .freq_hz = 5.0f, .period_us = 200000.0f,
                               .pulse_width_us = 100000.0f, .low_width_us = 100000.0f, .duty_pct = 50.0f};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_gate_counts(11ULL, 1000000ULL, FREQ_PCNT_EDGES_BOTH, &gres));
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_combine_reading(FREQ_MODE_AUTO, FREQ_METHOD_NONE, &p5, &gres, NULL, &reading));
    TEST_ASSERT_EQUAL(FREQ_XCHECK_OK, reading.xcheck);
    TEST_ASSERT_FALSE(reading.unreliable);
    TEST_ASSERT_TRUE(reading.duty_valid);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_combine_reading(FREQ_MODE_AUTO, FREQ_METHOD_NONE, &p5, NULL, NULL, &reading));
    TEST_ASSERT_EQUAL(FREQ_XCHECK_SKIPPED, reading.xcheck);
    TEST_ASSERT_TRUE(reading.duty_valid);
}

TEST_CASE("given 1Hz 2Hz and 5Hz edge streams and narrow slow pulses when period is calculated from timestamps then full-period frequency and duty are exact", "[tool_freq][period][lowfreq]")
{
    freq_period_result_t res = {0};

    /* 1 Hz 50%: the minimum useful stream in a 2 s window is 4 edges (3 pulses = 1 full cycle) */
    const freq_edge_stamp_t hz1[] = {{0ULL, 1U}, {500000ULL, 0U}, {1000000ULL, 1U}, {1500000ULL, 0U}};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(hz1, 4U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, res.duty_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 60.0f, res.rpm);

    /* Exactly 3 edges (high, low, then the next rising edge) bound one full period */
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(hz1, 3U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_EQUAL_UINT32(1U, res.cycles_measured);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, res.freq_hz);

    /* 2 Hz 50% with 5 edges (2 full cycles) */
    freq_edge_stamp_t hz2[5];
    for (size_t i = 0; i < 5U; ++i) {
        hz2[i].timestamp_us = (uint64_t)i * 250000ULL;
        hz2[i].level = (uint8_t)((i % 2U) == 0U ? 1U : 0U);
    }
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(hz2, 5U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_EQUAL_UINT32(2U, res.cycles_measured);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, res.freq_hz);

    /* 5 Hz 25% duty, 5 edges */
    const freq_edge_stamp_t hz5[] = {{0ULL, 1U}, {50000ULL, 0U}, {200000ULL, 1U}, {250000ULL, 0U}, {400000ULL, 1U}};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(hz5, 5U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, res.duty_pct);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 50000.0f, res.pulse_width_us);

    /* 1 Hz tach pulse, 10 ms wide: rise, fall, next rise (3 edges) is enough for 1 Hz and 1% duty */
    const freq_edge_stamp_t narrow[] = {{0ULL, 1U}, {10000ULL, 0U}, {1000000ULL, 1U}};
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(narrow, 3U, 1000U, &res));
    TEST_ASSERT_TRUE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, res.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, res.duty_pct);

    /* Only two edges (less than one full period) -> not valid, never a stale or invented value */
    TEST_ASSERT_EQUAL(ESP_OK, freq_calc_from_edge_timestamps(narrow, 2U, 1000U, &res));
    TEST_ASSERT_FALSE(res.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, res.freq_hz);
}

typedef struct {
    gpio_num_t pin;
    uint32_t tick_idx;
    uint32_t period_ticks;
    uint32_t high_ticks;
    uint8_t cur_level;
} slow_pwm_gen_ctx_t;

static void slow_pwm_timer_cb(void *arg)
{
    slow_pwm_gen_ctx_t *g = (slow_pwm_gen_ctx_t *)arg;
    uint32_t phase = g->tick_idx % g->period_ticks;
    uint8_t next_level = (phase < g->high_ticks) ? 1U : 0U;
    if (g->tick_idx == 0U || next_level != g->cur_level) {
        g->cur_level = next_level;
        tool_freq_record_edge_isr_for_test(g->pin, (uint64_t)esp_timer_get_time(), next_level);
    }
    g->tick_idx++;
}

#define SLOW_TICK_US (5000ULL)

static void freq_test_prepare_live_tool(freq_gate_window_t gate)
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_freq_set_capture_source(NULL, NULL);
    tool_freq_set_driver_error_for_test(ESP_OK);
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_mode(FREQ_MODE_AUTO));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_gate_window(gate));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_edge_acq_max_us(FREQ_EDGE_ACQ_DEFAULT_MAX_US));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_register());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
}

static void freq_test_finish_live_tool(void)
{
    (void)ui_shell_go_home(false);
    (void)tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT);
    (void)tool_freq_set_gate_window(FREQ_GATE_100MS);
    (void)tool_freq_set_mode(FREQ_MODE_AUTO);
    (void)tool_freq_set_edge_acq_max_us(FREQ_EDGE_ACQ_DEFAULT_MAX_US);
}

/* Drive a slow periodic signal into the live `freq_c1` worker (100 ms PCNT gate, 2 s edge window). */
static void run_slow_signal_case(uint32_t period_ticks,
                                 uint32_t high_ticks,
                                 float expect_hz,
                                 float expect_duty_pct,
                                 float duty_tol_pct,
                                 uint32_t min_acq_us,
                                 uint32_t timeout_ms)
{
    freq_test_prepare_live_tool(FREQ_GATE_100MS);

    slow_pwm_gen_ctx_t ctx = {
        .pin = BSP_PIN_PORT_C_TXD2,
        .tick_idx = 0U,
        .period_ticks = period_ticks,
        .high_ticks = high_ticks,
        .cur_level = 0U,
    };
    const esp_timer_create_args_t targs = {
        .callback = slow_pwm_timer_cb,
        .arg = &ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test_slow_pwm",
    };
    esp_timer_handle_t timer = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&targs, &timer));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_periodic(timer, SLOW_TICK_US));

    freq_reading_t reading = {0};
    bool got = false;
    for (uint32_t waited = 0; waited < timeout_ms; waited += 20U) {
        vTaskDelay(pdMS_TO_TICKS(20));
        (void)tool_freq_get_latest_reading(&reading);
        if (reading.signal_detected) {
            got = true;
            break;
        }
    }
    freq_window_stats_t stats = {0};
    tool_freq_get_last_window_stats(&stats);
    (void)esp_timer_stop(timer);
    (void)esp_timer_delete(timer);
    ui_shell_step();

    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL(ESP_OK, reading.hw_error);
    TEST_ASSERT_FALSE(reading.unreliable);
    TEST_ASSERT_EQUAL(FREQ_METHOD_PERIOD, reading.active_method);
    TEST_ASSERT_FLOAT_WITHIN(expect_hz * 0.15f, expect_hz, reading.freq_hz);
    TEST_ASSERT_TRUE(reading.duty_valid);
    TEST_ASSERT_FLOAT_WITHIN(duty_tol_pct, expect_duty_pct, reading.duty_pct);
    /* Edge acquisition ran past the 100 ms gate, but the PCNT gate itself was not stretched */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(min_acq_us, stats.acq_us);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(FREQ_EDGE_ACQ_DEFAULT_MAX_US + 200000U, stats.acq_us);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT64(90000ULL, reading.gate_duration_us);
    TEST_ASSERT_LESS_OR_EQUAL_UINT64(160000ULL, reading.gate_duration_us);
    TEST_ASSERT_FALSE(stats.isr_capped);

    freq_test_finish_live_tool();
}

TEST_CASE("given live 5Hz 50 percent signal when worker acquires edges beyond the 100ms gate then frequency and duty are measured", "[tool_freq][hardware][lowfreq]")
{
    run_slow_signal_case(40U, 20U, 5.0f, 50.0f, 6.0f, 380000U, 6000U);
}

TEST_CASE("given live 2Hz 50 percent signal when worker acquires edges beyond the 100ms gate then frequency and duty are measured", "[tool_freq][hardware][lowfreq]")
{
    run_slow_signal_case(100U, 50U, 2.0f, 50.0f, 6.0f, 950000U, 8000U);
}

TEST_CASE("given live 1Hz 50 percent signal when worker acquires edges for the bounded 2s window then frequency and duty are measured", "[tool_freq][hardware][lowfreq]")
{
    run_slow_signal_case(200U, 100U, 1.0f, 50.0f, 6.0f, 1900000U, 10000U);
}

TEST_CASE("given live 1Hz tach pulse only 10ms wide when worker acquires edges then frequency and narrow duty are measured", "[tool_freq][hardware][lowfreq]")
{
    run_slow_signal_case(200U, 2U, 1.0f, 1.0f, 1.5f, 1900000U, 12000U);
}

static void same_level_flood_cb(void *arg)
{
    gpio_num_t pin = *(const gpio_num_t *)arg;
    /* Every sample has the same level: only the first is stored, the rest are dropped by the ISR */
    tool_freq_record_edge_isr_for_test(pin, (uint64_t)esp_timer_get_time(), 1U);
}

TEST_CASE("given aliased same-level interrupt flood when edge window runs then ISR invocation cap disables the interrupt even though samples are dropped", "[tool_freq][hardware][isr_cap]")
{
    freq_test_prepare_live_tool(FREQ_GATE_20MS);
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_mode(FREQ_MODE_PERIOD_RMT));

    gpio_num_t pin = BSP_PIN_PORT_C_TXD2;
    const esp_timer_create_args_t targs = {
        .callback = same_level_flood_cb,
        .arg = &pin,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "test_flood",
    };
    esp_timer_handle_t timer = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&targs, &timer));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_periodic(timer, 200ULL));

    freq_window_stats_t stats = {0};
    for (int i = 0; i < 100; ++i) {
        vTaskDelay(pdMS_TO_TICKS(20));
        tool_freq_get_last_window_stats(&stats);
        if (stats.isr_capped) {
            break;
        }
    }
    (void)esp_timer_stop(timer);
    (void)esp_timer_delete(timer);

    TEST_ASSERT_TRUE(stats.isr_capped);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(FREQ_EDGE_ISR_MAX_INVOCATIONS, stats.isr_invocations);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(FREQ_EDGE_ISR_MAX_INVOCATIONS + 8U, stats.isr_invocations);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.edges_stored); /* proves invocations were counted although samples were dropped */
    /* The cap ends the window instead of waiting out the 2 s edge acquisition */
    TEST_ASSERT_LESS_THAN_UINT32(1500000U, stats.acq_us);

    freq_reading_t reading = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_get_latest_reading(&reading));
    TEST_ASSERT_FALSE(reading.signal_detected);

    freq_test_finish_live_tool();
}

TEST_CASE("given no signal during the long edge acquisition window when tool is stopped then worker cancels within 250ms and joins before buffers are freed", "[tool_freq][hardware][cancel]")
{
    freq_test_prepare_live_tool(FREQ_GATE_20MS);
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));

    /* Past the 20 ms gate the worker sits in the 2 s edge wait: no window has completed yet */
    vTaskDelay(pdMS_TO_TICKS(150));
    freq_window_stats_t stats = {0};
    tool_freq_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.windows);

    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    int64_t elapsed_us = esp_timer_get_time() - t0;
    TEST_ASSERT_LESS_THAN_INT32(250000, (int32_t)elapsed_us);
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));

    freq_test_finish_live_tool();
}
