/**
 * @file test_tool_scope.c
 * @brief Unity unit & integration tests for `tool_scope`: rising/falling/auto trigger search
 *        with hysteresis, 300-point decimated window extraction, waveform metrics (`Vpp`, `Vmin`,
 *        `Vmax`, `Vavg`, `Vrms`, `Freq`, `Duty`), rail/flat-line safety, and tool lifecycle.
 */

#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_scope.h"
#include "tool_scope_priv.h"
#include "ui_shell.h"
#include "unity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static esp_err_t mock_scope_i2s_init(uint32_t rate)
{
    (void)rate;
    return ESP_OK;
}

static esp_err_t mock_scope_i2s_write(const int16_t *pcm,
                                      size_t len,
                                      size_t *out_written,
                                      uint32_t timeout_ms)
{
    (void)pcm;
    (void)timeout_ms;
    if (out_written != NULL) {
        *out_written = len;
    }
    return ESP_OK;
}

static esp_err_t mock_scope_i2s_disable(void)
{
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_mock_scope_i2s_ops = {
    .ensure_init = mock_scope_i2s_init,
    .write_pcm = mock_scope_i2s_write,
    .disable = mock_scope_i2s_disable,
};

typedef struct {
    uint16_t low_mv;
    uint16_t high_mv;
    uint32_t period_samples;
    uint32_t high_samples;
    uint32_t sample_rate_hz;
} synth_pwm_ctx_t;

static esp_err_t synth_pwm_source_cb(gpio_num_t pin,
                                     uint16_t *out_raw_adc,
                                     uint16_t *out_mv,
                                     size_t count,
                                     uint32_t *out_sample_rate_hz,
                                     void *user_ctx)
{
    (void)pin;
    const synth_pwm_ctx_t *p = (const synth_pwm_ctx_t *)user_ctx;
    *out_sample_rate_hz = p->sample_rate_hz;
    for (size_t i = 0; i < count; ++i) {
        uint16_t mv = ((i % p->period_samples) < p->high_samples) ? p->high_mv : p->low_mv;
        out_mv[i] = mv;
        out_raw_adc[i] = (uint16_t)(((uint32_t)mv * 4095U) / 3300U);
    }
    return ESP_OK;
}

TEST_CASE("given rising falling and flat-line waveforms when searched for trigger with hysteresis then crossings and auto fallback are exact", "[tool_scope][trigger]")
{
    uint16_t buf[64];
    for (size_t i = 0; i < 64; ++i) {
        buf[i] = 1480U; /* Within hysteresis band of 1500 +/- 50 mV (not armed yet) */
    }
    /* Small wiggle across 1500 mV at index 10 without ever dropping <= 1450 mV must NOT trigger */
    buf[9] = 1485U;
    buf[10] = 1520U;
    buf[11] = 1490U;

    /* True arming drop to 1200 mV at index 20..24, then rising crossing >= 1500 mV at index 25 */
    for (size_t i = 20; i < 25; ++i) {
        buf[i] = 1200U;
    }
    for (size_t i = 25; i < 40; ++i) {
        buf[i] = 2000U;
    }
    /* Falling crossing <= 1500 mV at index 40 after being armed >= 1550 mV */
    for (size_t i = 40; i < 64; ++i) {
        buf[i] = 900U;
    }

    scope_trig_result_t res = {0};
    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_find_trigger(buf, 64, SCOPE_TRIG_RISING, 1500U, 50U, 4U, &res));
    TEST_ASSERT_TRUE(res.triggered);
    TEST_ASSERT_FALSE(res.auto_fallback);
    TEST_ASSERT_EQUAL_UINT32(25U, (uint32_t)res.trigger_index);

    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_find_trigger(buf, 64, SCOPE_TRIG_FALLING, 1500U, 50U, 4U, &res));
    TEST_ASSERT_TRUE(res.triggered);
    TEST_ASSERT_FALSE(res.auto_fallback);
    TEST_ASSERT_EQUAL_UINT32(40U, (uint32_t)res.trigger_index);

    /* Flat DC line at 1650 mV: AUTO falls back to centered trace; RISING reports untriggered */
    uint16_t flat[64];
    for (size_t i = 0; i < 64; ++i) {
        flat[i] = 1650U;
    }

    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_find_trigger(flat, 64, SCOPE_TRIG_AUTO, 1500U, 50U, 4U, &res));
    TEST_ASSERT_FALSE(res.triggered);
    TEST_ASSERT_TRUE(res.auto_fallback);
    TEST_ASSERT_EQUAL_UINT32(32U, (uint32_t)res.trigger_index);

    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_find_trigger(flat, 64, SCOPE_TRIG_RISING, 1500U, 50U, 4U, &res));
    TEST_ASSERT_FALSE(res.triggered);
    TEST_ASSERT_FALSE(res.auto_fallback);
}

TEST_CASE("given capture buffer and trigger index when decimated window is extracted then center alignment and boundary clamping hold", "[tool_scope][window]")
{
    uint16_t src[1000];
    for (size_t i = 0; i < 1000; ++i) {
        src[i] = (uint16_t)i;
    }

    uint16_t win[SCOPE_CHART_POINTS] = {0};
    /* Step = 2, centered at index 500 -> midpoint win[150] == 500, win[0] == 200, win[299] == 798 */
    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_extract_window(src, 1000, 500, 2, win, SCOPE_CHART_POINTS));
    TEST_ASSERT_EQUAL_UINT16(500U, win[SCOPE_CHART_POINTS / 2U]);
    TEST_ASSERT_EQUAL_UINT16(200U, win[0]);
    TEST_ASSERT_EQUAL_UINT16(798U, win[SCOPE_CHART_POINTS - 1U]);

    /* Clamping at left & right boundaries */
    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_extract_window(src, 1000, 5, 4, win, SCOPE_CHART_POINTS));
    TEST_ASSERT_EQUAL_UINT16(0U, win[0]);
    TEST_ASSERT_EQUAL_UINT16(5U, win[SCOPE_CHART_POINTS / 2U]);

    TEST_ASSERT_EQUAL(ESP_OK,
                      scope_dsp_extract_window(src, 1000, 995, 4, win, SCOPE_CHART_POINTS));
    TEST_ASSERT_EQUAL_UINT16(995U, win[SCOPE_CHART_POINTS / 2U]);
    TEST_ASSERT_EQUAL_UINT16(999U, win[SCOPE_CHART_POINTS - 1U]);
}

TEST_CASE("given each requested timebase when sample scheduling and viewport math are calculated then ten divisions are represented", "[tool_scope][timing]")
{
    for (scope_timebase_t tb = SCOPE_TB_100US_DIV; tb < SCOPE_TB_COUNT; ++tb) {
        uint32_t interval_ns = scope_dsp_target_sample_interval_ns(tb);
        TEST_ASSERT_GREATER_THAN_UINT32(0U, interval_ns);

        uint32_t target_rate_hz = (uint32_t)((1000000000ULL + interval_ns / 2U) / interval_ns);
        uint32_t viewport_us = scope_dsp_viewport_us(target_rate_hz, 1U, SCOPE_CHART_POINTS);
        uint32_t expected_viewport_us = scope_dsp_timebase_us_per_div(tb) * SCOPE_HORIZ_DIVS;
        TEST_ASSERT_UINT32_WITHIN(expected_viewport_us / 100U, expected_viewport_us, viewport_us);
    }

    TEST_ASSERT_EQUAL_UINT32(0U, scope_dsp_target_sample_interval_ns((scope_timebase_t)SCOPE_TB_COUNT));
    TEST_ASSERT_EQUAL_UINT32(0U, scope_dsp_viewport_us(0U, 1U, SCOPE_CHART_POINTS));
}

TEST_CASE("given PWM sine and 0V 3.3V rail buffers when metrics are computed then Vpp Vavg Vrms Freq and Duty are accurate", "[tool_scope][metrics]")
{
    /* 1. 1 kHz 25%-duty PWM wave sampled at 32 kHz (period = 32 samples, high = 8 samples) */
    uint16_t pwm[512];
    for (size_t i = 0; i < 512; ++i) {
        pwm[i] = ((i % 32U) < 8U) ? 3000U : 200U;
    }

    scope_metrics_t m = {0};
    TEST_ASSERT_EQUAL(ESP_OK, scope_dsp_compute_metrics(pwm, 512, 32000U, &m));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, m.vmin_mv);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 3000.0f, m.vmax_mv);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2800.0f, m.vpp_mv);
    /* Mean = 0.25 * 3000 + 0.75 * 200 = 750 + 150 = 900 mV */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 900.0f, m.vavg_mv);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 1000.0f, m.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 25.0f, m.duty_pct);

    /* 2. 500 Hz Sine wave sampled at 16 kHz (period = 32 samples), 1650 mV DC + 1000 mV peak */
    uint16_t sine_buf[512];
    for (size_t i = 0; i < 512; ++i) {
        double phase = (2.0 * M_PI * (double)(i % 32U)) / 32.0;
        sine_buf[i] = (uint16_t)lround(1650.0 + 1000.0 * sin(phase));
    }

    TEST_ASSERT_EQUAL(ESP_OK, scope_dsp_compute_metrics(sine_buf, 512, 16000U, &m));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 2000.0f, m.vpp_mv);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1650.0f, m.vavg_mv);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 707.1f, m.vrms_ac_mv);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 500.0f, m.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(2.5f, 50.0f, m.duty_pct);

    /* 3. 0 V and 3.3 V flat rail inputs -> 0.0 Hz, 0.0% duty, no NaN/overflow */
    uint16_t rail_0v[128];
    uint16_t rail_3v3[128];
    for (size_t i = 0; i < 128; ++i) {
        rail_0v[i] = 0U;
        rail_3v3[i] = 3300U;
    }

    TEST_ASSERT_EQUAL(ESP_OK, scope_dsp_compute_metrics(rail_0v, 128, 16000U, &m));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vpp_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.duty_pct);

    TEST_ASSERT_EQUAL(ESP_OK, scope_dsp_compute_metrics(rail_3v3, 128, 16000U, &m));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vpp_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3300.0f, m.vavg_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3300.0f, m.vrms_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.duty_pct);
}

TEST_CASE("given registered tool_scope when started configured and stepped then frame metrics speaker and recorder export work", "[tool_scope][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    bsp_spk_passthrough_set_i2s_ops(&s_mock_scope_i2s_ops);

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_timebase(SCOPE_TB_1MS_DIV));
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_vscale(SCOPE_VSCALE_1000MV_DIV));
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_trig_mode(SCOPE_TRIG_RISING));
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_trig_level_mv(1500U));

    synth_pwm_ctx_t pwm_ctx = {
        .low_mv = 300U,
        .high_mv = 2900U,
        .period_samples = 20U,
        .high_samples = 10U,
        .sample_rate_hz = 20000U,
    };
    tool_scope_set_sample_source(synth_pwm_source_cb, &pwm_ctx);

    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_register());
    size_t passed = 0, failed = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_run_selftests(&passed, &failed));
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)failed);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(SCOPE_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, SCOPE_TOOL_ID));
    TEST_ASSERT_GREATER_THAN_UINT32(0U, lv_obj_get_child_count(ui_shell_get_tool_container()));

    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, BSP_SPK_DEFAULT_RATE_HZ));
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_BT_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_run_single_step());
    ui_shell_step();

    scope_frame_t frame = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_get_latest_frame(&frame));
    TEST_ASSERT_TRUE(frame.trig.triggered);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 2600.0f, frame.metrics.vpp_mv);
    TEST_ASSERT_FLOAT_WITHIN(20.0f, 1000.0f, frame.metrics.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(2.5f, 50.0f, frame.metrics.duty_pct);
    TEST_ASSERT_EQUAL_UINT32(20000U, frame.sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(14950U, frame.viewport_us);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, rstats.frames_pushed);
    TEST_ASSERT_EQUAL_STRING(SCOPE_TOOL_ID, rstats.last_tool_id);
    TEST_ASSERT_EQUAL_UINT32(20000U, rstats.last_sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(450U, rstats.last_sample_count);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, (uint32_t)rstats.last_timestamp_us);

    /* A second injected rate verifies that both displayed span and recorder metadata track timing. */
    pwm_ctx.sample_rate_hz = 8000U;
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_get_latest_frame(&frame));
    TEST_ASSERT_EQUAL_UINT32(8000U, frame.sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(37375U, frame.viewport_us);
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_EQUAL_UINT32(8000U, rstats.last_sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(450U, rstats.last_sample_count);

    /* Switch input pin to GPIO35 and verify ownership transfer */
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_pin(BSP_PIN_ANALOG_IN_ALT));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, SCOPE_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_ALT, SCOPE_TOOL_ID));

    /* Pause scope (STOP) and verify frame holds while paused */
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_set_running(false));
    pwm_ctx.high_mv = 1000U;
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_get_latest_frame(&frame));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 2600.0f, frame.metrics.vpp_mv);

    /* Stop tool via ui_shell_go_home and verify cleanup */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_ALT, SCOPE_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    tool_scope_set_sample_source(NULL, NULL);
    (void)tool_scope_set_pin(BSP_PIN_ANALOG_IN_DEFAULT);
    bsp_spk_passthrough_set_i2s_ops(NULL);
}
