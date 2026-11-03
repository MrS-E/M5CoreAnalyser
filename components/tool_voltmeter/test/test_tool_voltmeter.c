/**
 * @file test_tool_voltmeter.c
 * @brief Unity unit & integration tests for `tool_voltmeter`: median/trimmed-mean outlier
 *        filtering, PSRAM moving average, two-point calibration & NVS persistence,
 *        DC/AC-RMS/True-RMS sine & rail accuracy, pin switching, recorder, and speaker passthrough.
 */

#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_voltmeter.h"
#include "tool_voltmeter_priv.h"
#include "ui_shell.h"
#include "unity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static esp_err_t mock_volt_i2s_init(uint32_t rate)
{
    (void)rate;
    return ESP_OK;
}

static esp_err_t mock_volt_i2s_write(const int16_t *pcm,
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

static esp_err_t mock_volt_i2s_disable(void)
{
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_mock_volt_i2s_ops = {
    .ensure_init = mock_volt_i2s_init,
    .write_pcm = mock_volt_i2s_write,
    .disable = mock_volt_i2s_disable,
};

typedef struct {
    float dc_mv;
    float amp_mv;
} synth_sine_ctx_t;

static esp_err_t synth_sine_source_cb(gpio_num_t pin,
                                      uint16_t *out_raw,
                                      float *out_uncal_mv,
                                      size_t count,
                                      void *user_ctx)
{
    (void)pin;
    const synth_sine_ctx_t *s = (const synth_sine_ctx_t *)user_ctx;
    for (size_t i = 0; i < count; ++i) {
        double phase = (2.0 * M_PI * (double)i) / (double)count;
        float mv = s->dc_mv + s->amp_mv * (float)sin(phase);
        if (mv < 0.0f) {
            mv = 0.0f;
        } else if (mv > 3300.0f) {
            mv = 3300.0f;
        }
        out_uncal_mv[i] = mv;
        out_raw[i] = (uint16_t)lroundf((mv * 4095.0f) / 3300.0f);
    }
    return ESP_OK;
}

TEST_CASE("given noisy sample batch with impulse spikes when median and trimmed-mean filtered then outliers are rejected", "[tool_voltmeter][dsp]")
{
    /* 16 samples mostly at 1650 mV, with two 0 mV dropouts and two 3300 mV spikes */
    const float noisy_batch[16] = {
        0.0f,    1648.0f, 1650.0f, 1652.0f,
        3300.0f, 1649.0f, 1651.0f, 1650.0f,
        1650.0f, 0.0f,    1651.0f, 1649.0f,
        1650.0f, 3300.0f, 1652.0f, 1648.0f
    };

    float median_mv = 0.0f;
    float trimmed_mv = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_median_filter_mv(noisy_batch, 16, &median_mv, &trimmed_mv));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1650.0f, median_mv);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1650.0f, trimmed_mv);

    /* Odd sample count */
    const float odd_batch[5] = {3300.0f, 1200.0f, 100.0f, 1200.0f, 1200.0f};
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_median_filter_mv(odd_batch, 5, &median_mv, &trimmed_mv));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1200.0f, median_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1200.0f, trimmed_mv);

    /* Invalid arguments */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, voltmeter_dsp_median_filter_mv(NULL, 8, &median_mv, &trimmed_mv));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, voltmeter_dsp_median_filter_mv(odd_batch, 0, &median_mv, &trimmed_mv));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      voltmeter_dsp_median_filter_mv(odd_batch, VOLTMETER_MAX_BATCH_SAMPLES + 1, &median_mv, &trimmed_mv));
}

TEST_CASE("given PSRAM moving-average filter when samples are pushed across wrap-around then average min and max track accurately", "[tool_voltmeter][avg]")
{
    voltmeter_avg_filter_t filter = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, voltmeter_dsp_avg_init(NULL, 4));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, voltmeter_dsp_avg_init(&filter, 0));
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_avg_init(&filter, 4));

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000.0f, voltmeter_dsp_avg_push(&filter, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, voltmeter_dsp_avg_push(&filter, 2000.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2000.0f, voltmeter_dsp_avg_push(&filter, 3000.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1750.0f, voltmeter_dsp_avg_push(&filter, 1000.0f));

    /* 5th push wraps window: [2000, 3000, 1000, 2200] -> mean = 2050 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2050.0f, voltmeter_dsp_avg_push(&filter, 2200.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000.0f, filter.min_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3000.0f, filter.max_mv);
    TEST_ASSERT_EQUAL_UINT32(5U, filter.total_batches);

    voltmeter_dsp_avg_reset(&filter);
    TEST_ASSERT_EQUAL_UINT32(0U, filter.total_batches);
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)filter.count);
    voltmeter_dsp_avg_free(&filter);
}

TEST_CASE("given two-point calibration references when solved saved and applied then slope offset and NVS persistence work", "[tool_voltmeter][cal]")
{
    voltmeter_cal_t cal = {0};
    voltmeter_dsp_cal_init_default(&cal);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, cal.slope);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, cal.offset_mv);

    /* Solve 2-point calibration: raw 490 mV -> 500 mV, raw 2450 mV -> 2500 mV
     * slope = 2000 / 1960 = 1.020408f, offset = 500 - 1.020408 * 490 = 0.0f */
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_calc_two_point_cal(490.0f, 500.0f, 2450.0f, 2500.0f, &cal));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2000.0f / 1960.0f, cal.slope);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, cal.offset_mv);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 500.0f, voltmeter_dsp_apply_cal(490.0f, &cal));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 2500.0f, voltmeter_dsp_apply_cal(2450.0f, &cal));

    /* Clamping at rails */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, voltmeter_dsp_apply_cal(-150.0f, &cal));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, VOLTMETER_CLAMP_MAX_MV, voltmeter_dsp_apply_cal(5000.0f, &cal));

    /* Reject degenerate or out-of-range calibration points */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      voltmeter_dsp_calc_two_point_cal(1000.0f, 1000.0f, 1002.0f, 2000.0f, &cal));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      voltmeter_dsp_calc_two_point_cal(100.0f, 100.0f, 500.0f, 2500.0f, &cal));

    /* NVS save, load, and reset round-trip */
    voltmeter_cal_t saved = {.slope = 1.015f, .offset_mv = -12.5f};
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_save_cal(BSP_PIN_ANALOG_IN_DEFAULT, &saved));

    voltmeter_cal_t loaded = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_load_cal(BSP_PIN_ANALOG_IN_DEFAULT, &loaded));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.015f, loaded.slope);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -12.5f, loaded.offset_mv);

    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_reset_cal(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_load_cal(BSP_PIN_ANALOG_IN_DEFAULT, &loaded));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, loaded.slope);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, loaded.offset_mv);
}

TEST_CASE("given synthetic sine wave and 0V 3.3V rail inputs when processed then DC AC-RMS True-RMS and Vpp are accurate", "[tool_voltmeter][rms]")
{
    const size_t N = 64;
    float sine_mv[64];
    uint16_t rail_zero[64];
    uint16_t rail_full[64];

    for (size_t i = 0; i < N; ++i) {
        double phase = (2.0 * M_PI * (double)i) / (double)N;
        sine_mv[i] = 1650.0f + 1000.0f * (float)sin(phase);
        rail_zero[i] = 0U;
        rail_full[i] = 4095U;
    }

    voltmeter_avg_filter_t avg = {0};
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_avg_init(&avg, 8));

    voltmeter_metrics_t m = {0};
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_process_batch(NULL, sine_mv, N, NULL, &avg, &m));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1650.0f, m.vdc_mv);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 707.1f, m.vrms_ac_mv);
    /* True RMS = sqrt(1650^2 + (1000/sqrt(2))^2) = sqrt(2722500 + 500000) = 1795.13 mV */
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1795.1f, m.vrms_true_mv);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 2000.0f, m.vpp_mv);

    /* 0V Rail input */
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_process_batch(rail_zero, NULL, N, NULL, &avg, &m));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vdc_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vrms_ac_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vrms_true_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.vpp_mv);

    /* 3.3V Rail input */
    TEST_ASSERT_EQUAL(ESP_OK, voltmeter_dsp_process_batch(rail_full, NULL, N, NULL, &avg, &m));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3300.0f, m.vdc_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, m.vrms_ac_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3300.0f, m.vrms_true_mv);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, m.vpp_mv);

    voltmeter_dsp_avg_free(&avg);
}

TEST_CASE("given registered tool_voltmeter when started switched pins and stepped then speaker recorder and hold states work", "[tool_voltmeter][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    bsp_spk_passthrough_set_i2s_ops(&s_mock_volt_i2s_ops);

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_reset_cal(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));

    synth_sine_ctx_t synth = {.dc_mv = 1500.0f, .amp_mv = 500.0f};
    tool_voltmeter_set_sample_source(synth_sine_source_cb, &synth);

    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_register());
    size_t passed = 0, failed = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_run_selftests(&passed, &failed));
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)failed);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(VOLTMETER_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, VOLTMETER_TOOL_ID));
    TEST_ASSERT_GREATER_THAN_UINT32(0U, lv_obj_get_child_count(ui_shell_get_tool_container()));

    /* Enable speaker passthrough and USB stub recording, then run a synchronous step + UI refresh */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, VOLTMETER_DEFAULT_SAMPLE_RATE_HZ));
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_run_single_step());
    ui_shell_step();

    voltmeter_metrics_t metrics = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_get_latest_metrics(&metrics));
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 1500.0f, metrics.vdc_mv);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 353.5f, metrics.vrms_ac_mv);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, rstats.frames_pushed);
    TEST_ASSERT_EQUAL_STRING(VOLTMETER_TOOL_ID, rstats.last_tool_id);

    /* Switch pin to GPIO35 while running */
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_set_pin(BSP_PIN_ANALOG_IN_ALT));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, VOLTMETER_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_ALT, VOLTMETER_TOOL_ID));

    /* If GPIO36 is claimed by another tool, switching back to GPIO36 fails cleanly and retains GPIO35 */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(BSP_PIN_ANALOG_IN_DEFAULT, "other_tool"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_voltmeter_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(BSP_PIN_ANALOG_IN_ALT, tool_voltmeter_get_pin());
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_release(BSP_PIN_ANALOG_IN_DEFAULT, "other_tool"));

    /* Hold mode freezes displayed metrics when input changes */
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_set_hold(true));
    synth.dc_mv = 2800.0f;
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_get_latest_metrics(&metrics));
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 1500.0f, metrics.vdc_mv);

    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_set_hold(false));
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_get_latest_metrics(&metrics));
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 2800.0f, metrics.vdc_mv);

    /* Stop tool via ui_shell_go_home and verify pin release and speaker gate-off */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_ALT, VOLTMETER_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    tool_voltmeter_set_sample_source(NULL, NULL);
    (void)tool_voltmeter_set_pin(BSP_PIN_ANALOG_IN_DEFAULT);
    bsp_spk_passthrough_set_i2s_ops(NULL);
}
