/**
 * @file test_tool_analoggen.c
 * @brief Unity test suite for `tool_analoggen` (256-entry LUT synthesis across Sine/Triangle/
 *        Sawtooth/Staircase/DC, amplitude/offset rail clamping, phase-continuous speaker
 *        resampling, task-driven DAC oneshot updates, background generator coexistence,
 *        and speaker ownership coordination with foreground input tools).
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_analoggen.h"
#include "tool_analoggen_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_freq.h"
#include "tool_scope.h"
#include "tool_voltmeter.h"
#include "tool_wavegen.h"
#include "tool_wavegen_priv.h"
#include "ui_shell.h"
#include "unity.h"

static esp_err_t mock_analog_i2s_init(uint32_t rate)
{
    (void)rate;
    return ESP_OK;
}

static esp_err_t mock_analog_i2s_write(const int16_t *pcm,
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

static esp_err_t mock_analog_i2s_disable(void)
{
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_mock_analog_i2s_ops = {
    .ensure_init = mock_analog_i2s_init,
    .write_pcm = mock_analog_i2s_write,
    .disable = mock_analog_i2s_disable,
};

TEST_CASE("given analog waveforms and rail excursions when 256-entry LUT built then shapes stats and clamping are accurate", "[tool_analoggen][lut]")
{
    uint8_t lut[ANALOGGEN_LUT_SIZE] = {0};
    analoggen_lut_stats_t stats = {0};

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

    /* Invalid arguments rejected */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, analoggen_lut_build(NULL, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, analoggen_lut_build(&cfg, NULL, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, analoggen_lut_build(&cfg, lut, 4U, &stats));

    /* 1. Sine wave (1650 mV offset, 2000 mVpp -> 650..2650 mV, unclipped) */
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_FALSE(stats.clipped);
    TEST_ASSERT_INT_WITHIN(2, 128, (int)lut[0]);
    TEST_ASSERT_INT_WITHIN(3, 205, (int)lut[64]);  /* +peak at quarter period */
    TEST_ASSERT_INT_WITHIN(2, 128, (int)lut[128]); /* zero crossing at half period */
    TEST_ASSERT_INT_WITHIN(3, 50, (int)lut[192]);  /* -peak at 3/4 period */
    TEST_ASSERT_INT_WITHIN(40, 2000, (int)stats.vpp_mv);
    TEST_ASSERT_INT_WITHIN(25, 1650, (int)stats.mean_mv);

    /* 2. Triangle wave: monotonic rise 0..128 and monotonic fall 128..255 */
    cfg.waveform = ANALOGGEN_WAVE_TRIANGLE;
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_FALSE(stats.clipped);
    TEST_ASSERT_EQUAL_UINT8(stats.min_code, lut[0]);
    TEST_ASSERT_EQUAL_UINT8(stats.max_code, lut[128]);
    for (size_t i = 1; i <= 128U; ++i) {
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(lut[i - 1U], lut[i]);
    }
    for (size_t i = 129U; i < ANALOGGEN_LUT_SIZE; ++i) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(lut[i - 1U], lut[i]);
    }

    /* 3. Sawtooth wave: monotonic rise from 0 to 255 */
    cfg.waveform = ANALOGGEN_WAVE_SAWTOOTH;
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_FALSE(stats.clipped);
    TEST_ASSERT_EQUAL_UINT8(stats.min_code, lut[0]);
    TEST_ASSERT_EQUAL_UINT8(stats.max_code, lut[ANALOGGEN_LUT_SIZE - 1U]);
    for (size_t i = 1; i < ANALOGGEN_LUT_SIZE; ++i) {
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(lut[i - 1U], lut[i]);
    }

    /* 4. Staircase wave (4 quantized steps across 256 samples = 64 samples/step) */
    cfg.waveform = ANALOGGEN_WAVE_STAIRCASE;
    cfg.stair_steps = 4U;
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_EQUAL_UINT8(lut[0], lut[63]);
    TEST_ASSERT_GREATER_THAN_UINT8(lut[63], lut[64]);
    TEST_ASSERT_EQUAL_UINT8(lut[64], lut[127]);
    TEST_ASSERT_GREATER_THAN_UINT8(lut[127], lut[128]);
    TEST_ASSERT_EQUAL_UINT8(lut[192], lut[255]);

    /* 5. DC wave (constant 2500 mV across all 256 entries) */
    cfg.waveform = ANALOGGEN_WAVE_DC;
    cfg.offset_mv = 2500U;
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_FALSE(stats.clipped);
    TEST_ASSERT_EQUAL_UINT8(lut[0], lut[255]);
    TEST_ASSERT_EQUAL_UINT16(0U, stats.vpp_mv);
    TEST_ASSERT_INT_WITHIN(20, 2500, (int)stats.mean_mv);

    /* 6. Upper and lower rail clamping detection */
    cfg.waveform = ANALOGGEN_WAVE_SINE;
    cfg.offset_mv = 2800U;
    cfg.amplitude_mv = 2000U; /* Upper peak 3800 mV > 3300 mV -> clipped at 255 */
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_TRUE(stats.clipped);
    TEST_ASSERT_EQUAL_UINT8(255U, stats.max_code);
    TEST_ASSERT_EQUAL_UINT16(3300U, stats.max_mv);

    cfg.offset_mv = 400U;
    cfg.amplitude_mv = 1600U; /* Lower peak -400 mV < 0 mV -> clipped at 0 */
    TEST_ASSERT_EQUAL(ESP_OK, analoggen_lut_build(&cfg, lut, ANALOGGEN_LUT_SIZE, &stats));
    TEST_ASSERT_TRUE(stats.clipped);
    TEST_ASSERT_EQUAL_UINT8(0U, stats.min_code);
    TEST_ASSERT_EQUAL_UINT16(0U, stats.min_mv);

    /* 7. Frequency clamping & speaker resampling */
    bool clamped = false;
    TEST_ASSERT_EQUAL_UINT32(1U, analoggen_clamp_freq_hz(0U, &clamped));
    TEST_ASSERT_TRUE(clamped);
    TEST_ASSERT_EQUAL_UINT32(500U, analoggen_clamp_freq_hz(500U, &clamped));
    TEST_ASSERT_FALSE(clamped);
    TEST_ASSERT_EQUAL_UINT32(ANALOGGEN_MAX_FREQ_HZ, analoggen_clamp_freq_hz(5000U, &clamped));
    TEST_ASSERT_TRUE(clamped);

    uint32_t phase_q16 = 0U;
    uint8_t audio[64] = {0};
    TEST_ASSERT_EQUAL(ESP_OK,
                      analoggen_resample_lut_for_speaker(
                          lut, ANALOGGEN_LUT_SIZE, 440U, 16000U, &phase_q16, audio, 64U));
    TEST_ASSERT_GREATER_THAN_UINT32(0U, phase_q16);
}

TEST_CASE("given tool_analoggen when started on GPIO25 or GPIO26 then PSRAM LUT and task-driven DAC oneshot update cleanly", "[tool_analoggen][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_analoggen_reset_for_test();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_register());

    /* Only GPIO25 (DAC1) and GPIO26 (DAC2) are supported */
    TEST_ASSERT_TRUE(tool_analoggen_is_pin_supported(BSP_PIN_DAC_OUT_CH1));
    TEST_ASSERT_TRUE(tool_analoggen_is_pin_supported(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_FALSE(tool_analoggen_is_pin_supported(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, tool_analoggen_set_pin(BSP_I2C_SDA_PIN));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tool_analoggen_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_analoggen_set_pin(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_analoggen_set_freq_hz(0U));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_analoggen_set_freq_hz(ANALOGGEN_MAX_FREQ_HZ + 1U));

    /* Open Analog Generator on default GPIO25 */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH1, ANALOGGEN_TOOL_ID));

    analoggen_status_t st = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_TRUE(st.running_fg);
    TEST_ASSERT_FALSE(st.running_bg);
    TEST_ASSERT_TRUE(st.hw_active);
    TEST_ASSERT_TRUE(st.psram_lut_allocated);
    TEST_ASSERT_EQUAL_UINT32(ANALOGGEN_FG_UPDATE_RATE_HZ, st.target_update_rate_hz);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, st.dac_updates_total);

    /* Run deterministic DAC worker step and verify update counter advances */
    uint32_t updates_before = st.dac_updates_total;
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_GREATER_THAN_UINT32(updates_before, st.dac_updates_total);

    /* Switch waveform, frequency, amplitude, offset, and hot-swap GPIO25 -> GPIO26 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_set_waveform(ANALOGGEN_WAVE_TRIANGLE));
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_set_freq_hz(440U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_set_amplitude_mv(1500U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_set_offset_mv(1600U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH1, ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, ANALOGGEN_TOOL_ID));

    uint8_t copied_lut[ANALOGGEN_LUT_SIZE] = {0};
    TEST_ASSERT_EQUAL_UINT32(ANALOGGEN_LUT_SIZE,
                             (uint32_t)tool_analoggen_copy_lut(copied_lut, sizeof(copied_lut)));

    ui_shell_step();
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, ANALOGGEN_TOOL_ID));
    tool_analoggen_reset_for_test();
}

TEST_CASE("given tool_analoggen in background when foreground tools switch then speaker ownership yields to ADC tools and resumes cleanly", "[tool_analoggen][concurrency][speaker]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_analoggen_reset_for_test();
    tool_wavegen_reset_for_test();
    bsp_spk_passthrough_set_i2s_ops(&s_mock_analog_i2s_ops);

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_voltmeter_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_register());

    /* 1. Start analoggen on GPIO25 and enable speaker monitoring */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_toggle_speaker());
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_TRUE(tool_analoggen_owns_speaker());

    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_run_single_step());
    analoggen_status_t st = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_TRUE(st.spk_streaming);
    TEST_ASSERT_FALSE(st.spk_yielded_to_fg);

    int16_t pcm_check[64] = {0};
    TEST_ASSERT_GREATER_THAN_UINT32(
        0U, (uint32_t)bsp_spk_passthrough_copy_last_pcm(pcm_check, 64U));

    /* 2. Return Home with keep_background = true -> analoggen stays in BG and retains speaker */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(true));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH1, ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_TRUE(tool_analoggen_owns_speaker());

    /* 3. Open Frequency Meter (supports_spk_passthrough == false) on GPIO36 ->
     *    background analoggen STILL owns and streams to the speaker! */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(tool_analoggen_owns_speaker());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_TRUE(st.spk_streaming);
    TEST_ASSERT_FALSE(st.spk_yielded_to_fg);
    TEST_ASSERT_EQUAL_UINT32(ANALOGGEN_BG_UPDATE_RATE_HZ, st.target_update_rate_hz);

    /* 4. Open Oscilloscope (supports_spk_passthrough == true) on GPIO36 ->
     *    background analoggen automatically YIELDS speaker ownership to foreground scope! */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(SCOPE_TOOL_ID));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_FALSE(tool_analoggen_owns_speaker());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_FALSE(st.spk_streaming);
    TEST_ASSERT_TRUE(st.spk_yielded_to_fg);
    TEST_ASSERT_TRUE(st.hw_active); /* Task-driven DAC oneshot continues on GPIO25 without ADC1 conflict */

    /* 5. Close Oscilloscope to Home(false) -> speaker stays enabled and analoggen resumes ownership */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_TRUE(tool_analoggen_owns_speaker());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_analoggen_get_status(&st));
    TEST_ASSERT_TRUE(st.spk_streaming);
    TEST_ASSERT_FALSE(st.spk_yielded_to_fg);

    /* 6. Also start wavegen on GPIO26 in background -> both generators run in background (BG:2),
     *    and analoggen rejects switching to GPIO26 while wavegen owns it */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(WAVEGEN_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(true));
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)tool_core_get_background_count());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_analoggen_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH1, ANALOGGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    /* 7. Stop all background tools -> releases pins and gates off speaker passthrough */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_all_background_tools());
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)tool_core_get_background_count());
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH1, ANALOGGEN_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    bsp_spk_passthrough_set_i2s_ops(NULL);
    tool_wavegen_reset_for_test();
    tool_analoggen_reset_for_test();
}
