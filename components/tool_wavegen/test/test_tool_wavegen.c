/**
 * @file test_tool_wavegen.c
 * @brief Unity test suite for `tool_wavegen` (LEDC/MCPWM/RMT timing calculations, servo & burst
 *        symbol synthesis, pin validation, live peripheral mode transitions, and background
 *        generator coexistence).
 */

#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_freq.h"
#include "tool_scope.h"
#include "tool_wavegen.h"
#include "tool_wavegen_priv.h"
#include "ui_shell.h"
#include "unity.h"

TEST_CASE("given target frequencies and duties when LEDC and MCPWM params solved then resolution and ticks are accurate", "[tool_wavegen][calc]")
{
    wavegen_ledc_params_t lp = {0};

    /* Invalid arguments rejected */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_ledc_params(0U, 1000U, 50.0f, &lp));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 0U, 50.0f, &lp));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, WAVEGEN_LEDC_MAX_FREQ_HZ + 1U, 50.0f, &lp));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 1000U, 50.0f, NULL));

    /* 1 kHz 50% duty -> 16-bit resolution (65536 ticks), 32768 duty ticks */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 1000U, 50.0f, &lp));
    TEST_ASSERT_EQUAL_UINT8(16U, lp.duty_resolution_bits);
    TEST_ASSERT_EQUAL_UINT32(65536U, lp.max_duty_ticks);
    TEST_ASSERT_EQUAL_UINT32(32768U, lp.duty_ticks);
    TEST_ASSERT_UINT32_WITHIN(2U, 1000U, lp.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, lp.actual_duty_pct);

    /* 1 MHz 25% duty -> 6-bit resolution (64 ticks), 16 duty ticks */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 1000000U, 25.0f, &lp));
    TEST_ASSERT_EQUAL_UINT8(6U, lp.duty_resolution_bits);
    TEST_ASSERT_EQUAL_UINT32(64U, lp.max_duty_ticks);
    TEST_ASSERT_EQUAL_UINT32(16U, lp.duty_ticks);
    TEST_ASSERT_EQUAL_UINT32(1000000U, lp.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, lp.actual_duty_pct);

    /* 1 Hz low-frequency edge case */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_ledc_params(WAVEGEN_APB_CLK_HZ, 1U, 100.0f, &lp));
    TEST_ASSERT_EQUAL_UINT8(18U, lp.duty_resolution_bits);
    TEST_ASSERT_EQUAL_UINT32(lp.max_duty_ticks, lp.duty_ticks);
    TEST_ASSERT_EQUAL_UINT32(1U, lp.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, lp.actual_duty_pct);

    /* MCPWM solver validation */
    wavegen_mcpwm_params_t mp = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_mcpwm_params(5U, 50.0f, &mp));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      wavegen_calc_mcpwm_params(WAVEGEN_MCPWM_MAX_FREQ_HZ + 1U, 50.0f, &mp));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_mcpwm_params(1000U, 50.0f, NULL));

    /* 1 kHz 25% duty on 10 MHz MCPWM timer -> 10000 period ticks, 2500 compare ticks, 250 us PW */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_mcpwm_params(1000U, 25.0f, &mp));
    TEST_ASSERT_EQUAL_UINT32(10000000U, mp.resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(10000U, mp.period_ticks);
    TEST_ASSERT_EQUAL_UINT32(2500U, mp.compare_ticks);
    TEST_ASSERT_EQUAL_UINT32(1000U, mp.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, mp.actual_duty_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 250.0f, mp.pulse_width_us);

    /* 50 Hz 7.5% duty selects 1 MHz timer -> 20000 period ticks, 1500 compare ticks */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_mcpwm_params(50U, 7.5f, &mp));
    TEST_ASSERT_EQUAL_UINT32(1000000U, mp.resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(20000U, mp.period_ticks);
    TEST_ASSERT_EQUAL_UINT32(1500U, mp.compare_ticks);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1500.0f, mp.pulse_width_us);

    /* 10 Hz selects 100 kHz timer -> 10000 period ticks */
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_mcpwm_params(10U, 50.0f, &mp));
    TEST_ASSERT_EQUAL_UINT32(100000U, mp.resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(10000U, mp.period_ticks);
    TEST_ASSERT_EQUAL_UINT32(5000U, mp.compare_ticks);
}

TEST_CASE("given servo angles and burst configs when RMT symbols built then durations and previews match specification", "[tool_wavegen][rmt]")
{
    /* Servo angle <-> pulse width conversions & clamping */
    TEST_ASSERT_EQUAL_UINT16(500U, wavegen_calc_servo_angle_to_us(-15.0f));
    TEST_ASSERT_EQUAL_UINT16(500U, wavegen_calc_servo_angle_to_us(0.0f));
    TEST_ASSERT_EQUAL_UINT16(1500U, wavegen_calc_servo_angle_to_us(90.0f));
    TEST_ASSERT_EQUAL_UINT16(2500U, wavegen_calc_servo_angle_to_us(180.0f));
    TEST_ASSERT_EQUAL_UINT16(2500U, wavegen_calc_servo_angle_to_us(220.0f));

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wavegen_calc_servo_us_to_angle(300U));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, wavegen_calc_servo_us_to_angle(1500U));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, wavegen_calc_servo_us_to_angle(2800U));

    /* Standard 20 ms (50 Hz) servo RMT symbol */
    rmt_symbol_word_t syms[8] = {0};
    size_t sym_count = 0;
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_servo_rmt_symbol(1500U, 20000U, syms, 8, &sym_count));
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)sym_count);
    TEST_ASSERT_EQUAL_UINT16(1U, syms[0].level0);
    TEST_ASSERT_EQUAL_UINT16(1500U, syms[0].duration0);
    TEST_ASSERT_EQUAL_UINT16(0U, syms[0].level1);
    TEST_ASSERT_EQUAL_UINT16(18500U, syms[0].duration1);

    /* Extended 40 ms period (> 32767 us low time) splits across 2 RMT symbols */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_servo_rmt_symbol(1500U, 40000U, syms, 1, &sym_count));
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_servo_rmt_symbol(1500U, 40000U, syms, 2, &sym_count));
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)sym_count);
    uint32_t total_low = (uint32_t)syms[0].duration1 + (uint32_t)syms[1].duration0 + (uint32_t)syms[1].duration1;
    TEST_ASSERT_EQUAL_UINT32(38500U, total_low);

    /* N-pulse RMT burst symbol builder */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_burst_rmt_symbols(0U, 100U, 4U, syms, 8, &sym_count));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_burst_rmt_symbols(100U, 100U, 0U, syms, 8, &sym_count));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wavegen_calc_burst_rmt_symbols(100U, 100U, 6U, syms, 4, &sym_count));

    TEST_ASSERT_EQUAL(ESP_OK, wavegen_calc_burst_rmt_symbols(120U, 80U, 5U, syms, 8, &sym_count));
    TEST_ASSERT_EQUAL_UINT32(5U, (uint32_t)sym_count);
    for (size_t i = 0; i < 5U; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1U, syms[i].level0);
        TEST_ASSERT_EQUAL_UINT16(120U, syms[i].duration0);
        TEST_ASSERT_EQUAL_UINT16(0U, syms[i].level1);
        TEST_ASSERT_EQUAL_UINT16(80U, syms[i].duration1);
    }

    /* Preview waveform synthesis */
    int32_t preview[WAVEGEN_CHART_POINTS] = {0};
    wavegen_config_t cfg = {
        .pin = WAVEGEN_DEFAULT_PIN,
        .mode = WAVEGEN_MODE_SQUARE,
        .freq_hz = 1000U,
        .duty_pct = 50.0f,
        .servo_pulse_us = 1500U,
        .burst_high_us = 100U,
        .burst_low_us = 100U,
        .burst_pulse_count = 4U,
        .burst_period_ms = 50U,
        .output_enabled = true,
        .keep_in_bg = true,
    };
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_build_preview_points(&cfg, preview, WAVEGEN_CHART_POINTS));
    TEST_ASSERT_EQUAL_INT32(3300, preview[0]);
    TEST_ASSERT_EQUAL_INT32(0, preview[WAVEGEN_CHART_POINTS - 1U]);

    cfg.output_enabled = false;
    TEST_ASSERT_EQUAL(ESP_OK, wavegen_build_preview_points(&cfg, preview, WAVEGEN_CHART_POINTS));
    TEST_ASSERT_EQUAL_INT32(0, preview[0]);
}

TEST_CASE("given tool_wavegen when started and switched across LEDC MCPWM and RMT modes then hardware and pins transition cleanly", "[tool_wavegen][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_wavegen_reset_for_test();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_register());

    /* Protected and input-only pins are rejected */
    TEST_ASSERT_FALSE(tool_wavegen_is_pin_supported(BSP_I2C_SDA_PIN));
    TEST_ASSERT_FALSE(tool_wavegen_is_pin_supported(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, tool_wavegen_set_pin(BSP_I2C_SDA_PIN));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tool_wavegen_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, tool_wavegen_set_pin(BSP_PIN_ANALOG_IN_ALT));

    /* Start Wave Generator in foreground on default GPIO14 */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, WAVEGEN_TOOL_ID));

    wavegen_status_t st = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    TEST_ASSERT_TRUE(st.running_fg);
    TEST_ASSERT_FALSE(st.running_bg);
    TEST_ASSERT_TRUE(st.hw_active);
    TEST_ASSERT_EQUAL(WAVEGEN_MODE_SQUARE, st.cfg.mode);
    TEST_ASSERT_UINT32_WITHIN(2U, 1000U, st.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, st.actual_duty_pct);

    /* Switch to LEDC PWM mode at 5 kHz, 25% duty */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_mode(WAVEGEN_MODE_PWM_LEDC));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_freq_hz(5000U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_duty_pct(25.0f));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    TEST_ASSERT_TRUE(st.hw_active);
    TEST_ASSERT_EQUAL(WAVEGEN_MODE_PWM_LEDC, st.cfg.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 25.0f, st.actual_duty_pct);

    /* Switch to MCPWM precision PWM mode at 2 kHz, 75% duty */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_mode(WAVEGEN_MODE_PWM_MCPWM));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_freq_hz(2000U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_duty_pct(75.0f));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    TEST_ASSERT_TRUE(st.hw_active);
    TEST_ASSERT_EQUAL_UINT32(2000U, st.actual_freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 75.0f, st.actual_duty_pct);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, st.period_ticks);

    /* Switch to RMT Servo mode at 2000 us */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_mode(WAVEGEN_MODE_SERVO));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_servo_pulse_us(2000U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    TEST_ASSERT_TRUE(st.hw_active);
    TEST_ASSERT_EQUAL_UINT32(50U, st.actual_freq_hz);
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)st.rmt_symbol_count);

    rmt_symbol_word_t staged[4] = {0};
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)tool_wavegen_copy_last_rmt_symbols_for_test(staged, 4));
    TEST_ASSERT_EQUAL_UINT16(2000U, staged[0].duration0);
    TEST_ASSERT_EQUAL_UINT16(18000U, staged[0].duration1);

    /* Switch to RMT Burst mode (6 pulses, 50 us high / 50 us low) and trigger manual burst */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_mode(WAVEGEN_MODE_RMT_BURST));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_burst_config(50U, 50U, 6U, 25U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    uint32_t sent_before = st.burst_frames_sent;
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_trigger_burst());
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_get_status(&st));
    TEST_ASSERT_EQUAL_UINT32(6U, (uint32_t)st.rmt_symbol_count);
    TEST_ASSERT_EQUAL_UINT32(sent_before + 1U, st.burst_frames_sent);

    /* Hot-swap output pin from GPIO14 to GPIO26 while running */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    ui_shell_step();
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));
    tool_wavegen_reset_for_test();
}

TEST_CASE("given tool_wavegen in background when foreground tools open then pins coexist or block conflicts and keep_in_bg toggle governs retention", "[tool_wavegen][concurrency]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    tool_wavegen_reset_for_test();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_scope_register());
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_register());

    /* 1. Start wavegen on GPIO26 (Port B) with keep_in_bg = true (default) */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    /* 2. Return Home with keep_background = true -> wavegen stays running in background on GPIO26 */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(true));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    ui_header_state_t hstate = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_TRUE(hstate.bg_pill_visible);
    TEST_ASSERT_EQUAL_STRING("BG:wave", hstate.bg_pill_text);

    /* 3. Open Frequency Meter on non-conflicting GPIO36 while wavegen runs in background on GPIO26 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg(WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, FREQ_TOOL_ID));

    /* 4. Attempting to switch Frequency Meter to GPIO26 is rejected due to background wavegen ownership */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_freq_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, FREQ_TOOL_ID));

    /* 5. Re-open wavegen -> promotes from background to foreground seamlessly */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(WAVEGEN_TOOL_ID));
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg(WAVEGEN_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    /* 6. Disable "Keep Running in Background" toggle inside wavegen, then go Home(true) -> stops & releases GPIO26 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_wavegen_set_keep_in_bg(false));
    TEST_ASSERT_FALSE(tool_wavegen_get_keep_in_bg());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(true));
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg(WAVEGEN_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, WAVEGEN_TOOL_ID));

    /* Now Frequency Meter can claim GPIO26 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_DAC_OUT_CH2));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(FREQ_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_DAC_OUT_CH2, FREQ_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_EQUAL(ESP_OK, tool_freq_set_pin(BSP_PIN_ANALOG_IN_DEFAULT));
    tool_wavegen_reset_for_test();
}
