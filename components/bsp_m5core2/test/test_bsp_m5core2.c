/**
 * @file test_bsp_m5core2.c
 * @brief Unity unit tests for bsp_m5core2 pin reservation manager, AXP192 telemetry math,
 *        and software Analog-to-Speaker passthrough DSP filter.
 */

#include <limits.h>
#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "unity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST_CASE("given protected internal pins when bsp_pin_claim called then returns ESP_ERR_NOT_ALLOWED", "[bsp_m5core2][pins]")
{
    bsp_pin_manager_reset();

    const gpio_num_t protected_pins[] = {
        BSP_I2C_SDA_PIN,      /* GPIO21 Internal I2C0 SDA */
        BSP_I2C_SCL_PIN,      /* GPIO22 Internal I2C0 SCL */
        BSP_LCD_PIN_MOSI,     /* GPIO23 SPI2 LCD MOSI */
        BSP_LCD_PIN_MISO,     /* GPIO38 SPI2 LCD MISO */
        BSP_LCD_PIN_SCLK,     /* GPIO18 SPI2 LCD SCLK */
        BSP_LCD_PIN_CS,       /* GPIO5  SPI2 LCD CS */
        BSP_LCD_PIN_DC,       /* GPIO15 SPI2 LCD DC */
        BSP_SDCARD_PIN_CS,    /* GPIO4  TF-Card CS */
        BSP_TOUCH_PIN_INT,    /* GPIO39 FT6336U Touch IRQ */
        BSP_SPK_PIN_BCK,      /* GPIO12 I2S1 Speaker BCK */
        BSP_SPK_PIN_WS,       /* GPIO0  I2S1 Speaker WS */
        BSP_SPK_PIN_DOUT,     /* GPIO2  I2S1 Speaker DOUT */
        GPIO_NUM_1,           /* UART0 TX */
        GPIO_NUM_3,           /* UART0 RX */
        GPIO_NUM_16,          /* PSRAM CS/CLK */
        GPIO_NUM_17,          /* PSRAM CS/CLK */
        GPIO_NUM_34,          /* Internal PDM Mic */
    };

    for (size_t i = 0; i < sizeof(protected_pins) / sizeof(protected_pins[0]); ++i) {
        gpio_num_t pin = protected_pins[i];
        TEST_ASSERT_TRUE(bsp_pin_is_protected(pin));
        TEST_ASSERT_EQUAL_STRING("system", bsp_pin_get_owner(pin));
        TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, bsp_pin_claim(pin, "scope"));
        TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, bsp_pin_release(pin, "scope"));
    }
}

TEST_CASE("given safe user expansion pins when claimed and released then ownership tracks accurately", "[bsp_m5core2][pins]")
{
    bsp_pin_manager_reset();

    const gpio_num_t user_pins[] = {
        BSP_PIN_ANALOG_IN_DEFAULT, /* GPIO36 */
        BSP_PIN_ANALOG_IN_ALT,     /* GPIO35 */
        BSP_PIN_DAC_OUT_CH1,       /* GPIO25 */
        BSP_PIN_DAC_OUT_CH2,       /* GPIO26 */
        BSP_PIN_PORT_A_SDA,        /* GPIO32 */
        BSP_PIN_PORT_A_SCL,        /* GPIO33 */
        BSP_PIN_PORT_C_RXD2,       /* GPIO13 */
        BSP_PIN_PORT_C_TXD2,       /* GPIO14 */
        BSP_PIN_MBUS_GPIO27,       /* GPIO27 */
        BSP_PIN_MBUS_GPIO19,       /* GPIO19 */
    };

    for (size_t i = 0; i < sizeof(user_pins) / sizeof(user_pins[0]); ++i) {
        gpio_num_t pin = user_pins[i];
        TEST_ASSERT_FALSE(bsp_pin_is_protected(pin));
        TEST_ASSERT_NULL(bsp_pin_get_owner(pin));
        TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(pin, "logic"));
        TEST_ASSERT_EQUAL_STRING("logic", bsp_pin_get_owner(pin));
        /* Idempotent re-claim by same owner */
        TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(pin, "logic"));
        TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_release(pin, "logic"));
        TEST_ASSERT_NULL(bsp_pin_get_owner(pin));
    }
}

TEST_CASE("given background generator when foreground tool claims pins then non-conflicting pins succeed and conflicts fail", "[bsp_m5core2][pins]")
{
    bsp_pin_manager_reset();

    /* Background wavegen on GPIO14 (Port C) and analoggen on GPIO25 (DAC1) */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim_ex(BSP_PIN_PORT_C_TXD2, "wavegen", true));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim_ex(BSP_PIN_DAC_OUT_CH1, "analoggen", true));

    /* Foreground scope on GPIO36 (ADC1_CH0) coexists without conflict */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(BSP_PIN_ANALOG_IN_DEFAULT, "scope"));

    /* Attempting to claim GPIO14 from logic analyzer while wavegen holds it in background fails */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, bsp_pin_claim(BSP_PIN_PORT_C_TXD2, "logic"));
    /* Attempting to release wavegen's pin from logic analyzer fails */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, bsp_pin_release(BSP_PIN_PORT_C_TXD2, "logic"));

    /* Input-only pins (GPIO36, GPIO35) reject output claims from wavegen */
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, bsp_pin_claim_ex(BSP_PIN_ANALOG_IN_DEFAULT, "wavegen", true));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, bsp_pin_claim_ex(BSP_PIN_ANALOG_IN_ALT, "wavegen", true));

    /* Bulk release for owner frees all its pins */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_release_all_for_owner("wavegen"));
    TEST_ASSERT_NULL(bsp_pin_get_owner(BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL_STRING("analoggen", bsp_pin_get_owner(BSP_PIN_DAC_OUT_CH1));

    bsp_pin_manager_reset();
}

TEST_CASE("given dc offset 12-bit adc signal when processed by speaker dsp then dc is removed and sine is centered", "[bsp_m5core2][dsp]")
{
    bsp_spk_dsp_state_t dsp;
    bsp_spk_dsp_init(&dsp, 100);

    /* 1. Pure DC signal at 2048 (mid-scale) and 3100 (high DC bias) must decay to 0 */
    uint16_t dc_buf[64];
    int16_t pcm_out[64];
    for (size_t i = 0; i < 64; ++i) {
        dc_buf[i] = 3100;
    }
    bsp_spk_dsp_process_adc12(&dsp, dc_buf, pcm_out, 64);
    for (size_t i = 0; i < 64; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, pcm_out[i]);
    }

    /* 2. DC-offset sine wave centered at 2048 with amplitude 500 ADC counts */
    bsp_spk_dsp_reset(&dsp);
    uint16_t sine_adc[512];
    int16_t sine_pcm[512];
    for (size_t i = 0; i < 512; ++i) {
        double phase = (2.0 * M_PI * (double)i) / 32.0;
        sine_adc[i] = (uint16_t)(2048 + (int)(500.0 * sin(phase)));
    }
    bsp_spk_dsp_process_adc12(&dsp, sine_adc, sine_pcm, 512);

    /* Measure mean and peak over the steady-state window (last 256 samples = 8 full periods) */
    int64_t sum = 0;
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    for (size_t i = 256; i < 512; ++i) {
        sum += sine_pcm[i];
        if (sine_pcm[i] < min_val) {
            min_val = sine_pcm[i];
        }
        if (sine_pcm[i] > max_val) {
            max_val = sine_pcm[i];
        }
    }
    int32_t mean = (int32_t)(sum / 256);
    TEST_ASSERT_INT32_WITHIN(150, 0, mean);
    TEST_ASSERT_TRUE(max_val > 6000);
    TEST_ASSERT_TRUE(min_val < -6000);
}

TEST_CASE("given 8-bit dac waveform and gain or mute settings when processed by speaker dsp then gain scales and saturates cleanly", "[bsp_m5core2][dsp]")
{
    bsp_spk_dsp_state_t dsp;
    bsp_spk_dsp_init(&dsp, 100);

    /* Square wave toggling 0..255 at 200% gain must hard-clip within INT16_MIN..INT16_MAX */
    uint8_t dac_sq[32];
    int16_t pcm_100[32];
    int16_t pcm_200[32];
    for (size_t i = 0; i < 32; ++i) {
        dac_sq[i] = (i % 2 == 0) ? 100 : 156;
    }

    bsp_spk_dsp_process_u8(&dsp, dac_sq, pcm_100, 32);

    bsp_spk_dsp_init(&dsp, 200);
    bsp_spk_dsp_process_u8(&dsp, dac_sq, pcm_200, 32);
    TEST_ASSERT_INT16_WITHIN(50, pcm_100[1] * 2, pcm_200[1]);

    /* Full-scale rail steps from midpoint (128 -> 255 -> 0) at 200% gain saturate at INT16_MAX / INT16_MIN */
    uint8_t rail_steps[4] = {128, 255, 0, 255};
    int16_t rail_pcm[4] = {0};
    bsp_spk_dsp_init(&dsp, 200);
    bsp_spk_dsp_process_u8(&dsp, rail_steps, rail_pcm, 4);
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, rail_pcm[1]);
    TEST_ASSERT_EQUAL_INT16(INT16_MIN, rail_pcm[2]);

    /* Mute forces all output samples to 0 */
    dsp.muted = true;
    bsp_spk_dsp_process_u8(&dsp, rail_steps, rail_pcm, 4);
    for (size_t i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, rail_pcm[i]);
    }
}

TEST_CASE("given speaker passthrough pipeline when enabled muted and disabled then amp gate and pcm buffer update properly", "[bsp_m5core2][passthrough]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bsp_spk_passthrough_enable(true, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, 16000));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_EQUAL_UINT32(16000, bsp_spk_passthrough_get_sample_rate());
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());

    uint16_t samples[8] = {2048, 2548, 2048, 1548, 2048, 2548, 2048, 1548};
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_push_adc12(samples, 8));

    size_t last_count = 0;
    const int16_t *last_pcm = bsp_spk_passthrough_get_last_pcm(&last_count);
    TEST_ASSERT_EQUAL_UINT32(8, (uint32_t)last_count);
    TEST_ASSERT_TRUE(last_pcm[1] > 0);

    /* Muting gates off AXP192 SPK_EN and zeroes buffer */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_mute(true));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_muted());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());

    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_mute(false));
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());

    /* Disabling passthrough immediately gates off AXP192 SPK_EN and zeroes DMA buffer */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(false, 0));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    last_pcm = bsp_spk_passthrough_get_last_pcm(&last_count);
    for (size_t i = 0; i < last_count; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, last_pcm[i]);
    }
}

TEST_CASE("given axp192 battery adc registers when converted to mv and percent then matches 1.1mv step and clamps", "[bsp_m5core2][axp192]")
{
    /* 0x000 -> 0 mV -> 0% */
    TEST_ASSERT_EQUAL_UINT16(0, bsp_axp192_raw12_to_mv(0x00, 0x00));
    TEST_ASSERT_EQUAL_UINT8(0, bsp_axp192_calc_battery_percent(3100));
    TEST_ASSERT_EQUAL_UINT8(0, bsp_axp192_calc_battery_percent(3200));

    /* 3800 counts * 1.1 mV = 4180 mV -> 100% */
    uint16_t mv_full = bsp_axp192_raw12_to_mv((uint8_t)(3800 >> 4), (uint8_t)(3800 & 0x0F));
    TEST_ASSERT_EQUAL_UINT16(4180, mv_full);
    TEST_ASSERT_EQUAL_UINT8(100, bsp_axp192_calc_battery_percent(mv_full));

    /* Midpoint 3675 mV -> 50% */
    TEST_ASSERT_EQUAL_UINT8(50, bsp_axp192_calc_battery_percent(3675));
}
