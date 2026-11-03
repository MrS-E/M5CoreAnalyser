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

    /*
     * 1. Step input (2048 -> 3000 after seeding):
     *    step_adc = 3000 - 2048 = 952 counts -> step_pcm = 952 * 16 = 15232.0f.
     *    Verify y[0] = 0 and y[k] = round(15232 * 0.995^(k - 1)) for k >= 1.
     */
    static uint16_t step_adc[512];
    static int16_t step_pcm[512];
    step_adc[0] = 2048;
    for (size_t i = 1; i < 512; ++i) {
        step_adc[i] = 3000;
    }
    bsp_spk_dsp_process_adc12(&dsp, step_adc, step_pcm, 512);
    TEST_ASSERT_EQUAL_INT16(0, step_pcm[0]);
    TEST_ASSERT_EQUAL_INT16(15232, step_pcm[1]);
    for (size_t k = 1; k < 512; ++k) {
        int16_t expected = (int16_t)lrintf(15232.0f * powf(0.995f, (float)(k - 1)));
        TEST_ASSERT_INT16_WITHIN(2, expected, step_pcm[k]);
    }
    /* Confirm exponential decay after 500 samples (15232 * 0.995^500 ~= 1243) */
    TEST_ASSERT_INT16_WITHIN(5, 1243, step_pcm[501]);

    /*
     * 2. DC-offset sine wave centred at 3000 (+15232 PCM DC bias) with amplitude 400 ADC counts:
     *    After the DC transient decays, the steady-state output mean must be ~0.
     */
    bsp_spk_dsp_reset(&dsp);
    static uint16_t sine_adc[1536];
    static int16_t sine_pcm[1536];
    for (size_t i = 0; i < 1536; ++i) {
        double phase = (2.0 * M_PI * (double)i) / 32.0;
        sine_adc[i] = (uint16_t)(3000 + (int)lrint(400.0 * sin(phase)));
    }
    bsp_spk_dsp_process_adc12(&dsp, sine_adc, sine_pcm, 1536);

    /* Measure mean and peak over the steady-state window (last 256 samples = 8 full periods) */
    int64_t sum = 0;
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    for (size_t i = 1280; i < 1536; ++i) {
        sum += sine_pcm[i];
        if (sine_pcm[i] < min_val) {
            min_val = sine_pcm[i];
        }
        if (sine_pcm[i] > max_val) {
            max_val = sine_pcm[i];
        }
    }
    int32_t mean = (int32_t)(sum / 256);
    TEST_ASSERT_INT32_WITHIN(50, 0, mean);
    TEST_ASSERT_TRUE(max_val > 5800);
    TEST_ASSERT_TRUE(min_val < -5800);

    /*
     * 3. Clamping of out-of-range ADC inputs above 4095:
     *    Inputs > 4095 (e.g. 5000, 0xFFFF) must clamp to 4095 before scaling.
     */
    uint16_t adc_rail_ref[2] = {2048, 4095};
    uint16_t adc_rail_over1[2] = {2048, 5000};
    uint16_t adc_rail_over2[2] = {2048, 0xFFFFU};
    int16_t pcm_ref[2] = {0};
    int16_t pcm_over1[2] = {0};
    int16_t pcm_over2[2] = {0};

    bsp_spk_dsp_init(&dsp, 100);
    bsp_spk_dsp_process_adc12(&dsp, adc_rail_ref, pcm_ref, 2);
    bsp_spk_dsp_init(&dsp, 100);
    bsp_spk_dsp_process_adc12(&dsp, adc_rail_over1, pcm_over1, 2);
    bsp_spk_dsp_init(&dsp, 100);
    bsp_spk_dsp_process_adc12(&dsp, adc_rail_over2, pcm_over2, 2);

    TEST_ASSERT_EQUAL_INT16(32752, pcm_ref[1]);
    TEST_ASSERT_EQUAL_INT16(pcm_ref[1], pcm_over1[1]);
    TEST_ASSERT_EQUAL_INT16(pcm_ref[1], pcm_over2[1]);

    /* 4. Gain 0% outputs exact zeros */
    bsp_spk_dsp_init(&dsp, 0);
    bsp_spk_dsp_process_adc12(&dsp, adc_rail_ref, pcm_ref, 2);
    TEST_ASSERT_EQUAL_INT16(0, pcm_ref[0]);
    TEST_ASSERT_EQUAL_INT16(0, pcm_ref[1]);
}

TEST_CASE("given 8-bit dac waveform and gain or mute settings when processed by speaker dsp then gain scales and saturates cleanly", "[bsp_m5core2][dsp]")
{
    bsp_spk_dsp_state_t dsp;
    bsp_spk_dsp_init(&dsp, 100);

    /* Moderate square wave toggling 100..156 (+/-28 counts around 128) scales linearly from 100% to 200% gain */
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

    /* Gain 0% forces all output samples to 0 */
    bsp_spk_dsp_init(&dsp, 0);
    bsp_spk_dsp_process_u8(&dsp, rail_steps, rail_pcm, 4);
    for (size_t i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, rail_pcm[i]);
    }

    /* Mute forces all output samples to 0 even at 200% gain */
    bsp_spk_dsp_init(&dsp, 200);
    dsp.muted = true;
    bsp_spk_dsp_process_u8(&dsp, rail_steps, rail_pcm, 4);
    for (size_t i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, rail_pcm[i]);
    }
}

typedef struct {
    esp_err_t next_init_err;
    uint32_t init_calls;
    uint32_t last_rate_hz;
    uint32_t write_calls;
    uint32_t disable_calls;
} mock_i2s_ctx_t;

static mock_i2s_ctx_t s_mock_i2s;

static esp_err_t mock_i2s_ensure_init(uint32_t sample_rate_hz)
{
    s_mock_i2s.init_calls++;
    s_mock_i2s.last_rate_hz = sample_rate_hz;
    return s_mock_i2s.next_init_err;
}

static esp_err_t mock_i2s_write_pcm(const int16_t *pcm,
                                     size_t byte_len,
                                     size_t *out_bytes_written,
                                     uint32_t timeout_ms)
{
    (void)pcm;
    (void)timeout_ms;
    s_mock_i2s.write_calls++;
    if (out_bytes_written != NULL) {
        *out_bytes_written = byte_len;
    }
    return ESP_OK;
}

static esp_err_t mock_i2s_disable(void)
{
    s_mock_i2s.disable_calls++;
    return ESP_OK;
}

TEST_CASE("given speaker passthrough pipeline when enabled muted and disabled then amp gate and pcm buffer update properly", "[bsp_m5core2][passthrough]")
{
    bsp_spk_passthrough_set_i2s_ops(NULL);
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_gain(100));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_mute(false));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bsp_spk_passthrough_enable(true, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, 16000));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_EQUAL_UINT32(16000, bsp_spk_passthrough_get_sample_rate());
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_HIGH(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    uint16_t samples[8] = {2048, 2548, 2048, 1548, 2048, 2548, 2048, 1548};
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_push_adc12(samples, 8));

    size_t last_count = 0;
    const int16_t *last_pcm = bsp_spk_passthrough_get_last_pcm(&last_count);
    TEST_ASSERT_EQUAL_UINT32(8, (uint32_t)last_count);
    TEST_ASSERT_TRUE(last_pcm[1] > 0);

    /* Muting gates off AXP192 SPK_EN (bit 2 of 0x94) and zeroes buffer */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_mute(true));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_muted());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_LOW(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_mute(false));
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_HIGH(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    /* Gain 0% also gates off AXP192 SPK_EN */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_gain(0));
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_LOW(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_set_gain(100));
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_HIGH(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    /* Disabling passthrough immediately gates off AXP192 SPK_EN and zeroes DMA buffer */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(false, 0));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_LOW(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());
    last_pcm = bsp_spk_passthrough_get_last_pcm(&last_count);
    for (size_t i = 0; i < last_count; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, last_pcm[i]);
    }
}

TEST_CASE("given i2s1 init or clock reconfig error when speaker passthrough enabled then error propagates and amp stays off", "[bsp_m5core2][passthrough]")
{
    static const bsp_spk_i2s_ops_t mock_ops = {
        .ensure_init = mock_i2s_ensure_init,
        .write_pcm = mock_i2s_write_pcm,
        .disable = mock_i2s_disable,
    };

    (void)bsp_spk_passthrough_enable(false, 0);
    memset(&s_mock_i2s, 0, sizeof(s_mock_i2s));
    bsp_spk_passthrough_set_i2s_ops(&mock_ops);

    /* 1. Initial I2S1 init failure must propagate and keep SPK_EN gated OFF */
    s_mock_i2s.next_init_err = ESP_ERR_NOT_FOUND;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, bsp_spk_passthrough_enable(true, 16000));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_LOW(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    uint16_t dummy_adc[4] = {2048, 2200, 2048, 1900};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, bsp_spk_passthrough_push_adc12(dummy_adc, 4));

    /* 2. Successful init enables passthrough and SPK_EN */
    s_mock_i2s.next_init_err = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, 16000));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_HIGH(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());

    /* 3. Subsequent clock reconfiguration failure disables passthrough and gates off SPK_EN */
    uint32_t prev_disable_calls = s_mock_i2s.disable_calls;
    s_mock_i2s.next_init_err = ESP_ERR_INVALID_STATE;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, bsp_spk_passthrough_enable(true, 44100));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_BITS_LOW(BSP_AXP192_GPIO2_SPK_EN_BIT, bsp_axp192_get_reg94_state());
    TEST_ASSERT_TRUE(s_mock_i2s.disable_calls > prev_disable_calls);

    bsp_spk_passthrough_set_i2s_ops(NULL);
}

TEST_CASE("given axp192 reg 0x94 gpio state and battery adc registers when calculated then bits and mv match core2 hardware", "[bsp_m5core2][axp192]")
{
    /* Verify M5Stack Core2 AXP192 register 0x94 bit constants */
    TEST_ASSERT_EQUAL_HEX8(0x94U, BSP_AXP192_REG_GPIO012_STATE);
    TEST_ASSERT_EQUAL_HEX8(0x02U, BSP_AXP192_GPIO1_LED_BIT);    /* Bit 1: SYS_LED (active-low) */
    TEST_ASSERT_EQUAL_HEX8(0x04U, BSP_AXP192_GPIO2_SPK_EN_BIT); /* Bit 2: SPK_EN (active-high) */

    /* Pure register 0x94 calculation from 0x00 base */
    TEST_ASSERT_EQUAL_HEX8(0x00U, bsp_axp192_calc_gpio_state(0x00U, false, true));  /* SPK off (b2=0), LED on (b1=0) */
    TEST_ASSERT_EQUAL_HEX8(0x04U, bsp_axp192_calc_gpio_state(0x00U, true, true));   /* SPK on  (b2=1), LED on (b1=0) */
    TEST_ASSERT_EQUAL_HEX8(0x06U, bsp_axp192_calc_gpio_state(0x00U, true, false));  /* SPK on  (b2=1), LED off (b1=1) */
    TEST_ASSERT_EQUAL_HEX8(0x02U, bsp_axp192_calc_gpio_state(0x00U, false, false)); /* SPK off (b2=0), LED off (b1=1) */

    /* Verify GPIO0 (bit 0 = 0x01) and upper bits [7:3] (0xF8) are preserved */
    TEST_ASSERT_EQUAL_HEX8(0xF9U, bsp_axp192_calc_gpio_state(0xFFU, false, true));
    TEST_ASSERT_EQUAL_HEX8(0xFDU, bsp_axp192_calc_gpio_state(0xF9U, true, true));
    TEST_ASSERT_EQUAL_HEX8(0xFFU, bsp_axp192_calc_gpio_state(0xF9U, true, false));
    TEST_ASSERT_EQUAL_HEX8(0xFBU, bsp_axp192_calc_gpio_state(0xFFU, false, false));

    /* Verify setter functions update shadow register 0x94 bits accurately without clobbering each other */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_led(true));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_speaker_enable(false));
    TEST_ASSERT_TRUE(bsp_axp192_get_led_state());
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_EQUAL_HEX8(0x00U, bsp_axp192_get_reg94_state() & 0x06U);

    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_speaker_enable(true));
    TEST_ASSERT_TRUE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_EQUAL_HEX8(0x04U, bsp_axp192_get_reg94_state() & 0x06U);

    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_led(false));
    TEST_ASSERT_FALSE(bsp_axp192_get_led_state());
    TEST_ASSERT_EQUAL_HEX8(0x06U, bsp_axp192_get_reg94_state() & 0x06U);

    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_speaker_enable(false));
    TEST_ASSERT_FALSE(bsp_axp192_get_speaker_enable());
    TEST_ASSERT_EQUAL_HEX8(0x02U, bsp_axp192_get_reg94_state() & 0x06U);

    TEST_ASSERT_EQUAL(ESP_OK, bsp_axp192_set_led(true));
    TEST_ASSERT_EQUAL_HEX8(0x00U, bsp_axp192_get_reg94_state() & 0x06U);

    /* Battery ADC: 0x000 -> 0 mV -> 0% */
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
