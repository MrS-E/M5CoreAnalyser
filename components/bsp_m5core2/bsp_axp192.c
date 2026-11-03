/**
 * @file bsp_axp192.c
 * @brief AXP192 PMIC driver for M5Stack Core2 over I2C_NUM_0 (GPIO21/GPIO22).
 *
 * Register layout for M5Stack Core2 (AXP192 @ 0x34):
 * - 0x00: Power Input Status (Bit 5 = VBUS present)
 * - 0x01: Power Mode / Charge Status (Bit 6 = Charging)
 * - 0x12: Power Output Control (Bit0=DCDC1 MCU 3.3V, Bit1=DCDC3 LCD BL, Bit2=LDO2 LCD/SD 3.3V, Bit3=LDO3 Vib, Bit6=EXTEN 5V)
 * - 0x26: DCDC1 Voltage (MCU 3.35V -> 0x6A; 700mV + N*25mV)
 * - 0x27: DCDC3 Voltage (LCD Backlight 2.80V -> 0x54; 700mV + N*25mV)
 * - 0x28: LDO2/LDO3 Voltage (Upper nibble LDO2=3.3V -> 0xF0; 1800mV + N*100mV)
 * - 0x30: VBUS-IPSOUT Path Setting (0x80 = no VBUS current limit)
 * - 0x33: Charge Control 1 (0xC0 = enable 4.2V @ 100mA)
 * - 0x78, 0x79: Battery Voltage ADC 12-bit (1.1 mV / LSB)
 * - 0x82: ADC Enable 1 (0xFF = enable VBAT/IBAT/VBUS ADCs)
 * - 0x90: GPIO0 Mode (0x02 = LDO output for M5-Bus VDD)
 * - 0x91: GPIO0 LDO Voltage (0xF0 = 3.3V)
 * - 0x92: GPIO1 Mode (0x00 = NMOS open-drain output for green power LED)
 * - 0x93: GPIO2 Mode (0x00 = NMOS open-drain output for NS4168 SPK_EN)
 * - 0x94: GPIO[2:0] Signal Status (Bit0=GPIO1 LED active-low, Bit1=GPIO2 SPK_EN active-high)
 * - 0x95: GPIO[4:3] Function Control (0x84 = GPIO4 NMOS open-drain output for TFT_RST)
 * - 0x96: GPIO[4:3] Signal Status (Bit1=GPIO4 TFT_RST)
 */

#include "bsp_m5core2.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_axp192";

#define AXP192_REG_POWER_STATUS     (0x00)
#define AXP192_REG_CHARGE_STATUS    (0x01)
#define AXP192_REG_OUTPUT_CTRL      (0x12)
#define AXP192_REG_DCDC1_VOLTAGE    (0x26)
#define AXP192_REG_DCDC3_VOLTAGE    (0x27)
#define AXP192_REG_LDO2_3_VOLTAGE   (0x28)
#define AXP192_REG_VBUS_IPSOUT      (0x30)
#define AXP192_REG_POWER_OFF_VOLT   (0x31)
#define AXP192_REG_CHARGE_CTRL1     (0x33)
#define AXP192_REG_PEK_PARAMS       (0x36)
#define AXP192_REG_VBAT_ADC_HI      (0x78)
#define AXP192_REG_VBAT_ADC_LO      (0x79)
#define AXP192_REG_ADC_EN1          (0x82)
#define AXP192_REG_GPIO0_MODE       (0x90)
#define AXP192_REG_GPIO0_LDO_VOLT   (0x91)
#define AXP192_REG_GPIO1_MODE       (0x92)
#define AXP192_REG_GPIO2_MODE       (0x93)
#define AXP192_REG_GPIO012_STATE    (0x94)
#define AXP192_REG_GPIO34_CTRL      (0x95)
#define AXP192_REG_GPIO34_STATE     (0x96)

#define AXP192_I2C_TIMEOUT_MS       (50)

static i2c_master_dev_handle_t s_axp_dev = NULL;
static volatile bool s_speaker_enabled = false;
static uint8_t s_backlight_percent = 85;

static esp_err_t axp192_write_reg(uint8_t reg, uint8_t val)
{
    if (s_axp_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_axp_dev, buf, sizeof(buf), AXP192_I2C_TIMEOUT_MS);
}

static esp_err_t axp192_read_reg(uint8_t reg, uint8_t *out_val)
{
    if (s_axp_dev == NULL || out_val == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_axp_dev, &reg, 1, out_val, 1, AXP192_I2C_TIMEOUT_MS);
}

static esp_err_t axp192_update_bits(uint8_t reg, uint8_t mask, uint8_t set_bits)
{
    uint8_t val = 0;
    esp_err_t err = axp192_read_reg(reg, &val);
    if (err != ESP_OK) {
        return err;
    }
    val = (uint8_t)((val & ~mask) | (set_bits & mask));
    return axp192_write_reg(reg, val);
}

uint16_t bsp_axp192_raw12_to_mv(uint8_t reg_hi, uint8_t reg_lo)
{
    uint32_t raw12 = ((uint32_t)reg_hi << 4) | ((uint32_t)reg_lo & 0x0FU);
    /* AXP192 battery ADC step is 1.1 mV / LSB */
    return (uint16_t)((raw12 * 11U + 5U) / 10U);
}

uint8_t bsp_axp192_calc_battery_percent(uint16_t voltage_mv)
{
    const uint16_t min_mv = 3200;
    const uint16_t max_mv = 4150;
    if (voltage_mv <= min_mv) {
        return 0;
    }
    if (voltage_mv >= max_mv) {
        return 100;
    }
    uint32_t pct = ((uint32_t)(voltage_mv - min_mv) * 100U) / (uint32_t)(max_mv - min_mv);
    return (uint8_t)pct;
}

esp_err_t bsp_axp192_init(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_axp_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BSP_AXP192_I2C_ADDR,
            .scl_speed_hz = BSP_I2C_CLK_SPEED_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_axp_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add AXP192 I2C device: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Probe AXP192 presence on internal I2C0 */
    esp_err_t err = i2c_master_probe(bus_handle, BSP_AXP192_I2C_ADDR, AXP192_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AXP192 PMIC (0x34) not responding on I2C0: %s", esp_err_to_name(err));
        return err;
    }

    /* 1. VBUS-IPSOUT path: disable VBUS current limit so USB powers board + display cleanly */
    (void)axp192_update_bits(AXP192_REG_VBUS_IPSOUT, 0x07, 0x02);

    /* 2. Configure GPIO0 as LDO output at 3.3V (M5-Bus peripheral power) */
    (void)axp192_write_reg(AXP192_REG_GPIO0_MODE, 0x02);
    (void)axp192_write_reg(AXP192_REG_GPIO0_LDO_VOLT, 0xF0);

    /* 3. Set DCDC1 (ESP32 3.35V -> 0x6A) and LDO2 (LCD/SD 3.3V -> upper nibble 0xF0, LDO3 Vib 0V) */
    (void)axp192_write_reg(AXP192_REG_DCDC1_VOLTAGE, 0x6A);
    (void)axp192_write_reg(AXP192_REG_LDO2_3_VOLTAGE, 0xF0);

    /* 4. Set DCDC3 (LCD Backlight ~2.8V -> 0x54) */
    (void)axp192_write_reg(AXP192_REG_DCDC3_VOLTAGE, 0x54);

    /* 5. Enable DCDC1 (bit0), DCDC3 (bit1), LDO2 (bit2), EXTEN (bit6); keep LDO3 (bit3) & DCDC2 (bit4) off */
    (void)axp192_update_bits(AXP192_REG_OUTPUT_CTRL, 0x5F, 0x47);

    /* 6. Configure GPIO1 (Power LED, NMOS open-drain) and GPIO2 (Speaker SPK_EN, NMOS open-drain) */
    (void)axp192_write_reg(AXP192_REG_GPIO1_MODE, 0x00);
    (void)axp192_write_reg(AXP192_REG_GPIO2_MODE, 0x00);
    /* Turn green power LED ON (bit0 = 0) and gate Speaker SPK_EN OFF (bit1 = 0) by default */
    s_speaker_enabled = false;
    (void)axp192_update_bits(AXP192_REG_GPIO012_STATE, 0x03, 0x00);

    /* 7. Enable battery and VBUS ADCs */
    (void)axp192_write_reg(AXP192_REG_ADC_EN1, 0xFF);
    (void)axp192_write_reg(AXP192_REG_CHARGE_CTRL1, 0xC0);

    /* 8. Configure GPIO4 as NMOS open-drain output for LCD reset (TFT_RST) and pulse reset */
    (void)axp192_write_reg(AXP192_REG_GPIO34_CTRL, 0x84);
    (void)bsp_axp192_lcd_reset();

    ESP_LOGI(TAG, "AXP192 PMIC initialized (DCDC1=3.35V, LDO2=3.3V, DCDC3=2.8V, SPK_EN=OFF)");
    return ESP_OK;
}

esp_err_t bsp_axp192_get_battery(uint16_t *out_voltage_mv, uint8_t *out_percent, bool *out_charging)
{
    uint8_t hi = 0;
    uint8_t lo = 0;
    uint8_t chg = 0;

    esp_err_t err = axp192_read_reg(AXP192_REG_VBAT_ADC_HI, &hi);
    if (err == ESP_OK) {
        err = axp192_read_reg(AXP192_REG_VBAT_ADC_LO, &lo);
    }
    if (err == ESP_OK) {
        err = axp192_read_reg(AXP192_REG_CHARGE_STATUS, &chg);
    }

    if (err != ESP_OK) {
        uint16_t fallback_mv = 3850;
        if (out_voltage_mv != NULL) {
            *out_voltage_mv = fallback_mv;
        }
        if (out_percent != NULL) {
            *out_percent = bsp_axp192_calc_battery_percent(fallback_mv);
        }
        if (out_charging != NULL) {
            *out_charging = false;
        }
        return err;
    }

    uint16_t vbat_mv = bsp_axp192_raw12_to_mv(hi, lo);
    if (out_voltage_mv != NULL) {
        *out_voltage_mv = vbat_mv;
    }
    if (out_percent != NULL) {
        *out_percent = bsp_axp192_calc_battery_percent(vbat_mv);
    }
    if (out_charging != NULL) {
        *out_charging = ((chg & (1U << 6)) != 0U);
    }
    return ESP_OK;
}

esp_err_t bsp_axp192_set_speaker_enable(bool enable)
{
    s_speaker_enabled = enable;
    if (s_axp_dev == NULL) {
        return ESP_OK;
    }
    /* AXP192 Reg 0x94 Bit 1 controls GPIO2 (SPK_EN on M5Stack Core2) */
    return axp192_update_bits(AXP192_REG_GPIO012_STATE, (1U << 1), enable ? (1U << 1) : 0U);
}

bool bsp_axp192_get_speaker_enable(void)
{
    return s_speaker_enabled;
}

esp_err_t bsp_axp192_set_lcd_backlight(uint8_t brightness_percent)
{
    if (brightness_percent > 100U) {
        brightness_percent = 100U;
    }
    s_backlight_percent = brightness_percent;

    if (s_axp_dev == NULL) {
        return ESP_OK;
    }

    if (brightness_percent == 0U) {
        return axp192_update_bits(AXP192_REG_OUTPUT_CTRL, (1U << 1), 0U);
    }

    /* Map 1..100% to 2500..3100 mV on DCDC3 (700mV + reg * 25mV) */
    uint32_t mv = 2500U + ((uint32_t)brightness_percent * 600U) / 100U;
    uint8_t reg_val = (uint8_t)((mv - 700U) / 25U);
    esp_err_t err = axp192_write_reg(AXP192_REG_DCDC3_VOLTAGE, reg_val);
    if (err != ESP_OK) {
        return err;
    }
    return axp192_update_bits(AXP192_REG_OUTPUT_CTRL, (1U << 1), (1U << 1));
}

esp_err_t bsp_axp192_set_led(bool on)
{
    if (s_axp_dev == NULL) {
        return ESP_OK;
    }
    /* AXP192 GPIO1 is active-low open-drain sink for the green LED */
    return axp192_update_bits(AXP192_REG_GPIO012_STATE, (1U << 0), on ? 0U : (1U << 0));
}

esp_err_t bsp_axp192_lcd_reset(void)
{
    if (s_axp_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = axp192_update_bits(AXP192_REG_GPIO34_STATE, (1U << 1), 0U);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    err = axp192_update_bits(AXP192_REG_GPIO34_STATE, (1U << 1), (1U << 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    return err;
}
