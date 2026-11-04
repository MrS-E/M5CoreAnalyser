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
 * - 0x30: VBUS-IPSOUT Path Setting (0x02 = 500mA VBUS current limit enabled)
 * - 0x33: Charge Control 1 (0xC0 = enable 4.2V @ 100mA)
 * - 0x78, 0x79: Battery Voltage ADC 12-bit (1.1 mV / LSB)
 * - 0x82: ADC Enable 1 (0xFF = enable VBAT/IBAT/VBUS ADCs)
 * - 0x90: GPIO0 Mode (0x02 = LDO output for M5-Bus VDD)
 * - 0x91: GPIO0 LDO Voltage (0xF0 = 3.3V)
 * - 0x92: GPIO1 Mode (0x00 = NMOS open-drain output for green power LED)
 * - 0x93: GPIO2 Mode (0x00 = NMOS open-drain output for NS4168 SPK_EN)
 * - 0x94: GPIO[2:0] Signal Status (Bit1=GPIO1 LED active-low, Bit2=GPIO2 SPK_EN active-high)
 * - 0x95: GPIO[4:3] Function Control (0x84 = GPIO4 NMOS open-drain output for TFT_RST)
 * - 0x96: GPIO[4:3] Signal Status (Bit1=GPIO4 TFT_RST)
 */

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "bsp_axp192";

#define AXP192_REG_POWER_STATUS     (0x00)
#define AXP192_REG_CHARGE_STATUS    (0x01)
#define AXP_REG_CHIP_ID             (0x03)
#define AXP192_CHIP_ID              (0x03)
#define AXP2101_CHIP_ID             (0x4A)
#define AXP192_REG_OUTPUT_CTRL      (0x12)
#define AXP192_REG_DCDC1_VOLTAGE    (0x26)
#define AXP192_REG_DCDC3_VOLTAGE    (0x27)
#define AXP192_REG_LDO2_3_VOLTAGE   (0x28)
#define AXP192_REG_VBUS_IPSOUT      (0x30)
#define AXP192_REG_POWER_OFF_VOLT   (0x31)
#define AXP192_REG_CHARGE_CTRL1     (0x33)
#define AXP192_REG_RTC_BAT_CHG      (0x35)
#define AXP192_REG_PEK_PARAMS       (0x36)
#define AXP192_REG_VBAT_ADC_HI      (0x78)
#define AXP192_REG_VBAT_ADC_LO      (0x79)
#define AXP192_REG_ADC_EN1          (0x82)
#define AXP192_REG_GPIO0_MODE       (0x90)
#define AXP192_REG_GPIO0_LDO_VOLT   (0x91)
#define AXP192_REG_GPIO1_MODE       (0x92)
#define AXP192_REG_GPIO2_MODE       (0x93)
#define AXP192_REG_GPIO012_STATE    (BSP_AXP192_REG_GPIO012_STATE)
#define AXP192_REG_GPIO34_CTRL      (0x95)
#define AXP192_REG_GPIO34_STATE     (0x96)

/* AXP2101 registers (M5Stack Core2 v1.1 @ 0x34, chip ID 0x4A) */
#define AXP2101_REG_PMU_COMMON      (0x10)
#define AXP2101_REG_BATFET_CTRL     (0x12)
#define AXP2101_REG_DCDC_UVP_CTRL   (0x23)
#define AXP2101_REG_PWROFF_THLD     (0x27)
#define AXP2101_REG_ADC_EN          (0x30)
#define AXP2101_REG_VBAT_HI         (0x34)
#define AXP2101_REG_VBAT_LO         (0x35)
#define AXP2101_REG_TS_PIN_CTRL     (0x50)
#define AXP2101_REG_CHG_VOLT_CTRL   (0x64)
#define AXP2101_REG_BAT_DET_CTRL    (0x68)
#define AXP2101_REG_CHGLED_CTRL     (0x69)
#define AXP2101_REG_DCDC_ONOFF      (0x80)
#define AXP2101_REG_DCDC1_VOLT      (0x82)
#define AXP2101_REG_DCDC3_VOLT      (0x84)
#define AXP2101_REG_LDO_ONOFF0      (0x90)
#define AXP2101_REG_ALDO2_VOLT      (0x93)
#define AXP2101_REG_ALDO3_VOLT      (0x94)
#define AXP2101_REG_ALDO4_VOLT      (0x95)
#define AXP2101_REG_BLDO1_VOLT      (0x96)
#define AXP2101_REG_DLDO1_VOLT      (0x99)

#define AXP192_I2C_TIMEOUT_MS       (50)

static StaticSemaphore_t s_axp_mutex_buf;
static SemaphoreHandle_t s_axp_mutex = NULL;
static volatile int s_axp_mutex_state = 0; /* 0 = uninitialized, 1 = creating, 2 = ready */

static i2c_master_dev_handle_t s_axp_dev = NULL;
static const bsp_axp192_reg_ops_t *s_axp_reg_ops = NULL;
static bool s_is_axp2101 = false;
static bool s_speaker_enabled = false;
static bool s_led_on = true;
static uint8_t s_reg94_shadow = 0x00U;
static uint8_t s_backlight_percent = 85;

void bsp_axp192_lock_init(void)
{
    if (__atomic_load_n(&s_axp_mutex_state, __ATOMIC_ACQUIRE) == 2) {
        return;
    }
    int expected = 0;
    if (__atomic_compare_exchange_n(&s_axp_mutex_state, &expected, 1, false,
                                    __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        /* Created outside any spinlock critical section */
        s_axp_mutex = xSemaphoreCreateMutexStatic(&s_axp_mutex_buf);
        __atomic_store_n(&s_axp_mutex_state, 2, __ATOMIC_RELEASE);
    } else {
        while (__atomic_load_n(&s_axp_mutex_state, __ATOMIC_ACQUIRE) != 2) {
            vTaskDelay(1);
        }
    }
}

static void axp_lock(void)
{
    bsp_axp192_lock_init();
    (void)xSemaphoreTake(s_axp_mutex, portMAX_DELAY);
}

static void axp_unlock(void)
{
    if (s_axp_mutex != NULL) {
        (void)xSemaphoreGive(s_axp_mutex);
    }
}

void bsp_axp192_set_reg_ops(const bsp_axp192_reg_ops_t *ops)
{
    axp_lock();
    s_axp_reg_ops = ops;
    s_is_axp2101 = false;
    if (ops != NULL && ops->read_reg != NULL) {
        uint8_t chip_id = 0U;
        if (ops->read_reg(AXP_REG_CHIP_ID, &chip_id) == ESP_OK && chip_id == AXP2101_CHIP_ID) {
            s_is_axp2101 = true;
        }
    }
    axp_unlock();
}

uint8_t bsp_axp192_calc_gpio_state(uint8_t reg94_prev, bool spk_enable, bool led_on)
{
    const uint8_t mask = (uint8_t)(BSP_AXP192_GPIO1_LED_BIT | BSP_AXP192_GPIO2_SPK_EN_BIT);
    uint8_t bits = 0U;
    if (spk_enable) {
        bits |= (uint8_t)BSP_AXP192_GPIO2_SPK_EN_BIT;
    }
    /* GPIO1 green LED is active-low NMOS open-drain: 0 = ON, 1 = OFF */
    if (!led_on) {
        bits |= (uint8_t)BSP_AXP192_GPIO1_LED_BIT;
    }
    return (uint8_t)((reg94_prev & (uint8_t)(~mask)) | bits);
}

uint8_t bsp_axp192_get_reg94_state(void)
{
    axp_lock();
    uint8_t val = s_reg94_shadow;
    axp_unlock();
    return val;
}

bool bsp_axp192_get_led_state(void)
{
    axp_lock();
    bool on = s_led_on;
    axp_unlock();
    return on;
}

static esp_err_t axp192_write_reg_locked(uint8_t reg, uint8_t val)
{
    if (s_axp_reg_ops != NULL && s_axp_reg_ops->write_reg != NULL) {
        return s_axp_reg_ops->write_reg(reg, val);
    }
    if (s_axp_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_axp_dev, buf, sizeof(buf), AXP192_I2C_TIMEOUT_MS);
}

static esp_err_t axp192_read_reg_locked(uint8_t reg, uint8_t *out_val)
{
    if (out_val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_axp_reg_ops != NULL && s_axp_reg_ops->read_reg != NULL) {
        return s_axp_reg_ops->read_reg(reg, out_val);
    }
    if (s_axp_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_axp_dev, &reg, 1, out_val, 1, AXP192_I2C_TIMEOUT_MS);
}

static esp_err_t axp192_update_bits_locked(uint8_t reg, uint8_t mask, uint8_t set_bits)
{
    uint8_t val = 0;
    esp_err_t err = axp192_read_reg_locked(reg, &val);
    if (err != ESP_OK) {
        return err;
    }
    val = (uint8_t)((val & ~mask) | (set_bits & mask));
    return axp192_write_reg_locked(reg, val);
}

static esp_err_t axp192_lcd_reset_locked(void)
{
    if (s_axp_dev == NULL && s_axp_reg_ops == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* AXP2101 uses ALDO2 (reg 0x90 bit 1) for LCD+Touch reset; AXP192 uses GPIO4 (reg 0x96 bit 1) */
    const uint8_t rst_reg = s_is_axp2101 ? AXP2101_REG_LDO_ONOFF0 : AXP192_REG_GPIO34_STATE;
    esp_err_t err = axp192_update_bits_locked(rst_reg, (1U << 1), 0U);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    err = axp192_update_bits_locked(rst_reg, (1U << 1), (1U << 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    return err;
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

    axp_lock();

    if (s_axp_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BSP_AXP192_I2C_ADDR,
            .scl_speed_hz = BSP_I2C_CLK_SPEED_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_axp_dev);
        if (err != ESP_OK) {
            axp_unlock();
            ESP_LOGE(TAG, "Failed to add AXP192 I2C device: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Probe PMIC presence on internal I2C0 (0x34 is shared by AXP192 on Core2 v1.0 and AXP2101 on Core2 v1.1) */
    esp_err_t err = i2c_master_probe(bus_handle, BSP_AXP192_I2C_ADDR, AXP192_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PMIC (0x34) not responding on I2C0: %s", esp_err_to_name(err));
        (void)i2c_master_bus_rm_device(s_axp_dev);
        s_axp_dev = NULL;
        axp_unlock();
        return err;
    }

    uint8_t chip_id = 0U;
    err = axp192_read_reg_locked(AXP_REG_CHIP_ID, &chip_id);
    if (err != ESP_OK || (chip_id != AXP192_CHIP_ID && chip_id != AXP2101_CHIP_ID)) {
        ESP_LOGW(TAG, "Unrecognized PMIC at 0x34 (err=%s, chip_id=0x%02X); skipping PMIC register init",
                 esp_err_to_name(err), (unsigned)chip_id);
        axp_unlock();
        return (err != ESP_OK) ? err : ESP_ERR_NOT_SUPPORTED;
    }
    s_is_axp2101 = (chip_id == AXP2101_CHIP_ID);
    s_speaker_enabled = false;
    s_led_on = true;
    s_reg94_shadow = bsp_axp192_calc_gpio_state(s_reg94_shadow, s_speaker_enabled, s_led_on);

    if (s_is_axp2101) {
        /*
         * M5Stack Core2 v1.1 (AXP2101 @ 0x34, chip_id = 0x4A):
         * - Disable DCDC1-5 UVP power-off latch (reg 0x23 bits 0..4) so USB inrush dips do not power off PMIC
         * - DCDC1 = 3.3V (ESP32), DCDC3 = 3.3V
         * - ALDO2 = 3.3V (LCD+Touch RST), ALDO3 = 3.3V (SPK_EN, gated OFF), ALDO4 = 3.3V (LCD/TP/TF logic)
         * - BLDO1 = 2.8V (LCD backlight); preserve ALDO1 (bit0) and BLDO2 (bit5) untouched
         * - DLDO1 = 0.5V / OFF (vibration motor off)
         */
        (void)axp192_update_bits_locked(AXP2101_REG_DCDC_UVP_CTRL, 0x1FU, 0x00U);
        (void)axp192_write_reg_locked(AXP2101_REG_DCDC1_VOLT, 0x12U); /* 3.3V */
        (void)axp192_write_reg_locked(AXP2101_REG_DCDC3_VOLT, 0x6AU); /* 3.3V */
        (void)axp192_update_bits_locked(AXP2101_REG_DCDC_ONOFF, 0x05U, 0x05U);

        (void)axp192_write_reg_locked(AXP2101_REG_ALDO2_VOLT, 0x1CU); /* 3.3V LCD+TP RST */
        (void)axp192_write_reg_locked(AXP2101_REG_ALDO3_VOLT, 0x1CU); /* 3.3V SPK_EN */
        (void)axp192_write_reg_locked(AXP2101_REG_ALDO4_VOLT, 0x1CU); /* 3.3V TF/TP/LCD */
        (void)axp192_write_reg_locked(AXP2101_REG_BLDO1_VOLT, 0x17U); /* 2.8V LCD BL */
        (void)axp192_write_reg_locked(AXP2101_REG_DLDO1_VOLT, 0x00U); /* Vib motor off */

        /* Enable ALDO2 (0x02), ALDO4 (0x08), BLDO1 (0x10); clear ALDO3 (0x04) & DLDO1 (0x80); preserve ALDO1/BLDO2 */
        (void)axp192_update_bits_locked(AXP2101_REG_LDO_ONOFF0, 0x9EU, 0x1AU);

        (void)axp192_write_reg_locked(AXP2101_REG_PWROFF_THLD, 0x00U);
        (void)axp192_write_reg_locked(AXP2101_REG_PMU_COMMON, 0x30U);
        (void)axp192_write_reg_locked(AXP2101_REG_BAT_DET_CTRL, 0x01U);
        (void)axp192_write_reg_locked(AXP2101_REG_CHGLED_CTRL, 0x35U); /* LED ON */
        (void)axp192_write_reg_locked(AXP2101_REG_ADC_EN, 0x0FU);
        (void)axp192_write_reg_locked(AXP2101_REG_CHG_VOLT_CTRL, 0x03U); /* 4.2V */
        (void)axp192_update_bits_locked(AXP2101_REG_TS_PIN_CTRL, 0x10U, 0x10U);

        (void)axp192_lcd_reset_locked();
        axp_unlock();

        ESP_LOGI(TAG, "AXP2101 PMIC (Core2 v1.1, id=0x%02X) initialized (DCDC1=3.3V, ALDO4=3.3V, BLDO1=2.8V, SPK_EN=OFF)",
                 (unsigned)chip_id);
        return ESP_OK;
    }

    /*
     * M5Stack Core2 v1.0 (AXP192 @ 0x34, chip_id = 0x03):
     * 1. VBUS-IPSOUT path: 500 mA VBUS current limit (0x02)
     * 2. Keep GPIO0 floating (0x07) so N_VBUSEN stays pulled low and USB VBUS is never cut off or back-fed
     */
    (void)axp192_update_bits_locked(AXP192_REG_VBUS_IPSOUT, 0x07, 0x02);
    (void)axp192_write_reg_locked(AXP192_REG_GPIO0_MODE, 0x07);
    (void)axp192_write_reg_locked(AXP192_REG_GPIO0_LDO_VOLT, 0xA0);

    /* 3. Set DCDC1 (ESP32 3.35V -> 0x6A) and LDO2 (LCD/SD 3.3V -> upper nibble 0xF0, LDO3 Vib 0V) */
    (void)axp192_write_reg_locked(AXP192_REG_DCDC1_VOLTAGE, 0x6A);
    (void)axp192_write_reg_locked(AXP192_REG_LDO2_3_VOLTAGE, 0xF0);

    /* 4. Set DCDC3 (LCD Backlight ~2.8V -> 0x54) */
    (void)axp192_write_reg_locked(AXP192_REG_DCDC3_VOLTAGE, 0x54);

    /* 5. Enable DCDC1 (bit0), DCDC3 (bit1), LDO2 (bit2); keep LDO3 (bit3) & DCDC2 (bit4) off, leave EXTEN untouched */
    (void)axp192_update_bits_locked(AXP192_REG_OUTPUT_CTRL, 0x1F, 0x07);

    /* 6. Configure GPIO1 (Power LED, NMOS open-drain) and GPIO2 (Speaker SPK_EN, NMOS open-drain) */
    (void)axp192_write_reg_locked(AXP192_REG_GPIO1_MODE, 0x00);
    (void)axp192_write_reg_locked(AXP192_REG_GPIO2_MODE, 0x00);
    /* Turn green power LED ON (bit1 = 0, active-low) and gate Speaker SPK_EN OFF (bit2 = 0) by default */
    uint8_t reg94 = 0U;
    if (axp192_read_reg_locked(AXP192_REG_GPIO012_STATE, &reg94) != ESP_OK) {
        reg94 = s_reg94_shadow;
    }
    s_reg94_shadow = bsp_axp192_calc_gpio_state(reg94, s_speaker_enabled, s_led_on);
    (void)axp192_write_reg_locked(AXP192_REG_GPIO012_STATE, s_reg94_shadow);

    /* 7. Enable RTC backup charge, battery and VBUS ADCs */
    (void)axp192_update_bits_locked(AXP192_REG_RTC_BAT_CHG, 0xE3, 0xA2);
    (void)axp192_write_reg_locked(AXP192_REG_ADC_EN1, 0xFF);
    (void)axp192_write_reg_locked(AXP192_REG_CHARGE_CTRL1, 0xC0);

    /* 8. Configure GPIO4 as NMOS open-drain output for LCD reset (TFT_RST) and pulse reset */
    (void)axp192_update_bits_locked(AXP192_REG_GPIO34_CTRL, 0x8D, 0x84);
    (void)axp192_lcd_reset_locked();

    axp_unlock();

    ESP_LOGI(TAG, "AXP192 PMIC (id=0x%02X) initialized (DCDC1=3.35V, LDO2=3.3V, DCDC3=2.8V, SPK_EN=OFF)",
             (unsigned)chip_id);
    return ESP_OK;
}

esp_err_t bsp_axp192_get_battery(uint16_t *out_voltage_mv, uint8_t *out_percent, bool *out_charging)
{
    uint8_t hi = 0;
    uint8_t lo = 0;
    uint8_t chg = 0;
    bool is_2101 = false;

    axp_lock();
    is_2101 = s_is_axp2101;
    esp_err_t err;
    if (is_2101) {
        err = axp192_read_reg_locked(AXP2101_REG_VBAT_HI, &hi);
        if (err == ESP_OK) {
            err = axp192_read_reg_locked(AXP2101_REG_VBAT_LO, &lo);
        }
        if (err == ESP_OK) {
            err = axp192_read_reg_locked(AXP192_REG_CHARGE_STATUS, &chg);
        }
    } else {
        err = axp192_read_reg_locked(AXP192_REG_VBAT_ADC_HI, &hi);
        if (err == ESP_OK) {
            err = axp192_read_reg_locked(AXP192_REG_VBAT_ADC_LO, &lo);
        }
        if (err == ESP_OK) {
            err = axp192_read_reg_locked(AXP192_REG_CHARGE_STATUS, &chg);
        }
    }
    axp_unlock();

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

    uint16_t vbat_mv;
    bool charging;
    if (is_2101) {
        uint16_t raw14 = (uint16_t)(((uint16_t)(hi & 0x3FU) << 8) | lo);
        /* Open 14-bit ADC channel on AXP2101 reads near full-scale (0x3FFF) when battery is absent */
        vbat_mv = (raw14 >= 0x3FDFU) ? 0U : raw14;
        charging = ((chg & 0x60U) == 0x20U);
    } else {
        vbat_mv = bsp_axp192_raw12_to_mv(hi, lo);
        charging = ((chg & (1U << 6)) != 0U);
    }

    if (out_voltage_mv != NULL) {
        *out_voltage_mv = vbat_mv;
    }
    if (out_percent != NULL) {
        *out_percent = bsp_axp192_calc_battery_percent(vbat_mv);
    }
    if (out_charging != NULL) {
        *out_charging = charging;
    }
    return ESP_OK;
}

esp_err_t bsp_axp192_set_speaker_enable(bool enable)
{
    axp_lock();
    if (s_axp_dev == NULL && s_axp_reg_ops == NULL) {
        s_speaker_enabled = enable;
        s_reg94_shadow = bsp_axp192_calc_gpio_state(s_reg94_shadow, s_speaker_enabled, s_led_on);
        axp_unlock();
        return ESP_OK;
    }

    esp_err_t err;
    if (s_is_axp2101) {
        /* AXP2101 (Core2 v1.1): ALDO3 (reg 0x90 bit 2 = 0x04) drives NS4168 SPK_EN at 3.3V */
        if (enable) {
            (void)axp192_write_reg_locked(AXP2101_REG_ALDO3_VOLT, 0x1CU);
        }
        err = axp192_update_bits_locked(AXP2101_REG_LDO_ONOFF0, (1U << 2), enable ? (1U << 2) : 0U);
        if (err == ESP_OK) {
            s_speaker_enabled = enable;
            s_reg94_shadow = bsp_axp192_calc_gpio_state(s_reg94_shadow, enable, s_led_on);
        }
    } else {
        /* AXP192 Reg 0x94 Bit 2 (0x04) controls GPIO2 (SPK_EN on M5Stack Core2 v1.0) */
        uint8_t reg94 = 0U;
        err = axp192_read_reg_locked(AXP192_REG_GPIO012_STATE, &reg94);
        if (err == ESP_OK) {
            uint8_t next_reg94 = bsp_axp192_calc_gpio_state(reg94, enable, s_led_on);
            err = axp192_write_reg_locked(AXP192_REG_GPIO012_STATE, next_reg94);
            if (err == ESP_OK) {
                s_speaker_enabled = enable;
                s_reg94_shadow = next_reg94;
            }
        }
    }
    axp_unlock();
    return err;
}

bool bsp_axp192_get_speaker_enable(void)
{
    axp_lock();
    bool enabled = s_speaker_enabled;
    axp_unlock();
    return enabled;
}

esp_err_t bsp_axp192_set_lcd_backlight(uint8_t brightness_percent)
{
    if (brightness_percent > 100U) {
        brightness_percent = 100U;
    }

    axp_lock();
    s_backlight_percent = brightness_percent;

    if (s_axp_dev == NULL && s_axp_reg_ops == NULL) {
        axp_unlock();
        return ESP_OK;
    }

    if (s_is_axp2101) {
        /* AXP2101 (Core2 v1.1): BLDO1 (reg 0x96, 100mV/step from 500mV, enable bit 4 in reg 0x90) */
        if (brightness_percent == 0U) {
            esp_err_t err = axp192_update_bits_locked(AXP2101_REG_LDO_ONOFF0, (1U << 4), 0U);
            axp_unlock();
            return err;
        }
        uint32_t mv = 2500U + ((uint32_t)brightness_percent * 800U) / 100U;
        uint8_t reg_val = (uint8_t)((mv - 500U) / 100U);
        esp_err_t err = axp192_write_reg_locked(AXP2101_REG_BLDO1_VOLT, reg_val);
        if (err == ESP_OK) {
            err = axp192_update_bits_locked(AXP2101_REG_LDO_ONOFF0, (1U << 4), (1U << 4));
        }
        axp_unlock();
        return err;
    }

    if (brightness_percent == 0U) {
        esp_err_t err = axp192_update_bits_locked(AXP192_REG_OUTPUT_CTRL, (1U << 1), 0U);
        axp_unlock();
        return err;
    }

    /* Map 1..100% to 2500..3100 mV on DCDC3 (700mV + reg * 25mV) */
    uint32_t mv = 2500U + ((uint32_t)brightness_percent * 600U) / 100U;
    uint8_t reg_val = (uint8_t)((mv - 700U) / 25U);
    esp_err_t err = axp192_write_reg_locked(AXP192_REG_DCDC3_VOLTAGE, reg_val);
    if (err == ESP_OK) {
        err = axp192_update_bits_locked(AXP192_REG_OUTPUT_CTRL, (1U << 1), (1U << 1));
    }
    axp_unlock();
    return err;
}

esp_err_t bsp_axp192_set_led(bool on)
{
    axp_lock();
    if (s_axp_dev == NULL && s_axp_reg_ops == NULL) {
        s_led_on = on;
        s_reg94_shadow = bsp_axp192_calc_gpio_state(s_reg94_shadow, s_speaker_enabled, s_led_on);
        axp_unlock();
        return ESP_OK;
    }

    esp_err_t err;
    if (s_is_axp2101) {
        err = axp192_write_reg_locked(AXP2101_REG_CHGLED_CTRL, on ? 0x35U : 0x05U);
        if (err == ESP_OK) {
            s_led_on = on;
            s_reg94_shadow = bsp_axp192_calc_gpio_state(s_reg94_shadow, s_speaker_enabled, on);
        }
    } else {
        /* AXP192 Reg 0x94 Bit 1 (0x02) controls GPIO1 (active-low open-drain sink for green LED) */
        uint8_t reg94 = 0U;
        err = axp192_read_reg_locked(AXP192_REG_GPIO012_STATE, &reg94);
        if (err == ESP_OK) {
            uint8_t next_reg94 = bsp_axp192_calc_gpio_state(reg94, s_speaker_enabled, on);
            err = axp192_write_reg_locked(AXP192_REG_GPIO012_STATE, next_reg94);
            if (err == ESP_OK) {
                s_led_on = on;
                s_reg94_shadow = next_reg94;
            }
        }
    }
    axp_unlock();
    return err;
}

esp_err_t bsp_axp192_lcd_reset(void)
{
    axp_lock();
    esp_err_t err = axp192_lcd_reset_locked();
    axp_unlock();
    return err;
}
