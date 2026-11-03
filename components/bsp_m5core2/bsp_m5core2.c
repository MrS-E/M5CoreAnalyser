/**
 * @file bsp_m5core2.c
 * @brief M5Stack Core2 Board Support Package: I2C0, ILI9342C SPI LCD, FT6336U Touch, and ADC1.
 */

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"

#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_m5core2";

#define FT6336U_REG_TD_STATUS              (0x02)
#define FT6336U_I2C_TIMEOUT_MS             (30)

static bool s_bsp_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_touch_dev = NULL;
static esp_lcd_panel_io_handle_t s_lcd_io = NULL;
static esp_lcd_panel_handle_t s_lcd_panel = NULL;

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_adc1_cali_handle = NULL;
static bool s_adc1_calibrated = false;

static esp_err_t bsp_i2c_init_internal(void)
{
    if (s_i2c_bus != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BSP_I2C_PORT,
        .sda_io_num = BSP_I2C_SDA_PIN,
        .scl_io_num = BSP_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize internal I2C0 bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t touch_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_FT6336U_I2C_ADDR,
        .scl_speed_hz = BSP_I2C_CLK_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &touch_cfg, &s_touch_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register FT6336U touch device on I2C0: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_bus(void)
{
    return s_i2c_bus;
}

/**
 * @brief Send ILI9342C vendor initialization sequence over SPI panel IO.
 */
static void ili9342c_send_vendor_init(esp_lcd_panel_io_handle_t io)
{
    static const uint8_t extc_cmd[] = {0xFF, 0x93, 0x42};
    static const uint8_t pwr1_cmd[] = {0x12, 0x12};
    static const uint8_t pwr2_cmd[] = {0x03};
    static const uint8_t vcom1_cmd[] = {0xF2};
    static const uint8_t madctl_cmd[] = {0x08}; /* BGR color order, landscape 320x240 */
    static const uint8_t colmod_cmd[] = {0x55}; /* 16-bit RGB565 */
    static const uint8_t dfunc_cmd[] = {0x08, 0x82, 0x27};
    static const uint8_t pgamma_cmd[] = {
        0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89,
        0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34, 0x0F
    };
    static const uint8_t ngamma_cmd[] = {
        0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67,
        0x48, 0x07, 0x0E, 0x0B, 0x2E, 0x33, 0x0F
    };

    (void)esp_lcd_panel_io_tx_param(io, 0xC8, extc_cmd, sizeof(extc_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xC0, pwr1_cmd, sizeof(pwr1_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xC1, pwr2_cmd, sizeof(pwr2_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xC5, vcom1_cmd, sizeof(vcom1_cmd));
    (void)esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, madctl_cmd, sizeof(madctl_cmd));
    (void)esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, colmod_cmd, sizeof(colmod_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xB6, dfunc_cmd, sizeof(dfunc_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xE0, pgamma_cmd, sizeof(pgamma_cmd));
    (void)esp_lcd_panel_io_tx_param(io, 0xE1, ngamma_cmd, sizeof(ngamma_cmd));
    (void)esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    (void)esp_lcd_panel_io_tx_param(io, LCD_CMD_INVON, NULL, 0);
    (void)esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0);
}

esp_err_t bsp_display_init(void)
{
    if (s_lcd_panel != NULL) {
        return ESP_OK;
    }

    /* Pull TF-Card CS (GPIO4) high so SD card never contends on SPI2 */
    gpio_config_t sd_cs_cfg = {
        .pin_bit_mask = (1ULL << BSP_SDCARD_PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&sd_cs_cfg);
    (void)gpio_set_level(BSP_SDCARD_PIN_CS, 1);

    /* Configure Touch IRQ (GPIO39) as input */
    gpio_config_t touch_irq_cfg = {
        .pin_bit_mask = (1ULL << BSP_TOUCH_PIN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&touch_irq_cfg);

    spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_PIN_SCLK,
        .mosi_io_num = BSP_LCD_PIN_MOSI,
        .miso_io_num = BSP_LCD_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BSP_LCD_H_RES * 40 * sizeof(uint16_t),
    };

    esp_err_t err = spi_bus_initialize(BSP_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI2 bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_PIN_DC,
        .cs_gpio_num = BSP_LCD_PIN_CS,
        .pclk_hz = BSP_LCD_SPI_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST, &io_config, &s_lcd_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD panel IO init failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1, /* Reset is controlled via AXP192 GPIO4 */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    err = esp_lcd_new_panel_st7789(s_lcd_io, &panel_config, &s_lcd_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD panel creation failed: %s", esp_err_to_name(err));
        return err;
    }

    (void)esp_lcd_panel_reset(s_lcd_panel);
    (void)esp_lcd_panel_init(s_lcd_panel);
    ili9342c_send_vendor_init(s_lcd_io);
    (void)esp_lcd_panel_invert_color(s_lcd_panel, true);
    (void)esp_lcd_panel_disp_on_off(s_lcd_panel, true);

    ESP_LOGI(TAG, "ILI9342C 320x240 SPI2 display initialized @ %d MHz", BSP_LCD_SPI_CLOCK_HZ / 1000000);
    return ESP_OK;
}

esp_lcd_panel_handle_t bsp_display_get_panel(void)
{
    return s_lcd_panel;
}

esp_lcd_panel_io_handle_t bsp_display_get_panel_io(void)
{
    return s_lcd_io;
}

esp_err_t bsp_touch_read_state(bsp_touch_state_t *out_state)
{
    if (out_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_state = (bsp_touch_state_t){0};

    if (s_touch_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = FT6336U_REG_TD_STATUS;
    uint8_t data[5] = {0};
    esp_err_t err = i2c_master_transmit_receive(
        s_touch_dev, &reg, 1, data, sizeof(data), FT6336U_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t touch_points = data[0] & 0x0FU;
    if (touch_points == 0U || touch_points > 2U) {
        return ESP_OK;
    }

    uint16_t raw_x = (uint16_t)(((uint16_t)(data[1] & 0x0FU) << 8) | data[2]);
    uint16_t raw_y = (uint16_t)(((uint16_t)(data[3] & 0x0FU) << 8) | data[4]);
    out_state->raw_x = raw_x;
    out_state->raw_y = raw_y;

    if (raw_y >= BSP_LCD_V_RES) {
        /* Bottom capacitive touch buttons region (y = 240..280) on M5Stack Core2 */
        if (raw_x < 106U) {
            out_state->btn_a = true;
        } else if (raw_x < 214U) {
            out_state->btn_b = true;
        } else {
            out_state->btn_c = true;
        }
        out_state->pressed = false;
        out_state->x = (raw_x >= BSP_LCD_H_RES) ? (BSP_LCD_H_RES - 1) : raw_x;
        out_state->y = BSP_LCD_V_RES - 1;
        return ESP_OK;
    }

    out_state->pressed = true;
    out_state->x = (raw_x >= BSP_LCD_H_RES) ? (BSP_LCD_H_RES - 1) : raw_x;
    out_state->y = raw_y;
    return ESP_OK;
}

esp_err_t bsp_touch_read(uint16_t *out_x, uint16_t *out_y, bool *out_pressed)
{
    bsp_touch_state_t st = {0};
    esp_err_t err = bsp_touch_read_state(&st);
    if (out_x != NULL) {
        *out_x = st.x;
    }
    if (out_y != NULL) {
        *out_y = st.y;
    }
    if (out_pressed != NULL) {
        *out_pressed = st.pressed;
    }
    return err;
}

static esp_err_t bsp_pin_to_adc1_channel(gpio_num_t pin, adc_channel_t *out_chan)
{
    if (out_chan == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pin == BSP_PIN_ANALOG_IN_DEFAULT) {
        *out_chan = ADC_CHANNEL_0; /* GPIO36 */
        return ESP_OK;
    }
    if (pin == BSP_PIN_ANALOG_IN_ALT) {
        *out_chan = ADC_CHANNEL_7; /* GPIO35 */
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t bsp_adc1_init(void)
{
    if (s_adc1_handle != NULL) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 unit init failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    (void)adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_0, &chan_cfg); /* GPIO36 */
    (void)adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_7, &chan_cfg); /* GPIO35 */

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc1_cali_handle) == ESP_OK) {
        s_adc1_calibrated = true;
    }
#endif

#if CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK
    ESP_LOGW(TAG, "ADC1 mid-scale mock fallback ACTIVE (test-only build, CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK=y)");
#endif
    ESP_LOGI(TAG, "ADC1 initialized on GPIO36 (CH0) & GPIO35 (CH7), cali=%d, mock=%d",
             (int)s_adc1_calibrated, (int)bsp_adc1_uses_mock_fallback());
    return ESP_OK;
}

bool bsp_adc1_uses_mock_fallback(void)
{
#if CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK
    return true;
#else
    return false;
#endif
}

esp_err_t bsp_adc1_read_raw(gpio_num_t pin, int *out_raw)
{
    if (out_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    adc_channel_t chan;
    esp_err_t err = bsp_pin_to_adc1_channel(pin, &chan);
    if (err != ESP_OK) {
        return err;
    }
    if (s_adc1_handle == NULL) {
        err = bsp_adc1_init();
        if (err != ESP_OK) {
            return err;
        }
    }
#if CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK
    /* Test-only (QEMU): SAR ADC MMIO is not emulated and adc_oneshot_read() would spin forever. */
    (void)chan;
    *out_raw = 2048;
    return ESP_OK;
#else
    return adc_oneshot_read(s_adc1_handle, chan, out_raw);
#endif
}

esp_err_t bsp_adc1_raw_to_mv(gpio_num_t pin, int raw, int *out_mv)
{
    if (out_mv == NULL || raw < 0 || raw > 4095) {
        return ESP_ERR_INVALID_ARG;
    }
    adc_channel_t chan;
    esp_err_t err = bsp_pin_to_adc1_channel(pin, &chan);
    if (err != ESP_OK) {
        return err;
    }

    if (s_adc1_calibrated && s_adc1_cali_handle != NULL) {
        if (adc_cali_raw_to_voltage(s_adc1_cali_handle, raw, out_mv) == ESP_OK) {
            return ESP_OK;
        }
    }

    /* Deterministic linear fallback: 0..4095 -> 0..3300 mV */
    *out_mv = (raw * 3300 + 2047) / 4095;
    return ESP_OK;
}

esp_err_t bsp_m5core2_init(void)
{
    bsp_axp192_lock_init();
    bsp_spk_passthrough_lock_init();
    bsp_pin_manager_reset();

    size_t psram_size = esp_psram_get_size();
    ESP_LOGI(TAG, "M5Stack Core2 BSP init starting (PSRAM=%u bytes)", (unsigned)psram_size);

    esp_err_t first_err = ESP_OK;

    esp_err_t err = bsp_i2c_init_internal();
    if (err != ESP_OK && first_err == ESP_OK) {
        first_err = err;
    }

    if (s_i2c_bus != NULL) {
        err = bsp_axp192_init(s_i2c_bus);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }

    err = bsp_display_init();
    if (err != ESP_OK && first_err == ESP_OK) {
        first_err = err;
    }

    err = bsp_adc1_init();
    if (err != ESP_OK && first_err == ESP_OK) {
        first_err = err;
    }

    s_bsp_initialized = (first_err == ESP_OK);
    return first_err;
}

bool bsp_m5core2_is_initialized(void)
{
    return s_bsp_initialized;
}
