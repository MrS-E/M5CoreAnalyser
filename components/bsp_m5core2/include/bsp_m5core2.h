/**
 * @file bsp_m5core2.h
 * @brief Board Support Package (BSP) for the M5Stack Core2 in M5CoreAnalyser.
 *
 * Provides initialization and hardware abstraction for the M5Stack Core2:
 * - AXP192 PMIC power rails, battery telemetry, LCD backlight/reset, LED, and speaker amp gate
 * - ILI9342C 320x240 16-bit RGB565 SPI LCD panel
 * - FT6336U capacitive touch controller over internal I2C0
 * - Safe GPIO pin reservation manager preventing conflicts with internal peripherals
 * - Shared calibrated ADC1 access on GPIO36 (ADC1_CH0) and GPIO35 (ADC1_CH7)
 * - Software-only Analog-to-Speaker audio passthrough over I2S1 (NS4168 amplifier)
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name M5Stack Core2 Display Geometry
 *  @{
 */
#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (240)
#define BSP_LCD_SPI_CLOCK_HZ       (40 * 1000 * 1000)
/** @} */

/** @name Internal I2C0 Bus Pins & Device Addresses
 *  @{
 */
#define BSP_I2C_PORT               (I2C_NUM_0)
#define BSP_I2C_SDA_PIN            (GPIO_NUM_21)
#define BSP_I2C_SCL_PIN            (GPIO_NUM_22)
#define BSP_I2C_CLK_SPEED_HZ       (400000)

#define BSP_AXP192_I2C_ADDR        (0x34)
#define BSP_FT6336U_I2C_ADDR       (0x38)
#define BSP_BM8563_I2C_ADDR        (0x51)
#define BSP_MPU6886_I2C_ADDR       (0x68)
/** @} */

/** @name SPI2 LCD & TF-Card Bus Pins
 *  @{
 */
#define BSP_LCD_SPI_HOST           (SPI2_HOST)
#define BSP_LCD_PIN_MOSI           (GPIO_NUM_23)
#define BSP_LCD_PIN_MISO           (GPIO_NUM_38)
#define BSP_LCD_PIN_SCLK           (GPIO_NUM_18)
#define BSP_LCD_PIN_CS             (GPIO_NUM_5)
#define BSP_LCD_PIN_DC             (GPIO_NUM_15)
#define BSP_SDCARD_PIN_CS          (GPIO_NUM_4)
#define BSP_TOUCH_PIN_INT          (GPIO_NUM_39)
/** @} */

/** @name NS4168 Speaker I2S1 Pins
 *  @{
 */
#define BSP_SPK_I2S_PORT           (1)
#define BSP_SPK_PIN_BCK            (GPIO_NUM_12)
#define BSP_SPK_PIN_WS             (GPIO_NUM_0)
#define BSP_SPK_PIN_DOUT           (GPIO_NUM_2)
#define BSP_SPK_DEFAULT_RATE_HZ    (16000)
#define BSP_SPK_DEFAULT_GAIN_PCT   (100)
#define BSP_SPK_MAX_GAIN_PCT       (200)
/** @} */

/** @name Default Instrument Expansion Pins (Port A, Port B, Port C, M5-Bus)
 *  @{
 */
#define BSP_PIN_ANALOG_IN_DEFAULT  (GPIO_NUM_36) /**< Port B / M5-Bus input-only ADC1_CH0 */
#define BSP_PIN_ANALOG_IN_ALT      (GPIO_NUM_35) /**< M5-Bus input-only ADC1_CH7 */
#define BSP_PIN_DAC_OUT_CH1        (GPIO_NUM_25) /**< M5-Bus DAC1 (8-bit) */
#define BSP_PIN_DAC_OUT_CH2        (GPIO_NUM_26) /**< Port B / M5-Bus DAC2 (8-bit) / Digital I/O */
#define BSP_PIN_PORT_A_SDA         (GPIO_NUM_32) /**< Port A GPIO32 */
#define BSP_PIN_PORT_A_SCL         (GPIO_NUM_33) /**< Port A GPIO33 */
#define BSP_PIN_PORT_C_RXD2        (GPIO_NUM_13) /**< Port C GPIO13 */
#define BSP_PIN_PORT_C_TXD2        (GPIO_NUM_14) /**< Port C GPIO14 */
#define BSP_PIN_MBUS_GPIO27        (GPIO_NUM_27) /**< M5-Bus GPIO27 */
#define BSP_PIN_MBUS_GPIO19        (GPIO_NUM_19) /**< M5-Bus GPIO19 */
/** @} */

/**
 * @brief Touch state structure including active screen coordinates and Core2 bottom touch buttons.
 */
typedef struct {
    bool pressed;      /**< True if a touch point is active in the 320x240 display area */
    uint16_t x;        /**< Clamped horizontal coordinate (0..319) */
    uint16_t y;        /**< Clamped vertical coordinate (0..239) */
    uint16_t raw_x;    /**< Unclamped FT6336U X coordinate */
    uint16_t raw_y;    /**< Unclamped FT6336U Y coordinate (240..280 corresponds to touch circles) */
    bool btn_a;        /**< True if bottom-left capacitive circle (BtnA) is touched */
    bool btn_b;        /**< True if bottom-center capacitive circle (BtnB) is touched */
    bool btn_c;        /**< True if bottom-right capacitive circle (BtnC) is touched */
} bsp_touch_state_t;

/**
 * @brief Software DSP filter state for the Analog-to-Speaker passthrough pipeline.
 *
 * Implements the 1st-order DC-blocking high-pass filter:
 * \f$ y[n] = x[n] - x[n-1] + 0.995 \cdot y[n-1] \f$
 * followed by adjustable digital gain (`0%`..`200%`), mute gating, and hard-clipping
 * saturation to `INT16_MIN..INT16_MAX`.
 */
typedef struct {
    float prev_x;         /**< Previous input sample x[n-1] (centered signed float) */
    float prev_y;         /**< Previous filter output y[n-1] */
    uint8_t gain_percent; /**< Digital gain percentage (0..200, where 100 = unity gain) */
    bool muted;           /**< When true, output samples are forced to 0 */
    bool initialized;     /**< True once first sample has seeded prev_x */
} bsp_spk_dsp_state_t;

/**
 * @brief Initialize the M5Stack Core2 Board Support Package.
 *
 * Initializes the GPIO pin reservation manager, internal I2C0 bus (`GPIO21`/`GPIO22`),
 * AXP192 PMIC power rails, ILI9342C 320x240 SPI LCD, FT6336U touch controller,
 * and shared ADC1 calibration unit.
 *
 * @return
 *  - `ESP_OK` if all hardware subsystems initialized cleanly
 *  - Error code from the failing subsystem otherwise (pin manager and software DSP remain usable)
 */
esp_err_t bsp_m5core2_init(void);

/**
 * @brief Return true if `bsp_m5core2_init()` has completed hardware initialization.
 *
 * @return true if initialized, false otherwise.
 */
bool bsp_m5core2_is_initialized(void);

/**
 * @brief Retrieve the internal I2C0 master bus handle (`GPIO21` SDA, `GPIO22` SCL).
 *
 * @return `i2c_master_bus_handle_t` or `NULL` if I2C0 is not initialized.
 */
i2c_master_bus_handle_t bsp_i2c_get_bus(void);

/* --------------------------------------------------------------------------
 * AXP192 PMIC Power, Battery & Peripheral Control
 * -------------------------------------------------------------------------- */

/** @name AXP192 GPIO[2:0] Signal Status Register (0x94) Bit Masks
 *  @{
 */
#define BSP_AXP192_REG_GPIO012_STATE   (0x94U)
#define BSP_AXP192_GPIO1_LED_BIT       (1U << 1) /**< Bit 1 (0x02): GPIO1 Power LED (active-low: 0=ON, 1=OFF) */
#define BSP_AXP192_GPIO2_SPK_EN_BIT    (1U << 2) /**< Bit 2 (0x04): GPIO2 NS4168 SPK_EN (active-high: 1=ON, 0=OFF) */
/** @} */

/**
 * @brief Compute the updated AXP192 register `0x94` (`GPIO[2:0]` state) byte for speaker and LED control.
 *
 * Pure register calculation helper matching upstream M5Stack Core2 `AXP192` semantics:
 * - Preserves bit 0 (`GPIO0` bus LDO state) and upper bits `[7:3]` in `reg94_prev`.
 * - Sets/clears bit 2 (`0x04`, `BSP_AXP192_GPIO2_SPK_EN_BIT`) for `SPK_EN` (active-high: `true` -> `1`).
 * - Clears/sets bit 1 (`0x02`, `BSP_AXP192_GPIO1_LED_BIT`) for `SYS_LED` (active-low: `true` -> `0`, `false` -> `1`).
 *
 * @param[in] reg94_prev Previous value of AXP192 register `0x94`.
 * @param[in] spk_enable True to enable NS4168 speaker amplifier (`bit 2 = 1`), false to disable (`bit 2 = 0`).
 * @param[in] led_on     True to turn green power LED on (`bit 1 = 0`), false to turn off (`bit 1 = 1`).
 * @return Updated AXP192 register `0x94` byte.
 */
uint8_t bsp_axp192_calc_gpio_state(uint8_t reg94_prev, bool spk_enable, bool led_on);

/**
 * @brief Return the last computed/written AXP192 register `0x94` (`GPIO[2:0]` state) byte.
 *
 * @return Shadow register `0x94` value.
 */
uint8_t bsp_axp192_get_reg94_state(void);

/**
 * @brief Query the last commanded state of the AXP192 `GPIO1` green power LED.
 *
 * @return True if LED is commanded on (`0x94` bit 1 = 0), false if commanded off (`0x94` bit 1 = 1).
 */
bool bsp_axp192_get_led_state(void);

/**
 * @brief Initialize the AXP192 PMIC on the specified I2C master bus.
 *
 * Configures DCDC1 (3.35V MCU), DCDC3 (2.8V LCD backlight), LDO2 (3.3V LCD/SD logic),
 * GPIO0 LDO (3.3V bus), GPIO1 (system LED), GPIO2 (NS4168 speaker amp off by default),
 * GPIO4 (LCD reset), and battery voltage/current ADCs.
 *
 * @param[in] bus_handle Initialized I2C0 master bus handle.
 * @return `ESP_OK` on success, or an `esp_err_t` I2C error code.
 */
esp_err_t bsp_axp192_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Convert raw AXP192 12-bit battery ADC register bytes (`0x78`, `0x79`) to millivolts.
 *
 * Pure conversion helper (`1.1 mV / LSB`).
 *
 * @param[in] reg_hi High 8 bits from register `0x78`.
 * @param[in] reg_lo Low 4 bits (in bits `[3:0]`) from register `0x79`.
 * @return Battery voltage in millivolts (`0`..`4504` mV).
 */
uint16_t bsp_axp192_raw12_to_mv(uint8_t reg_hi, uint8_t reg_lo);

/**
 * @brief Estimate state-of-charge percentage (`0`..`100%`) from battery millivolts.
 *
 * Maps `3200 mV` -> `0%` and `4150 mV` -> `100%` with clamping.
 *
 * @param[in] voltage_mv Battery voltage in millivolts.
 * @return Clamped battery percentage (`0`..`100`).
 */
uint8_t bsp_axp192_calc_battery_percent(uint16_t voltage_mv);

/**
 * @brief Read live battery voltage, estimated percentage, and charging status from AXP192.
 *
 * If AXP192 hardware is unavailable, writes safe fallback values (`3850 mV`, `68%`, `false`)
 * and returns `ESP_ERR_INVALID_STATE`.
 *
 * @param[out] out_voltage_mv Pointer to receive battery voltage in mV (may be `NULL`).
 * @param[out] out_percent    Pointer to receive battery percentage `0..100` (may be `NULL`).
 * @param[out] out_charging   Pointer to receive charging flag (may be `NULL`).
 * @return `ESP_OK` on hardware read success, or error code.
 */
esp_err_t bsp_axp192_get_battery(uint16_t *out_voltage_mv, uint8_t *out_percent, bool *out_charging);

/**
 * @brief Enable or disable the onboard NS4168 speaker amplifier via AXP192 GPIO2 (`SPK_EN`).
 *
 * @param[in] enable True to enable speaker amplifier power gate, false to mute/power down.
 * @return `ESP_OK` on success, or error code if AXP192 is not initialized.
 */
esp_err_t bsp_axp192_set_speaker_enable(bool enable);

/**
 * @brief Query the last commanded state of the AXP192 `SPK_EN` speaker power gate.
 *
 * @return True if speaker amplifier is enabled, false if disabled/gated off.
 */
bool bsp_axp192_get_speaker_enable(void);

/**
 * @brief Set M5Stack Core2 LCD backlight brightness via AXP192 DCDC3 (`0%`..`100%`).
 *
 * @param[in] brightness_percent Brightness level (`0` turns off DCDC3 backlight, `1`..`100` scales 2500..3100 mV).
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t bsp_axp192_set_lcd_backlight(uint8_t brightness_percent);

/**
 * @brief Control the green power indicator LED on AXP192 GPIO1.
 *
 * @param[in] on True to turn LED on, false to turn LED off.
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t bsp_axp192_set_led(bool on);

/**
 * @brief Pulse AXP192 GPIO4 (`TFT_RST`) to hardware-reset the ILI9342C LCD and FT6336U touch.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t bsp_axp192_lcd_reset(void);

/* --------------------------------------------------------------------------
 * ILI9342C SPI Display & FT6336U Capacitive Touch
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize the ILI9342C 320x240 SPI2 LCD panel and FT6336U touch controller.
 *
 * @return `ESP_OK` on success, or an `esp_err_t` code.
 */
esp_err_t bsp_display_init(void);

/**
 * @brief Get the initialized `esp_lcd_panel_handle_t` for the ILI9342C display.
 *
 * @return Panel handle, or `NULL` if display is not initialized.
 */
esp_lcd_panel_handle_t bsp_display_get_panel(void);

/**
 * @brief Get the initialized `esp_lcd_panel_io_handle_t` for the ILI9342C display.
 *
 * @return Panel IO handle, or `NULL` if display is not initialized.
 */
esp_lcd_panel_io_handle_t bsp_display_get_panel_io(void);

/**
 * @brief Read the current touch coordinates from the FT6336U capacitive touch controller.
 *
 * @param[out] out_x       Horizontal touch coordinate `0..319` (may be `NULL`).
 * @param[out] out_y       Vertical touch coordinate `0..239` (may be `NULL`).
 * @param[out] out_pressed True if screen is currently touched within `320x240` (may be `NULL`).
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t bsp_touch_read(uint16_t *out_x, uint16_t *out_y, bool *out_pressed);

/**
 * @brief Read extended touch state including the three M5Core2 bottom touch circles (`BtnA`/`BtnB`/`BtnC`).
 *
 * @param[out] out_state Pointer to `bsp_touch_state_t` structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_state` is `NULL`.
 */
esp_err_t bsp_touch_read_state(bsp_touch_state_t *out_state);

/* --------------------------------------------------------------------------
 * Shared Calibrated ADC1 Helper (GPIO36 / GPIO35)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize shared ADC1 oneshot unit and curve/line-fitting calibration for GPIO36 and GPIO35.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t bsp_adc1_init(void);

/**
 * @brief Read a single raw 12-bit ADC1 sample (`0..4095`) from `GPIO36` or `GPIO35`.
 *
 * Production builds always call `adc_oneshot_read()` and propagate its error code unchanged;
 * a failed hardware read is never replaced with a substitute value. The only exception is a
 * test-only build with `CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK=y` (set in `test_app/sdkconfig.defaults`
 * for QEMU, where SAR ADC registers are not emulated), which always returns mid-scale `2048`
 * and `ESP_OK`. Use `bsp_adc1_uses_mock_fallback()` to detect that mode.
 *
 * @param[in]  pin     `GPIO_NUM_36` (`ADC1_CHANNEL_0`) or `GPIO_NUM_35` (`ADC1_CHANNEL_7`).
 * @param[out] out_raw Pointer to store raw 12-bit reading.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` for unsupported pins or `NULL` output,
 *         or the `adc_oneshot_read()` error code on hardware failure.
 */
esp_err_t bsp_adc1_read_raw(gpio_num_t pin, int *out_raw);

/**
 * @brief Report whether `bsp_adc1_read_raw()` is running the test-only mid-scale mock fallback.
 *
 * Returns true only when built with `CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK=y` (test_app on QEMU).
 * Always false in production firmware, where every sample comes from real ADC hardware.
 *
 * @return true if raw reads are mock mid-scale values, false if they come from hardware.
 */
bool bsp_adc1_uses_mock_fallback(void);

/**
 * @brief Convert a raw 12-bit ADC1 sample (`0..4095`) into calibrated millivolts (`0..3300` mV).
 *
 * Uses ESP-IDF hardware `adc_cali_raw_to_voltage` when calibrated, or linear 12-bit 3300 mV
 * conversion fallback when running without hardware calibration eFuses.
 *
 * @param[in]  pin    `GPIO_NUM_36` or `GPIO_NUM_35`.
 * @param[in]  raw    Raw 12-bit ADC count (`0..4095`).
 * @param[out] out_mv Pointer to store calibrated voltage in mV.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t bsp_adc1_raw_to_mv(gpio_num_t pin, int raw, int *out_mv);

/* --------------------------------------------------------------------------
 * GPIO Pin Reservation & Conflict Manager
 * -------------------------------------------------------------------------- */

/** Maximum length (including null terminator) of a GPIO owner tool ID string. */
#define BSP_PIN_OWNER_MAX_LEN      (24)

/**
 * @brief Check whether a GPIO is a protected internal M5Core2 pin or invalid pin.
 *
 * Protected pins include internal I2C0 (`GPIO21`/`GPIO22`), SPI2 LCD & SD (`GPIO18`/`GPIO23`/
 * `GPIO38`/`GPIO5`/`GPIO15`/`GPIO4`), I2S1 Speaker (`GPIO12`/`GPIO0`/`GPIO2`), SPI Flash/PSRAM
 * (`GPIO6`–`GPIO11`, `GPIO16`, `GPIO17`), UART0 (`GPIO1`/`GPIO3`), Touch IRQ (`GPIO39`), and
 * internal PDM Mic (`GPIO34`).
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if the pin is protected/reserved and cannot be claimed by tools, false if claimable.
 */
bool bsp_pin_is_protected(gpio_num_t pin);

/**
 * @brief Check whether a GPIO is input-only on ESP32 (`GPIO34`..`GPIO39`).
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if input-only, false otherwise.
 */
bool bsp_pin_is_input_only(gpio_num_t pin);

/**
 * @brief Claim exclusive ownership of a safe external GPIO for an instrument tool.
 *
 * Rejects protected internal M5Core2 pins with `ESP_ERR_NOT_ALLOWED`, invalid GPIO numbers
 * or `NULL`/empty `owner_tool_id` with `ESP_ERR_INVALID_ARG`, and pins already claimed by a
 * different tool (e.g. a background generator) with `ESP_ERR_INVALID_STATE`.
 * Re-claiming a pin already owned by the same `owner_tool_id` is idempotent and returns `ESP_OK`.
 *
 * @param[in] pin           Target GPIO number.
 * @param[in] owner_tool_id Non-empty tool identifier string (e.g. `"scope"`, `"wavegen"`).
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `pin` is out of range or `owner_tool_id` is `NULL`/empty
 *  - `ESP_ERR_NOT_ALLOWED` if `pin` is a protected internal M5Core2 pin
 *  - `ESP_ERR_INVALID_STATE` if `pin` is currently owned by another tool
 */
esp_err_t bsp_pin_claim(gpio_num_t pin, const char *owner_tool_id);

/**
 * @brief Claim a GPIO with optional output-capability validation.
 *
 * Identical to `bsp_pin_claim()`, except when `require_output` is true, input-only pins
 * (`GPIO35`, `GPIO36`, etc.) are rejected with `ESP_ERR_NOT_SUPPORTED`.
 *
 * @param[in] pin            Target GPIO number.
 * @param[in] owner_tool_id  Non-empty tool identifier string.
 * @param[in] require_output True if the tool will drive the pin as an output.
 * @return `ESP_OK` on success, or an `esp_err_t` code.
 */
esp_err_t bsp_pin_claim_ex(gpio_num_t pin, const char *owner_tool_id, bool require_output);

/**
 * @brief Release a previously claimed GPIO.
 *
 * Only the current `owner_tool_id` may release the pin.
 *
 * @param[in] pin           Target GPIO number.
 * @param[in] owner_tool_id Tool identifier string that claimed the pin.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if arguments are invalid
 *  - `ESP_ERR_INVALID_STATE` if the pin is unowned or owned by a different tool
 */
esp_err_t bsp_pin_release(gpio_num_t pin, const char *owner_tool_id);

/**
 * @brief Release all GPIOs currently owned by `owner_tool_id`.
 *
 * @param[in] owner_tool_id Tool identifier string.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `owner_tool_id` is `NULL`/empty.
 */
esp_err_t bsp_pin_release_all_for_owner(const char *owner_tool_id);

/**
 * @brief Copy the current owner ID of a GPIO into a caller-provided buffer under spinlock.
 *
 * For protected system pins, copies `"system"` and returns `ESP_OK`.
 * For unclaimed user pins, writes `""` (`out_buf[0] = '\0'`) and returns `ESP_ERR_NOT_FOUND`.
 *
 * @param[in]  pin     Target GPIO number.
 * @param[out] out_buf Destination character buffer.
 * @param[in]  buf_len Size of `out_buf` in bytes (must be >= 2).
 * @return
 *  - `ESP_OK` if the pin has an owner (copied into `out_buf`)
 *  - `ESP_ERR_NOT_FOUND` if the pin is valid and unclaimed (`out_buf` set to empty string)
 *  - `ESP_ERR_INVALID_ARG` if `pin` is out of range, `out_buf` is `NULL`, or `buf_len < 2`
 */
esp_err_t bsp_pin_get_owner_copy(gpio_num_t pin, char *out_buf, size_t buf_len);

/**
 * @brief Check atomically whether a GPIO is currently owned by `owner_tool_id`.
 *
 * @param[in] pin           Target GPIO number.
 * @param[in] owner_tool_id Owner ID string to match (e.g. `"scope"`, `"wavegen"`, `"system"`).
 * @return True if `pin` is currently owned by `owner_tool_id`, false otherwise.
 */
bool bsp_pin_is_owned_by(gpio_num_t pin, const char *owner_tool_id);

/* --------------------------------------------------------------------------
 * Software-Only Analog-to-Speaker Passthrough (I2S1 + DSP Filter)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize a software speaker DSP filter state structure.
 *
 * @param[out] state        Pointer to DSP state structure.
 * @param[in]  gain_percent Initial digital gain percentage (`0`..`200`, clamped).
 */
void bsp_spk_dsp_init(bsp_spk_dsp_state_t *state, uint8_t gain_percent);

/**
 * @brief Reset the DC-blocker filter history (`prev_x`, `prev_y`) without altering gain/mute.
 *
 * @param[in,out] state Pointer to DSP state structure.
 */
void bsp_spk_dsp_reset(bsp_spk_dsp_state_t *state);

/**
 * @brief Run the DC-blocking high-pass filter, gain stage, and 16-bit PCM saturation on 12-bit ADC samples.
 *
 * Applies \f$ y[n] = x[n] - x[n-1] + 0.995 \cdot y[n-1] \f$ where `x[n]` is scaled from
 * 12-bit unsigned ADC counts (`0..4095`) to signed 16-bit full-scale amplitude (`-32768..+32752`),
 * then multiplies by `gain_percent / 100.0f` and clamps to `[INT16_MIN, INT16_MAX]`.
 *
 * @param[in,out] state       Pointer to DSP filter state.
 * @param[in]     adc_samples Input array of 12-bit ADC samples (`0..4095`).
 * @param[out]    out_pcm16   Output array of signed 16-bit PCM samples.
 * @param[in]     count       Number of samples to process.
 */
void bsp_spk_dsp_process_adc12(bsp_spk_dsp_state_t *state,
                               const uint16_t *adc_samples,
                               int16_t *out_pcm16,
                               size_t count);

/**
 * @brief Run the DC-blocking high-pass filter, gain stage, and 16-bit PCM saturation on 8-bit DAC samples.
 *
 * Applies \f$ y[n] = x[n] - x[n-1] + 0.995 \cdot y[n-1] \f$ where `x[n]` is scaled from
 * 8-bit unsigned DAC values (`0..255`) to signed 16-bit full-scale amplitude (`-32768..+32512`),
 * then multiplies by `gain_percent / 100.0f` and clamps to `[INT16_MIN, INT16_MAX]`.
 *
 * @param[in,out] state       Pointer to DSP filter state.
 * @param[in]     dac_samples Input array of 8-bit DAC LUT samples (`0..255`).
 * @param[out]    out_pcm16   Output array of signed 16-bit PCM samples.
 * @param[in]     count       Number of samples to process.
 */
void bsp_spk_dsp_process_u8(bsp_spk_dsp_state_t *state,
                            const uint8_t *dac_samples,
                            int16_t *out_pcm16,
                            size_t count);

/**
 * @brief Enable or disable the `I2S1` Analog-to-Speaker audio passthrough pipeline.
 *
 * When enabled, initializes/reconfigures `I2S_NUM_1` (`GPIO12` BCK, `GPIO0` WS, `GPIO2` DOUT)
 * at `sample_rate_hz`, resets the DC-blocker filter, and enables AXP192 `SPK_EN` (unless muted).
 * When disabled, writes a zeroed DMA silence frame to `I2S1`, disables `I2S1`, and gates off
 * AXP192 `SPK_EN` immediately to prevent pop/click or idle amplifier hiss.
 *
 * @param[in] enable         True to enable speaker passthrough, false to mute and disable.
 * @param[in] sample_rate_hz Sample rate in Hz (`4000`..`96000`; defaults to `16000` if `0`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `sample_rate_hz` is out of range.
 */
esp_err_t bsp_spk_passthrough_enable(bool enable, uint32_t sample_rate_hz);

/**
 * @brief Query whether speaker passthrough is currently enabled.
 *
 * @return True if enabled, false otherwise.
 */
bool bsp_spk_passthrough_is_enabled(void);

/**
 * @brief Query the currently configured speaker passthrough sample rate in Hz.
 *
 * @return Sample rate in Hz.
 */
uint32_t bsp_spk_passthrough_get_sample_rate(void);

/**
 * @brief Set the digital gain percentage (`0%`..`200%`) for speaker passthrough.
 *
 * Setting `gain_percent == 0` also gates off AXP192 `SPK_EN` to eliminate amplifier idle noise.
 * Values above `200` are clamped to `200%`.
 *
 * @param[in] gain_percent Gain percentage (`0`..`200`).
 * @return `ESP_OK` on success.
 */
esp_err_t bsp_spk_passthrough_set_gain(uint8_t gain_percent);

/**
 * @brief Get the current speaker passthrough digital gain percentage (`0`..`200`).
 *
 * @return Current gain percentage.
 */
uint8_t bsp_spk_passthrough_get_gain(void);

/**
 * @brief Mute or unmute speaker passthrough while keeping the stream active.
 *
 * Muting zeroes outgoing PCM frames and gates off AXP192 `SPK_EN`.
 *
 * @param[in] mute True to mute, false to unmute.
 * @return `ESP_OK` on success.
 */
esp_err_t bsp_spk_passthrough_set_mute(bool mute);

/**
 * @brief Query whether speaker passthrough is currently muted.
 *
 * @return True if muted, false otherwise.
 */
bool bsp_spk_passthrough_is_muted(void);

/**
 * @brief Push a batch of 12-bit unsigned ADC samples (`0..4095`) through the speaker passthrough.
 *
 * Filters out DC offset, applies digital gain and `INT16_MIN..INT16_MAX` saturation, and streams
 * the resulting 16-bit PCM audio to `I2S1` DMA when enabled.
 *
 * @param[in] adc_samples Pointer to 12-bit ADC sample buffer.
 * @param[in] count       Number of samples in `adc_samples`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `adc_samples` is `NULL` with `count > 0`,
 *         or `ESP_ERR_INVALID_STATE` if passthrough is disabled.
 */
esp_err_t bsp_spk_passthrough_push_adc12(const uint16_t *adc_samples, size_t count);

/**
 * @brief Push a batch of 8-bit unsigned DAC samples (`0..255`) through the speaker passthrough.
 *
 * Filters out DC offset, applies digital gain and `INT16_MIN..INT16_MAX` saturation, and streams
 * the resulting 16-bit PCM audio to `I2S1` DMA when enabled.
 *
 * @param[in] dac_samples Pointer to 8-bit DAC sample buffer.
 * @param[in] count       Number of samples in `dac_samples`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `dac_samples` is `NULL` with `count > 0`,
 *         or `ESP_ERR_INVALID_STATE` if passthrough is disabled.
 */
esp_err_t bsp_spk_passthrough_push_u8(const uint8_t *dac_samples, size_t count);

/**
 * @brief Copy out the most recently rendered 16-bit PCM sample chunk from `bsp_spk_passthrough_push_*` or silence flush.
 *
 * Thread-safe copy performed while holding the speaker passthrough mutex so cross-core callers
 * cannot observe torn samples.
 *
 * @param[out] dst_pcm     Destination buffer to receive 16-bit PCM samples (may be `NULL` to query available sample count).
 * @param[in]  max_samples Maximum number of samples to copy into `dst_pcm`.
 * @return Number of samples copied into `dst_pcm` (or available sample count when `dst_pcm == NULL`).
 */
size_t bsp_spk_passthrough_copy_last_pcm(int16_t *dst_pcm, size_t max_samples);

#ifdef __cplusplus
}
#endif
