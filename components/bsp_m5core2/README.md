# `bsp_m5core2` — M5Stack Core2 Board Support Package

## Purpose
`bsp_m5core2` encapsulates all M5Stack Core2 (ESP32-D0WDQ6-V3, 16 MB Flash, 8 MB PSRAM) board-level hardware initialization, power management, display/touch drivers, GPIO conflict protection, calibrated `ADC1` access, and software Analog-to-Speaker audio passthrough for **M5CoreAnalyser**.

---

## Hardware Pins & Peripherals

| Subsystem | ESP32 Peripheral | Pins / I²C Address | Notes |
| :--- | :--- | :--- | :--- |
| **Internal I²C Bus** | `I2C_NUM_0` (400 kHz) | `GPIO21` (`SDA`), `GPIO22` (`SCL`) | Shared by AXP192 / AXP2101, FT6336U, BM8563, MPU6886 |
| **PMIC** | `AXP192` (v1.0) / `AXP2101` (v1.1) | I²C `0x34` (`0x03`=`0x03` or `0x4A`) | Auto-detects Core2 v1.0 (`AXP192`) and Core2 v1.1 (`AXP2101`) rails, backlight, `SPK_EN`, `TFT_RST`, and battery ADC |
| **LCD Display** | `SPI2_HOST` (`ILI9342C`) | `GPIO23` (`MOSI`), `GPIO38` (`MISO`), `GPIO18` (`SCLK`), `GPIO5` (`CS`), `GPIO15` (`DC`) | 320×240 RGB565 @ 40 MHz SPI; `GPIO4` (`SD_CS`) pulled high |
| **Capacitive Touch** | `FT6336U` (`I2C_NUM_0`) | I²C `0x38`, `GPIO39` (`INT`) | 320×240 screen area + bottom touch circles (`y = 240..280`) |
| **Speaker Output** | `I2S_NUM_1` (`NS4168`) | `GPIO12` (`BCK`), `GPIO0` (`WS`), `GPIO2` (`DOUT`) | 16-bit mono Philips I²S + AXP192 `SPK_EN` power gate |
| **Analog Inputs** | `ADC_UNIT_1` | `GPIO36` (`ADC1_CH0`), `GPIO35` (`ADC1_CH7`) | 12-bit calibrated (`ADC_ATTEN_DB_12`, input-only) |
| **Safe User GPIOs** | Port A / B / C / M5-Bus | `GPIO36`, `GPIO35`, `GPIO25`, `GPIO26`, `GPIO32`, `GPIO33`, `GPIO13`, `GPIO14`, `GPIO27`, `GPIO19` | Managed by `bsp_pin_claim()` / `bsp_pin_release()` |

---

## Internal Pipeline

### 1. AXP192 / AXP2101 PMIC Initialization (`bsp_axp192.c`)
- Auto-detects the PMIC chip ID at register `0x03` on `I2C0` (`0x34`):
  - **M5Stack Core2 v1.0 (`AXP192`, `chip_id = 0x03`)**: Keeps `DCDC1` at `3.35 V` for the ESP32 core, enables `LDO2` at `3.30 V` for LCD/SD logic and `DCDC3` at `2.80 V` for the LCD backlight, leaves `GPIO0` floating (`0x07`) and `EXTEN` off so USB `VBUS` is never disconnected or back-fed, configures `GPIO1` (`0x94` bit 1, active-low) for the power LED, `GPIO2` (`0x94` bit 2 `SPK_EN`, active-high) gated **OFF** by default, `GPIO4` (`TFT_RST`) for LCD/touch reset, and 12-bit battery ADC (`0x78`/`0x79`, `1.1 mV/LSB`).
  - **M5Stack Core2 v1.1 (`AXP2101`, `chip_id = 0x4A`)**: Disables DCDC1–5 UVP power-off latch (`0x23`), configures `DCDC1`/`DCDC3` at `3.30 V`, `ALDO2` (`3.30 V` LCD/touch reset), `ALDO3` (`3.30 V` `SPK_EN`, gated **OFF** by default in `0x90` bit 2), `ALDO4` (`3.30 V` LCD/TP/TF logic), `BLDO1` (`2.80 V` LCD backlight), `CHGLED` (`0x69`), and 14-bit battery voltage ADC (`0x34`/`0x35`, `1 mV/LSB`) while preserving `BLDO2` so USB `VBUS` is never disturbed.

### 2. GPIO Reservation & Conflict Manager (`bsp_pin_manager.c`)
- Protects all internal bus pins (`GPIO21`/`GPIO22` I²C0, `GPIO18`/`GPIO23`/`GPIO38`/`GPIO5`/`GPIO15`/`GPIO4` SPI2, `GPIO12`/`GPIO0`/`GPIO2` I²S1, `GPIO6`–`GPIO11`/`GPIO16`/`GPIO17` Flash/PSRAM, `GPIO1`/`GPIO3` UART0, `GPIO34` PDM Mic, `GPIO39` Touch IRQ) by returning `ESP_ERR_NOT_ALLOWED`.
- Tracks active ownership across foreground measurement tools and background generators (`wavegen`, `analoggen`), returning `ESP_ERR_INVALID_STATE` when a second tool attempts to claim an in-use pin.
- Enforces input-only constraints on `GPIO34`–`GPIO39` via `bsp_pin_claim_ex(..., require_output=true)` (`ESP_ERR_NOT_SUPPORTED`).

### 3. Software Analog-to-Speaker Passthrough (`bsp_spk_passthrough.c`)
- Accepts either 12-bit unsigned ADC samples (`0..4095`) or 8-bit unsigned DAC LUT samples (`0..255`).
- Centers samples around zero and removes DC bias using a 1st-order high-pass filter:
  $$y[n] = x[n] - x[n-1] + 0.995 \cdot y[n-1]$$
- Applies adjustable digital gain (`0%`–`200%`), mute gating, and hard-clipping saturation to `[INT16_MIN, INT16_MAX]`.
- Streams 16-bit PCM frames to `I2S_NUM_1` (`NS4168`) and automatically gates off `AXP192 SPK_EN` and flushes zeroed DMA buffers when muted or disabled.

---

## Public API Reference

Header: [`include/bsp_m5core2.h`](include/bsp_m5core2.h)

- **Lifecycle & Buses**:
  - `esp_err_t bsp_m5core2_init(void)`
  - `bool bsp_m5core2_is_initialized(void)`
  - `i2c_master_bus_handle_t bsp_i2c_get_bus(void)`
- **AXP192 Power & Telemetry**:
  - `esp_err_t bsp_axp192_init(i2c_master_bus_handle_t bus_handle)`
  - `esp_err_t bsp_axp192_get_battery(uint16_t *out_voltage_mv, uint8_t *out_percent, bool *out_charging)`
  - `uint16_t bsp_axp192_raw12_to_mv(uint8_t reg_hi, uint8_t reg_lo)`
  - `uint8_t bsp_axp192_calc_battery_percent(uint16_t voltage_mv)`
  - `uint8_t bsp_axp192_calc_gpio_state(uint8_t reg94_prev, bool spk_enable, bool led_on)`
  - `uint8_t bsp_axp192_get_reg94_state(void)`
  - `esp_err_t bsp_axp192_set_speaker_enable(bool enable)` / `bool bsp_axp192_get_speaker_enable(void)`
  - `esp_err_t bsp_axp192_set_lcd_backlight(uint8_t brightness_percent)`
  - `esp_err_t bsp_axp192_set_led(bool on)` / `bool bsp_axp192_get_led_state(void)`
- **Display, Touch & ADC1**:
  - `esp_err_t bsp_display_init(void)`
  - `esp_lcd_panel_handle_t bsp_display_get_panel(void)`
  - `esp_err_t bsp_touch_read(uint16_t *out_x, uint16_t *out_y, bool *out_pressed)`
  - `esp_err_t bsp_touch_read_state(bsp_touch_state_t *out_state)`
  - `esp_err_t bsp_adc1_init(void)`
  - `esp_err_t bsp_adc1_read_raw(gpio_num_t pin, int *out_raw)`
  - `esp_err_t bsp_adc1_raw_to_mv(gpio_num_t pin, int raw, int *out_mv)`
- **GPIO Pin Manager**:
  - `esp_err_t bsp_pin_claim(gpio_num_t pin, const char *owner_tool_id)`
  - `esp_err_t bsp_pin_claim_ex(gpio_num_t pin, const char *owner_tool_id, bool require_output)`
  - `esp_err_t bsp_pin_release(gpio_num_t pin, const char *owner_tool_id)`
  - `esp_err_t bsp_pin_release_all_for_owner(const char *owner_tool_id)`
  - `esp_err_t bsp_pin_get_owner_copy(gpio_num_t pin, char *out_buf, size_t buf_len)`
  - `bool bsp_pin_is_owned_by(gpio_num_t pin, const char *owner_tool_id)`
- **Speaker Passthrough & DSP**:
  - `esp_err_t bsp_spk_passthrough_enable(bool enable, uint32_t sample_rate_hz)`
  - `esp_err_t bsp_spk_passthrough_set_gain(uint8_t gain_percent)`
  - `esp_err_t bsp_spk_passthrough_set_mute(bool mute)`
  - `esp_err_t bsp_spk_passthrough_push_adc12(const uint16_t *adc_samples, size_t count)`
  - `esp_err_t bsp_spk_passthrough_push_u8(const uint8_t *dac_samples, size_t count)`
  - `size_t bsp_spk_passthrough_copy_last_pcm(int16_t *dst_pcm, size_t max_samples)`
  - `void bsp_spk_dsp_init(bsp_spk_dsp_state_t *state, uint8_t gain_percent)`
  - `void bsp_spk_dsp_process_adc12(bsp_spk_dsp_state_t *state, const uint16_t *adc_samples, int16_t *out_pcm16, size_t count)`
  - `void bsp_spk_dsp_process_u8(bsp_spk_dsp_state_t *state, const uint8_t *dac_samples, int16_t *out_pcm16, size_t count)`

---

## Configuration & Usage Example

```c
#include "bsp_m5core2.h"

void scope_audio_example(const uint16_t *adc_batch, size_t count)
{
    bsp_m5core2_init();

    if (bsp_pin_claim(BSP_PIN_ANALOG_IN_DEFAULT, "scope") == ESP_OK) {
        bsp_spk_passthrough_set_gain(100);
        bsp_spk_passthrough_enable(true, 16000);
        bsp_spk_passthrough_push_adc12(adc_batch, count);
        bsp_spk_passthrough_enable(false, 0);
        bsp_pin_release(BSP_PIN_ANALOG_IN_DEFAULT, "scope");
    }
}
```

---

## ADC1 Read Semantics & Test-Only Mock Fallback
- `bsp_adc1_read_raw()` always calls `adc_oneshot_read()` in production builds (`CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK=n`, the default). A hardware read error is returned to the caller unchanged; it is never replaced with a substitute value.
- The only substitute path is the Kconfig option `CONFIG_BSP_ADC1_QEMU_MIDSCALE_FALLBACK` (`components/bsp_m5core2/Kconfig`). It is enabled only in `test_app/sdkconfig.defaults`, because the ESP32 QEMU machine does not emulate the SAR ADC (SENS) registers and `adc_oneshot_read()` would spin forever there. With the option on, `bsp_adc1_read_raw()` returns a constant `2048` and `ESP_OK`.
- `bsp_adc1_uses_mock_fallback()` reports which mode is compiled in. Production firmware never probes or writes raw SENS registers.
- Under QEMU, tests can check ADC timing and metadata, but not waveform content.

## Limitations
- **Safe Voltage Range**: Direct ESP32 GPIO and ADC1 inputs (`GPIO36`/`GPIO35`) operate over `0.0 V – 3.3 V` only; external attenuation is required for higher voltages.
- **Input-Only Pins**: `GPIO35` and `GPIO36` are input-only on ESP32 and cannot be used by `tool_wavegen` or `tool_analoggen`.
- **Onboard PDM Microphone Excluded**: `GPIO34` (`SPM1423` DATA) is reserved as a protected internal pin so `I2S_NUM_1` (`GPIO12`/`GPIO0`/`GPIO2`) is dedicated exclusively to the `NS4168` speaker amplifier without bus contention.
