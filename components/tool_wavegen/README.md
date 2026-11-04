# `tool_wavegen` — Digital Wave Generator Instrument

## 1. Purpose
`tool_wavegen` is the Digital Wave Generator instrument (`id = "wavegen"`, category `TOOL_CATEGORY_GENERATOR`) for **M5CoreAnalyser**. It drives external expansion GPIOs on Port A, Port B, Port C, and the M5-Bus header using three dedicated ESP32 hardware peripherals (**LEDC**, **MCPWM**, and **RMT TX**) and supports background execution (`supports_background = true`) so digital test stimuli can remain active while the user opens foreground measurement tools (`tool_scope`, `tool_freq`, `tool_logic`, or `tool_voltmeter`).

---

## 2. Supported Output Pins & Hardware Peripherals

### Claimable Output Pins (`tool_wavegen_is_pin_supported(pin) == true`)
All output pins are claimed with `bsp_pin_claim_ex(pin, "wavegen", /*require_output=*/true)`:

| GPIO | M5Stack Core2 Connector | Notes |
| :--- | :--- | :--- |
| **`GPIO14`** *(Default)* | **Port C** (Blue, TXD2) | Default digital wave output |
| **`GPIO13`** | **Port C** (Blue, RXD2) | Alternate Port C digital output |
| **`GPIO26`** | **Port B** (Black) & M5-Bus | Ideal for Port B jumper loopback to `GPIO36` |
| **`GPIO32`** | **Port A** (Red, SDA) | Port A digital output |
| **`GPIO33`** | **Port A** (Red, SCL) | Port A digital output |
| **`GPIO27`** | **M5-Bus** Pin 19 | M5-Bus expansion output |
| **`GPIO19`** | **M5-Bus** Pin 17 | M5-Bus expansion output |
| **`GPIO25`** | **M5-Bus** Pin 9 | M5-Bus expansion output (shared with DAC1) |

Input-only pins (`GPIO36`, `GPIO35`, `GPIO34`, `GPIO38`, `GPIO39`) are rejected with `ESP_ERR_NOT_SUPPORTED`, and protected internal M5Core2 bus pins (`GPIO21`/`GPIO22`, SPI2 LCD, `I2S1`) are rejected with `ESP_ERR_NOT_ALLOWED`.

### Operating Modes (`wavegen_mode_t`)

| Mode | ESP32 Peripheral | Frequency / Timing Range | Description |
| :--- | :--- | :--- | :--- |
| **`WAVEGEN_MODE_SQUARE`** | `LEDC` (`LEDC_HIGH_SPEED_MODE`, Timer 0, Ch 0) | `1 Hz` – `1 MHz` | 50.0% duty square wave / clock with automatic bit resolution (`6..18` bits). |
| **`WAVEGEN_MODE_PWM_LEDC`** | `LEDC` (`LEDC_HIGH_SPEED_MODE`, Timer 0, Ch 0) | `1 Hz` – `1 MHz`, `0%..100%` duty | Variable-duty PWM using the 18-bit fractional APB clock divider. |
| **`WAVEGEN_MODE_PWM_MCPWM`** | `MCPWM` Group 0 (`mcpwm_timer`, `oper`, `cmpr`, `gen`) | `10 Hz` – `250 kHz`, `0%..100%` duty | High-precision variable-duty PWM using `10 MHz`, `1 MHz`, or `100 kHz` timer ticks. |
| **`WAVEGEN_MODE_SERVO`** | `RMT TX` (`1 MHz` / `1 µs` tick) | `50 Hz` (`20 ms` period), `500..2500 µs` (`0°..180°`) | Standard RC servo control pulse transmitted every `20 ms`. |
| **`WAVEGEN_MODE_RMT_BURST`** | `RMT TX` (`1 MHz` / `1 µs` tick) | `1..64` pulses, `1..32767 µs` high/low, `5..1000 ms` repeat | Programmable $N$-pulse digital burst for protocol/counter stimulus. |

---

## 3. Timing & Resolution Math (`wavegen_calc.c`)

1. **LEDC Bit-Resolution Solver (`wavegen_calc_ledc_params`)**:
   - Searches downward from `18` bits (`262,144` ticks, avoiding the ESP32 19-bit full-duty hardware erratum) to `1` bit for the highest resolution where the 18-bit fractional divider $D_{Q8} = \text{round}\left(\frac{80\text{ MHz} \times 256}{f_{\text{target}} \cdot 2^{\text{bits}}}\right)$ satisfies $256 \le D_{Q8} \le 1023 \times 256$.
2. **MCPWM Prescaler & Comparator Solver (`wavegen_calc_mcpwm_params`)**:
   - Selects the highest counter resolution among `{10 MHz, 1 MHz, 100 kHz}` whose period count $N_{\text{period}} = \text{round}(f_{\text{res}} / f_{\text{target}})$ fits within the 16-bit counter limit `[2, 65535]`, then computes $N_{\text{cmp}} = \text{round}(\text{duty} \cdot N_{\text{period}} / 100)$.
3. **RMT TX Servo & Burst Symbol Builder (`wavegen_calc_servo_rmt_symbol`, `wavegen_calc_burst_rmt_symbols`)**:
   - Stages `rmt_symbol_word_t` items in PSRAM (`s_psram_symbols`) and copies them into an internal SRAM bounce buffer (`DRAM_ATTR s_rmt_bounce_symbols`) before invoking `rmt_transmit()` so RMT DMA/ISR paths never touch external SPI PSRAM.

---

## 4. Background Generator Concurrency
- When `keep_in_bg` is enabled (default `true`, toggleable via the `BG:1`/`BG:0` UI button or `tool_wavegen_set_keep_in_bg()`), returning to the Home launcher (`ui_shell_go_home(true)`) keeps the active LEDC/MCPWM/RMT peripheral running and retains exclusive ownership of the selected output GPIO in `bsp_pin_claim`.
- Any foreground tool attempting to claim that same GPIO receives `ESP_ERR_INVALID_STATE` without interrupting the background waveform.
- Tapping the orange `BG:wave` pill in the persistent top header bar (or setting `keep_in_bg = false` before exiting) stops hardware generation and releases the GPIO pin.

---

## 5. Public API Reference (`include/tool_wavegen.h`)
- `esp_err_t tool_wavegen_register(void)`
- `const analyser_tool_t *tool_wavegen_get_descriptor(void)`
- `esp_err_t tool_wavegen_set_pin(gpio_num_t pin)` / `gpio_num_t tool_wavegen_get_pin(void)`
- `esp_err_t tool_wavegen_set_mode(wavegen_mode_t mode)` / `wavegen_mode_t tool_wavegen_get_mode(void)`
- `esp_err_t tool_wavegen_set_freq_hz(uint32_t freq_hz)` / `uint32_t tool_wavegen_get_freq_hz(void)`
- `esp_err_t tool_wavegen_set_duty_pct(float duty_pct)` / `float tool_wavegen_get_duty_pct(void)`
- `esp_err_t tool_wavegen_set_servo_pulse_us(uint16_t pulse_us)` / `uint16_t tool_wavegen_get_servo_pulse_us(void)`
- `esp_err_t tool_wavegen_set_burst_config(uint16_t high_us, uint16_t low_us, uint16_t pulse_count, uint16_t period_ms)`
- `esp_err_t tool_wavegen_trigger_burst(void)`
- `esp_err_t tool_wavegen_set_output_enabled(bool enabled)` / `bool tool_wavegen_is_output_enabled(void)`
- `esp_err_t tool_wavegen_set_keep_in_bg(bool keep_in_bg)` / `bool tool_wavegen_get_keep_in_bg(void)`
- `esp_err_t tool_wavegen_get_status(wavegen_status_t *out_status)`

---

## 6. Limitations
- Output logic levels are fixed at ESP32 `3.3 V` CMOS (`0 V` low, `3.3 V` high); external level shifting is required for `5 V` servos or TTL loads.
- In `WAVEGEN_MODE_RMT_BURST` and `WAVEGEN_MODE_SERVO`, inter-frame repetition timing (`20 ms`..`1000 ms`) is scheduled by the Core 1 FreeRTOS worker task (`wavegen_c1`), while intra-burst pulse widths (`duration0`/`duration1`) are timed to `1 µs` precision by the RMT hardware state machine.
