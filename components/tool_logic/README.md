# `tool_logic` — Multi-Channel Logic Analyzer & Protocol Decoder Component

## 1. Purpose
`tool_logic` implements the **4-Channel Digital Logic Analyzer & Protocol Decoder Suite** (`id = "logic"`, category `TOOL_CATEGORY_DIGITAL_IN`) for **M5CoreAnalyser** on the M5Stack Core2. It supports two selectable hardware capture engines:
1. **Sampled Logic Mode (`LOGIC_CAP_MODE_SAMPLED`)**: Periodic multi-channel GPIO sampling (`10 kHz` to `1 MHz`) reading ESP32 `GPIO.in` (`GPIO0..31`) and `GPIO.in1.val` (`GPIO32..39`) registers via an internal SRAM bounce buffer into a `64 KB` PSRAM capture buffer (`LOGIC_PSRAM_BUFFER_BYTES = 65536` when `CONFIG_SPIRAM=y`).
2. **Edge-Timestamp Mode (`LOGIC_CAP_MODE_RMT_EDGE`)**: Hardware pulse duration capture via ESP32 **RMT RX** at `1 MHz` (`1 µs` resolution) into an internal SRAM bounce buffer, staged into PSRAM and expanded into uniform multi-channel sample buffers via `logic_expand_rmt_to_samples()`.

---

## 2. Hardware Pins & Peripherals

| Channel | Default GPIO | Protocol Signal Mapping | Supported Safe Remap Pins |
|---|---|---|---|
| **CH0** | `GPIO13` (Port C RXD2) | UART `RX` / I²C `SDA` / SPI `CS` / PWM `IN` | `GPIO36`, `GPIO35`, `GPIO26`, `GPIO13`, `GPIO14`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19` |
| **CH1** | `GPIO14` (Port C TXD2) | I²C `SCL` / SPI `SCK` | Same safe pin pool (unique per channel) |
| **CH2** | `GPIO32` (Port A SDA) | SPI `MOSI` | Same safe pin pool (unique per channel) |
| **CH3** | `GPIO33` (Port A SCL) | SPI `MISO` | Same safe pin pool (unique per channel) |

All 4 channel pins are claimed atomically on `on_start()` via `bsp_pin_claim(pin, "logic")`. If any pin is already owned by a background generator (`wavegen` / `analoggen`), `claim_all_channel_pins()` rolls back all previously claimed pins in the transaction and returns `ESP_ERR_INVALID_STATE`.

---

## 3. Software Protocol Decoders (`logic_decoders.c`)

1. **UART 8N1 (`logic_decode_uart`)**:
   - Configurable baud rate (`9600`, `19200`, `38400`, `57600`, `115200` bps) and polarity (`invert`).
   - Synchronizes on the falling edge of the Start bit (`1 -> 0`), verifies the Start bit at `0.5 * T_bit` to reject single-sample glitches, samples 8 LSB-first data bits at `(1.5 .. 8.5) * T_bit`, and checks the Stop bit at `9.5 * T_bit` (flagging `framing_error = true` if Low).
2. **I²C Bus (`logic_decode_i2c`)**:
   - Detects `START` (`SDA` falling while `SCL` High), `RESTART` (repeated `START` without intervening `STOP`), and `STOP` (`SDA` rising while `SCL` High).
   - Samples 8 MSB-first bits plus the 9th `ACK` (`SDA = 0`) / `NACK` (`SDA = 1`) bit on rising `SCL` edges, decoding 7-bit slave addresses, `R/W` direction, and payload data bytes.
3. **SPI 4-Wire (`logic_decode_spi`)**:
   - Frames transfers while `CS` is active (`prev_cs == active && cs == active`), sampling 8 MSB-first `MOSI` and `MISO` bits on the active `SCK` edge (`rising` when `CPOL == CPHA`, `falling` when `CPOL != CPHA`).
4. **PWM / Pulse (`logic_decode_pwm`)**:
   - Measures complete cycle period (`us`), frequency (`Hz`), high pulse width (`PW+`), low pulse width (`PW-`), positive duty cycle (`0..100%`), and rising/falling edge counts.

---

## 4. Internal Pipeline, PSRAM ISR Safety & LVGL 9 UI

- **Internal SRAM Bounce Buffers (`DRAM_ATTR`)**:
  - `on_logic_gptimer_alarm_isr` and `on_logic_rmt_done_isr` (`IRAM_ATTR`) write exclusively to `s_isr_bounce_buf` and `s_rmt_bounce_buf` in internal SRAM, and the Core 1 worker task (`logic_c1`) copies batches into the `64 KB` PSRAM capture buffer (`s_psram_capture_buf`).
- **LVGL 9 Multi-Lane Waveform & Modal UI**:
  - Renders 4 non-overlapping digital waveform lanes (`CH3`..`CH0`) across a 120-point viewport with horizontal **Zoom** (`1x`, `2x`, `4x`, `8x`) and **Pan** (`<`, `>`), a decoded packet table, and a floating **Capture & Pin Mapping Modal** (`PINS` button).
- **Recorder Sink Integration**:
  - Exports captured packed sample buffers (`1 byte/sample`) with measured `sample_rate_hz` and `timestamp_us` to `recorder_sink_push()`.

---

## 5. Limitations
- Requires at least $3\times$ oversampling (`sample_rate_hz >= 3 * baud_rate`) for reliable UART 8N1 center-bit decoding.
- All inputs operate at standard ESP32 `0–3.3 V` digital logic levels.
