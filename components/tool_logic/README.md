# `tool_logic` — Multi-Channel Logic Analyzer & Protocol Decoder Component

## 1. Purpose
`tool_logic` implements the **4-Channel Digital Logic Analyzer & Protocol Decoder Suite** (`id = "logic"`, category `TOOL_CATEGORY_DIGITAL_IN`) for **M5CoreAnalyser** on the M5Stack Core2. It supports two selectable hardware capture engines:
1. **Sampled Logic Mode (`LOGIC_CAP_MODE_SAMPLED`)**: Hardware **GPTimer** auto-reload ISR (`1 MHz` resolution, `on_logic_gptimer_alarm_isr`) periodically sampling `GPIO.in` (`GPIO0..31`) and `GPIO.in1.val` (`GPIO32..39`) into a `1024`-sample internal SRAM ring bounce buffer (`s_isr_bounce_buf`), drained by the Core 1 worker into a `64 KB` PSRAM capture buffer (`LOGIC_PSRAM_BUFFER_BYTES = 65536` when `CONFIG_SPIRAM=y`). Presets above `100 kHz` (`LOGIC_SAMPLED_MAX_RATE_HZ`, `10 µs` period) are clamped to `100 kHz` (`frame.rate_capped = true`), and the actual achieved sample rate (`frame.sample_rate_hz`) is computed from ISR-stamped first/last sample timestamps (`frame.capture_elapsed_us`).
2. **Edge-Timestamp Mode (`LOGIC_CAP_MODE_RMT_EDGE`)**: Combines ESP32 **RMT RX** (`1 MHz` / `1 µs` resolution, `2.5 ms` idle threshold for burst frames) with an `IRAM_ATTR` **GPIO Edge ISR** (`on_logic_gpio_edge_isr` / `record_logic_edge_iram`) in internal SRAM (`256` transitions max, hard-capped at `512` ISR entries). Continuous signals without an RMT idle gap can use the GPIO timestamp fallback; its timestamps have GPIO ISR service jitter and are not equivalent to hardware RMT edge timing. The configured `10 kHz`–`1 MHz` rate is only the uniform expansion grid for the captured transitions, not a guarantee of edge acquisition bandwidth. Idle lines with zero captured pulses report `frame.no_capture = true` (`total_captured = 0`, `sample_rate_hz = 0`) without fabricating flat-line samples or pushing empty frames to `recorder_sink`.

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

- **Sampled Mode GPTimer Auto-Reload ISR & Rate Cap (`LOGIC_CAP_MODE_SAMPLED`)**:
  - `s_gptimer` runs at `1 MHz` with `auto_reload_on_alarm = true` (`DRAM_ATTR` `s_isr_alarm_cfg`). On each alarm, `on_logic_gptimer_alarm_isr` (`IRAM_ATTR`) packs `CH0..CH3` from `GPIO.in` / `GPIO.in1.val` into `DRAM_ATTR` `s_isr_bounce_buf[1024]`, stamps `s_isr_first_sample_us` / `s_isr_last_sample_us`, and stops the timer at `target_samples` (`1024`). If ISR entry latency exceeds the alarm period (`cur_cnt >= s_isr_interval_us`), ISR-safe `gptimer_set_raw_count(timer, 0ULL)` re-arms the counter so capture never stalls in QEMU or under heavy interrupt load.
  - Because periodic software ISRs faster than `10 µs` would starve FreeRTOS and LVGL, `logic_clamp_sample_rate()` clamps `250 kHz`, `500 kHz`, and `1 MHz` presets to `100 kHz` in `LOGIC_CAP_MODE_SAMPLED`, setting `frame.rate_capped = true` and showing `100k[CAP]` in the UI.
  - `frame.sample_rate_hz` is computed from `(final_write - 1) * 1e6 / (s_isr_last_sample_us - s_isr_first_sample_us)`. If the timer stalls past its deadline, `capture_logic_buffer()` returns `ESP_ERR_TIMEOUT`, sets `frame.hw_error = ESP_ERR_TIMEOUT`, skips `recorder_sink_push()`, and displays `[HW ERR]` in the UI.
- **RMT Edge Mode, Continuous Fallback, Hard ISR Cap & `no_capture` (`LOGIC_CAP_MODE_RMT_EDGE`)**:
  - Arms both `rmt_receive()` (`2.5 ms` idle threshold) and `on_logic_gpio_edge_isr` on `CH0` for up to `s_edge_timeout_us` (default `25 ms`, configurable `5 ms..500 ms` via `tool_logic_set_edge_timeout_us()`).
  - `record_logic_edge_iram()` increments `s_edge_isr_invocations` on *every* ISR entry (including rejected duplicate-level bounces) and disables the GPIO interrupt immediately at `512` invocations (`LOGIC_EDGE_ISR_MAX_INVOCATIONS`) or `256` stored edges (`LOGIC_EDGE_BOUNCE_MAX`).
  - If RMT RX completes, its symbols are staged to PSRAM (`s_psram_rmt_buf`) and expanded via `logic_expand_rmt_to_samples()`. RMT RX completes burst captures after its configured idle gap; for continuous streams, `logic_edge_timestamps_to_rmt_symbols()` converts `DRAM_ATTR` GPIO ISR edge timestamps into RMT-shaped symbols in PSRAM. That fallback is best-effort: timestamp quantization is `1 us`, ISR-entry latency adds jitter, and edges arriving faster than the ISR can service them may be lost or merged. The `sample_rate_hz` on expanded edge frames describes the software expansion grid, not a measured GPIO sampling clock.
  - If no complete pulse is captured (`expanded == 0`), `frame.no_capture = true`, `frame.total_captured = 0`, `frame.sample_rate_hz = 0`, `recorder_sink_push()` is skipped, all chart series are cleared to `LV_CHART_POINT_NONE`, and the UI displays `[NO CAP]`.
- **LVGL 9 Multi-Lane Waveform & Modal UI**:
  - Renders up to 4 non-overlapping digital waveform lanes (`CH3`..`CH0`) across a 120-point viewport with horizontal **Zoom** (`1x`, `2x`, `4x`, `8x`) and **Pan** (`<`, `>`), a decoded packet table, and a floating **Capture & Pin Mapping Modal** (`PINS` button). In RMT edge mode, only CH0 is acquired; CH1–CH3 are labeled `N/A` and their chart traces are cleared, not drawn as low.
- **Recorder Sink Integration**:
  - Exports valid captured packed sample buffers (`1 byte/sample`) with `sample_rate_hz` and `timestamp_us` to `recorder_sink_push()` (skipped when `no_capture` or `hw_error != ESP_OK`). Sampled mode reports the measured timer rate; expanded RMT-edge mode reports the requested/effective software raster rate, not a measured GPIO sampling clock.

---

## 5. Limitations
- **Sampled Mode Ceiling**: Periodic multi-pin GPTimer ISR sampling is capped at `100 kHz` (`10 µs` target period). `frame.sample_rate_hz` reports the measured rate over the capture window; ISR latency, interrupt load, and board/application load can make it differ from the preset. UART decoding needs adequate oversampling and clean signal edges; `57600` and `115200` baud should not be assumed reliable with the `100 kHz` sampled mode.
- **RMT Edge Mode Limits**: Native RMT RX timing is preferable for one-wire bursts that end with the configured `2.5 ms` idle gap and fit in its receive buffer. Continuous signals fall back to GPIO ISR timestamps with microsecond quantization and several-microsecond-class, load-dependent service jitter; edges closer together than the ISR can service can be lost. Do not rely on the fallback for `500 kHz`–`1 MHz` continuous capture or high-baud UART. Its expanded sample rate is a software grid, not evidence that all transitions were acquired. Multi-wire `I²C` and `SPI` require sampled mode because edge mode captures CH0 only.
- **RMT Edge Mode Channel Count**: `LOGIC_CAP_MODE_RMT_EDGE` acquires only CH0; the UI marks CH1–CH3 unavailable. Multi-wire synchronous buses (`I²C` `SDA`+`SCL`, `SPI` `CS`+`SCK`+`MOSI`+`MISO`) must use `LOGIC_CAP_MODE_SAMPLED` and keep bus clocks comfortably below the `100 kHz` sample rate.
- All inputs operate at standard ESP32 `0–3.3 V` digital logic levels.
