# `tool_analoggen` — 8-Bit DAC Analog Waveform Generator

## 1. Purpose
`tool_analoggen` is the Analog Waveform Generator instrument (`id = "analoggen"`, category `TOOL_CATEGORY_GENERATOR`) for **M5CoreAnalyser**. It synthesizes 256-entry 8-bit lookup tables (**Sine**, **Triangle**, **Sawtooth**, **Staircase**, and **DC**) in PSRAM/SRAM and drives the ESP32 internal 8-bit DAC on `GPIO25` (`DAC_CHAN_0`) or `GPIO26` (`DAC_CHAN_1`). It supports both **background execution** (`supports_background = true`) and **software speaker audio monitoring** (`supports_spk_passthrough = true`) over `I2S_NUM_1`.

---

## 2. Supported Output Pins & Peripheral Architecture

| GPIO | ESP32 DAC Channel | M5Stack Core2 Connector | Notes |
| :--- | :--- | :--- | :--- |
| **`GPIO25`** *(Default)* | `DAC_CHAN_0` (`DAC1`) | **M5-Bus** Pin 9 | Default analog generator output; conflict-free with Port B `GPIO26`/`GPIO36` |
| **`GPIO26`** | `DAC_CHAN_1` (`DAC2`) | **Port B** (Black) & M5-Bus | Alternate DAC output; shared with Port B digital tools via `bsp_pin_claim_ex` |

### Why Task-Driven `dac_oneshot` Instead of `dac_continuous` DMA or High-Priority Timer ISRs?
1. **Zero `I2S0` / `ADC1` Contention on Original ESP32**:
   - On original ESP32 (`ESP32-D0WDQ6-V3`), `dac_continuous` DMA uses `I2S0` internally, which contends with `ADC1` (`tool_scope` and `tool_voltmeter`).
   - By driving the DAC via `dac_oneshot_output_voltage()` from the Core 1 worker task (`analoggen_c1`) indexed by `esp_timer_get_time()` (`analoggen_calc_phase_index`), `tool_analoggen` never touches `I2S0` DMA and coexists cleanly with foreground `ADC1` measurements.
2. **Cache & Watchdog Safety**:
   - `dac_oneshot_output_voltage()` is called exclusively from the Core 1 FreeRTOS task (`analoggen_c1`) in bounded micro-batches (`8 kS/s` nominal in foreground, `4 kS/s` in background coexistence mode), never from a high-priority timer ISR.
   - To guarantee $\ge 4..8$ DAC updates per waveform cycle without task starvation, output frequency is honestly bounded to **`1 Hz – 1000 Hz`** (`ANALOGGEN_MAX_FREQ_HZ = 1000`).

---

## 3. 256-Entry LUT Synthesis & Rail Clamping (`analoggen_lut.c`)
For each index $i \in [0, 255]$:
$$v_i = V_{\text{offset}} + \frac{V_{\text{amp}}}{2} \cdot u_i, \quad u_i \in [-1.0, +1.0]$$
- If $v_i < 0\text{ mV}$ or $v_i > 3300\text{ mV}$, `analoggen_lut_build()` clamps $v_i$ to $[0, 3300]\text{ mV}$, converts to an 8-bit DAC code $c_i = \text{round}(v_i \cdot 255 / 3300) \in [0, 255]$, and sets `lut_stats.clipped = true` (displayed as `[CLIP]` in the LVGL UI).
- The primary 256-byte LUT is allocated in external PSRAM (`s_psram_lut`) and mirrored into internal SRAM (`s_sram_lut`) for zero-wait-state task access.

---

## 4. Speaker Monitoring Ownership Coordination
`tool_analoggen` coordinates `I2S1` speaker passthrough (`bsp_spk_passthrough_push_u8`) with foreground input tools:
- **Foreground Active**: When `tool_analoggen` is in the foreground and speaker passthrough is enabled, `analoggen_c1` resamples the LUT via a Q16.16 phase accumulator (`analoggen_resample_lut_for_speaker`) and streams 8-bit audio to `bsp_spk_passthrough_push_u8()`.
- **Background + Non-Speaker Foreground Tool (or Home)**: When `tool_analoggen` is in the background and the user is on the Home screen or using `tool_freq` / `tool_logic` (`supports_spk_passthrough == false`), `tool_analoggen` retains speaker ownership and continues audio monitoring.
- **Background + Speaker-Capable Foreground Input Tool (`tool_scope`, `tool_voltmeter`)**: When `tool_scope` or `tool_voltmeter` is open in the foreground, `tool_analoggen_owns_speaker()` returns `false` (`status.spk_yielded_to_fg = true`), automatically yielding the speaker stream to the foreground ADC tool so the two instruments never interleave audio frames into `I2S1`. Closing the foreground ADC tool automatically restores speaker ownership to background `tool_analoggen`.

---

## 5. Public API Reference (`include/tool_analoggen.h`)
- `esp_err_t tool_analoggen_register(void)`
- `const analyser_tool_t *tool_analoggen_get_descriptor(void)`
- `esp_err_t tool_analoggen_set_pin(gpio_num_t pin)` / `gpio_num_t tool_analoggen_get_pin(void)`
- `esp_err_t tool_analoggen_set_waveform(analoggen_wave_t waveform)` / `analoggen_wave_t tool_analoggen_get_waveform(void)`
- `esp_err_t tool_analoggen_set_freq_hz(uint32_t freq_hz)` / `uint32_t tool_analoggen_get_freq_hz(void)`
- `esp_err_t tool_analoggen_set_amplitude_mv(uint16_t amplitude_mv)` / `uint16_t tool_analoggen_get_amplitude_mv(void)`
- `esp_err_t tool_analoggen_set_offset_mv(uint16_t offset_mv)` / `uint16_t tool_analoggen_get_offset_mv(void)`
- `esp_err_t tool_analoggen_set_stair_steps(uint8_t steps)` / `uint8_t tool_analoggen_get_stair_steps(void)`
- `esp_err_t tool_analoggen_set_output_enabled(bool enabled)` / `bool tool_analoggen_is_output_enabled(void)`
- `esp_err_t tool_analoggen_set_keep_in_bg(bool keep_in_bg)` / `bool tool_analoggen_get_keep_in_bg(void)`
- `bool tool_analoggen_owns_speaker(void)`
- `size_t tool_analoggen_copy_lut(uint8_t *out_lut, size_t max_len)`
- `esp_err_t tool_analoggen_get_status(analoggen_status_t *out_status)`

---

## 6. Limitations
- The ESP32 internal DAC has 8-bit vertical resolution (`~12.9 mV / LSB` over `0..3.3 V`) and unbuffered high output impedance; low-impedance loads require an external op-amp buffer.
- Task-driven `dac_oneshot` updates are bounded to `1..1000 Hz` fundamental frequencies and exhibit FreeRTOS tick scheduling intervals between micro-batches while preserving exact long-term phase via `esp_timer_get_time()`.
