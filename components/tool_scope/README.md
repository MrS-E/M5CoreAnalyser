# `tool_scope` — 1-Channel Digital Oscilloscope Component

## Purpose
`tool_scope` implements the 1-channel digital storage oscilloscope instrument (`id = "scope"`) for **M5CoreAnalyser**. It captures calibrated 12-bit ADC1 waveforms on `GPIO36` (`ADC1_CH0`) or `GPIO35` (`ADC1_CH7`) on Core 1 into PSRAM buffers, performs hysteresis-armed rising/falling/auto trigger alignment, extracts a 300-point display window, computes live waveform metrics (`Vpp`, `Vmin`, `Vmax`, `Vavg`, `Vrms`, `Freq`, `Duty`), streams audio to the onboard NS4168 speaker, and renders a 10×6 division LVGL 9 oscilloscope chart on Core 0. The scope schedules requested sample intervals for `100 us/div` to `100 ms/div`, measures the actual ADC interval for metrics/export, and displays the effective timebase when oneshot ADC throughput cannot meet the request.

---

## Hardware Pins & Peripherals
| Signal / Function | GPIO | ESP32 Peripheral | Notes |
|---|---|---|---|
| Default Scope Input | `GPIO36` | `ADC1_CHANNEL_0` | Input-only (`0–3.3 V`, `ADC_ATTEN_DB_12`) |
| Alternate Scope Input | `GPIO35` | `ADC1_CHANNEL_7` | Input-only (`0–3.3 V`, `ADC_ATTEN_DB_12`) |
| Speaker Audio Monitor | `GPIO12`/`GPIO0`/`GPIO2` | `I2S_NUM_1` (NS4168) | Real-time ADC-to-Speaker passthrough via `bsp_spk_passthrough_push_adc12()` |

---

## Internal Pipeline & Algorithms
1. **PSRAM Capture Ring & Tear-Free Frame Handoff**:
   - Allocates a `16,384`-sample capture buffer in 8 MB PSRAM (`CONFIG_SPIRAM=y`) and a 3-slot `psram_double_buf_t` holding `scope_frame_t` (`300`-point display window + metrics + measured `viewport_us`).
2. **Timebase-Scheduled Acquisition & Measured Scale**:
   - Requests evenly spaced sample deadlines spanning ten divisions; the raw capture includes trigger margins, then the centered `300`-point chart window spans the selected ten divisions.
   - For intervals of `1 ms` or longer, the Core 1 worker sleeps with `vTaskDelay()` until near each `esp_timer` deadline instead of busy-waiting across long gaps. Faster requested intervals poll ADC1 at its achievable rate.
   - The worker derives `sample_rate_hz` from elapsed time across the actual captured samples. The frame's `viewport_us`, the scope frequency/duty metrics, and recorder metadata are based on that measured rate. If the ADC cannot attain the selected fast scale, the UI labels the effective measured time/division with `~` rather than claiming the requested scale.
3. **Hysteresis-Armed Trigger Engine (`scope_dsp_find_trigger`)**:
   - **Rising Edge (`SCOPE_TRIG_RISING` / `SCOPE_TRIG_AUTO`)**: Requires the signal to first drop to or below `threshold_mv - hysteresis_mv` (`40 mV` hysteresis band) before triggering on `samples[i - 1] < threshold_mv && samples[i] >= threshold_mv`.
   - **Falling Edge (`SCOPE_TRIG_FALLING`)**: Requires the signal to first rise to or above `threshold_mv + hysteresis_mv` before triggering on `samples[i - 1] > threshold_mv && samples[i] <= threshold_mv`.
   - **Auto Fallback (`SCOPE_TRIG_AUTO`)**: When no edge occurs in the capture window, automatically centers the window (`auto_fallback = true`) so flat DC traces remain visible.
4. **300-Point Window Extraction (`scope_dsp_extract_window`)**:
   - Extracts `300` points (`10` horizontal divisions × `30` points/div) centered around `trigger_index` with stride `decimation_step`.
5. **Live Waveform Metrics (`scope_dsp_compute_metrics`)**:
   - Computes `Vmin`, `Vmax`, `Vpp`, `Vavg` (DC mean), `Vrms` (total RMS), and `Vrms_ac` (AC-coupled RMS).
   - Estimates fundamental `Freq` (Hz) and positive `Duty` (`0–100%`) using Schmitt-trigger midpoint hysteresis with sub-sample linear interpolation across complete cycles, reporting `0.0 Hz` when `Vpp < 60 mV`.
6. **Recorder Timing**:
   - Each recorder frame contains one contiguous capture block, with `sample_rate_hz` measured from that block and `timestamp_us` anchored at its first sample. Separate capture blocks have their own timestamps; consumers must not assume samples continue uninterrupted between frames.

---

## Public API Reference (`include/tool_scope.h`)
- `esp_err_t scope_dsp_find_trigger(const uint16_t *samples_mv, size_t count, scope_trig_mode_t mode, uint16_t threshold_mv, uint16_t hysteresis_mv, size_t pre_trigger_samples, scope_trig_result_t *out_res)`
- `uint32_t scope_dsp_target_sample_interval_ns(scope_timebase_t tb)` / `uint32_t scope_dsp_viewport_us(uint32_t sample_rate_hz, uint32_t decimation_step, size_t window_points)`
- `esp_err_t scope_dsp_extract_window(const uint16_t *samples_mv, size_t count, size_t trigger_index, uint32_t decimation_step, uint16_t *out_window_mv, size_t window_points)`
- `esp_err_t scope_dsp_compute_metrics(const uint16_t *samples_mv, size_t count, uint32_t sample_rate_hz, scope_metrics_t *out_metrics)`
- `esp_err_t tool_scope_set_pin(gpio_num_t pin)` / `gpio_num_t tool_scope_get_pin(void)`
- `esp_err_t tool_scope_set_timebase(scope_timebase_t tb)` / `scope_timebase_t tool_scope_get_timebase(void)`
- `esp_err_t tool_scope_set_vscale(scope_vscale_t vs)` / `scope_vscale_t tool_scope_get_vscale(void)`
- `esp_err_t tool_scope_set_trig_mode(scope_trig_mode_t mode)` / `scope_trig_mode_t tool_scope_get_trig_mode(void)`
- `esp_err_t tool_scope_set_trig_level_mv(uint16_t level_mv)` / `uint16_t tool_scope_get_trig_level_mv(void)`
- `esp_err_t tool_scope_set_running(bool running)` / `bool tool_scope_is_running(void)`
- `esp_err_t tool_scope_get_latest_frame(scope_frame_t *out_frame)`
- `esp_err_t tool_scope_register(void)`

---

## Limitations
- Designed for low-frequency analog signals (`DC` up to `~10–15 kHz`) within `0–3.3 V`.
- Uses `adc_oneshot` polling on `ADC1` (`GPIO36`/`GPIO35`) so `I2S0` remains free from DMA contention when background generators are active.
