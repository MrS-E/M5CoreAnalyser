# `tool_voltmeter` — 0–3.3 V Calibrated DC & AC-RMS Voltmeter Component

## Purpose
`tool_voltmeter` implements the digital DC and AC-RMS voltmeter instrument (`id = "voltmeter"`) for **M5CoreAnalyser**. It acquires 12-bit calibrated ADC1 batches on `GPIO36` (`ADC1_CH0`, Port B / M5-Bus) or `GPIO35` (`ADC1_CH7`, M5-Bus) on Core 1, filters impulse noise using an interquartile median/trimmed-mean filter and a PSRAM moving-average window, applies user-configurable two-point linear calibration stored in NVS, and renders a digital voltmeter UI on Core 0.

---

## Hardware Pins & Peripherals
| Signal / Function | GPIO | ESP32 Peripheral | Notes |
|---|---|---|---|
| Default Analog Input | `GPIO36` | `ADC1_CHANNEL_0` | Input-only (`0–3.3 V`, `ADC_ATTEN_DB_12`) |
| Alternate Analog Input | `GPIO35` | `ADC1_CHANNEL_7` | Input-only (`0–3.3 V`, `ADC_ATTEN_DB_12`) |
| Speaker Audio Monitor | `GPIO12`/`GPIO0`/`GPIO2` | `I2S_NUM_1` (NS4168) | Optional live audio passthrough via `bsp_spk_passthrough_push_adc12()` |

---

## Internal Pipeline & Algorithms
1. **Core 1 Batch Acquisition (`volt_c1`)**:
   - Samples batches of `64`–`256` readings on the selected input pin using `bsp_adc1_read_raw()` and `bsp_adc1_raw_to_mv()`. The recorder rate is measured over the contiguous ADC reads in each batch, not taken from the speaker's nominal `16 kHz` rate.
2. **Two-Point Linear Calibration (`voltmeter_dsp_apply_cal`)**:
   - Applies $V_{\text{cal}} = \text{slope} \cdot V_{\text{raw}} + \text{offset\_mv}$ with rail clamping to `[0, 3600] mV`.
   - Calibration parameters (`slope` in `[0.5, 2.0]`, `offset_mv` in `[-1000, +1000] mV`) are persisted per pin (`cal_g36`, `cal_g35`) in the `"volt_cal"` NVS namespace.
3. **Outlier-Rejecting Median & Trimmed-Mean Filter (`voltmeter_dsp_median_filter_mv`)**:
   - Sorts each batch and discards the bottom 25% and top 25% of samples (`count >= 4`) to compute an impulse-immune DC voltage (`vdc_mv`) alongside the exact median (`median_mv`).
4. **AC-RMS & True-RMS Calculation (`voltmeter_dsp_process_batch`)**:
   - **AC-coupled RMS**: $V_{\text{rms,AC}} = \sqrt{\frac{1}{N}\sum_{i=0}^{N-1}(v_i - \bar{v})^2}$
   - **True RMS**: $V_{\text{rms,True}} = \sqrt{\frac{1}{N}\sum_{i=0}^{N-1}v_i^2}$
5. **PSRAM Moving-Average & Session Tracker (`voltmeter_avg_filter_t`)**:
   - Maintains a circular window in PSRAM (`psram_buf_calloc`) to smooth `vavg_mv` and track session `vmin_mv`, `vmax_mv`, and `sample_count`.
6. **Tear-Free UI & Export**:
   - Publishes `voltmeter_frame_t` snapshots to Core 0 via `psram_double_buf_t` (`3`-slot tear-free buffer) and exports raw sample frames to `recorder_sink_push()` when recording is active. Each frame timestamp anchors its first sample; its measured `sample_rate_hz` describes only that batch.
   - The worker pauses approximately `25 ms` between batches, so voltmeter recorder frames are discontinuous. Consumers must use each frame's timestamp to preserve the gap and must not concatenate frames as a continuous uniformly sampled signal.

---

## Public API Reference (`include/tool_voltmeter.h`)
- `void voltmeter_dsp_cal_init_default(voltmeter_cal_t *cal)`
- `esp_err_t voltmeter_dsp_calc_two_point_cal(float raw_low_mv, float ref_low_mv, float raw_high_mv, float ref_high_mv, voltmeter_cal_t *out_cal)`
- `float voltmeter_dsp_apply_cal(float raw_mv, const voltmeter_cal_t *cal)`
- `esp_err_t voltmeter_dsp_avg_init(voltmeter_avg_filter_t *filter, size_t window_len)`
- `void voltmeter_dsp_avg_free(voltmeter_avg_filter_t *filter)`
- `void voltmeter_dsp_avg_reset(voltmeter_avg_filter_t *filter)`
- `float voltmeter_dsp_avg_push(voltmeter_avg_filter_t *filter, float sample_mv)`
- `esp_err_t voltmeter_dsp_median_filter_mv(const float *samples_mv, size_t count, float *out_median_mv, float *out_trimmed_mean_mv)`
- `esp_err_t voltmeter_dsp_process_batch(const uint16_t *raw_adc, const float *uncal_mv, size_t count, const voltmeter_cal_t *cal, voltmeter_avg_filter_t *avg_filter, voltmeter_metrics_t *out_metrics)`
- `esp_err_t tool_voltmeter_save_cal(gpio_num_t pin, const voltmeter_cal_t *cal)`
- `esp_err_t tool_voltmeter_load_cal(gpio_num_t pin, voltmeter_cal_t *out_cal)`
- `esp_err_t tool_voltmeter_reset_cal(gpio_num_t pin)`
- `esp_err_t tool_voltmeter_set_pin(gpio_num_t pin)` / `gpio_num_t tool_voltmeter_get_pin(void)`
- `esp_err_t tool_voltmeter_set_hold(bool hold)` / `bool tool_voltmeter_is_hold(void)`
- `esp_err_t tool_voltmeter_reset_stats(void)`
- `esp_err_t tool_voltmeter_get_latest_metrics(voltmeter_metrics_t *out_metrics)`
- `esp_err_t tool_voltmeter_register(void)`

---

## Limitations
- Operates strictly within the safe `0–3.3 V` ESP32 ADC1 input range (`ADC_ATTEN_DB_12`). External voltage dividers are required for signals above `3.3 V`.
- ESP32 ADC1 exhibits non-linearity near `< 100 mV` and `> 3150 mV`; two-point calibration improves accuracy across the central linear operating region.
