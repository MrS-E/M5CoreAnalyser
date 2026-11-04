# `tool_freq` — Digital Frequency Meter Component

## 1. Purpose
`tool_freq` implements the **Digital Frequency Meter** instrument (`id = "freq"`, category `TOOL_CATEGORY_DIGITAL_IN`) for **M5CoreAnalyser** on the M5Stack Core2. It combines two complementary measurement paths on the selected input GPIO:
1. **Low-Frequency Continuous Edge-Period Measurement ($f = 1/T$)**: Uses a per-pin `IRAM_ATTR` **GPIO Edge Interrupt (`GPIO_INTR_ANYEDGE`)** paired with `esp_timer_get_time()` (`1 µs` resolution) to record edge timestamps and post-edge logic levels into an internal SRAM (`DRAM_ATTR`) bounce buffer (`s_edge_bounce_buf[128]`). Unlike ESP32 RMT RX (which lacks ping-pong RX and only completes on an idle gap), GPIO edge timestamping works on **continuous periodic signals** without requiring any idle timeout, and disables the pin interrupt as soon as a hard cap is reached (see section 3) so high-frequency or aliased signals cannot flood the CPU with interrupts.
2. **High-Frequency Gate-Time Pulse Counting ($f = N / t_{\text{gate}}$)**: Uses the ESP32 **PCNT** hardware pulse counter unit (counting both rising and falling edges with 16-bit watch-point overflow accumulation and a default **`25 ns` / 2-APB-cycle glitch filter** permitting signals up to **`20 MHz`** at 50% duty) over selectable `20 ms`, `100 ms`, or `1000 ms` gate windows.

---

## 2. Hardware Pins & Peripherals

| Resource | Assignment | Notes |
|---|---|---|
| **Default Input Pin** | `GPIO36` (`BSP_PIN_ANALOG_IN_DEFAULT`) | Port B / M5-Bus input-only pin |
| **Selectable Safe Pins** | `GPIO36`, `GPIO35`, `GPIO26`, `GPIO13`, `GPIO14`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19` | Claimed atomically via `bsp_pin_claim(pin, "freq")` |
| **GPIO Edge ISR (`1/T`)** | `driver/gpio.h` (`GPIO_INTR_ANYEDGE`, `esp_timer_get_time()`) | Stores up to `128` `(timestamp_us, level)` transitions per window in `DRAM_ATTR` `s_edge_bounce_buf`; counts *every* ISR entry and disables the interrupt at `512` entries or when the ring is full |
| **PCNT (`N / t_gate`)** | `driver/pulse_cnt.h` (`[-30000, +30000]` watch limit, default `25 ns` glitch filter) | Counts rising + falling edges ($2\times$ resolution) with `IRAM_ATTR` overflow accumulator; configurable `0..12500 ns` via `tool_freq_set_pcnt_glitch_ns()` |
| **GPTimer & Gate Timing** | `driver/gptimer.h` (`1 MHz` count-up alarm) + `esp_timer` | `gptimer_start()` then `pcnt_unit_start()` run in task context; the gate start is stamped with `esp_timer_get_time()` right after PCNT starts. `on_gate_timer_alarm_isr` stops PCNT via ISR-safe `pcnt_unit_stop()` and latches `s_gate_isr_stop_us`, so `actual_gate_us = stop - pcnt_start` (task start-up latency is excluded, alarm ISR latency is not) |

---

## 3. Internal Pipeline, PSRAM ISR Safety & Error Handling

1. **ISR Bounce Buffering in Internal SRAM & Stack-Free Period Math**:
   - All hardware ISR callbacks (`on_freq_gpio_edge_isr`, `on_pcnt_watch_reach_isr`, `on_gate_timer_alarm_isr`) are marked `IRAM_ATTR` and touch only `DRAM_ATTR` internal SRAM state (`s_edge_bounce_buf`, `s_pcnt_overflow_accum`, `s_gate_isr_stop_us`, `s_gate_alarm_fired`), avoiding PSRAM cache stalls in interrupt context.
   - `freq_calc_from_edge_timestamps()` and `freq_calc_from_rmt_symbols()` use a single-pass coalescing state machine (`pulse_cycle_accum_t`, `< 64 B` stack frame) with zero stack arrays, and the Core 1 worker task (`freq_c1`) runs with a `6144`-byte stack (`FREQ_WORKER_STACK_BYTES`).
2. **Core 1 Worker, Actual Gate Timing & Cooperative Cancellation**:
   - At the start of each gate window (`20 ms`, `100 ms`, `1000 ms`), the Core 1 worker arms the `1 MHz` `GPTimer` alarm and starts `pcnt_unit_start()`.
   - While the gate is active, the worker waits in short (`<= 5 ms`) cooperative `vTaskDelay` / `taskYIELD` slices (`wait_gate_or_abort()`), checking `s_stop_requested`, `!s_running`, `s_capture_seq`, and `s_step_preempt`.
   - When the `GPTimer` alarm fires (`on_gate_timer_alarm_isr`), it stops the PCNT counter in ISR context (`pcnt_unit_stop()`) and latches the stop timestamp (`s_gate_isr_stop_us`). If the task deadline expires first (or on cooperative abort), task context stops PCNT/GPTimer and computes `actual_gate_us = gate_end_us - start_us`.
   - **Edge acquisition is independent of the gate.** The GPIO edge ISR is armed together with the gate. After the PCNT gate closes (its count is read and its duration fixed at that moment, it is never stretched), `capture_freq_window()` keeps collecting edges if fewer than `FREQ_EDGE_ACQ_TARGET_EDGES` (`5` edges = 2 full periods) are stored and the gate counted fewer than 5 edges. It waits in `<= 5 ms` cancellable slices (`wait_edges_or_abort()`) until 5 edges are stored, the ISR disables itself, or the bounded limit (default `2 s`, measured from capture start; `tool_freq_set_edge_acq_max_us()`, `20 ms..2 s`) expires. This does not depend on any previous reading. Stop / pin / mode changes still cancel in `< 250 ms`.
   - **Hard ISR caps.** `record_edge_sample_iram()` increments `s_edge_isr_invocations` on *every* ISR entry, including same-level samples that are dropped. At `FREQ_EDGE_ISR_MAX_INVOCATIONS` (`512`) entries, or when the `128`-entry SRAM ring is full, the ISR disables its own interrupt (`isr_capped`) and the window ends immediately, so worst-case ISR load per window is bounded whatever the signal does.
   - Switching tools (`freq_on_stop()`) or reconfiguring settings aborts any in-flight `100 ms` or `1000 ms` gate window within milliseconds and joins `freq_c1` before freeing PSRAM buffers or releasing peripherals.
3. **5 kHz Hysteresis Crossover (`FREQ_MODE_AUTO`) & Cross-Check Reliability**:
   - Below `5.5 kHz` (`FREQ_CROSSOVER_HIGH_HZ`), `FREQ_MODE_AUTO` uses edge-period measurement ($f = 1/T$) for high fractional frequency resolution and direct high/low pulse-width (`PW+`, `PW-`) and duty-cycle measurement.
   - **Cross-check (`freq_calc_cross_check()`, applied inside `freq_calc_combine_reading()`).** It runs only when both the 1/T result and the PCNT gate result are valid with non-zero frequency and the gate counted `N >= FREQ_XCHECK_MIN_COUNTS` (`8`) edges; otherwise the verdict is `FREQ_XCHECK_SKIPPED` (inconclusive, never a false mismatch). The tolerance is `max(5 %, 3 / N)` so the `+-1` count quantisation of a short gate does not flag correct low-frequency readings (`N = 10` allows `30 %`, `N >= 60` allows `5 %`). `N` is the number of edges counted (rising + falling).
   - On `FREQ_XCHECK_MISMATCH` (typically lost edges or counted glitches) `reading.unreliable = true` and, in `AUTO` mode, the reading is replaced by the gate values (`freq_hz`, `period_us`, `rpm`, `active_method = GATE`); duty and pulse widths become unknown (`duty_valid = false`, `NAN`, UI `"--"`). In forced `1/T` mode the method is kept but the reading is flagged `unreliable` with unknown duty. The UI badge shows `[... UNRELIABLE]`.
   - When only PCNT gate counts are available (in `FREQ_MODE_GATE_PCNT`, or in `FREQ_MODE_AUTO` above `25 kHz` where `1 µs` software edge ISR timestamps cannot resolve pulse width), `reading.duty_valid` is set to `false`, `duty_pct`, `pulse_width_us`, and `low_width_us` are set to `NAN`, and the LVGL UI renders `"PW+: --  PW-: --"` and `"Duty: --"` instead of fabricating a 50% duty cycle.
4. **Peripheral Error Propagation**:
   - `ensure_freq_peripherals()` and `capture_freq_window()` check and propagate all `gptimer`, `pcnt`, and `gpio` driver errors (`esp_err_t`), cleaning up partial state via `teardown_freq_peripherals()`, recording `reading.hw_error`, and displaying `"[HW ERR]"` in the UI status badge.

---

## 4. Public API Reference (`include/tool_freq.h`)

- **Pure Calculation API (`freq_calc.c`)**:
  - `freq_calc_from_edge_timestamps()`: Extracts period, frequency, high/low pulse widths, duty cycle, and RPM from continuous GPIO edge ISR timestamps (`freq_edge_stamp_t`) in a single stack-free pass.
  - `freq_calc_from_rmt_symbols()`: Extracts period, frequency, high/low pulse widths, duty cycle, and RPM from `rmt_symbol_word_t` buffers in a single stack-free pass.
  - `freq_calc_from_gate_counts()` & `freq_calc_pcnt_max_freq_hz()`: Computes frequency, period, and RPM from PCNT edge counts and gate duration, and reports maximum unattenuated frequency for a given PCNT glitch filter setting.
  - `freq_calc_cross_check()` & `freq_calc_cross_check_tolerance()`: Verdict (`OK` / `MISMATCH` / `SKIPPED`) of 1/T versus gate frequency with tolerance `max(5 %, 3/N)`.
  - `freq_calc_select_method()` & `freq_calc_combine_reading()`: Applies `4.5–5.5 kHz` hysteresis crossover, applies the cross-check, sets `duty_valid = false` (`NaN`) when only gate counts exist, and updates PSRAM session min/max/average statistics.
  - `freq_calc_format_freq()`: Formats frequency values into auto-ranging `"Hz"`, `"kHz"`, or `"MHz"` strings.
- **Runtime Configuration & Lifecycle API (`tool_freq.c`)**:
  - `tool_freq_register()`, `tool_freq_get_descriptor()`
  - `tool_freq_set_pin()` / `tool_freq_get_pin()` / `tool_freq_is_pin_supported()`
  - `tool_freq_set_mode()` / `tool_freq_get_mode()` (`FREQ_MODE_AUTO`, `FREQ_MODE_PERIOD_RMT` / `FREQ_MODE_PERIOD_EDGE`, `FREQ_MODE_GATE_PCNT`)
  - `tool_freq_set_gate_window()` / `tool_freq_get_gate_window()` (`FREQ_GATE_20MS`, `FREQ_GATE_100MS`, `FREQ_GATE_1000MS`)
  - `tool_freq_set_pcnt_glitch_ns()` / `tool_freq_get_pcnt_glitch_ns()` (default `25 ns`, `0..12500 ns`)
  - `tool_freq_set_edge_acq_max_us()` / `tool_freq_get_edge_acq_max_us()` (bounded edge acquisition, default `2 s`)
  - `tool_freq_set_running()` / `tool_freq_is_running()`, `tool_freq_reset_stats()`, `tool_freq_get_latest_reading()`

---

## 5. Limitations & Measurement Floor
- **Minimum frequency**: with the default `2 s` edge window and `5`-edge target, signals of `>= 1 Hz` are measurable when at least 3 edges (one full period, rise-fall-rise) fall inside the window. A 1 Hz 50 % signal always has `>= 4` edges per 2 s window; a 1 Hz *narrow* tach pulse is marginal (exactly 3 edges in the worst phase) and may need a second window before a reading appears. Below `1 Hz` the meter reports `NO SIG`. The PCNT gate result is unaffected (`0` counts at these rates).
- **Update rate**: while a signal has fewer than 5 edges per gate, a window lasts up to the edge limit (`2 s` for a no-signal input in `AUTO` / `1/T` mode, since the absence of a 1 Hz signal cannot be distinguished earlier). Lower it with `tool_freq_set_edge_acq_max_us()` or use `FREQ_MODE_GATE_PCNT` for fast `NO SIG` on high-frequency inputs.
- **ISR load / ceiling of 1/T**: the edge ISR is limited to `512` entries or `128` stored edges per window; at high rates it disables itself within milliseconds and the PCNT gate result is used instead. Duty and pulse width are only trusted up to `25 kHz` in `AUTO` mode (`1 us` timestamps).
- **Cross-check scope**: it detects disagreement, not which estimator is wrong. A noisy input that PCNT counts as extra edges can make the gate value the wrong one; the `unreliable` flag is the honest signal then. Gate counts below `8` are inconclusive.
- **Not verified on M5Core2 hardware in this pass**: tests run in QEMU, where the edge ISR is fed through `tool_freq_record_edge_isr_for_test()` and PCNT counts nothing.
- Input voltage levels must remain within safe ESP32 `0–3.3 V` CMOS logic bounds (`V_IL <= 0.8 V`, `V_IH >= 2.4 V`).
- Pulse width (`PW+`, `PW-`) and duty cycle are measured via GPIO edge timestamps (`1 µs` resolution) up to `25 kHz`; in `FREQ_MODE_GATE_PCNT` or above `25 kHz` where only PCNT gate counts are used, duty cycle and pulse widths are reported as unknown (`duty_valid = false`, `NaN`, UI `"--"`).
