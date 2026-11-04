# `tool_freq` — Digital Frequency Meter Component

## 1. Purpose
`tool_freq` implements the **Digital Frequency Meter** instrument (`id = "freq"`, category `TOOL_CATEGORY_DIGITAL_IN`) for **M5CoreAnalyser** on the M5Stack Core2. It combines two complementary ESP32 hardware measurement engines:
1. **Low-Frequency Edge-Period Measurement ($f = 1/T$)**: Uses the ESP32 **RMT RX** peripheral at `1 MHz` (`1 µs` tick resolution) to record high and low pulse durations ($T_{\text{high}}$, $T_{\text{low}}$), calculating period $T = T_{\text{high}} + T_{\text{low}}$, frequency $f = 1/T$, positive duty cycle $\text{Duty} = 100 \cdot T_{\text{high}} / T$, and $\text{RPM} = 60 \cdot f$.
2. **High-Frequency Gate-Time Pulse Counting ($f = N / t_{\text{gate}}$)**: Uses the ESP32 **PCNT** hardware pulse counter unit (counting both rising and falling edges with hardware glitch filtering and 16-bit watch-point overflow accumulation) gated by a `1 MHz` **GPTimer** over selectable `20 ms`, `100 ms`, or `1000 ms` windows for signals up to multi-MHz.

---

## 2. Hardware Pins & Peripherals

| Resource | Assignment | Notes |
|---|---|---|
| **Default Input Pin** | `GPIO36` (`BSP_PIN_ANALOG_IN_DEFAULT`) | Port B / M5-Bus input-only pin |
| **Selectable Safe Pins** | `GPIO36`, `GPIO35`, `GPIO26`, `GPIO13`, `GPIO14`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19` | Claimed atomically via `bsp_pin_claim(pin, "freq")` |
| **RMT RX** | `driver/rmt_rx.h` (`1 MHz` ticks, `1000 ns` glitch filter) | Writes into internal SRAM bounce buffer `s_rmt_bounce_buf` |
| **PCNT** | `driver/pulse_cnt.h` (`[-30000, +30000]` watch limit, `1000 ns` glitch filter) | Counts rising + falling edges ($2\times$ resolution) with ISR overflow accumulator |
| **GPTimer** | `driver/gptimer.h` (`1 MHz` count-up gate timer) | Controls hardware gate duration (`20 ms`, `100 ms`, `1000 ms`) |

---

## 3. Internal Pipeline & PSRAM ISR Safety

1. **ISR Bounce Buffering in Internal SRAM**:
   - All hardware ISR callbacks (`on_rmt_rx_done_isr`, `on_pcnt_watch_reach_isr`, `on_gate_timer_alarm_isr`) are marked `IRAM_ATTR` and write exclusively to `DRAM_ATTR` internal SRAM bounce buffers and counters (`s_rmt_bounce_buf`, `s_pcnt_overflow_accum`, `s_gate_alarm_fired`), avoiding PSRAM cache stalls inside interrupt context.
2. **Core 1 Worker & Cooperative Gate Cancellation**:
   - The Core 1 worker task (`freq_c1`) waits for the gate window in short (`<= 5 ms`) cooperative slices (`wait_gate_or_abort()`), checking `s_stop_requested`, `!s_running`, `s_capture_seq`, and `s_step_preempt`.
   - Switching tools (`freq_on_stop()`) or changing settings (`tool_freq_set_pin()`, `tool_freq_set_mode()`, `tool_freq_set_gate_window()`) aborts any in-flight `100 ms` or `1000 ms` gate window within milliseconds and joins the worker before freeing PSRAM buffers or releasing hardware peripherals.
3. **5 kHz Hysteresis Crossover (`FREQ_MODE_AUTO`)**:
   - Below `5.5 kHz` (`FREQ_CROSSOVER_HIGH_HZ`), `FREQ_MODE_AUTO` uses RMT RX period measurement ($f = 1/T$) for high fractional resolution on low-frequency signals.
   - Above `5.5 kHz`, it transitions to PCNT gate counting ($f = N / t_{\text{gate}}$) and remains in gate mode until the signal drops below `4.5 kHz` (`FREQ_CROSSOVER_LOW_HZ`), preventing mode chatter near `5 kHz`.
4. **PSRAM Staging, Session Tracking & Recorder Export**:
   - Captured RMT symbols are staged into `s_psram_symbols` (`psram_buf_calloc`), session `Fmin`/`Fmax`/`Favg` are tracked in a 16-window PSRAM circular buffer (`freq_session_tracker_t`), UI frames are published tear-free via `psram_double_buf_t`, and frames are exported to `recorder_sink_push()` when recording is enabled.

---

## 4. Public API Reference (`include/tool_freq.h`)

- **Pure Calculation API (`freq_calc.c`)**:
  - `freq_calc_from_rmt_symbols()`: Extracts period, frequency, high/low pulse widths, duty cycle, and RPM from `rmt_symbol_word_t` buffers with glitch rejection and same-level coalescing.
  - `freq_calc_from_gate_counts()`: Computes frequency, period, and RPM from PCNT edge counts and gate duration.
  - `freq_calc_select_method()` & `freq_calc_combine_reading()`: Applies `4.5–5.5 kHz` hysteresis crossover and updates session min/max/average statistics.
  - `freq_calc_format_freq()`: Formats frequency values into auto-ranging `"Hz"`, `"kHz"`, or `"MHz"` strings.
- **Runtime Configuration & Lifecycle API (`tool_freq.c`)**:
  - `tool_freq_register()`, `tool_freq_get_descriptor()`
  - `tool_freq_set_pin()` / `tool_freq_get_pin()` / `tool_freq_is_pin_supported()`
  - `tool_freq_set_mode()` / `tool_freq_get_mode()` (`FREQ_MODE_AUTO`, `FREQ_MODE_PERIOD_RMT`, `FREQ_MODE_GATE_PCNT`)
  - `tool_freq_set_gate_window()` / `tool_freq_get_gate_window()` (`FREQ_GATE_20MS`, `FREQ_GATE_100MS`, `FREQ_GATE_1000MS`)
  - `tool_freq_set_running()` / `tool_freq_is_running()`, `tool_freq_reset_stats()`, `tool_freq_get_latest_reading()`

---

## 5. Limitations
- Input voltage levels must remain within safe ESP32 `0–3.3 V` CMOS logic bounds (`V_IL <= 0.8 V`, `V_IH >= 2.4 V`).
- In `FREQ_MODE_GATE_PCNT` above the RMT RX symbol capture range, duty cycle defaults to `50.0%` unless valid RMT RX pulse-width symbols were also captured in the window.
