# `recorder_sink` — Stubbed USB / Bluetooth Sample Export Sink

## Purpose
`recorder_sink` provides the thread-safe sample frame export contract and recording state machine for **M5CoreAnalyser**. It decouples Core 1 measurement instruments (`tool_voltmeter`, `tool_scope`, `tool_logic`, `tool_freq`) and Core 0 UI header controls (`ui_shell`) from future USB CDC bulk and Bluetooth (BLE/SPP) transport implementations.

---

## Hardware Pins & Peripherals
`recorder_sink` is a pure software component and does not claim any ESP32 GPIOs or hardware peripherals directly.

---

## Internal Pipeline

### 1. Recording State Machine (`recorder_sink.c`)
- Maintains two spinlock-protected (`portMUX_TYPE`) mode fields:
  - `mode`: Active capture state (`REC_MODE_OFF`, `REC_MODE_USB_STUB`, or `REC_MODE_BT_STUB`).
  - `selected_transport`: Preferred streaming target (`REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`, defaulting to `REC_MODE_USB_STUB`).
- `recorder_sink_toggle()` switches atomically between `REC_MODE_OFF` and `selected_transport`.
- `recorder_sink_set_transport()` updates `selected_transport` and, if recording is currently active, immediately switches `mode` to the newly selected transport.

### 2. Sample Frame Ingestion & Accounting (`recorder_sink_push`)
- Validates every incoming `recorder_frame_t` descriptor (non-empty `tool_id`, non-null `samples`, non-zero `sample_count` and `bytes_per_sample`, and overflow-safe `sample_count * bytes_per_sample`).
- When `mode == REC_MODE_OFF`, increments `dropped_frames` and returns `ESP_ERR_INVALID_STATE` without altering captured frame or byte totals.
- When `mode == REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`, atomically increments `frames_pushed`, `usb_frames` / `bt_frames`, `samples_pushed`, and `bytes_pushed`, and records metadata for the latest frame (`last_tool_id`, `last_sample_rate_hz`, `last_timestamp_us`).

---

## Public API Reference

Header: [`include/recorder_sink.h`](include/recorder_sink.h)

- **Lifecycle & Mode Control**:
  - `esp_err_t recorder_sink_init(void)`
  - `esp_err_t recorder_sink_set_mode(recorder_mode_t mode)`
  - `recorder_mode_t recorder_sink_get_mode(void)`
  - `esp_err_t recorder_sink_set_transport(recorder_mode_t transport)`
  - `recorder_mode_t recorder_sink_get_transport(void)`
  - `esp_err_t recorder_sink_toggle(void)`
  - `bool recorder_sink_is_recording(void)`
  - `const char *recorder_sink_mode_to_str(recorder_mode_t mode)`
- **Data Ingestion & Telemetry**:
  - `esp_err_t recorder_sink_push(const recorder_frame_t *frame)`
  - `esp_err_t recorder_sink_get_stats(recorder_sink_stats_t *out_stats)`
  - `esp_err_t recorder_sink_reset_stats(void)`

---

## Configuration & Usage Example

```c
#include "recorder_sink.h"

void export_scope_frame(const uint16_t *samples, size_t count, uint64_t ts_us)
{
    if (!recorder_sink_is_recording()) {
        return;
    }

    recorder_frame_t frame = {
        .tool_id = "scope",
        .sample_rate_hz = 20000,
        .timestamp_us = ts_us,
        .samples = samples,
        .sample_count = count,
        .bytes_per_sample = sizeof(uint16_t),
    };
    (void)recorder_sink_push(&frame);
}
```

---

## Limitations
- **Stub Transport Only**: `REC_MODE_USB_STUB` and `REC_MODE_BT_STUB` validate and account for frame metadata and byte throughput in RAM without initializing an ESP32 Bluetooth stack or USB serial stream.
