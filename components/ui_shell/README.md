# `ui_shell` — Core 0 LVGL 9 Application Shell, Header Bar & Tool Launcher

## Purpose
`ui_shell` implements the Core 0 (`PRO_CPU`) **LVGL 9** display/touch integration, the persistent `28 px` top status bar, and the scrollable `320×212` 2×3 Tool Launcher Grid for **M5CoreAnalyser**.

---

## Hardware Pins & Peripherals

| Subsystem | Peripheral / Driver | Pins | Role in `ui_shell` |
| :--- | :--- | :--- | :--- |
| **Display Flush** | `ILI9342C` via `bsp_display_get_panel()` | `SPI2` (`GPIO23`/`GPIO18`/`GPIO5`/`GPIO15`) | 320×240 RGB565 partial rendering (2×24-line SRAM DMA buffers + `on_color_trans_done` callback) |
| **Touch Input** | `FT6336U` via `bsp_touch_read_state()` | `I2C_NUM_0` (`0x38`, `GPIO39` IRQ) | 320×240 pointer events + bottom touch buttons (`BtnA`=Home, `BtnB`=Speaker, `BtnC`=Record) |
| **Battery Telemetry** | `AXP192` via `bsp_axp192_get_battery()` | `I2C_NUM_0` (`0x34`) | Live voltage (`V`), percentage (`%`), and charge indicator (`+`) in top header |
| **Speaker Control** | `NS4168` via `bsp_spk_passthrough_*()` | `I2S_NUM_1` (`GPIO12`/`GPIO0`/`GPIO2`) | Header `SPK` toggle button (`SPK OFF` / `SPK ON` / `SPK MUTE`) |

---

## Internal Pipeline

### 1. Persistent 28 px Top Status Header (`ui_header.c`)
Occupies `320×28` (`y = 0..27`) as a single-row flex layout guaranteed to fit within `320 px` even in worst-case badge states:
- **Home Button (`LV_SYMBOL_HOME`)**: Enabled when an instrument is open; returns to the Launcher Grid while retaining background-capable generators.
- **Screen / Instrument Title**: Uses `flex_grow = 1` with `LV_LABEL_LONG_CLIP` to display `"M5CoreAnalyser"` on the launcher screen or the active tool's `title`, automatically shrinking when the `BG` pill is visible.
- **`BG` Pill**: Dynamically appears when `tool_core_get_background_count() > 0` (`"BG:wave"` or `"BG:2"`); tapping stops background generators and releases their GPIOs.
- **Speaker Monitor Button (`SPK`)**: Shows `"SPK-"`, `"SPK0"`, `"SPK1"`, or `"MUTE"` and controls `bsp_spk_passthrough`.
- **Record Toggle (`REC`) & Mode Badge (`USB`/`BT`)**: Bound to `recorder_sink` to toggle capture and switch between `REC_MODE_USB_STUB` and `REC_MODE_BT_STUB` while displaying the live frame counter (`"REC"` when idle, `"#12"` when frames captured).
- **Battery Readout**: Displays AXP192 battery voltage and state of charge (`"3.85V 68%"` or `"4.15V 100%+"`).

### 2. 320×212 Content Container & Launcher Grid (`ui_shell.c`)
Occupies `320×212` (`y = 28..239`):
- Queries `tool_registry_get_all()` to populate a 2×3 grid of touch cards color-coded by `tool_category_t` (`Analog In`, `Digital In`, `Generator`), badging any active background generator with `[BG]`.
- When switching tools in `ui_shell_open_tool(tool_id)`, stops the previous foreground tool (`tool_core_stop_active_tool`) **before** cleaning `s_tool_cont` so `on_stop` callbacks can safely access their LVGL widgets before deletion. Displays a conflict banner (`"Pin conflict starting '...'"`) if a background generator owns a required GPIO.

---

## Public API Reference

Header: [`include/ui_shell.h`](include/ui_shell.h)

- **Lifecycle & Core 0 Task**:
  - `esp_err_t ui_shell_init(void)`
  - `bool ui_shell_is_initialized(void)`
  - `esp_err_t ui_shell_start_task(void)`
  - `void ui_shell_step(void)`
  - `esp_err_t ui_shell_rebuild_launcher(void)`
- **Navigation & Header Actions**:
  - `esp_err_t ui_shell_open_tool(const char *tool_id)`
  - `esp_err_t ui_shell_go_home(bool keep_background)`
  - `esp_err_t ui_shell_toggle_speaker(void)`
  - `esp_err_t ui_shell_toggle_recorder(void)`
  - `esp_err_t ui_shell_cycle_recorder_transport(void)`
  - `esp_err_t ui_shell_get_header_state(ui_header_state_t *out_state)`
  - `esp_err_t ui_shell_get_last_status_msg(char *out_buf, size_t buf_len)`
  - `lv_obj_t *ui_shell_get_tool_container(void)`
- **Pure Formatting Helpers**:
  - `void ui_header_format_battery(uint16_t voltage_mv, uint8_t percent, bool charging, char *out_buf, size_t buf_len)`
  - `void ui_header_format_bg_pill(size_t bg_count, const char *first_bg_id, char *out_buf, size_t buf_len)`
  - `void ui_header_format_spk_badge(bool supported, bool enabled, bool muted, char *out_buf, size_t buf_len)`
  - `void ui_header_format_rec_badge(recorder_mode_t mode, recorder_mode_t transport, uint32_t frames, char *out_rec_buf, size_t rec_len, char *out_mode_buf, size_t mode_len)`
  - `esp_err_t ui_header_compute_state(ui_header_state_t *out_state)`

---

## Configuration & Usage Example

```c
#include "bsp_m5core2.h"
#include "recorder_sink.h"
#include "tool_core.h"
#include "ui_shell.h"

void app_main(void)
{
    bsp_m5core2_init();
    recorder_sink_init();
    tool_core_init();
    ui_shell_init();
    ui_shell_start_task();
}
```

---

## Limitations
- **Single-Thread LVGL Access**: All LVGL widget mutations must occur on Core 0 inside `ui_shell` or the active tool's `on_start`, `on_stop`, and `on_ui_refresh` callbacks; Core 1 worker tasks must communicate exclusively through `psram_double_buf_t` and atomic state.
