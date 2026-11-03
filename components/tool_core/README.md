# `tool_core` — Extensible Tool Plugin Registry, PSRAM Double Buffer & Concurrency Manager

## Purpose
`tool_core` defines the `analyser_tool_t` plugin interface, dynamic instrument registry, foreground/background generator concurrency state machine, Core 1 worker task lifecycle helpers, and PSRAM ping-pong double buffering for **M5CoreAnalyser**.

---

## Hardware Pins & Peripherals
`tool_core` does not directly claim hardware peripherals, but coordinates with `bsp_m5core2` (`bsp_pin_release_all_for_owner` and `bsp_spk_passthrough_enable`) during foreground/background tool transitions and allocates sample buffers from the M5Stack Core2's **8 MB external SPI PSRAM** (`MALLOC_CAP_SPIRAM`).

---

## Internal Pipeline

### 1. Dynamic Tool Registry (`tool_registry.c`)
- Stores up to `TOOL_REGISTRY_MAX_TOOLS` (`12`) `analyser_tool_t` descriptors in registration order under a recursive FreeRTOS mutex.
- Validates required descriptor fields (`id`, `title`, `category`, `on_start`, `on_stop`), rejects duplicate `id` slugs (`ESP_ERR_INVALID_STATE`), and runs `on_init(user_ctx)` during registration.
- Supports full enumeration (`tool_registry_get_all`), category filtering (`tool_registry_get_by_category`), and slug lookup (`tool_registry_find`).

### 2. Foreground & Background Concurrency Manager (`tool_registry.c`)
- Separates the **Foreground Active Tool** (which owns the 320×212 LVGL content area) from **Background Generators** (`supports_background == true`, such as `tool_wavegen` and `tool_analoggen`).
- When `tool_core_stop_active_tool(true)` is called on a background-capable generator, its peripheral and GPIO claims remain active while `tool_core` tracks it in the background list (`tool_core_get_background_tools`).
- When a tool stops with `keep_background == false` (or when a non-background tool exits), `tool_core` automatically releases any remaining GPIOs owned by that tool via `bsp_pin_release_all_for_owner()` and disables speaker passthrough if no remaining active/background tool requires it.

### 3. Tear-Free 3-Slot PSRAM Sample Buffer (`psram_buffer.c`)
- `psram_double_buf_alloc()` allocates three equal-sized sample buffer slots (`buf_a`, `buf_b`, `buf_c`) indexed by `active_write_idx`, `ready_idx`, and `active_read_idx` (always a permutation of `{0, 1, 2}`). Memory is allocated using compile-time `#if CONFIG_SPIRAM` selection (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT` in production firmware, `MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT` when `CONFIG_SPIRAM=n` in QEMU).
- **Core 1 Producer**: Writes into `psram_double_buf_get_write_ptr()` (`active_write_idx`) and calls `psram_double_buf_swap()` to atomically exchange `active_write_idx <-> ready_idx` and set `frame_ready = true` under `dbuf->spinlock`. The producer never touches `active_read_idx`.
- **Core 0 Consumer**: Acquires or latches the ready frame (`active_read_idx <-> ready_idx`) via `psram_double_buf_acquire_read()` / `psram_double_buf_release_read()`, `psram_double_buf_consume_ready()`, or `psram_double_buf_read_copy()`. In `psram_double_buf_read_copy()`, `dbuf->spinlock` is held only for the index exchange and `reader_busy` flag update while `memcpy` executes outside the critical section, preventing both frame tearing and long interrupt-disabled stalls.

### 4. Core 1 Worker Task Helper & Self-Test Seam
- `tool_worker_start()` pins an acquisition/DSP task to **Core 1 (`APP_CPU`)** and `tool_worker_stop()` signals termination and awaits a binary completion semaphore before freeing the worker context.
- `tool_core_run_selftests()` runs built-in and component-registered deterministic self-tests without disrupting normal boot unless `CONFIG_ANALYSER_RUN_SELFTEST_ON_BOOT=y` is enabled.

---

## Public API Reference

Headers: [`include/tool_core.h`](include/tool_core.h), [`include/psram_buffer.h`](include/psram_buffer.h)

- **Registry & Lifecycle**:
  - `esp_err_t tool_core_init(void)`
  - `esp_err_t tool_registry_register(const analyser_tool_t *tool)`
  - `size_t tool_registry_get_all(const analyser_tool_t **out_tools, size_t max_tools)`
  - `size_t tool_registry_get_by_category(tool_category_t category, const analyser_tool_t **out_tools, size_t max_tools)`
  - `const analyser_tool_t *tool_registry_find(const char *id)`
  - `esp_err_t tool_core_start_tool(const char *tool_id, lv_obj_t *parent_cont)`
  - `esp_err_t tool_core_stop_active_tool(bool keep_background)`
  - `esp_err_t tool_core_stop_background_tool(const char *tool_id)`
  - `esp_err_t tool_core_stop_all_background_tools(void)`
  - `const analyser_tool_t *tool_core_get_active_tool(void)`
  - `bool tool_core_is_tool_running_bg(const char *tool_id)`
  - `size_t tool_core_get_background_tools(const analyser_tool_t **out_tools, size_t max_tools)`
  - `size_t tool_core_get_background_count(void)`
  - `void tool_core_ui_refresh_active(void)`
- **PSRAM Double/Triple Buffer**:
  - `esp_err_t psram_double_buf_alloc(psram_double_buf_t *dbuf, size_t capacity_samples, size_t sample_size)`
  - `void psram_double_buf_free(psram_double_buf_t *dbuf)`
  - `void *psram_double_buf_get_write_ptr(psram_double_buf_t *dbuf)`
  - `const void *psram_double_buf_get_read_ptr(psram_double_buf_t *dbuf)`
  - `esp_err_t psram_double_buf_swap(psram_double_buf_t *dbuf)`
  - `bool psram_double_buf_acquire_read(psram_double_buf_t *dbuf, const void **out_read_ptr)`
  - `void psram_double_buf_release_read(psram_double_buf_t *dbuf)`
  - `bool psram_double_buf_consume_ready(psram_double_buf_t *dbuf, const void **out_read_ptr)`
  - `esp_err_t psram_double_buf_read_copy(psram_double_buf_t *dbuf, void *dst_buf, size_t max_samples, size_t *out_copied)`
- **Core 1 Worker & Self-Test**:
  - `esp_err_t tool_worker_start(tool_worker_t **out_worker, const tool_worker_config_t *cfg)`
  - `esp_err_t tool_worker_stop(tool_worker_t *worker, uint32_t timeout_ms)`
  - `esp_err_t tool_core_register_selftest(const char *name, tool_selftest_fn_t fn)`
  - `esp_err_t tool_core_run_selftests(size_t *out_passed, size_t *out_failed)`

---

## Configuration & Usage Example

```c
#include "tool_core.h"

static esp_err_t my_tool_start(lv_obj_t *parent, void *ctx) { return ESP_OK; }
static esp_err_t my_tool_stop(bool keep_bg, void *ctx)     { return ESP_OK; }

static const analyser_tool_t s_my_tool = {
    .id = "custom_probe",
    .title = "Custom Probe",
    .symbol = LV_SYMBOL_CHARGE,
    .category = TOOL_CATEGORY_ANALOG_IN,
    .on_start = my_tool_start,
    .on_stop = my_tool_stop,
};

void register_custom_tool(void)
{
    tool_core_init();
    tool_registry_register(&s_my_tool);
}
```

---

## Limitations
- **Maximum Registered Tools**: `TOOL_REGISTRY_MAX_TOOLS` is set to `12` static descriptors.
- **PSRAM ISR Access**: `psram_double_buf_t` buffers reside in external SPI PSRAM and must be accessed from tasks (Core 0 / Core 1) rather than directly inside `IRAM_ATTR` hardware timer ISRs.
