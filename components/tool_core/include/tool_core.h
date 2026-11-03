/**
 * @file tool_core.h
 * @brief Extensible instrument plugin contract, dynamic tool registry, background generator
 *        concurrency manager, Core 1 worker helpers, and self-test runner for M5CoreAnalyser.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "psram_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of instruments that can be registered in `tool_core`. */
#define TOOL_REGISTRY_MAX_TOOLS    (12)

/** Maximum number of registered self-test callbacks in `tool_core`. */
#define TOOL_CORE_MAX_SELFTESTS    (16)

/** Default Core ID for real-time instrument acquisition/DSP tasks (`APP_CPU`). */
#if (portNUM_PROCESSORS > 1)
#define TOOL_WORKER_DEFAULT_CORE   (1)
#else
#define TOOL_WORKER_DEFAULT_CORE   (tskNO_AFFINITY)
#endif

/**
 * @brief Functional category of an instrument tool.
 */
typedef enum {
    TOOL_CATEGORY_ANALOG_IN = 0, /**< Analog measurement instrument (Voltmeter, Oscilloscope) */
    TOOL_CATEGORY_DIGITAL_IN,    /**< Digital measurement instrument (Logic Analyzer, Frequency Meter) */
    TOOL_CATEGORY_GENERATOR,     /**< Signal generator instrument (Wave Generator, Analog Generator) */
} tool_category_t;

/**
 * @brief Descriptor contract implemented by every M5CoreAnalyser instrument component.
 */
typedef struct analyser_tool {
    const char *id;                  /**< Unique slug identifier, e.g. `"scope"`, `"voltmeter"` */
    const char *title;               /**< Human-readable display title, e.g. `"Oscilloscope"` */
    const char *symbol;              /**< LVGL icon symbol string, e.g. `LV_SYMBOL_IMAGE` */
    tool_category_t category;        /**< Functional category */
    bool supports_background;        /**< True for generators (`wavegen`, `analoggen`) that can run in background */
    bool supports_recording;         /**< True for input instruments that push `recorder_frame_t` blocks */
    bool supports_spk_passthrough;   /**< True for tools that can stream audio to the NS4168 speaker */

    esp_err_t (*on_init)(void *ctx);                             /**< Optional one-time initialization callback */
    esp_err_t (*on_start)(lv_obj_t *parent_cont, void *ctx);     /**< Start foreground UI in `parent_cont` and acquisition */
    esp_err_t (*on_stop)(bool keep_background, void *ctx);       /**< Stop foreground UI; retain generator if `keep_background` */
    void      (*on_ui_refresh)(void *ctx);                       /**< Called periodically on Core 0 LVGL timer (~30 Hz) */
    bool      (*is_running_bg)(void *ctx);                       /**< Optional query returning true if generator is active in background */
    void     *user_ctx;                                          /**< Opaque tool-specific context pointer */
} analyser_tool_t;

/* --------------------------------------------------------------------------
 * Tool Core Initialization & Registry API
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize the `tool_core` subsystem and synchronization primitives.
 *
 * If `CONFIG_ANALYSER_RUN_SELFTEST_ON_BOOT` is enabled in Kconfig, also runs registered
 * self-tests once at startup.
 *
 * @return `ESP_OK` on success, or error code if boot self-tests fail.
 */
esp_err_t tool_core_init(void);

/**
 * @brief Register an instrument descriptor with the global tool registry.
 *
 * Validates required fields (`id`, `title`, `category`, `on_start`, `on_stop`), rejects
 * duplicate `id` slugs with `ESP_ERR_INVALID_STATE`, and invokes `tool->on_init(tool->user_ctx)`
 * if non-NULL before adding the descriptor.
 *
 * @param[in] tool Pointer to static/persistent `analyser_tool_t` descriptor.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `tool` or required fields are invalid
 *  - `ESP_ERR_INVALID_STATE` if a tool with `tool->id` is already registered
 *  - `ESP_ERR_NO_MEM` if the registry has reached `TOOL_REGISTRY_MAX_TOOLS`
 */
esp_err_t tool_registry_register(const analyser_tool_t *tool);

/**
 * @brief Retrieve all registered tools in registration order.
 *
 * If `out_tools` is `NULL`, returns the total number of registered tools without copying.
 *
 * @param[out] out_tools Array to receive tool descriptor pointers (may be `NULL`).
 * @param[in]  max_tools Maximum number of pointers to write into `out_tools`.
 * @return Number of tools written (or total registered count when `out_tools == NULL`).
 */
size_t tool_registry_get_all(const analyser_tool_t **out_tools, size_t max_tools);

/**
 * @brief Retrieve registered tools matching a specific `tool_category_t`.
 *
 * If `out_tools` is `NULL`, returns the total count of tools in `category`.
 *
 * @param[in]  category  Target tool category.
 * @param[out] out_tools Array to receive matching tool descriptor pointers (may be `NULL`).
 * @param[in]  max_tools Maximum number of pointers to write into `out_tools`.
 * @return Number of matching tools written (or total matching count when `out_tools == NULL`).
 */
size_t tool_registry_get_by_category(tool_category_t category,
                                     const analyser_tool_t **out_tools,
                                     size_t max_tools);

/**
 * @brief Look up a registered tool by its unique slug `id`.
 *
 * @param[in] id Tool slug string (e.g. `"scope"`).
 * @return Pointer to registered `analyser_tool_t`, or `NULL` if not found.
 */
const analyser_tool_t *tool_registry_find(const char *id);

/**
 * @brief Return a human-readable label for a `tool_category_t`.
 *
 * @param[in] category Tool category enum.
 * @return Static null-terminated string (`"Analog In"`, `"Digital In"`, `"Generator"`, or `"Unknown"`).
 */
const char *tool_category_to_str(tool_category_t category);

/* --------------------------------------------------------------------------
 * Foreground & Background Concurrency Lifecycle Manager
 * -------------------------------------------------------------------------- */

/**
 * @brief Start a registered tool in the foreground inside `parent_cont`.
 *
 * If another foreground tool is currently active, stops that tool (`keep_background = false`)
 * before starting `tool_id`. If `tool_id` was running in the background, promotes it back to
 * foreground active status and invokes `on_start(parent_cont, user_ctx)`.
 *
 * @param[in] tool_id     Unique slug of the tool to launch.
 * @param[in] parent_cont LVGL parent container for the tool's UI (may be `NULL` in headless tests).
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `tool_id` is `NULL`/empty
 *  - `ESP_ERR_NOT_FOUND` if `tool_id` is not registered
 *  - Error returned by `on_start()` (e.g. `ESP_ERR_INVALID_STATE` on GPIO pin conflict)
 */
esp_err_t tool_core_start_tool(const char *tool_id, lv_obj_t *parent_cont);

/**
 * @brief Stop the currently active foreground tool.
 *
 * If `keep_background` is true and the active tool has `supports_background == true`, the tool
 * transitions to the background running list (`on_stop(true, user_ctx)`) and retains its claimed
 * output pins. Otherwise, `on_stop(false, user_ctx)` is called, any pins still owned by the tool
 * are released via `bsp_pin_release_all_for_owner()`, and speaker passthrough is disabled if no
 * other active/background tool requires it.
 *
 * @param[in] keep_background True to keep a background-capable generator running.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_STATE` if no foreground tool is currently active
 */
esp_err_t tool_core_stop_active_tool(bool keep_background);

/**
 * @brief Stop a specific generator currently running in the background and release its pins.
 *
 * @param[in] tool_id Slug ID of the background generator to stop.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `tool_id` is `NULL`/empty
 *  - `ESP_ERR_NOT_FOUND` if `tool_id` is not registered
 *  - `ESP_ERR_INVALID_STATE` if `tool_id` is not currently running in the background
 */
esp_err_t tool_core_stop_background_tool(const char *tool_id);

/**
 * @brief Stop all generators currently running in the background and release their pins.
 *
 * @return `ESP_OK` on success, or the first error encountered.
 */
esp_err_t tool_core_stop_all_background_tools(void);

/**
 * @brief Get the currently active foreground tool descriptor.
 *
 * @return Pointer to active `analyser_tool_t`, or `NULL` when on the Home launcher screen.
 */
const analyser_tool_t *tool_core_get_active_tool(void);

/**
 * @brief Check whether the tool with slug `tool_id` is currently running in the background.
 *
 * @param[in] tool_id Tool slug string.
 * @return True if running in background, false otherwise.
 */
bool tool_core_is_tool_running_bg(const char *tool_id);

/**
 * @brief Retrieve all tools currently running in the background.
 *
 * @param[out] out_tools Array to receive background tool pointers (may be `NULL`).
 * @param[in]  max_tools Maximum number of pointers to write into `out_tools`.
 * @return Number of background tools written (or total active background count if `out_tools == NULL`).
 */
size_t tool_core_get_background_tools(const analyser_tool_t **out_tools, size_t max_tools);

/**
 * @brief Return the number of generators currently running in the background.
 *
 * @return Active background generator count (`0..TOOL_REGISTRY_MAX_TOOLS`).
 */
size_t tool_core_get_background_count(void);

/**
 * @brief Invoke `on_ui_refresh()` on the active foreground tool (called from Core 0 LVGL timer).
 */
void tool_core_ui_refresh_active(void);

/* --------------------------------------------------------------------------
 * Core 1 Instrument Worker Task Helper
 * -------------------------------------------------------------------------- */

/** Opaque handle for a Core 1 instrument worker task. */
typedef struct tool_worker tool_worker_t;

/**
 * @brief Callback invoked repeatedly by a Core 1 worker task until stop is requested.
 *
 * Returning `ESP_OK` continues execution; returning any non-OK `esp_err_t` terminates the
 * worker loop early.
 */
typedef esp_err_t (*tool_worker_step_fn_t)(void *user_ctx);

/**
 * @brief Configuration parameters for spawning a Core 1 instrument worker task.
 */
typedef struct {
    const char *name;              /**< FreeRTOS task name (defaults to `"tool_worker"` if `NULL`) */
    uint32_t stack_size_bytes;     /**< Stack size in bytes (defaults to `4096` if `0`) */
    UBaseType_t priority;          /**< FreeRTOS task priority (defaults to `5` if `0`) */
    BaseType_t core_id;            /**< CPU core affinity (`TOOL_WORKER_DEFAULT_CORE` = Core 1) */
    uint32_t period_ms;            /**< Delay between `step_fn` iterations in ms (`0` = 1 tick yield) */
    tool_worker_step_fn_t step_fn; /**< Non-NULL step function executed each iteration */
    void *user_ctx;                /**< Opaque context passed to `step_fn` */
} tool_worker_config_t;

/**
 * @brief Spawn a Core 1 worker task that executes `cfg->step_fn` in a managed loop.
 *
 * @param[out] out_worker Pointer to receive the allocated worker handle.
 * @param[in]  cfg        Worker task configuration.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments, `ESP_ERR_NO_MEM` on failure.
 */
esp_err_t tool_worker_start(tool_worker_t **out_worker, const tool_worker_config_t *cfg);

/**
 * @brief Signal a running worker task to stop, wait for completion, and free its handle.
 *
 * @param[in] worker     Worker handle returned by `tool_worker_start()`.
 * @param[in] timeout_ms Maximum time in ms to wait for the task to exit (defaults to `500` if `0`).
 * @return `ESP_OK` if task exited cleanly and handle was freed, `ESP_ERR_TIMEOUT` if it did not exit in time.
 */
esp_err_t tool_worker_stop(tool_worker_t *worker, uint32_t timeout_ms);

/**
 * @brief Check whether a worker task is currently running.
 *
 * @param[in] worker Worker handle.
 * @return True if running, false otherwise.
 */
bool tool_worker_is_running(const tool_worker_t *worker);

/**
 * @brief Check whether a stop request has been issued for the worker task.
 *
 * @param[in] worker Worker handle.
 * @return True if stop has been requested, false otherwise.
 */
bool tool_worker_stop_requested(const tool_worker_t *worker);

/* --------------------------------------------------------------------------
 * Non-Disruptive Self-Test Seam
 * -------------------------------------------------------------------------- */

/** Self-test callback signature returning `ESP_OK` on pass. */
typedef esp_err_t (*tool_selftest_fn_t)(void);

/**
 * @brief Register a deterministic component self-test callback.
 *
 * @param[in] name Short descriptive name of the self-test.
 * @param[in] fn   Self-test function returning `ESP_OK` on pass.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_NO_MEM` on error.
 */
esp_err_t tool_core_register_selftest(const char *name, tool_selftest_fn_t fn);

/**
 * @brief Execute all built-in and registered `tool_core` self-tests without disrupting hardware state.
 *
 * @param[out] out_passed Optional pointer to receive count of passed self-tests.
 * @param[out] out_failed Optional pointer to receive count of failed self-tests.
 * @return `ESP_OK` if all self-tests passed, `ESP_FAIL` if any failed.
 */
esp_err_t tool_core_run_selftests(size_t *out_passed, size_t *out_failed);

#ifdef __cplusplus
}
#endif
