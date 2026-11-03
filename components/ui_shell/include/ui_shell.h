/**
 * @file ui_shell.h
 * @brief LVGL 9 Core 0 application shell, persistent 28 px status header, and 2x3 tool launcher grid.
 *
 * Manages display/touch port binding on Core 0 (`PRO_CPU`), the persistent top status bar
 * (Home button, tool title, `BG GEN` indicator pill, Speaker Monitor toggle, `REC` button +
 * `USB`/`BT` mode selector, AXP192 battery readout), and switching the `320x212` content area
 * between the Tool Launcher Grid and the active instrument's LVGL container.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"
#include "recorder_sink.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name UI Shell Layout Geometry (320x240 ILI9342C Display)
 *  @{
 */
#define UI_SHELL_SCREEN_W          (320)
#define UI_SHELL_SCREEN_H          (240)
#define UI_SHELL_HEADER_H          (28)
#define UI_SHELL_CONTENT_H         (UI_SHELL_SCREEN_H - UI_SHELL_HEADER_H) /**< 212 px */
#define UI_SHELL_REFRESH_HZ        (30)
#define UI_SHELL_STATUS_MSG_MAX    (64)
/** @} */

/**
 * @brief Snapshot of the persistent 28 px top status bar view-model state.
 */
typedef struct {
    bool home_enabled;                /**< True when an instrument is open in the foreground */
    char title[32];                   /**< Displayed screen/instrument title */
    bool bg_pill_visible;             /**< True when >= 1 background generator is active */
    size_t bg_tool_count;             /**< Count of active background generators */
    char bg_pill_text[24];            /**< Formatted background indicator pill label (`"BG:wave"`, `"BG:2"`) */
    bool spk_supported;               /**< True if active or background tool supports speaker audio */
    bool spk_enabled;                 /**< True if speaker passthrough is active and unmuted */
    bool spk_muted;                   /**< True if speaker passthrough is muted */
    char spk_badge_text[16];          /**< Formatted speaker button label (`"SPK1"`, `"MUTE"`, `"SPK0"`, `"SPK-"`) */
    recorder_mode_t rec_mode;         /**< Current recorder sink mode (`OFF`, `USB_STUB`, `BT_STUB`) */
    recorder_mode_t rec_transport;    /**< Selected recorder transport (`USB_STUB` or `BT_STUB`) */
    uint32_t rec_frames;              /**< Total recorded frames from `recorder_sink` */
    char rec_badge_text[24];          /**< Formatted record button text (`"REC"`, `"#12"`, etc.) */
    char transport_badge_text[8];     /**< Formatted transport selector badge (`"USB"` or `"BT"`) */
    uint16_t battery_mv;              /**< Battery voltage in millivolts from AXP192 */
    uint8_t battery_pct;              /**< Battery state-of-charge percentage (`0..100`) */
    bool battery_charging;            /**< True if AXP192 reports battery charging */
    char battery_text[24];            /**< Formatted battery readout (`"3.85V 68%"` or `"4.15V 100%+"`) */
} ui_header_state_t;

/* --------------------------------------------------------------------------
 * Pure Header Formatting & View-Model Functions (ui_header.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Format AXP192 battery telemetry into a compact status bar string.
 *
 * Examples: `"3.85V 68%"` (discharging) or `"4.15V 100%+"` (charging).
 *
 * @param[in]  voltage_mv Battery voltage in millivolts.
 * @param[in]  percent    State of charge `0..100`.
 * @param[in]  charging   True if charging.
 * @param[out] out_buf    Destination string buffer.
 * @param[in]  buf_len    Size of `out_buf` in bytes.
 */
void ui_header_format_battery(uint16_t voltage_mv,
                              uint8_t percent,
                              bool charging,
                              char *out_buf,
                              size_t buf_len);

/**
 * @brief Format the `BG GEN` indicator pill text from the active background generator list.
 *
 * @param[in]  bg_count    Number of active background generators.
 * @param[in]  first_bg_id Slug ID of the first background generator (may be `NULL`).
 * @param[out] out_buf     Destination string buffer.
 * @param[in]  buf_len     Size of `out_buf` in bytes.
 */
void ui_header_format_bg_pill(size_t bg_count,
                              const char *first_bg_id,
                              char *out_buf,
                              size_t buf_len);

/**
 * @brief Format the Speaker Monitor button badge text.
 *
 * @param[in]  supported True if an active/background tool supports speaker passthrough.
 * @param[in]  enabled   True if `bsp_spk_passthrough_is_enabled()` is true.
 * @param[in]  muted     True if `bsp_spk_passthrough_is_muted()` is true.
 * @param[out] out_buf   Destination string buffer.
 * @param[in]  buf_len   Size of `out_buf` in bytes.
 */
void ui_header_format_spk_badge(bool supported,
                                bool enabled,
                                bool muted,
                                char *out_buf,
                                size_t buf_len);

/**
 * @brief Format the Record toggle button label and USB/BT transport badge.
 *
 * @param[in]  mode         Active `recorder_mode_t`.
 * @param[in]  transport    Selected `recorder_mode_t` transport (`USB_STUB` or `BT_STUB`).
 * @param[in]  frames       Total captured frames.
 * @param[out] out_rec_buf  Destination buffer for the Record button label.
 * @param[in]  rec_len      Size of `out_rec_buf` in bytes.
 * @param[out] out_mode_buf Destination buffer for the transport selector badge (`"USB"` / `"BT"`).
 * @param[in]  mode_len     Size of `out_mode_buf` in bytes.
 */
void ui_header_format_rec_badge(recorder_mode_t mode,
                                recorder_mode_t transport,
                                uint32_t frames,
                                char *out_rec_buf,
                                size_t rec_len,
                                char *out_mode_buf,
                                size_t mode_len);

/**
 * @brief Compute a complete `ui_header_state_t` snapshot from `tool_core`, `recorder_sink`, and `bsp_m5core2`.
 *
 * @param[out] out_state Pointer to `ui_header_state_t` to populate.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_state` is `NULL`.
 */
esp_err_t ui_header_compute_state(ui_header_state_t *out_state);

/* --------------------------------------------------------------------------
 * UI Shell Lifecycle, Navigation & Controls (ui_shell.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize LVGL 9, bind the ILI9342C display and FT6336U touch port, and create the
 *        persistent 28 px header bar and 320x212 content/launcher views.
 *
 * Safe to run even if physical LCD/I2C hardware is absent (uses a virtual flush callback in
 * headless/emulated environments).
 *
 * @return `ESP_OK` on success, or an `esp_err_t` code.
 */
esp_err_t ui_shell_init(void);

/**
 * @brief Return true if `ui_shell_init()` has completed.
 *
 * @return True if initialized, false otherwise.
 */
bool ui_shell_is_initialized(void);

/**
 * @brief Spawn the Core 0 (`PRO_CPU`) LVGL UI refresh and input task (~30 Hz).
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t ui_shell_start_task(void);

/**
 * @brief Execute a single Core 0 UI refresh step (header update, active tool refresh, and `lv_timer_handler()`).
 */
void ui_shell_step(void);

/**
 * @brief Rebuild the 2x3 Tool Launcher Grid from all tools currently registered in `tool_core`.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t ui_shell_rebuild_launcher(void);

/**
 * @brief Launch a registered tool by its `tool_id` in the 320x212 content container.
 *
 * On failure (such as a GPIO conflict with a background generator), preserves the current view,
 * records a descriptive status banner message, and returns the error code.
 *
 * @param[in] tool_id Unique slug of the tool to launch.
 * @return `ESP_OK` on success, or error code from `tool_core_start_tool()`.
 */
esp_err_t ui_shell_open_tool(const char *tool_id);

/**
 * @brief Return from the active foreground tool to the Home Tool Launcher Grid.
 *
 * @param[in] keep_background True to keep a background-capable generator running in the background.
 * @return `ESP_OK` on success.
 */
esp_err_t ui_shell_go_home(bool keep_background);

/**
 * @brief Toggle the Speaker Monitor passthrough / mute state from the top header bar.
 *
 * @return `ESP_OK` on success, `ESP_ERR_NOT_SUPPORTED` if no active/background tool supports speaker output.
 */
esp_err_t ui_shell_toggle_speaker(void);

/**
 * @brief Toggle sample recording (`REC_MODE_OFF` <-> selected `USB`/`BT` transport) from the header bar.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t ui_shell_toggle_recorder(void);

/**
 * @brief Cycle the recorder transport selection between `REC_MODE_USB_STUB` and `REC_MODE_BT_STUB`.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t ui_shell_cycle_recorder_transport(void);

/**
 * @brief Copy out the latest computed `ui_header_state_t` snapshot.
 *
 * @param[out] out_state Destination structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_state` is `NULL`.
 */
esp_err_t ui_shell_get_header_state(ui_header_state_t *out_state);

/**
 * @brief Copy out the most recent UI status/error banner message into a caller buffer.
 *
 * @param[out] out_buf Destination character buffer.
 * @param[in]  buf_len Size of `out_buf` in bytes.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if arguments are invalid.
 */
esp_err_t ui_shell_get_last_status_msg(char *out_buf, size_t buf_len);

/**
 * @brief Get the `320x212` LVGL container object used to host the active foreground tool UI.
 *
 * @return LVGL object pointer, or `NULL` if `ui_shell_init()` has not been called.
 */
lv_obj_t *ui_shell_get_tool_container(void);

#ifdef __cplusplus
}
#endif
