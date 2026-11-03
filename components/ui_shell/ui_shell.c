/**
 * @file ui_shell.c
 * @brief Core 0 LVGL 9 application shell, display/touch binding, and 2x3 tool launcher grid.
 */

#include "ui_shell.h"
#include "ui_shell_priv.h"

#include <stdio.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "recorder_sink.h"
#include "tool_core.h"

static const char *TAG = "ui_shell";

#define UI_DRAW_BUF_LINES   (24)
#define UI_DRAW_BUF_PIXELS  (UI_SHELL_SCREEN_W * UI_DRAW_BUF_LINES)

static portMUX_TYPE s_ui_state_lock = portMUX_INITIALIZER_UNLOCKED;
static ui_header_state_t s_last_header_state;
static char s_last_status_msg[UI_SHELL_STATUS_MSG_MAX] = "Ready";

static bool s_ui_initialized = false;
static bool s_async_flush_cb_registered = false;
static TaskHandle_t s_ui_task_handle = NULL;
static lv_display_t *s_lv_disp = NULL;
static lv_indev_t *s_lv_indev = NULL;
static uint16_t *s_draw_buf1 = NULL;
static uint16_t *s_draw_buf2 = NULL;

static lv_obj_t *s_root_screen = NULL;
static lv_obj_t *s_launcher_cont = NULL;
static lv_obj_t *s_launcher_grid = NULL;
static lv_obj_t *s_status_banner = NULL;
static lv_obj_t *s_tool_cont = NULL;

static bool s_prev_btn_a = false;
static bool s_prev_btn_b = false;
static bool s_prev_btn_c = false;

static void set_status_msg(const char *msg)
{
    if (msg == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_ui_state_lock);
    strncpy(s_last_status_msg, msg, sizeof(s_last_status_msg) - 1);
    s_last_status_msg[sizeof(s_last_status_msg) - 1] = '\0';
    portEXIT_CRITICAL(&s_ui_state_lock);

    if (s_status_banner != NULL) {
        lv_label_set_text(s_status_banner, msg);
    }
}

static void refresh_header_snapshot(void)
{
    ui_header_state_t next_state = {0};
    if (ui_header_compute_state(&next_state) == ESP_OK) {
        portENTER_CRITICAL(&s_ui_state_lock);
        s_last_header_state = next_state;
        portEXIT_CRITICAL(&s_ui_state_lock);
        ui_header_apply_state(&next_state);
    }
}

static uint32_t lvgl_tick_ms_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static bool lvgl_on_color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io,
                                        esp_lcd_panel_io_event_data_t *edata,
                                        void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    lv_display_t *disp = (lv_display_t *)user_ctx;
    if (disp != NULL) {
        lv_display_flush_ready(disp);
    }
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = bsp_display_get_panel();
    if (panel != NULL && area != NULL && px_map != NULL) {
        int x1 = area->x1;
        int y1 = area->y1;
        int x2 = area->x2 + 1;
        int y2 = area->y2 + 1;

        /* Swap RGB565 byte order for SPI ILI9342C big-endian pixel transfer */
        uint32_t px_count = (uint32_t)(x2 - x1) * (uint32_t)(y2 - y1);
        lv_draw_sw_rgb565_swap(px_map, px_count);
        esp_err_t err = esp_lcd_panel_draw_bitmap(panel, x1, y1, x2, y2, px_map);
        if (err == ESP_OK && s_async_flush_cb_registered) {
            /* lv_display_flush_ready(disp) will be signaled from lvgl_on_color_trans_done_cb */
            return;
        }
    }
    lv_display_flush_ready(disp);
}

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    bsp_touch_state_t tstate = {0};
    if (bsp_touch_read_state(&tstate) == ESP_OK) {
        if (tstate.pressed) {
            data->point.x = (int32_t)tstate.x;
            data->point.y = (int32_t)tstate.y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }

        /* M5Stack Core2 capacitive bottom circles: BtnA = Home, BtnB = Speaker, BtnC = Record */
        if (tstate.btn_a && !s_prev_btn_a) {
            (void)ui_shell_go_home(true);
        }
        if (tstate.btn_b && !s_prev_btn_b) {
            (void)ui_shell_toggle_speaker();
        }
        if (tstate.btn_c && !s_prev_btn_c) {
            (void)ui_shell_toggle_recorder();
        }
        s_prev_btn_a = tstate.btn_a;
        s_prev_btn_b = tstate.btn_b;
        s_prev_btn_c = tstate.btn_c;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void on_launcher_card_clicked(lv_event_t *e)
{
    const char *tool_id = (const char *)lv_event_get_user_data(e);
    if (tool_id != NULL) {
        (void)ui_shell_open_tool(tool_id);
    }
}

static uint32_t category_card_color(tool_category_t cat, bool is_bg)
{
    if (is_bg) {
        return 0xE65100;
    }
    switch (cat) {
    case TOOL_CATEGORY_ANALOG_IN:
        return 0x00695C;
    case TOOL_CATEGORY_DIGITAL_IN:
        return 0x1565C0;
    case TOOL_CATEGORY_GENERATOR:
        return 0x4527A0;
    default:
        return 0x263238;
    }
}

esp_err_t ui_shell_rebuild_launcher(void)
{
    refresh_header_snapshot();

    if (s_launcher_grid == NULL) {
        return ESP_OK;
    }

    lv_obj_clean(s_launcher_grid);

    const analyser_tool_t *tools[TOOL_REGISTRY_MAX_TOOLS] = {0};
    size_t count = tool_registry_get_all(tools, TOOL_REGISTRY_MAX_TOOLS);

    if (count == 0) {
        lv_obj_t *empty_lbl = lv_label_create(s_launcher_grid);
        lv_label_set_text(empty_lbl, "No instruments registered");
        lv_obj_set_style_text_color(empty_lbl, lv_color_hex(0x90A4AE), 0);
        return ESP_OK;
    }

    for (size_t i = 0; i < count; ++i) {
        const analyser_tool_t *tool = tools[i];
        if (tool == NULL) {
            continue;
        }

        bool is_bg = tool_core_is_tool_running_bg(tool->id);
        lv_obj_t *card = lv_button_create(s_launcher_grid);
        lv_obj_set_size(card, 98, 82);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(category_card_color(tool->category, is_bg)), 0);
        lv_obj_set_style_pad_all(card, 6, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card,
                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_START);
        lv_obj_add_event_cb(card, on_launcher_card_clicked, LV_EVENT_CLICKED, (void *)tool->id);

        lv_obj_t *top_row = lv_label_create(card);
        const char *sym = (tool->symbol != NULL && tool->symbol[0] != '\0')
                              ? tool->symbol
                              : LV_SYMBOL_SETTINGS;
        if (is_bg) {
            lv_label_set_text_fmt(top_row, "%s  [BG]", sym);
        } else {
            lv_label_set_text_fmt(top_row, "%s  %s", sym, tool_category_to_str(tool->category));
        }
        lv_obj_set_style_text_font(top_row, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(top_row, lv_color_hex(0xCFD8DC), 0);

        lv_obj_t *title_lbl = lv_label_create(card);
        lv_label_set_text(title_lbl, tool->title);
        lv_label_set_long_mode(title_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(title_lbl, 86);
        lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xFFFFFF), 0);
    }

    return ESP_OK;
}

esp_err_t ui_shell_init(void)
{
    if (s_ui_initialized) {
        refresh_header_snapshot();
        return ESP_OK;
    }

    lv_init();
    lv_tick_set_cb(lvgl_tick_ms_cb);

    s_draw_buf1 = (uint16_t *)heap_caps_calloc(
        UI_DRAW_BUF_PIXELS, sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_draw_buf2 = (uint16_t *)heap_caps_calloc(
        UI_DRAW_BUF_PIXELS, sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_draw_buf1 == NULL) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer");
        return ESP_ERR_NO_MEM;
    }

    s_lv_disp = lv_display_create(UI_SHELL_SCREEN_W, UI_SHELL_SCREEN_H);
    if (s_lv_disp == NULL) {
        heap_caps_free(s_draw_buf1);
        heap_caps_free(s_draw_buf2);
        s_draw_buf1 = NULL;
        s_draw_buf2 = NULL;
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_flush_cb(s_lv_disp, lvgl_flush_cb);
    lv_display_set_buffers(s_lv_disp,
                           s_draw_buf1,
                           s_draw_buf2,
                           UI_DRAW_BUF_PIXELS * sizeof(uint16_t),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    esp_lcd_panel_io_handle_t panel_io = bsp_display_get_panel_io();
    if (panel_io != NULL) {
        const esp_lcd_panel_io_callbacks_t io_cbs = {
            .on_color_trans_done = lvgl_on_color_trans_done_cb,
        };
        if (esp_lcd_panel_io_register_event_callbacks(panel_io, &io_cbs, s_lv_disp) == ESP_OK) {
            s_async_flush_cb_registered = true;
        }
    }

    s_lv_indev = lv_indev_create();
    if (s_lv_indev != NULL) {
        lv_indev_set_type(s_lv_indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(s_lv_indev, lvgl_touch_read_cb);
    }

    s_root_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_root_screen, lv_color_hex(0x0B0F14), 0);
    lv_obj_clear_flag(s_root_screen, LV_OBJ_FLAG_SCROLLABLE);

    /* 1. Persistent 28 px Top Status Header */
    (void)ui_header_create_widgets(s_root_screen);

    /* 2. 320x212 Launcher Container (2x3 Grid + Status Banner) */
    s_launcher_cont = lv_obj_create(s_root_screen);
    lv_obj_set_size(s_launcher_cont, UI_SHELL_SCREEN_W, UI_SHELL_CONTENT_H);
    lv_obj_set_pos(s_launcher_cont, 0, UI_SHELL_HEADER_H);
    lv_obj_set_style_bg_color(s_launcher_cont, lv_color_hex(0x0B0F14), 0);
    lv_obj_set_style_border_width(s_launcher_cont, 0, 0);
    lv_obj_set_style_radius(s_launcher_cont, 0, 0);
    lv_obj_set_style_pad_all(s_launcher_cont, 6, 0);
    lv_obj_set_flex_flow(s_launcher_cont, LV_FLEX_FLOW_COLUMN);

    s_launcher_grid = lv_obj_create(s_launcher_cont);
    lv_obj_set_size(s_launcher_grid, UI_SHELL_SCREEN_W - 12, UI_SHELL_CONTENT_H - 30);
    lv_obj_set_style_bg_opa(s_launcher_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_launcher_grid, 0, 0);
    lv_obj_set_style_pad_all(s_launcher_grid, 0, 0);
    lv_obj_set_style_pad_row(s_launcher_grid, 6, 0);
    lv_obj_set_style_pad_column(s_launcher_grid, 6, 0);
    lv_obj_set_flex_flow(s_launcher_grid, LV_FLEX_FLOW_ROW_WRAP);

    s_status_banner = lv_label_create(s_launcher_cont);
    lv_label_set_text(s_status_banner, "Select an instrument to begin");
    lv_obj_set_style_text_font(s_status_banner, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_status_banner, lv_color_hex(0x90A4AE), 0);

    /* 3. 320x212 Active Tool Container (hidden on Home screen) */
    s_tool_cont = lv_obj_create(s_root_screen);
    lv_obj_set_size(s_tool_cont, UI_SHELL_SCREEN_W, UI_SHELL_CONTENT_H);
    lv_obj_set_pos(s_tool_cont, 0, UI_SHELL_HEADER_H);
    lv_obj_set_style_bg_color(s_tool_cont, lv_color_hex(0x0F151C), 0);
    lv_obj_set_style_border_width(s_tool_cont, 0, 0);
    lv_obj_set_style_radius(s_tool_cont, 0, 0);
    lv_obj_set_style_pad_all(s_tool_cont, 4, 0);
    lv_obj_add_flag(s_tool_cont, LV_OBJ_FLAG_HIDDEN);

    s_ui_initialized = true;
    (void)ui_shell_rebuild_launcher();
    set_status_msg("Select an instrument to begin");

    ESP_LOGI(TAG, "LVGL 9 UI shell initialized (%dx%d, header %d px, content %d px)",
             UI_SHELL_SCREEN_W, UI_SHELL_SCREEN_H, UI_SHELL_HEADER_H, UI_SHELL_CONTENT_H);
    return ESP_OK;
}

bool ui_shell_is_initialized(void)
{
    return s_ui_initialized;
}

void ui_shell_step(void)
{
    refresh_header_snapshot();
    tool_core_ui_refresh_active();
    if (s_ui_initialized) {
        (void)lv_timer_handler();
    }
}

static void ui_shell_task_entry(void *arg)
{
    (void)arg;
    const TickType_t frame_ticks = pdMS_TO_TICKS(1000 / UI_SHELL_REFRESH_HZ);
    while (true) {
        ui_shell_step();
        vTaskDelay(frame_ticks > 0 ? frame_ticks : 1);
    }
}

esp_err_t ui_shell_start_task(void)
{
    if (!s_ui_initialized) {
        esp_err_t err = ui_shell_init();
        if (err != ESP_OK) {
            return err;
        }
    }

    if (s_ui_task_handle != NULL) {
        return ESP_OK;
    }

    BaseType_t created = xTaskCreatePinnedToCore(
        ui_shell_task_entry,
        "ui_shell_c0",
        6144,
        NULL,
        5,
        &s_ui_task_handle,
        0 /* Core 0 (PRO_CPU) */);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ui_shell_open_tool(const char *tool_id)
{
    if (tool_id == NULL || tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    const analyser_tool_t *target = tool_registry_find(tool_id);
    if (target == NULL) {
        char msg[UI_SHELL_STATUS_MSG_MAX];
        (void)snprintf(msg, sizeof(msg), "Failed to open '%s': ESP_ERR_NOT_FOUND", tool_id);
        set_status_msg(msg);
        refresh_header_snapshot();
        return ESP_ERR_NOT_FOUND;
    }

    const analyser_tool_t *active = tool_core_get_active_tool();
    if (active == target) {
        if (s_launcher_cont != NULL && s_tool_cont != NULL) {
            lv_obj_add_flag(s_launcher_cont, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_tool_cont, LV_OBJ_FLAG_HIDDEN);
        }
        refresh_header_snapshot();
        return ESP_OK;
    }

    /* Stop the previous foreground tool BEFORE cleaning s_tool_cont so its on_stop()
     * callback can safely access/detach its own LVGL child widgets before deletion. */
    if (active != NULL) {
        (void)tool_core_stop_active_tool(false);
    }

    if (s_tool_cont != NULL) {
        lv_obj_clean(s_tool_cont);
    }

    esp_err_t err = tool_core_start_tool(tool_id, s_tool_cont);
    if (err != ESP_OK) {
        if (s_tool_cont != NULL) {
            lv_obj_clean(s_tool_cont);
            lv_obj_add_flag(s_tool_cont, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_launcher_cont != NULL) {
            lv_obj_clear_flag(s_launcher_cont, LV_OBJ_FLAG_HIDDEN);
        }
        char msg[UI_SHELL_STATUS_MSG_MAX];
        if (err == ESP_ERR_INVALID_STATE) {
            (void)snprintf(msg, sizeof(msg), "Pin conflict starting '%s' (check BG GEN)", tool_id);
        } else {
            (void)snprintf(msg, sizeof(msg), "Failed to open '%s': %s", tool_id, esp_err_to_name(err));
        }
        set_status_msg(msg);
        refresh_header_snapshot();
        return err;
    }

    if (s_launcher_cont != NULL && s_tool_cont != NULL) {
        lv_obj_add_flag(s_launcher_cont, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_tool_cont, LV_OBJ_FLAG_HIDDEN);
    }

    char ok_msg[UI_SHELL_STATUS_MSG_MAX];
    active = tool_core_get_active_tool();
    (void)snprintf(ok_msg, sizeof(ok_msg), "Opened %s", (active != NULL) ? active->title : tool_id);
    set_status_msg(ok_msg);
    refresh_header_snapshot();
    return ESP_OK;
}

esp_err_t ui_shell_go_home(bool keep_background)
{
    const analyser_tool_t *active = tool_core_get_active_tool();
    if (active != NULL) {
        (void)tool_core_stop_active_tool(keep_background);
    }

    if (s_tool_cont != NULL) {
        lv_obj_clean(s_tool_cont);
        lv_obj_add_flag(s_tool_cont, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_launcher_cont != NULL) {
        lv_obj_clear_flag(s_launcher_cont, LV_OBJ_FLAG_HIDDEN);
    }

    (void)ui_shell_rebuild_launcher();
    set_status_msg("Home launcher active");
    return ESP_OK;
}

esp_err_t ui_shell_toggle_speaker(void)
{
    ui_header_state_t hstate = {0};
    (void)ui_header_compute_state(&hstate);

    if (!hstate.spk_supported) {
        set_status_msg("Speaker monitor not supported by active tool");
        refresh_header_snapshot();
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err;
    if (!bsp_spk_passthrough_is_enabled()) {
        err = bsp_spk_passthrough_enable(true, BSP_SPK_DEFAULT_RATE_HZ);
        if (err == ESP_OK) {
            err = bsp_spk_passthrough_set_mute(false);
        }
    } else {
        bool next_mute = !bsp_spk_passthrough_is_muted();
        err = bsp_spk_passthrough_set_mute(next_mute);
    }

    refresh_header_snapshot();
    return err;
}

esp_err_t ui_shell_toggle_recorder(void)
{
    esp_err_t err = recorder_sink_toggle();
    refresh_header_snapshot();
    return err;
}

esp_err_t ui_shell_cycle_recorder_transport(void)
{
    recorder_mode_t cur = recorder_sink_get_transport();
    recorder_mode_t next = (cur == REC_MODE_USB_STUB) ? REC_MODE_BT_STUB : REC_MODE_USB_STUB;
    esp_err_t err = recorder_sink_set_transport(next);
    refresh_header_snapshot();
    return err;
}

esp_err_t ui_shell_get_header_state(ui_header_state_t *out_state)
{
    if (out_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    refresh_header_snapshot();
    portENTER_CRITICAL(&s_ui_state_lock);
    *out_state = s_last_header_state;
    portEXIT_CRITICAL(&s_ui_state_lock);
    return ESP_OK;
}

esp_err_t ui_shell_get_last_status_msg(char *out_buf, size_t buf_len)
{
    if (out_buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_ui_state_lock);
    strncpy(out_buf, s_last_status_msg, buf_len - 1);
    out_buf[buf_len - 1] = '\0';
    portEXIT_CRITICAL(&s_ui_state_lock);
    return ESP_OK;
}

lv_obj_t *ui_shell_get_tool_container(void)
{
    return s_tool_cont;
}

lv_obj_t *ui_shell_get_launcher_container(void)
{
    return s_launcher_cont;
}

lv_obj_t *ui_shell_get_launcher_grid(void)
{
    return s_launcher_grid;
}
