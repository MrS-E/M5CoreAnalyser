/**
 * @file ui_header.c
 * @brief Persistent 28 px top status bar view-model formatting and LVGL 9 widgets.
 */

#include "ui_shell.h"
#include "ui_shell_priv.h"

#include <stdio.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "recorder_sink.h"
#include "tool_core.h"

static lv_obj_t *s_header_bar = NULL;
static lv_obj_t *s_btn_home = NULL;
static lv_obj_t *s_lbl_title = NULL;
static lv_obj_t *s_btn_bg_pill = NULL;
static lv_obj_t *s_lbl_bg_pill = NULL;
static lv_obj_t *s_btn_spk = NULL;
static lv_obj_t *s_lbl_spk = NULL;
static lv_obj_t *s_btn_rec = NULL;
static lv_obj_t *s_lbl_rec = NULL;
static lv_obj_t *s_btn_mode = NULL;
static lv_obj_t *s_lbl_mode = NULL;
static lv_obj_t *s_lbl_batt = NULL;

void ui_header_format_battery(uint16_t voltage_mv,
                              uint8_t percent,
                              bool charging,
                              char *out_buf,
                              size_t buf_len)
{
    if (out_buf == NULL || buf_len == 0) {
        return;
    }

    uint8_t clamped_pct = (percent > 100U) ? 100U : percent;
    unsigned volts_int = (unsigned)(voltage_mv / 1000U);
    unsigned volts_centi = (unsigned)((voltage_mv % 1000U) / 10U);

    (void)snprintf(out_buf,
                   buf_len,
                   "%u.%02uV %u%%%s",
                   volts_int,
                   volts_centi,
                   (unsigned)clamped_pct,
                   charging ? "+" : "");
}

void ui_header_format_bg_pill(size_t bg_count,
                              const char *first_bg_id,
                              char *out_buf,
                              size_t buf_len)
{
    if (out_buf == NULL || buf_len == 0) {
        return;
    }

    if (bg_count == 0) {
        out_buf[0] = '\0';
        return;
    }

    if (bg_count == 1 && first_bg_id != NULL && first_bg_id[0] != '\0') {
        (void)snprintf(out_buf, buf_len, "BG:%.4s", first_bg_id);
    } else {
        (void)snprintf(out_buf, buf_len, "BG:%u", (unsigned)bg_count);
    }
}

void ui_header_format_spk_badge(bool supported,
                                bool enabled,
                                bool muted,
                                char *out_buf,
                                size_t buf_len)
{
    if (out_buf == NULL || buf_len == 0) {
        return;
    }

    if (!supported) {
        (void)snprintf(out_buf, buf_len, "SPK-");
    } else if (!enabled) {
        (void)snprintf(out_buf, buf_len, "SPK0");
    } else if (muted) {
        (void)snprintf(out_buf, buf_len, "MUTE");
    } else {
        (void)snprintf(out_buf, buf_len, "SPK1");
    }
}

void ui_header_format_rec_badge(recorder_mode_t mode,
                                recorder_mode_t transport,
                                uint32_t frames,
                                char *out_rec_buf,
                                size_t rec_len,
                                char *out_mode_buf,
                                size_t mode_len)
{
    recorder_mode_t effective_transport =
        (transport == REC_MODE_BT_STUB) ? REC_MODE_BT_STUB : REC_MODE_USB_STUB;

    if (mode == REC_MODE_USB_STUB || mode == REC_MODE_BT_STUB) {
        effective_transport = mode;
    }

    const char *mode_str = recorder_sink_mode_to_str(effective_transport);
    if (out_mode_buf != NULL && mode_len > 0) {
        (void)snprintf(out_mode_buf, mode_len, "%s", mode_str);
    }

    if (out_rec_buf != NULL && rec_len > 0) {
        if (mode == REC_MODE_OFF && frames == 0) {
            (void)snprintf(out_rec_buf, rec_len, "REC");
        } else {
            (void)snprintf(out_rec_buf, rec_len, "#%u", (unsigned)frames);
        }
    }
}

esp_err_t ui_header_compute_state(ui_header_state_t *out_state)
{
    if (out_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_state, 0, sizeof(*out_state));

    const analyser_tool_t *active = tool_core_get_active_tool();
    if (active != NULL) {
        out_state->home_enabled = true;
        (void)snprintf(out_state->title, sizeof(out_state->title), "%s", active->title);
    } else {
        out_state->home_enabled = false;
        (void)snprintf(out_state->title, sizeof(out_state->title), "M5CoreAnalyser");
    }

    const analyser_tool_t *bg_tools[TOOL_REGISTRY_MAX_TOOLS] = {0};
    size_t bg_count = tool_core_get_background_tools(bg_tools, TOOL_REGISTRY_MAX_TOOLS);
    out_state->bg_tool_count = bg_count;
    out_state->bg_pill_visible = (bg_count > 0);
    const char *first_bg_id = (bg_count > 0 && bg_tools[0] != NULL) ? bg_tools[0]->id : NULL;
    ui_header_format_bg_pill(
        bg_count, first_bg_id, out_state->bg_pill_text, sizeof(out_state->bg_pill_text));

    bool spk_supported = (active != NULL && active->supports_spk_passthrough);
    for (size_t i = 0; i < bg_count && !spk_supported; ++i) {
        if (bg_tools[i] != NULL && bg_tools[i]->supports_spk_passthrough) {
            spk_supported = true;
        }
    }

    bool spk_en = bsp_spk_passthrough_is_enabled();
    bool spk_muted = bsp_spk_passthrough_is_muted();
    out_state->spk_supported = spk_supported;
    out_state->spk_enabled = spk_en && !spk_muted;
    out_state->spk_muted = spk_muted;
    ui_header_format_spk_badge(
        spk_supported, spk_en, spk_muted, out_state->spk_badge_text, sizeof(out_state->spk_badge_text));

    recorder_sink_stats_t rec_stats = {0};
    (void)recorder_sink_get_stats(&rec_stats);
    out_state->rec_mode = rec_stats.mode;
    out_state->rec_transport = rec_stats.selected_transport;
    out_state->rec_frames = rec_stats.frames_pushed;
    ui_header_format_rec_badge(rec_stats.mode,
                               rec_stats.selected_transport,
                               rec_stats.frames_pushed,
                               out_state->rec_badge_text,
                               sizeof(out_state->rec_badge_text),
                               out_state->transport_badge_text,
                               sizeof(out_state->transport_badge_text));

    uint16_t batt_mv = 0;
    uint8_t batt_pct = 0;
    bool batt_chg = false;
    (void)bsp_axp192_get_battery(&batt_mv, &batt_pct, &batt_chg);
    out_state->battery_mv = batt_mv;
    out_state->battery_pct = batt_pct;
    out_state->battery_charging = batt_chg;
    ui_header_format_battery(
        batt_mv, batt_pct, batt_chg, out_state->battery_text, sizeof(out_state->battery_text));

    return ESP_OK;
}

static void on_home_clicked(lv_event_t *e)
{
    (void)e;
    (void)ui_shell_go_home(true);
}

static void on_bg_pill_clicked(lv_event_t *e)
{
    (void)e;
    (void)tool_core_stop_all_background_tools();
    (void)ui_shell_rebuild_launcher();
}

static void on_spk_clicked(lv_event_t *e)
{
    (void)e;
    (void)ui_shell_toggle_speaker();
}

static void on_rec_clicked(lv_event_t *e)
{
    (void)e;
    (void)ui_shell_toggle_recorder();
}

static void on_mode_clicked(lv_event_t *e)
{
    (void)e;
    (void)ui_shell_cycle_recorder_transport();
}

static lv_obj_t *create_header_chip_btn(lv_obj_t *parent,
                                        const char *initial_txt,
                                        uint32_t bg_hex,
                                        lv_event_cb_t cb,
                                        lv_obj_t **out_lbl)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_height(btn, 22);
    lv_obj_set_style_pad_hor(btn, 2, 0);
    lv_obj_set_style_pad_ver(btn, 1, 0);
    lv_obj_set_style_radius(btn, 4, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(bg_hex), 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    if (cb != NULL) {
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, initial_txt);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(lbl);

    if (out_lbl != NULL) {
        *out_lbl = lbl;
    }
    return btn;
}

lv_obj_t *ui_header_create_widgets(lv_obj_t *parent_screen)
{
    s_header_bar = lv_obj_create(parent_screen);
    lv_obj_set_size(s_header_bar, UI_SHELL_SCREEN_W, UI_SHELL_HEADER_H);
    lv_obj_set_pos(s_header_bar, 0, 0);
    lv_obj_set_style_bg_color(s_header_bar, lv_color_hex(0x141B24), 0);
    lv_obj_set_style_border_width(s_header_bar, 0, 0);
    lv_obj_set_style_radius(s_header_bar, 0, 0);
    lv_obj_set_style_pad_hor(s_header_bar, 2, 0);
    lv_obj_set_style_pad_ver(s_header_bar, 2, 0);
    lv_obj_set_style_pad_column(s_header_bar, 2, 0);
    lv_obj_clear_flag(s_header_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(s_header_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_header_bar,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    /* Home button */
    s_btn_home = create_header_chip_btn(s_header_bar, LV_SYMBOL_HOME, 0x263238, on_home_clicked, NULL);

    /* Screen / Tool title (flex_grow fills remaining space and shrinks when BG pill appears) */
    s_lbl_title = lv_label_create(s_header_bar);
    lv_label_set_text(s_lbl_title, "M5CoreAnalyser");
    lv_label_set_long_mode(s_lbl_title, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(s_lbl_title, 0);
    lv_obj_set_style_min_width(s_lbl_title, 20, 0);
    lv_obj_set_flex_grow(s_lbl_title, 1);
    lv_obj_set_style_text_font(s_lbl_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_title, lv_color_hex(0x80CBC4), 0);

    /* Background generator pill */
    s_btn_bg_pill = create_header_chip_btn(
        s_header_bar, "BG:wave", 0xE65100, on_bg_pill_clicked, &s_lbl_bg_pill);
    lv_obj_add_flag(s_btn_bg_pill, LV_OBJ_FLAG_HIDDEN);

    /* Speaker monitor toggle */
    s_btn_spk = create_header_chip_btn(
        s_header_bar, "SPK -", 0x37474F, on_spk_clicked, &s_lbl_spk);

    /* Recorder toggle button */
    s_btn_rec = create_header_chip_btn(
        s_header_bar, "REC OFF", 0x37474F, on_rec_clicked, &s_lbl_rec);

    /* Transport mode selector (USB / BT) */
    s_btn_mode = create_header_chip_btn(
        s_header_bar, "USB", 0x1565C0, on_mode_clicked, &s_lbl_mode);

    /* Battery telemetry label */
    s_lbl_batt = lv_label_create(s_header_bar);
    lv_label_set_text(s_lbl_batt, "3.85V 68%");
    lv_obj_set_style_text_font(s_lbl_batt, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_lbl_batt, lv_color_hex(0xA5D6A7), 0);

    return s_header_bar;
}

lv_obj_t *ui_header_get_bar(void)
{
    return s_header_bar;
}

void ui_header_apply_state(const ui_header_state_t *state)
{
    if (state == NULL || s_header_bar == NULL) {
        return;
    }

    if (s_btn_home != NULL) {
        if (state->home_enabled) {
            lv_obj_clear_state(s_btn_home, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_btn_home, lv_color_hex(0x00897B), 0);
        } else {
            lv_obj_add_state(s_btn_home, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_btn_home, lv_color_hex(0x263238), 0);
        }
    }

    if (s_lbl_title != NULL) {
        lv_label_set_text(s_lbl_title, state->title);
    }

    if (s_btn_bg_pill != NULL && s_lbl_bg_pill != NULL) {
        if (state->bg_pill_visible) {
            lv_label_set_text(s_lbl_bg_pill, state->bg_pill_text);
            lv_obj_clear_flag(s_btn_bg_pill, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_btn_bg_pill, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (s_btn_spk != NULL && s_lbl_spk != NULL) {
        lv_label_set_text(s_lbl_spk, state->spk_badge_text);
        uint32_t spk_color = state->spk_enabled ? 0x2E7D32 : 0x37474F;
        lv_obj_set_style_bg_color(s_btn_spk, lv_color_hex(spk_color), 0);
    }

    if (s_btn_rec != NULL && s_lbl_rec != NULL) {
        lv_label_set_text(s_lbl_rec, state->rec_badge_text);
        uint32_t rec_color = (state->rec_mode != REC_MODE_OFF) ? 0xC62828 : 0x37474F;
        lv_obj_set_style_bg_color(s_btn_rec, lv_color_hex(rec_color), 0);
    }

    if (s_btn_mode != NULL && s_lbl_mode != NULL) {
        lv_label_set_text(s_lbl_mode, state->transport_badge_text);
        uint32_t mode_color = (state->rec_transport == REC_MODE_BT_STUB) ? 0x6A1B9A : 0x1565C0;
        lv_obj_set_style_bg_color(s_btn_mode, lv_color_hex(mode_color), 0);
    }

    if (s_lbl_batt != NULL) {
        lv_label_set_text(s_lbl_batt, state->battery_text);
    }
}
