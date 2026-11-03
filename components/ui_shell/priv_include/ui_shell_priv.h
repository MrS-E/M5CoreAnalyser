/**
 * @file ui_shell_priv.h
 * @brief Private internal declarations between ui_shell.c, ui_header.c, and unit tests.
 */

#pragma once

#include "lvgl.h"
#include "ui_shell.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the persistent 28 px LVGL top header bar on `parent_screen`.
 *
 * @param[in] parent_screen Root 320x240 LVGL screen object.
 * @return Created header container object.
 */
lv_obj_t *ui_header_create_widgets(lv_obj_t *parent_screen);

/**
 * @brief Apply a `ui_header_state_t` snapshot to the LVGL top header bar widgets.
 *
 * @param[in] state Pointer to computed header state.
 */
void ui_header_apply_state(const ui_header_state_t *state);

#ifdef __cplusplus
}
#endif
