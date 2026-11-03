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

/**
 * @brief Return the persistent 28 px LVGL header bar object (unit-test & layout inspection seam).
 *
 * @return Header bar `lv_obj_t *` or `NULL`.
 */
lv_obj_t *ui_header_get_bar(void);

/**
 * @brief Return the 320x212 launcher container object (unit-test visibility inspection seam).
 *
 * @return Launcher container `lv_obj_t *` or `NULL`.
 */
lv_obj_t *ui_shell_get_launcher_container(void);

/**
 * @brief Return the 2x3 launcher grid object holding tool cards (unit-test card count seam).
 *
 * @return Launcher grid `lv_obj_t *` or `NULL`.
 */
lv_obj_t *ui_shell_get_launcher_grid(void);

#ifdef __cplusplus
}
#endif
