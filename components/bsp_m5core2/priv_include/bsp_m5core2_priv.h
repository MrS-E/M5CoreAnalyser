/**
 * @file bsp_m5core2_priv.h
 * @brief Private internal and unit-test seam declarations for bsp_m5core2.
 *
 * Not part of the public production BSP interface. Exposed only to bsp_m5core2
 * implementation files and the bsp_m5core2 Unity test component via PRIV_INCLUDE_DIRS.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize internal mutexes for AXP192 and speaker passthrough.
 *
 * Safe to call multiple times from initialization paths.
 */
void bsp_axp192_lock_init(void);

/**
 * @brief Initialize the speaker passthrough synchronization mutex.
 *
 * Safe to call multiple times from initialization paths.
 */
void bsp_spk_passthrough_lock_init(void);

/**
 * @brief Reset all dynamic tool GPIO claims (internal initialization and unit-test helper).
 *
 * Must not be called by normal tools at runtime as it clears background generator claims.
 */
void bsp_pin_manager_reset(void);

/**
 * @brief Backend I2S operations table for speaker passthrough (allows deterministic unit testing).
 */
typedef struct {
    esp_err_t (*ensure_init)(uint32_t sample_rate_hz); /**< Initialize or reconfigure I2S1 clock and enable channel */
    esp_err_t (*write_pcm)(const int16_t *pcm,
                           size_t byte_len,
                           size_t *out_bytes_written,
                           uint32_t timeout_ms);       /**< Write 16-bit mono PCM samples to I2S1 DMA */
    esp_err_t (*disable)(void);                        /**< Disable I2S1 channel when passthrough stops */
} bsp_spk_i2s_ops_t;

/**
 * @brief Override or restore the `I2S1` backend used by speaker passthrough (unit-test seam).
 *
 * If passthrough is currently active when switching backends, disables the previous backend
 * and gates off the speaker amplifier before installing the new operations table.
 * Passing `NULL` restores the default `I2S_NUM_1` hardware implementation.
 *
 * @param[in] ops Pointer to custom `bsp_spk_i2s_ops_t` table, or `NULL` for default hardware `I2S1`.
 */
void bsp_spk_passthrough_set_i2s_ops(const bsp_spk_i2s_ops_t *ops);

/**
 * @brief Backend register I/O operations table for AXP192 (allows deterministic unit testing of I2C errors).
 */
typedef struct {
    esp_err_t (*read_reg)(uint8_t reg, uint8_t *out_val); /**< Read a single 8-bit AXP192 register */
    esp_err_t (*write_reg)(uint8_t reg, uint8_t val);     /**< Write a single 8-bit AXP192 register */
} bsp_axp192_reg_ops_t;

/**
 * @brief Override or restore the AXP192 register read/write backend (unit-test seam).
 *
 * Passing `NULL` restores the default I2C0 / shadow implementation.
 *
 * @param[in] ops Pointer to custom `bsp_axp192_reg_ops_t` table, or `NULL` for default.
 */
void bsp_axp192_set_reg_ops(const bsp_axp192_reg_ops_t *ops);

#ifdef __cplusplus
}
#endif
