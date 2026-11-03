/**
 * @file psram_buffer.h
 * @brief PSRAM buffer allocation and lock-free/spinlock-guarded double-buffer primitives.
 *
 * Allocates capture and lookup buffers from the M5Stack Core2's 8 MB external SPI PSRAM
 * (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`) when available, falling back to default 8-bit
 * heap in headless/emulated environments without external SPIRAM.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ping-pong double buffer for transferring sample frames from Core 1 to Core 0.
 */
typedef struct {
    void *buf_a;                      /**< First sample buffer (allocated in PSRAM when available) */
    void *buf_b;                      /**< Second sample buffer (allocated in PSRAM when available) */
    size_t capacity_samples;          /**< Capacity of each buffer in samples */
    size_t sample_size;               /**< Size of each sample in bytes */
    volatile uint8_t active_write_idx;/**< Index (0 = buf_a, 1 = buf_b) currently owned by writer (Core 1) */
    volatile bool frame_ready;        /**< True when writer has swapped a new completed frame for reader (Core 0) */
    portMUX_TYPE spinlock;            /**< Spinlock guarding index swaps and copy-out reads */
} psram_double_buf_t;

/**
 * @brief Allocate a zero-initialized buffer preferring external PSRAM (`MALLOC_CAP_SPIRAM`).
 *
 * When external PSRAM is initialized (`heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0`),
 * allocates strictly from `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`. When running in an
 * environment without PSRAM hardware (e.g., QEMU unit test runner), falls back to
 * `MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT`.
 *
 * @param[in] count       Number of elements (> 0).
 * @param[in] sample_size Size of each element in bytes (> 0).
 * @return Pointer to zeroed buffer, or `NULL` on overflow or out-of-memory.
 */
void *psram_buf_calloc(size_t count, size_t sample_size);

/**
 * @brief Free a buffer previously allocated with `psram_buf_calloc()`.
 *
 * @param[in] ptr Buffer pointer (safe to pass `NULL`).
 */
void psram_buf_free(void *ptr);

/**
 * @brief Check whether external SPI PSRAM heap is available on the running target.
 *
 * @return True if `MALLOC_CAP_SPIRAM` heap has non-zero total capacity, false otherwise.
 */
bool psram_buf_is_spiram_available(void);

/**
 * @brief Allocate and initialize a ping-pong double buffer in PSRAM.
 *
 * @param[out] dbuf             Pointer to double-buffer descriptor to initialize.
 * @param[in]  capacity_samples Number of samples per buffer (> 0).
 * @param[in]  sample_size      Size of one sample in bytes (> 0).
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `dbuf` is `NULL`, dimensions are 0, or size overflows
 *  - `ESP_ERR_NO_MEM` if allocation fails
 */
esp_err_t psram_double_buf_alloc(psram_double_buf_t *dbuf, size_t capacity_samples, size_t sample_size);

/**
 * @brief Free both buffers of a `psram_double_buf_t` and reset its state.
 *
 * @param[in,out] dbuf Pointer to double-buffer descriptor (safe to pass `NULL`).
 */
void psram_double_buf_free(psram_double_buf_t *dbuf);

/**
 * @brief Obtain the current back-buffer pointer for writing on Core 1.
 *
 * @param[in] dbuf Pointer to initialized double buffer.
 * @return Pointer to active write buffer (`buf_a` when `active_write_idx == 0`, else `buf_b`), or `NULL`.
 */
void *psram_double_buf_get_write_ptr(psram_double_buf_t *dbuf);

/**
 * @brief Obtain the current front-buffer pointer for reading on Core 0.
 *
 * @param[in] dbuf Pointer to initialized double buffer.
 * @return Pointer to active read buffer (`buf_b` when `active_write_idx == 0`, else `buf_a`), or `NULL`.
 */
const void *psram_double_buf_get_read_ptr(psram_double_buf_t *dbuf);

/**
 * @brief Atomically swap the write and read buffers and mark `frame_ready = true`.
 *
 * Called by Core 1 after finishing writing a full sample frame into the write buffer.
 *
 * @param[in,out] dbuf Pointer to initialized double buffer.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `dbuf` is uninitialized.
 */
esp_err_t psram_double_buf_swap(psram_double_buf_t *dbuf);

/**
 * @brief Atomically check and clear `frame_ready`, returning the current read buffer pointer.
 *
 * @param[in,out] dbuf         Pointer to initialized double buffer.
 * @param[out]    out_read_ptr Optional pointer to receive the front read buffer address.
 * @return True if a new frame was ready (and `frame_ready` was cleared), false otherwise.
 */
bool psram_double_buf_consume_ready(psram_double_buf_t *dbuf, const void **out_read_ptr);

/**
 * @brief Atomically copy samples from the current front read buffer into a caller buffer under spinlock.
 *
 * Prevents data tearing even if the Core 1 producer swaps buffers concurrently. Also clears
 * `frame_ready`.
 *
 * @param[in,out] dbuf       Pointer to initialized double buffer.
 * @param[out]    dst_buf    Destination buffer to receive copied samples.
 * @param[in]     max_samples Maximum number of samples to copy into `dst_buf`.
 * @param[out]    out_copied Optional pointer to receive number of samples copied.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if arguments are invalid.
 */
esp_err_t psram_double_buf_read_copy(psram_double_buf_t *dbuf,
                                     void *dst_buf,
                                     size_t max_samples,
                                     size_t *out_copied);

#ifdef __cplusplus
}
#endif
