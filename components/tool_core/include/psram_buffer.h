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
 * @brief Tear-free 3-slot sample buffer for transferring frames from Core 1 to Core 0.
 *
 * Uses three backing buffers (`buf_a`, `buf_b`, `buf_c`) indexed by `active_write_idx`,
 * `ready_idx`, and `active_read_idx` (always a permutation of `{0, 1, 2}`):
 * - Core 1 producer writes to `active_write_idx` and exchanges `active_write_idx <-> ready_idx`
 *   in `psram_double_buf_swap()`.
 * - Core 0 consumer latches `active_read_idx <-> ready_idx` in `psram_double_buf_acquire_read()`,
 *   `psram_double_buf_consume_ready()`, or `psram_double_buf_read_copy()`.
 * - Because the producer never touches `active_read_idx`, Core 0 can read or `memcpy` from
 *   the latched read buffer completely outside the spinlock without frame tearing, even if
 *   Core 1 swaps new frames concurrently.
 */
typedef struct {
    void *buf_a;                       /**< Slot 0 sample buffer (PSRAM when CONFIG_SPIRAM=y) */
    void *buf_b;                       /**< Slot 1 sample buffer (PSRAM when CONFIG_SPIRAM=y) */
    void *buf_c;                       /**< Slot 2 sample buffer (isolates reader from writer swaps) */
    size_t capacity_samples;           /**< Capacity of each buffer slot in samples */
    size_t sample_size;                /**< Size of each sample in bytes */
    volatile uint8_t active_write_idx; /**< Slot index (0..2) currently owned by Core 1 writer */
    volatile uint8_t ready_idx;        /**< Slot index (0..2) holding latest swapped frame for reader */
    volatile uint8_t active_read_idx;  /**< Slot index (0..2) currently owned by Core 0 reader */
    volatile bool frame_ready;         /**< True when writer has published an unconsumed frame in ready_idx */
    volatile bool reader_busy;         /**< True while a reader holds active_read_idx via acquire_read/read_copy */
    portMUX_TYPE spinlock;             /**< Spinlock guarding index exchanges only (never held during memcpy) */
} psram_double_buf_t;

/**
 * @brief Allocate a zero-initialized buffer using compile-time PSRAM selection (`CONFIG_SPIRAM`).
 *
 * When built with `CONFIG_SPIRAM=y` (production M5Stack Core2 firmware), allocates strictly from
 * external SPI PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`). When built with `CONFIG_SPIRAM=n`
 * (e.g., QEMU unit test runner), allocates from `MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT`.
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
 * @brief Allocate and initialize a tear-free 3-slot sample buffer in PSRAM.
 *
 * @param[out] dbuf             Pointer to buffer descriptor to initialize.
 * @param[in]  capacity_samples Number of samples per buffer slot (> 0).
 * @param[in]  sample_size      Size of one sample in bytes (> 0).
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if `dbuf` is `NULL`, dimensions are 0, or size overflows
 *  - `ESP_ERR_NO_MEM` if allocation fails (rolling back any partial allocations)
 */
esp_err_t psram_double_buf_alloc(psram_double_buf_t *dbuf, size_t capacity_samples, size_t sample_size);

/**
 * @brief Free all backing buffers of a `psram_double_buf_t` and reset its state.
 *
 * @param[in,out] dbuf Pointer to buffer descriptor (safe to pass `NULL`).
 */
void psram_double_buf_free(psram_double_buf_t *dbuf);

/**
 * @brief Obtain the current back-buffer pointer for writing on Core 1.
 *
 * @param[in] dbuf Pointer to initialized buffer descriptor.
 * @return Pointer to active write buffer slot (`active_write_idx`), or `NULL`.
 */
void *psram_double_buf_get_write_ptr(psram_double_buf_t *dbuf);

/**
 * @brief Obtain the currently latched front-buffer pointer owned by the Core 0 reader.
 *
 * Note: To latch a newly published frame from `ready_idx` into `active_read_idx`, call
 * `psram_double_buf_acquire_read()`, `psram_double_buf_consume_ready()`, or
 * `psram_double_buf_read_copy()`.
 *
 * @param[in] dbuf Pointer to initialized buffer descriptor.
 * @return Pointer to the reader's current buffer slot (`active_read_idx`), or `NULL`.
 */
const void *psram_double_buf_get_read_ptr(psram_double_buf_t *dbuf);

/**
 * @brief Atomically publish the current write buffer (`active_write_idx <-> ready_idx`) and mark `frame_ready = true`.
 *
 * Called by Core 1 after finishing writing a full sample frame into the write buffer.
 * Does not touch `active_read_idx`, so any concurrent Core 0 reader is unaffected.
 *
 * @param[in,out] dbuf Pointer to initialized buffer descriptor.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `dbuf` is uninitialized.
 */
esp_err_t psram_double_buf_swap(psram_double_buf_t *dbuf);

/**
 * @brief Acquire the latest ready frame for zero-copy reading (`active_read_idx <-> ready_idx`).
 *
 * Marks `reader_busy = true` until `psram_double_buf_release_read()` is called. While held,
 * neither Core 1 producer swaps nor concurrent consumer calls can recycle `active_read_idx`.
 *
 * @param[in,out] dbuf         Pointer to initialized buffer descriptor.
 * @param[out]    out_read_ptr Pointer to receive the acquired read buffer address.
 * @return True if a ready frame was acquired, false if no new frame was ready or reader is already busy.
 */
bool psram_double_buf_acquire_read(psram_double_buf_t *dbuf, const void **out_read_ptr);

/**
 * @brief Release a read buffer previously acquired with `psram_double_buf_acquire_read()`.
 *
 * @param[in,out] dbuf Pointer to initialized buffer descriptor.
 */
void psram_double_buf_release_read(psram_double_buf_t *dbuf);

/**
 * @brief Latch the latest ready frame into the reader slot (`active_read_idx <-> ready_idx`) and clear `frame_ready`.
 *
 * Safe for single-consumer (Core 0) zero-copy reading: the returned buffer slot (`active_read_idx`)
 * is isolated from Core 1 `psram_double_buf_swap()` calls until the consumer itself calls
 * `consume_ready()`, `acquire_read()`, or `read_copy()` for a subsequent frame.
 *
 * @param[in,out] dbuf         Pointer to initialized buffer descriptor.
 * @param[out]    out_read_ptr Optional pointer to receive the latched read buffer address.
 * @return True if a new frame was ready and latched, false otherwise.
 */
bool psram_double_buf_consume_ready(psram_double_buf_t *dbuf, const void **out_read_ptr);

/**
 * @brief Copy samples from the latest ready (or currently latched) read buffer into a caller buffer.
 *
 * Exchanges `active_read_idx <-> ready_idx` and sets `reader_busy = true` under spinlock,
 * performs the `memcpy` **outside** the critical section so interrupts and Core 1 are never
 * stalled, and then clears `reader_busy = false`.
 *
 * @param[in,out] dbuf        Pointer to initialized buffer descriptor.
 * @param[out]    dst_buf     Destination buffer to receive copied samples.
 * @param[in]     max_samples Maximum number of samples to copy into `dst_buf`.
 * @param[out]    out_copied  Optional pointer to receive number of samples copied.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_INVALID_ARG` if arguments are invalid
 *  - `ESP_ERR_INVALID_STATE` if another reader currently holds `reader_busy`
 */
esp_err_t psram_double_buf_read_copy(psram_double_buf_t *dbuf,
                                     void *dst_buf,
                                     size_t max_samples,
                                     size_t *out_copied);

#ifdef __cplusplus
}
#endif
