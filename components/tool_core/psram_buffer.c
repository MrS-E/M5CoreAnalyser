/**
 * @file psram_buffer.c
 * @brief PSRAM buffer allocation and atomic ping-pong double-buffer implementation.
 */

#include "psram_buffer.h"
#include "tool_core_priv.h"

#include <stdint.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "psram_buffer";

static int s_fail_after_count = -1;

static inline bool dbuf_is_valid(const psram_double_buf_t *dbuf)
{
    return (dbuf != NULL) &&
           (dbuf->buf_a != NULL) &&
           (dbuf->buf_b != NULL) &&
           (dbuf->buf_c != NULL) &&
           (dbuf->capacity_samples > 0) &&
           (dbuf->sample_size > 0);
}

static inline void *dbuf_slot_ptr(const psram_double_buf_t *dbuf, uint8_t idx)
{
    switch (idx) {
    case 0U:
        return dbuf->buf_a;
    case 1U:
        return dbuf->buf_b;
    default:
        return dbuf->buf_c;
    }
}

void psram_buf_test_set_fail_after(int fail_after_count)
{
    s_fail_after_count = fail_after_count;
}

bool psram_buf_is_spiram_available(void)
{
    return heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;
}

void *psram_buf_calloc(size_t count, size_t sample_size)
{
    if (count == 0 || sample_size == 0 || count > (SIZE_MAX / sample_size)) {
        return NULL;
    }

    if (s_fail_after_count >= 0) {
        if (s_fail_after_count == 0) {
            return NULL;
        }
        s_fail_after_count--;
    }

#if CONFIG_SPIRAM
    uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
#else
    uint32_t caps = MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT;
#endif

    return heap_caps_calloc(count, sample_size, caps);
}

void psram_buf_free(void *ptr)
{
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}

esp_err_t psram_double_buf_alloc(psram_double_buf_t *dbuf, size_t capacity_samples, size_t sample_size)
{
    if (dbuf == NULL || capacity_samples == 0 || sample_size == 0 ||
        capacity_samples > (SIZE_MAX / sample_size)) {
        return ESP_ERR_INVALID_ARG;
    }

    *dbuf = (psram_double_buf_t){
        .buf_a = NULL,
        .buf_b = NULL,
        .buf_c = NULL,
        .capacity_samples = 0,
        .sample_size = 0,
        .active_write_idx = 0U,
        .ready_idx = 1U,
        .active_read_idx = 2U,
        .frame_ready = false,
        .reader_busy = false,
        .spinlock = portMUX_INITIALIZER_UNLOCKED,
    };

    void *buf_a = psram_buf_calloc(capacity_samples, sample_size);
    if (buf_a == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer slot A (%u x %u bytes)",
                 (unsigned)capacity_samples, (unsigned)sample_size);
        return ESP_ERR_NO_MEM;
    }

    void *buf_b = psram_buf_calloc(capacity_samples, sample_size);
    if (buf_b == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer slot B (%u x %u bytes)",
                 (unsigned)capacity_samples, (unsigned)sample_size);
        psram_buf_free(buf_a);
        return ESP_ERR_NO_MEM;
    }

    void *buf_c = psram_buf_calloc(capacity_samples, sample_size);
    if (buf_c == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer slot C (%u x %u bytes)",
                 (unsigned)capacity_samples, (unsigned)sample_size);
        psram_buf_free(buf_b);
        psram_buf_free(buf_a);
        return ESP_ERR_NO_MEM;
    }

    dbuf->buf_a = buf_a;
    dbuf->buf_b = buf_b;
    dbuf->buf_c = buf_c;
    dbuf->capacity_samples = capacity_samples;
    dbuf->sample_size = sample_size;
    dbuf->active_write_idx = 0U;
    dbuf->ready_idx = 1U;
    dbuf->active_read_idx = 2U;
    dbuf->frame_ready = false;
    dbuf->reader_busy = false;
    return ESP_OK;
}

void psram_double_buf_free(psram_double_buf_t *dbuf)
{
    if (dbuf == NULL) {
        return;
    }

    portENTER_CRITICAL(&dbuf->spinlock);
    void *buf_a = dbuf->buf_a;
    void *buf_b = dbuf->buf_b;
    void *buf_c = dbuf->buf_c;
    dbuf->buf_a = NULL;
    dbuf->buf_b = NULL;
    dbuf->buf_c = NULL;
    dbuf->capacity_samples = 0;
    dbuf->sample_size = 0;
    dbuf->active_write_idx = 0U;
    dbuf->ready_idx = 1U;
    dbuf->active_read_idx = 2U;
    dbuf->frame_ready = false;
    dbuf->reader_busy = false;
    portEXIT_CRITICAL(&dbuf->spinlock);

    psram_buf_free(buf_a);
    psram_buf_free(buf_b);
    psram_buf_free(buf_c);
}

void *psram_double_buf_get_write_ptr(psram_double_buf_t *dbuf)
{
    if (!dbuf_is_valid(dbuf)) {
        return NULL;
    }

    void *ptr;
    portENTER_CRITICAL(&dbuf->spinlock);
    ptr = dbuf_slot_ptr(dbuf, dbuf->active_write_idx);
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ptr;
}

const void *psram_double_buf_get_read_ptr(psram_double_buf_t *dbuf)
{
    if (!dbuf_is_valid(dbuf)) {
        return NULL;
    }

    const void *ptr;
    portENTER_CRITICAL(&dbuf->spinlock);
    ptr = dbuf_slot_ptr(dbuf, dbuf->active_read_idx);
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ptr;
}

esp_err_t psram_double_buf_swap(psram_double_buf_t *dbuf)
{
    if (!dbuf_is_valid(dbuf)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&dbuf->spinlock);
    uint8_t prev_write = dbuf->active_write_idx;
    dbuf->active_write_idx = dbuf->ready_idx;
    dbuf->ready_idx = prev_write;
    dbuf->frame_ready = true;
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ESP_OK;
}

bool psram_double_buf_acquire_read(psram_double_buf_t *dbuf, const void **out_read_ptr)
{
    if (out_read_ptr != NULL) {
        *out_read_ptr = NULL;
    }
    if (!dbuf_is_valid(dbuf)) {
        return false;
    }

    bool acquired = false;
    const void *read_ptr = NULL;

    portENTER_CRITICAL(&dbuf->spinlock);
    if (!dbuf->reader_busy && dbuf->frame_ready) {
        uint8_t prev_read = dbuf->active_read_idx;
        dbuf->active_read_idx = dbuf->ready_idx;
        dbuf->ready_idx = prev_read;
        dbuf->frame_ready = false;
        dbuf->reader_busy = true;
        read_ptr = dbuf_slot_ptr(dbuf, dbuf->active_read_idx);
        acquired = true;
    }
    portEXIT_CRITICAL(&dbuf->spinlock);

    if (out_read_ptr != NULL) {
        *out_read_ptr = read_ptr;
    }
    return acquired;
}

void psram_double_buf_release_read(psram_double_buf_t *dbuf)
{
    if (dbuf == NULL) {
        return;
    }

    portENTER_CRITICAL(&dbuf->spinlock);
    dbuf->reader_busy = false;
    portEXIT_CRITICAL(&dbuf->spinlock);
}

bool psram_double_buf_consume_ready(psram_double_buf_t *dbuf, const void **out_read_ptr)
{
    if (out_read_ptr != NULL) {
        *out_read_ptr = NULL;
    }
    if (!dbuf_is_valid(dbuf)) {
        return false;
    }

    bool ready = false;
    const void *read_ptr = NULL;

    portENTER_CRITICAL(&dbuf->spinlock);
    if (!dbuf->reader_busy && dbuf->frame_ready) {
        uint8_t prev_read = dbuf->active_read_idx;
        dbuf->active_read_idx = dbuf->ready_idx;
        dbuf->ready_idx = prev_read;
        dbuf->frame_ready = false;
        read_ptr = dbuf_slot_ptr(dbuf, dbuf->active_read_idx);
        ready = true;
    }
    portEXIT_CRITICAL(&dbuf->spinlock);

    if (out_read_ptr != NULL) {
        *out_read_ptr = read_ptr;
    }
    return ready;
}

esp_err_t psram_double_buf_read_copy(psram_double_buf_t *dbuf,
                                     void *dst_buf,
                                     size_t max_samples,
                                     size_t *out_copied)
{
    if (out_copied != NULL) {
        *out_copied = 0;
    }
    if (!dbuf_is_valid(dbuf) || dst_buf == NULL || max_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const void *src = NULL;
    size_t to_copy = 0;
    size_t sample_size = 0;

    portENTER_CRITICAL(&dbuf->spinlock);
    if (dbuf->reader_busy) {
        portEXIT_CRITICAL(&dbuf->spinlock);
        return ESP_ERR_INVALID_STATE;
    }
    if (dbuf->frame_ready) {
        uint8_t prev_read = dbuf->active_read_idx;
        dbuf->active_read_idx = dbuf->ready_idx;
        dbuf->ready_idx = prev_read;
        dbuf->frame_ready = false;
    }
    dbuf->reader_busy = true;
    src = dbuf_slot_ptr(dbuf, dbuf->active_read_idx);
    to_copy = (max_samples < dbuf->capacity_samples) ? max_samples : dbuf->capacity_samples;
    sample_size = dbuf->sample_size;
    portEXIT_CRITICAL(&dbuf->spinlock);

    /* Copy samples outside the spinlock so interrupts and Core 1 producer swaps never stall */
    memcpy(dst_buf, src, to_copy * sample_size);

    portENTER_CRITICAL(&dbuf->spinlock);
    dbuf->reader_busy = false;
    portEXIT_CRITICAL(&dbuf->spinlock);

    if (out_copied != NULL) {
        *out_copied = to_copy;
    }
    return ESP_OK;
}
