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

static const char *TAG = "psram_buffer";

static int s_fail_after_count = -1;

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

    uint32_t caps = psram_buf_is_spiram_available()
                        ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
                        : (MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT);

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
        .capacity_samples = 0,
        .sample_size = 0,
        .active_write_idx = 0,
        .frame_ready = false,
        .spinlock = portMUX_INITIALIZER_UNLOCKED,
    };

    void *buf_a = psram_buf_calloc(capacity_samples, sample_size);
    if (buf_a == NULL) {
        ESP_LOGE(TAG, "Failed to allocate double-buffer A (%u x %u bytes)",
                 (unsigned)capacity_samples, (unsigned)sample_size);
        return ESP_ERR_NO_MEM;
    }

    void *buf_b = psram_buf_calloc(capacity_samples, sample_size);
    if (buf_b == NULL) {
        ESP_LOGE(TAG, "Failed to allocate double-buffer B (%u x %u bytes)",
                 (unsigned)capacity_samples, (unsigned)sample_size);
        psram_buf_free(buf_a);
        return ESP_ERR_NO_MEM;
    }

    dbuf->buf_a = buf_a;
    dbuf->buf_b = buf_b;
    dbuf->capacity_samples = capacity_samples;
    dbuf->sample_size = sample_size;
    dbuf->active_write_idx = 0;
    dbuf->frame_ready = false;
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
    dbuf->buf_a = NULL;
    dbuf->buf_b = NULL;
    dbuf->capacity_samples = 0;
    dbuf->sample_size = 0;
    dbuf->active_write_idx = 0;
    dbuf->frame_ready = false;
    portEXIT_CRITICAL(&dbuf->spinlock);

    psram_buf_free(buf_a);
    psram_buf_free(buf_b);
}

void *psram_double_buf_get_write_ptr(psram_double_buf_t *dbuf)
{
    if (dbuf == NULL || dbuf->buf_a == NULL || dbuf->buf_b == NULL) {
        return NULL;
    }

    void *ptr;
    portENTER_CRITICAL(&dbuf->spinlock);
    ptr = (dbuf->active_write_idx == 0U) ? dbuf->buf_a : dbuf->buf_b;
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ptr;
}

const void *psram_double_buf_get_read_ptr(psram_double_buf_t *dbuf)
{
    if (dbuf == NULL || dbuf->buf_a == NULL || dbuf->buf_b == NULL) {
        return NULL;
    }

    const void *ptr;
    portENTER_CRITICAL(&dbuf->spinlock);
    ptr = (dbuf->active_write_idx == 0U) ? dbuf->buf_b : dbuf->buf_a;
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ptr;
}

esp_err_t psram_double_buf_swap(psram_double_buf_t *dbuf)
{
    if (dbuf == NULL || dbuf->buf_a == NULL || dbuf->buf_b == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&dbuf->spinlock);
    dbuf->active_write_idx ^= 1U;
    dbuf->frame_ready = true;
    portEXIT_CRITICAL(&dbuf->spinlock);
    return ESP_OK;
}

bool psram_double_buf_consume_ready(psram_double_buf_t *dbuf, const void **out_read_ptr)
{
    if (dbuf == NULL || dbuf->buf_a == NULL || dbuf->buf_b == NULL) {
        if (out_read_ptr != NULL) {
            *out_read_ptr = NULL;
        }
        return false;
    }

    bool ready = false;
    const void *read_ptr = NULL;

    portENTER_CRITICAL(&dbuf->spinlock);
    if (dbuf->frame_ready) {
        ready = true;
        dbuf->frame_ready = false;
        read_ptr = (dbuf->active_write_idx == 0U) ? dbuf->buf_b : dbuf->buf_a;
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
    if (dbuf == NULL || dst_buf == NULL || max_samples == 0 ||
        dbuf->buf_a == NULL || dbuf->buf_b == NULL || dbuf->sample_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&dbuf->spinlock);
    size_t to_copy = (max_samples < dbuf->capacity_samples) ? max_samples : dbuf->capacity_samples;
    const void *src = (dbuf->active_write_idx == 0U) ? dbuf->buf_b : dbuf->buf_a;
    memcpy(dst_buf, src, to_copy * dbuf->sample_size);
    dbuf->frame_ready = false;
    portEXIT_CRITICAL(&dbuf->spinlock);

    if (out_copied != NULL) {
        *out_copied = to_copy;
    }
    return ESP_OK;
}
