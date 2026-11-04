/**
 * @file tool_registry.c
 * @brief Dynamic tool registry, foreground/background concurrency manager, Core 1 worker
 *        task lifecycle helper, and self-test runner for M5CoreAnalyser.
 */

#include "tool_core.h"
#include "tool_core_priv.h"

#include <stdatomic.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "recorder_sink.h"
#include "sdkconfig.h"

static const char *TAG = "tool_core";

struct tool_worker {
    TaskHandle_t task_handle;
    SemaphoreHandle_t done_sem;
    StaticSemaphore_t done_sem_buf;
    tool_worker_step_fn_t step_fn;
    void *user_ctx;
    uint32_t period_ms;
    volatile bool stop_requested;
    volatile bool running;
};

typedef struct {
    const char *name;
    tool_selftest_fn_t fn;
} selftest_entry_t;

static StaticSemaphore_t s_tool_mutex_buf;
static SemaphoreHandle_t s_tool_mutex = NULL;
static atomic_int s_tool_mutex_state = 0; /* 0 = uninit, 1 = creating, 2 = ready */

static const analyser_tool_t *s_tools[TOOL_REGISTRY_MAX_TOOLS];
static bool s_bg_running[TOOL_REGISTRY_MAX_TOOLS];
static size_t s_tool_count = 0;
static const analyser_tool_t *s_active_tool = NULL;

static selftest_entry_t s_selftests[TOOL_CORE_MAX_SELFTESTS];
static size_t s_selftest_count = 0;

static void tool_core_lock_init(void)
{
    int expected = 0;
    if (atomic_compare_exchange_strong(&s_tool_mutex_state, &expected, 1)) {
        s_tool_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_tool_mutex_buf);
        atomic_store(&s_tool_mutex_state, 2);
    } else {
        while (atomic_load(&s_tool_mutex_state) != 2) {
            taskYIELD();
        }
    }
}

static void tool_lock(void)
{
    if (atomic_load(&s_tool_mutex_state) != 2) {
        tool_core_lock_init();
    }
    (void)xSemaphoreTakeRecursive(s_tool_mutex, portMAX_DELAY);
}

static void tool_unlock(void)
{
    (void)xSemaphoreGiveRecursive(s_tool_mutex);
}

static bool is_valid_category(tool_category_t category)
{
    return (category == TOOL_CATEGORY_ANALOG_IN) ||
           (category == TOOL_CATEGORY_DIGITAL_IN) ||
           (category == TOOL_CATEGORY_GENERATOR);
}

static int find_tool_index_locked(const char *id)
{
    if (id == NULL || id[0] == '\0') {
        return -1;
    }
    for (size_t i = 0; i < s_tool_count; ++i) {
        if (s_tools[i] != NULL && strcmp(s_tools[i]->id, id) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static bool is_tool_bg_active_locked(size_t idx)
{
    if (idx >= s_tool_count || s_tools[idx] == NULL) {
        return false;
    }
    if (!s_bg_running[idx]) {
        return false;
    }
    if (s_tools[idx]->is_running_bg != NULL) {
        return s_tools[idx]->is_running_bg(s_tools[idx]->user_ctx);
    }
    return true;
}

static bool any_spk_tool_active_locked(void)
{
    if (s_active_tool != NULL && s_active_tool->supports_spk_passthrough) {
        return true;
    }
    for (size_t i = 0; i < s_tool_count; ++i) {
        if (is_tool_bg_active_locked(i) && s_tools[i]->supports_spk_passthrough) {
            return true;
        }
    }
    return false;
}

static esp_err_t builtin_psram_double_buf_selftest(void)
{
    psram_double_buf_t dbuf = {0};
    esp_err_t err = psram_double_buf_alloc(&dbuf, 16, sizeof(uint16_t));
    if (err != ESP_OK) {
        return err;
    }

    uint16_t *wptr = (uint16_t *)psram_double_buf_get_write_ptr(&dbuf);
    if (wptr == NULL) {
        psram_double_buf_free(&dbuf);
        return ESP_FAIL;
    }
    for (size_t i = 0; i < 16; ++i) {
        wptr[i] = (uint16_t)(100U + i);
    }

    err = psram_double_buf_swap(&dbuf);
    if (err != ESP_OK) {
        psram_double_buf_free(&dbuf);
        return err;
    }

    uint16_t copy_out[16] = {0};
    size_t copied = 0;
    err = psram_double_buf_read_copy(&dbuf, copy_out, 16, &copied);
    psram_double_buf_free(&dbuf);
    if (err != ESP_OK || copied != 16 || copy_out[0] != 100U || copy_out[15] != 115U) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void tool_registry_test_reset(void)
{
    tool_lock();
    if (s_active_tool != NULL) {
        (void)tool_core_stop_active_tool(false);
    }
    (void)tool_core_stop_all_background_tools();

    memset(s_tools, 0, sizeof(s_tools));
    memset(s_bg_running, 0, sizeof(s_bg_running));
    s_tool_count = 0;
    s_active_tool = NULL;

    memset(s_selftests, 0, sizeof(s_selftests));
    s_selftest_count = 0;
    tool_unlock();
}

esp_err_t tool_core_init(void)
{
    tool_core_lock_init();

#if CONFIG_ANALYSER_RUN_SELFTEST_ON_BOOT
    size_t passed = 0;
    size_t failed = 0;
    esp_err_t err = tool_core_run_selftests(&passed, &failed);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Boot self-tests failed (%u passed, %u failed)",
                 (unsigned)passed, (unsigned)failed);
        return err;
    }
#endif

    return ESP_OK;
}

esp_err_t tool_registry_register(const analyser_tool_t *tool)
{
    if (tool == NULL ||
        tool->id == NULL || tool->id[0] == '\0' ||
        tool->title == NULL || tool->title[0] == '\0' ||
        !is_valid_category(tool->category) ||
        tool->on_start == NULL ||
        tool->on_stop == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tool_lock();
    if (find_tool_index_locked(tool->id) >= 0) {
        tool_unlock();
        ESP_LOGW(TAG, "Duplicate tool registration rejected for id='%s'", tool->id);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_tool_count >= TOOL_REGISTRY_MAX_TOOLS) {
        tool_unlock();
        ESP_LOGE(TAG, "Tool registry full (%u max)", (unsigned)TOOL_REGISTRY_MAX_TOOLS);
        return ESP_ERR_NO_MEM;
    }

    if (tool->on_init != NULL) {
        esp_err_t init_err = tool->on_init(tool->user_ctx);
        if (init_err != ESP_OK) {
            tool_unlock();
            ESP_LOGE(TAG, "Tool '%s' on_init failed: %s", tool->id, esp_err_to_name(init_err));
            return init_err;
        }
    }

    s_tools[s_tool_count] = tool;
    s_bg_running[s_tool_count] = false;
    s_tool_count++;
    tool_unlock();

    ESP_LOGI(TAG, "Registered tool '%s' (%s)", tool->id, tool->title);
    return ESP_OK;
}

size_t tool_registry_get_all(const analyser_tool_t **out_tools, size_t max_tools)
{
    tool_lock();
    if (out_tools == NULL) {
        size_t total = s_tool_count;
        tool_unlock();
        return total;
    }

    size_t count = (max_tools < s_tool_count) ? max_tools : s_tool_count;
    for (size_t i = 0; i < count; ++i) {
        out_tools[i] = s_tools[i];
    }
    tool_unlock();
    return count;
}

size_t tool_registry_get_by_category(tool_category_t category,
                                     const analyser_tool_t **out_tools,
                                     size_t max_tools)
{
    if (!is_valid_category(category)) {
        return 0;
    }

    tool_lock();
    size_t matched = 0;
    for (size_t i = 0; i < s_tool_count; ++i) {
        if (s_tools[i]->category == category) {
            if (out_tools != NULL) {
                if (matched < max_tools) {
                    out_tools[matched] = s_tools[i];
                    matched++;
                }
            } else {
                matched++;
            }
        }
    }
    tool_unlock();
    return matched;
}

const analyser_tool_t *tool_registry_find(const char *id)
{
    tool_lock();
    int idx = find_tool_index_locked(id);
    const analyser_tool_t *found = (idx >= 0) ? s_tools[idx] : NULL;
    tool_unlock();
    return found;
}

const char *tool_category_to_str(tool_category_t category)
{
    switch (category) {
    case TOOL_CATEGORY_ANALOG_IN:
        return "Analog In";
    case TOOL_CATEGORY_DIGITAL_IN:
        return "Digital In";
    case TOOL_CATEGORY_GENERATOR:
        return "Generator";
    default:
        return "Unknown";
    }
}

esp_err_t tool_core_start_tool(const char *tool_id, lv_obj_t *parent_cont)
{
    if (tool_id == NULL || tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    tool_lock();
    int idx = find_tool_index_locked(tool_id);
    if (idx < 0) {
        tool_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    const analyser_tool_t *target = s_tools[idx];
    if (s_active_tool == target) {
        tool_unlock();
        return ESP_OK;
    }

    /* Stop any existing foreground tool before starting the new one */
    if (__atomic_load_n(&s_active_tool, __ATOMIC_ACQUIRE) != NULL) {
        (void)tool_core_stop_active_tool(false);
    }

    bool was_bg = s_bg_running[idx];
    s_bg_running[idx] = false;

    esp_err_t err = target->on_start(parent_cont, target->user_ctx);
    if (err != ESP_OK) {
        /* If it failed to start and was not running in background, ensure its pins are clean */
        if (!was_bg) {
            (void)bsp_pin_release_all_for_owner(target->id);
        } else {
            s_bg_running[idx] = true;
        }
        tool_unlock();
        ESP_LOGW(TAG, "Failed to start tool '%s': %s", target->id, esp_err_to_name(err));
        return err;
    }

    __atomic_store_n(&s_active_tool, target, __ATOMIC_RELEASE);
    tool_unlock();

    ESP_LOGI(TAG, "Started foreground tool '%s'", target->id);
    return ESP_OK;
}

esp_err_t tool_core_stop_active_tool(bool keep_background)
{
    tool_lock();
    const analyser_tool_t *stopping = __atomic_load_n(&s_active_tool, __ATOMIC_ACQUIRE);
    if (stopping == NULL) {
        tool_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    int idx = find_tool_index_locked(stopping->id);
    bool effective_keep_bg = keep_background && stopping->supports_background;

    esp_err_t err = stopping->on_stop(effective_keep_bg, stopping->user_ctx);
    __atomic_store_n(&s_active_tool, NULL, __ATOMIC_RELEASE);

    if (idx >= 0) {
        if (effective_keep_bg && err == ESP_OK) {
            bool bg_ok = true;
            if (stopping->is_running_bg != NULL) {
                bg_ok = stopping->is_running_bg(stopping->user_ctx);
            }
            s_bg_running[idx] = bg_ok;
        } else {
            s_bg_running[idx] = false;
        }

        if (!s_bg_running[idx]) {
            (void)bsp_pin_release_all_for_owner(stopping->id);
        }
    }

    bool spk_still_needed = any_spk_tool_active_locked();
    tool_unlock();

    if (!spk_still_needed && bsp_spk_passthrough_is_enabled()) {
        (void)bsp_spk_passthrough_enable(false, 0);
    }

    ESP_LOGI(TAG, "Stopped foreground tool '%s' (keep_bg=%d)",
             stopping->id, effective_keep_bg ? 1 : 0);
    return err;
}

esp_err_t tool_core_stop_background_tool(const char *tool_id)
{
    if (tool_id == NULL || tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    tool_lock();
    int idx = find_tool_index_locked(tool_id);
    if (idx < 0) {
        tool_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    if (!is_tool_bg_active_locked((size_t)idx)) {
        tool_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const analyser_tool_t *bg_tool = s_tools[idx];
    esp_err_t err = bg_tool->on_stop(false, bg_tool->user_ctx);
    s_bg_running[idx] = false;
    (void)bsp_pin_release_all_for_owner(bg_tool->id);
    bool spk_still_needed = any_spk_tool_active_locked();
    tool_unlock();

    if (!spk_still_needed && bsp_spk_passthrough_is_enabled()) {
        (void)bsp_spk_passthrough_enable(false, 0);
    }

    ESP_LOGI(TAG, "Stopped background tool '%s'", bg_tool->id);
    return err;
}

esp_err_t tool_core_stop_all_background_tools(void)
{
    esp_err_t first_err = ESP_OK;

    tool_lock();
    for (size_t i = 0; i < s_tool_count; ++i) {
        if (is_tool_bg_active_locked(i)) {
            const analyser_tool_t *bg_tool = s_tools[i];
            esp_err_t err = bg_tool->on_stop(false, bg_tool->user_ctx);
            s_bg_running[i] = false;
            (void)bsp_pin_release_all_for_owner(bg_tool->id);
            if (err != ESP_OK && first_err == ESP_OK) {
                first_err = err;
            }
        }
    }
    bool spk_still_needed = any_spk_tool_active_locked();
    tool_unlock();

    if (!spk_still_needed && bsp_spk_passthrough_is_enabled()) {
        (void)bsp_spk_passthrough_enable(false, 0);
    }

    return first_err;
}

const analyser_tool_t *tool_core_get_active_tool(void)
{
    return __atomic_load_n(&s_active_tool, __ATOMIC_ACQUIRE);
}

bool tool_core_is_tool_running_bg(const char *tool_id)
{
    tool_lock();
    int idx = find_tool_index_locked(tool_id);
    bool active_bg = (idx >= 0) ? is_tool_bg_active_locked((size_t)idx) : false;
    tool_unlock();
    return active_bg;
}

size_t tool_core_get_background_tools(const analyser_tool_t **out_tools, size_t max_tools)
{
    tool_lock();
    size_t count = 0;
    for (size_t i = 0; i < s_tool_count; ++i) {
        if (is_tool_bg_active_locked(i)) {
            if (out_tools != NULL) {
                if (count < max_tools) {
                    out_tools[count] = s_tools[i];
                    count++;
                }
            } else {
                count++;
            }
        }
    }
    tool_unlock();
    return count;
}

size_t tool_core_get_background_count(void)
{
    return tool_core_get_background_tools(NULL, 0);
}

void tool_core_ui_refresh_active(void)
{
    tool_lock();
    const analyser_tool_t *active = s_active_tool;
    if (active != NULL && active->on_ui_refresh != NULL) {
        active->on_ui_refresh(active->user_ctx);
    }
    tool_unlock();
}

/* --------------------------------------------------------------------------
 * Core 1 Instrument Worker Task Implementation
 * -------------------------------------------------------------------------- */

static void worker_task_entry(void *arg)
{
    tool_worker_t *worker = (tool_worker_t *)arg;
    TickType_t delay_ticks = pdMS_TO_TICKS(worker->period_ms > 0 ? worker->period_ms : 1);
    if (delay_ticks == 0) {
        delay_ticks = 1;
    }

    while (!worker->stop_requested) {
        esp_err_t err = worker->step_fn(worker->user_ctx);
        if (err != ESP_OK || worker->stop_requested) {
            break;
        }
        vTaskDelay(delay_ticks);
    }

    worker->running = false;
    (void)xSemaphoreGive(worker->done_sem);
    vTaskDelete(NULL);
}

esp_err_t tool_worker_start(tool_worker_t **out_worker, const tool_worker_config_t *cfg)
{
    if (out_worker == NULL || cfg == NULL || cfg->step_fn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_worker = NULL;

    tool_worker_t *worker = (tool_worker_t *)heap_caps_calloc(
        1, sizeof(tool_worker_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (worker == NULL) {
        return ESP_ERR_NO_MEM;
    }

    worker->done_sem = xSemaphoreCreateBinaryStatic(&worker->done_sem_buf);
    worker->step_fn = cfg->step_fn;
    worker->user_ctx = cfg->user_ctx;
    worker->period_ms = (cfg->period_ms > 0) ? cfg->period_ms : 1;
    worker->stop_requested = false;
    worker->running = true;

    const char *task_name = (cfg->name != NULL && cfg->name[0] != '\0') ? cfg->name : "tool_worker";
    uint32_t stack_bytes = (cfg->stack_size_bytes >= 2048) ? cfg->stack_size_bytes : 4096;
    UBaseType_t prio = (cfg->priority > 0) ? cfg->priority : 5;
    BaseType_t core_id = cfg->core_id;
#if (portNUM_PROCESSORS > 1)
    if (core_id < 0 && core_id != tskNO_AFFINITY) {
        core_id = TOOL_WORKER_DEFAULT_CORE;
    }
#else
    core_id = tskNO_AFFINITY;
#endif

    BaseType_t created = xTaskCreatePinnedToCore(
        worker_task_entry,
        task_name,
        stack_bytes,
        worker,
        prio,
        &worker->task_handle,
        core_id);
    if (created != pdPASS) {
        heap_caps_free(worker);
        return ESP_ERR_NO_MEM;
    }

    *out_worker = worker;
    return ESP_OK;
}

esp_err_t tool_worker_stop(tool_worker_t *worker, uint32_t timeout_ms)
{
    if (worker == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    worker->stop_requested = true;
    uint32_t wait_ms = (timeout_ms > 0) ? timeout_ms : 500;
    TickType_t wait_ticks = pdMS_TO_TICKS(wait_ms);
    if (wait_ticks < 2) {
        wait_ticks = 2;
    }

    if (xSemaphoreTake(worker->done_sem, wait_ticks) != pdTRUE) {
        ESP_LOGW(TAG, "Worker task stop timed out after %u ms", (unsigned)wait_ms);
        return ESP_ERR_TIMEOUT;
    }

    heap_caps_free(worker);
    return ESP_OK;
}

bool tool_worker_is_running(const tool_worker_t *worker)
{
    return (worker != NULL) && worker->running;
}

bool tool_worker_stop_requested(const tool_worker_t *worker)
{
    return (worker != NULL) && worker->stop_requested;
}

/* --------------------------------------------------------------------------
 * Self-Test Runner Seam
 * -------------------------------------------------------------------------- */

esp_err_t tool_core_register_selftest(const char *name, tool_selftest_fn_t fn)
{
    if (name == NULL || name[0] == '\0' || fn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tool_lock();
    if (s_selftest_count >= TOOL_CORE_MAX_SELFTESTS) {
        tool_unlock();
        return ESP_ERR_NO_MEM;
    }
    s_selftests[s_selftest_count].name = name;
    s_selftests[s_selftest_count].fn = fn;
    s_selftest_count++;
    tool_unlock();
    return ESP_OK;
}

esp_err_t tool_core_run_selftests(size_t *out_passed, size_t *out_failed)
{
    size_t passed = 0;
    size_t failed = 0;

    if (builtin_psram_double_buf_selftest() == ESP_OK) {
        passed++;
    } else {
        failed++;
    }

    tool_lock();
    size_t count = s_selftest_count;
    selftest_entry_t snapshot[TOOL_CORE_MAX_SELFTESTS];
    if (count > 0) {
        memcpy(snapshot, s_selftests, count * sizeof(selftest_entry_t));
    }
    tool_unlock();

    for (size_t i = 0; i < count; ++i) {
        esp_err_t err = snapshot[i].fn();
        if (err == ESP_OK) {
            passed++;
        } else {
            failed++;
            ESP_LOGE(TAG, "Self-test '%s' failed: %s",
                     snapshot[i].name, esp_err_to_name(err));
        }
    }

    if (out_passed != NULL) {
        *out_passed = passed;
    }
    if (out_failed != NULL) {
        *out_failed = failed;
    }
    return (failed == 0) ? ESP_OK : ESP_FAIL;
}
