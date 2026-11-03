#include <stdio.h>
#include "esp_log.h"
#include "bsp_m5core2.h"
#include "recorder_sink.h"
#include "tool_core.h"
#include "tool_scope.h"
#include "tool_voltmeter.h"
#include "ui_shell.h"

static const char *TAG = "m5core_analyser";

void app_main(void)
{
    esp_err_t err = bsp_m5core2_init();
    ESP_LOGI(TAG, "M5CoreAnalyser BSP init finished: %s", esp_err_to_name(err));

    ESP_ERROR_CHECK(recorder_sink_init());
    ESP_ERROR_CHECK(tool_core_init());

    ESP_ERROR_CHECK(tool_voltmeter_register());
    ESP_ERROR_CHECK(tool_scope_register());

    err = ui_shell_init();
    if (err == ESP_OK) {
        err = ui_shell_start_task();
    }
    ESP_LOGI(TAG, "M5CoreAnalyser UI shell ready: %s", esp_err_to_name(err));
}
