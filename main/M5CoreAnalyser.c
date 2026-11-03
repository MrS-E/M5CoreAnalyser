#include <stdio.h>
#include "esp_log.h"
#include "bsp_m5core2.h"

static const char *TAG = "m5core_analyser";

void app_main(void)
{
    esp_err_t err = bsp_m5core2_init();
    ESP_LOGI(TAG, "M5CoreAnalyser BSP init finished: %s", esp_err_to_name(err));
}
