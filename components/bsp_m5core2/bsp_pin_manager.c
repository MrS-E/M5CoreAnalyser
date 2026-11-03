/**
 * @file bsp_pin_manager.c
 * @brief Thread-safe GPIO pin reservation and conflict manager for M5Stack Core2.
 */

#include "bsp_m5core2.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "bsp_pin_mgr";

#define BSP_PIN_OWNER_MAX_LEN (24)
#define BSP_SYSTEM_OWNER_NAME "system"

static char s_pin_owners[GPIO_NUM_MAX][BSP_PIN_OWNER_MAX_LEN];
static portMUX_TYPE s_pin_lock = portMUX_INITIALIZER_UNLOCKED;

bool bsp_pin_is_protected(gpio_num_t pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) {
        return true;
    }

    switch (pin) {
        /* Safe expansion / user instrument pins on M5Stack Core2:
         * - Port A (I2C/Grove): GPIO32, GPIO33
         * - Port B (Analog/DAC/GPIO): GPIO26, GPIO36
         * - Port C (UART2/GPIO): GPIO13, GPIO14
         * - M5-Bus free pins: GPIO25 (DAC1), GPIO35 (ADC1_CH7), GPIO27, GPIO19
         */
        case GPIO_NUM_13:
        case GPIO_NUM_14:
        case GPIO_NUM_19:
        case GPIO_NUM_25:
        case GPIO_NUM_26:
        case GPIO_NUM_27:
        case GPIO_NUM_32:
        case GPIO_NUM_33:
        case GPIO_NUM_35:
        case GPIO_NUM_36:
            return false;

        /* All other pins are reserved internal peripherals or unbonded:
         * - GPIO0, GPIO2, GPIO12: I2S1 NS4168 Speaker (WS, DOUT, BCK)
         * - GPIO1, GPIO3: UART0 USB-Serial Console
         * - GPIO4, GPIO5, GPIO15, GPIO18, GPIO23, GPIO38: SPI2 LCD (ILI9342C) & TF-Card
         * - GPIO6..GPIO11, GPIO16, GPIO17: Internal SPI Flash & 8MB PSRAM
         * - GPIO21, GPIO22: Internal I2C0 (AXP192, FT6336U, BM8563, MPU6886)
         * - GPIO34: Internal SPM1423 PDM Microphone DATA
         * - GPIO39: FT6336U Touch IRQ
         */
        default:
            return true;
    }
}

bool bsp_pin_is_input_only(gpio_num_t pin)
{
    return (pin >= GPIO_NUM_34 && pin <= GPIO_NUM_39);
}

void bsp_pin_manager_reset(void)
{
    portENTER_CRITICAL(&s_pin_lock);
    memset(s_pin_owners, 0, sizeof(s_pin_owners));
    portEXIT_CRITICAL(&s_pin_lock);
}

esp_err_t bsp_pin_claim_ex(gpio_num_t pin, const char *owner_tool_id, bool require_output)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX || owner_tool_id == NULL || owner_tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(owner_tool_id, BSP_SYSTEM_OWNER_NAME) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bsp_pin_is_protected(pin)) {
        ESP_LOGW(TAG, "Refusing claim on protected M5Core2 pin GPIO%d by '%s'", (int)pin, owner_tool_id);
        return ESP_ERR_NOT_ALLOWED;
    }

    if (require_output && bsp_pin_is_input_only(pin)) {
        ESP_LOGW(TAG, "Refusing output claim on input-only GPIO%d by '%s'", (int)pin, owner_tool_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret = ESP_OK;
    portENTER_CRITICAL(&s_pin_lock);
    if (s_pin_owners[pin][0] != '\0') {
        if (strncmp(s_pin_owners[pin], owner_tool_id, BSP_PIN_OWNER_MAX_LEN - 1) == 0) {
            ret = ESP_OK;
        } else {
            ret = ESP_ERR_INVALID_STATE;
        }
    } else {
        strncpy(s_pin_owners[pin], owner_tool_id, BSP_PIN_OWNER_MAX_LEN - 1);
        s_pin_owners[pin][BSP_PIN_OWNER_MAX_LEN - 1] = '\0';
        ret = ESP_OK;
    }
    portEXIT_CRITICAL(&s_pin_lock);

    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO%d conflict: requested by '%s', already owned by '%s'",
                 (int)pin, owner_tool_id, bsp_pin_get_owner(pin));
    }
    return ret;
}

esp_err_t bsp_pin_claim(gpio_num_t pin, const char *owner_tool_id)
{
    return bsp_pin_claim_ex(pin, owner_tool_id, false);
}

esp_err_t bsp_pin_release(gpio_num_t pin, const char *owner_tool_id)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX || owner_tool_id == NULL || owner_tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (bsp_pin_is_protected(pin)) {
        return ESP_ERR_NOT_ALLOWED;
    }

    esp_err_t ret = ESP_OK;
    portENTER_CRITICAL(&s_pin_lock);
    if (s_pin_owners[pin][0] == '\0') {
        ret = ESP_ERR_INVALID_STATE;
    } else if (strncmp(s_pin_owners[pin], owner_tool_id, BSP_PIN_OWNER_MAX_LEN - 1) != 0) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        s_pin_owners[pin][0] = '\0';
        ret = ESP_OK;
    }
    portEXIT_CRITICAL(&s_pin_lock);

    return ret;
}

esp_err_t bsp_pin_release_all_for_owner(const char *owner_tool_id)
{
    if (owner_tool_id == NULL || owner_tool_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_pin_lock);
    for (int pin = 0; pin < GPIO_NUM_MAX; ++pin) {
        if (s_pin_owners[pin][0] != '\0' &&
            strncmp(s_pin_owners[pin], owner_tool_id, BSP_PIN_OWNER_MAX_LEN - 1) == 0) {
            s_pin_owners[pin][0] = '\0';
        }
    }
    portEXIT_CRITICAL(&s_pin_lock);

    return ESP_OK;
}

const char *bsp_pin_get_owner(gpio_num_t pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) {
        return NULL;
    }

    if (bsp_pin_is_protected(pin)) {
        return BSP_SYSTEM_OWNER_NAME;
    }

    const char *owner = NULL;
    portENTER_CRITICAL(&s_pin_lock);
    if (s_pin_owners[pin][0] != '\0') {
        owner = s_pin_owners[pin];
    }
    portEXIT_CRITICAL(&s_pin_lock);

    return owner;
}
