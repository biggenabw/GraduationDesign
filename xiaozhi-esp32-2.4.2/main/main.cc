#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "kitchen_core.h"

#define TAG "main"

#ifndef KITCHEN_SELFTEST
#define KITCHEN_SELFTEST 0
#endif

#if KITCHEN_SELFTEST

static void KitchenEventLogger(const kitchen_event_t* event, void* user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(TAG, "kitchen event type=%d level=%d mode=%d value=%ld msg=%s",
             static_cast<int>(event->type),
             static_cast<int>(event->alarm_level),
             static_cast<int>(event->mode),
             static_cast<long>(event->value),
             event->message);
}

static void RunKitchenSelfTest(void)
{
    ESP_LOGI(TAG, "KITCHEN_SELFTEST mode: xiaozhi application is disabled");
    ESP_ERROR_CHECK(kitchen_core_init());
    kitchen_core_set_event_callback(KitchenEventLogger, nullptr);
    ESP_ERROR_CHECK(kitchen_core_start());

    kitchen_core_request(KITCHEN_ACTION_SET_MODE, 0);
    kitchen_core_post_key(KITCHEN_KEY_OK, false);

    while (true) {
        kitchen_snapshot_t snapshot = {};
        esp_err_t err = kitchen_core_get_snapshot(&snapshot);
        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "selftest uptime=%lu ms mode=%d alarm=%d fan=%d valve=%d window=%u valid=%d",
                     static_cast<unsigned long>(snapshot.uptime_ms),
                     static_cast<int>(snapshot.mode),
                     static_cast<int>(snapshot.alarm_level),
                     snapshot.fan_on,
                     snapshot.valve_open,
                     snapshot.window_position_pct,
                     snapshot.valid);
        } else {
            ESP_LOGE(TAG, "snapshot failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif // KITCHEN_SELFTEST

extern "C" void app_main(void)
{
    // Initialize NVS flash for WiFi configuration and kitchen parameters.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if KITCHEN_SELFTEST
    RunKitchenSelfTest();
#else
    // Normal xiaozhi firmware: voice, OLED, MCP and network.
    auto& app = Application::GetInstance();
    app.Initialize();
    app.Run();  // This function runs the main event loop and never returns
#endif
}
