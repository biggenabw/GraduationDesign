#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>

#include "application.h"
#include "kitchen_core.h"

#define TAG "main"

#ifndef KITCHEN_SELFTEST
#define KITCHEN_SELFTEST 0
#endif

#if KITCHEN_SELFTEST

static void FormatX100(char* out, size_t len, float value)
{
    const int scaled = static_cast<int>(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
    const int whole = scaled / 100;
    int frac = scaled % 100;
    if (frac < 0) {
        frac = -frac;
    }
    snprintf(out, len, "%d.%02d", whole, frac);
}

static const char* ModeName(kitchen_mode_t mode)
{
    return mode == KITCHEN_MODE_AUTO ? "AUTO" : "MANUAL";
}

static const char* AlarmName(kitchen_alarm_level_t level)
{
    switch (level) {
        case KITCHEN_ALARM_L1:
            return "L1";
        case KITCHEN_ALARM_L2:
            return "L2";
        case KITCHEN_ALARM_L3:
            return "L3";
        case KITCHEN_ALARM_NONE:
        default:
            return "NONE";
    }
}

static void KitchenEventLogger(const kitchen_event_t* event, void* user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(TAG, "kitchen event type=%d level=%d mode=%d value=%ld msg=%s",
             static_cast<int>(event->type),
             static_cast<int>(event->alarm_level),
             static_cast<int>(event->mode),
             static_cast<long>(event->value),
             event->message);

    // Selftest-only key mapping: UP/DOWN move the window, OK toggles the fan,
    // BACK toggles the demo valve based on the current snapshot.
    if (event->type == KITCHEN_EVENT_KEY) {
        switch (static_cast<kitchen_key_t>(event->value)) {
            case KITCHEN_KEY_UP:
                kitchen_core_request(KITCHEN_ACTION_SET_WINDOW, 100);
                break;
            case KITCHEN_KEY_DOWN:
                kitchen_core_request(KITCHEN_ACTION_SET_WINDOW, 0);
                break;
            case KITCHEN_KEY_OK: {
                static bool fan_on = false;
                fan_on = !fan_on;
                kitchen_core_request(KITCHEN_ACTION_SET_FAN, fan_on ? 1 : 0);
                break;
            }
            case KITCHEN_KEY_BACK: {
                kitchen_snapshot_t snapshot = {};
                if (kitchen_core_get_snapshot(&snapshot) == ESP_OK) {
                    kitchen_core_request(KITCHEN_ACTION_SET_VALVE,
                                         snapshot.valve_open ? 0 : 1);
                }
                break;
            }
            default:
                break;
        }
    }
}

static void RunKitchenSelfTest(void)
{
    ESP_LOGI(TAG, "KITCHEN_SELFTEST mode: xiaozhi application is disabled");
    ESP_ERROR_CHECK(kitchen_core_init());
    kitchen_core_set_event_callback(KitchenEventLogger, nullptr);
    ESP_ERROR_CHECK(kitchen_core_start());

    kitchen_core_request(KITCHEN_ACTION_SET_MODE, 0);

    while (true) {
        kitchen_snapshot_t snapshot = {};
        esp_err_t err = kitchen_core_get_snapshot(&snapshot);
        if (err == ESP_OK) {
            char temperature_text[16];
            char humidity_text[16];
            char pot_text[16];
            char ratio2_text[16];
            char ratio4_text[16];
            FormatX100(temperature_text, sizeof(temperature_text), snapshot.temperature_c);
            FormatX100(humidity_text, sizeof(humidity_text), snapshot.humidity_rh);
            FormatX100(pot_text, sizeof(pot_text), snapshot.pot_temperature_c);
            FormatX100(ratio2_text, sizeof(ratio2_text), snapshot.gas_ratio_mq2);
            FormatX100(ratio4_text, sizeof(ratio4_text), snapshot.gas_ratio_mq4);

            ESP_LOGI(TAG,
                     "selftest uptime=%lu ms temp=%sC humi=%s%%RH pot=%sC pot_valid=%d MQ2=%u ratio=%s MQ4=%u ratio=%s gas_valid=%d human=%d human_valid=%d radar_online=%d dist=%u mode=%s alarm=%s flags=%u index=%u stove=%d fan=%d valve=%d window=%u valid=%d",
                     static_cast<unsigned long>(snapshot.uptime_ms),
                     temperature_text,
                     humidity_text,
                     pot_text,
                     snapshot.pot_valid,
                     static_cast<unsigned>(snapshot.gas_adc_mq2),
                     ratio2_text,
                     static_cast<unsigned>(snapshot.gas_adc_mq4),
                     ratio4_text,
                     snapshot.gas_valid,
                     snapshot.human_present,
                     snapshot.human_valid,
                     snapshot.radar_online,
                     static_cast<unsigned>(snapshot.human_distance_cm),
                     ModeName(snapshot.mode),
                     AlarmName(snapshot.alarm_level),
                     static_cast<unsigned>(snapshot.alarm_flags),
                     static_cast<unsigned>(snapshot.safety_index),
                     snapshot.stove_on,
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
