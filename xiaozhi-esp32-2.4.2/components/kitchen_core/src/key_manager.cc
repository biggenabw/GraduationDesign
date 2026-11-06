#include "key_manager.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "kitchen_core.h"

namespace {

constexpr const char* TAG = "KeyManager";
constexpr int kDebounceSamples = 3;
constexpr int64_t kLongPressMs = 1000;

struct KeyBinding {
    gpio_num_t gpio;
    kitchen_key_t key;
    const char* name;
};

constexpr KeyBinding kKeyBindings[] = {
    {GPIO_NUM_38, KITCHEN_KEY_UP, "UP"},
    {GPIO_NUM_39, KITCHEN_KEY_DOWN, "DOWN"},
    {GPIO_NUM_40, KITCHEN_KEY_OK, "OK"},
    {GPIO_NUM_0, KITCHEN_KEY_BACK, "BACK"},
};

struct KeyState {
    bool last_raw = false;
    bool stable_pressed = false;
    int stable_count = 0;
    int64_t pressed_ms = 0;
};

KeyState s_states[sizeof(kKeyBindings) / sizeof(kKeyBindings[0])];

} // namespace

esp_err_t key_manager_init(void)
{
    uint64_t pin_mask = 0;
    for (const auto& binding : kKeyBindings) {
        pin_mask |= 1ULL << static_cast<uint32_t>(binding.gpio);
    }

    gpio_config_t io_config = {};
    io_config.pin_bit_mask = pin_mask;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_ENABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "key GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < sizeof(kKeyBindings) / sizeof(kKeyBindings[0]); ++i) {
        const bool pressed = gpio_get_level(kKeyBindings[i].gpio) == 0;
        s_states[i].last_raw = pressed;
        s_states[i].stable_pressed = pressed;
        s_states[i].stable_count = 0;
        s_states[i].pressed_ms = 0;
    }

    ESP_LOGI(TAG, "4 keys ready: UP=38 DOWN=39 OK=40 BACK=0");
    return ESP_OK;
}

void key_manager_poll(void)
{
    const int64_t now_ms = esp_timer_get_time() / 1000;

    for (size_t i = 0; i < sizeof(kKeyBindings) / sizeof(kKeyBindings[0]); ++i) {
        const KeyBinding& binding = kKeyBindings[i];
        KeyState& state = s_states[i];
        const bool raw_pressed = gpio_get_level(binding.gpio) == 0;

        if (raw_pressed != state.last_raw) {
            state.last_raw = raw_pressed;
            state.stable_count = 0;
            continue;
        }

        if (state.stable_count < kDebounceSamples) {
            ++state.stable_count;
        }

        if (state.stable_count == kDebounceSamples &&
            state.stable_pressed != raw_pressed) {
            state.stable_pressed = raw_pressed;

            if (raw_pressed) {
                state.pressed_ms = now_ms;
            } else {
                const int64_t duration_ms = now_ms - state.pressed_ms;
                const bool long_press = duration_ms >= kLongPressMs;
                ESP_LOGI(TAG, "key %s %s (%lld ms)", binding.name,
                         long_press ? "long" : "short",
                         static_cast<long long>(duration_ms));
                esp_err_t err = kitchen_core_post_key(binding.key, long_press);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "post key failed: %s", esp_err_to_name(err));
                }
            }
        }
    }
}
