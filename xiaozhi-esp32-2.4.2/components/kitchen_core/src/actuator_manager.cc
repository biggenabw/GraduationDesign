#include "actuator_manager.h"

#include "driver/gpio.h"
#include "esp_log.h"

namespace {

constexpr const char* TAG = "ActuatorManager";
constexpr gpio_num_t kStatusLed = GPIO_NUM_17;

} // namespace

esp_err_t actuator_manager_init(void)
{
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << static_cast<uint32_t>(kStatusLed);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    gpio_set_level(kStatusLed, 0);
    ESP_LOGI(TAG, "status LED ready on GPIO%d", kStatusLed);
    return ESP_OK;
}

void actuator_manager_set_led(bool on)
{
    gpio_set_level(kStatusLed, on ? 1 : 0);
}
