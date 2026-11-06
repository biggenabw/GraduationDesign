#include "actuator_manager.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

namespace {

constexpr const char* TAG = "ActuatorManager";

constexpr gpio_num_t kStatusLed = GPIO_NUM_17;
constexpr gpio_num_t kWindowServo = GPIO_NUM_11;
constexpr gpio_num_t kValveRelay = GPIO_NUM_12;
constexpr gpio_num_t kFanIna = GPIO_NUM_13;
constexpr gpio_num_t kBuzzer = GPIO_NUM_14;

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kServoTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kServoChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kBuzzerTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kBuzzerChannel = LEDC_CHANNEL_1;

constexpr uint32_t kServoPeriodUs = 20000;  // 50 Hz
constexpr uint32_t kServoMinUs = 500;       // 0 %
constexpr uint32_t kServoMaxUs = 2500;      // 100 %
constexpr uint32_t kServoResolution = 16384;  // 14 bit

esp_err_t InitOutput(gpio_num_t pin, int initial_level)
{
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << static_cast<uint32_t>(pin);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d config failed: %s", pin, esp_err_to_name(err));
        return err;
    }
    gpio_set_level(pin, initial_level);
    return ESP_OK;
}

esp_err_t InitServo(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kLedcMode;
    timer.duty_resolution = LEDC_TIMER_14_BIT;
    timer.timer_num = kServoTimer;
    timer.freq_hz = 50;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "servo timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel = {};
    channel.gpio_num = kWindowServo;
    channel.speed_mode = kLedcMode;
    channel.channel = kServoChannel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kServoTimer;
    channel.duty = static_cast<uint32_t>(kServoMinUs * kServoResolution / kServoPeriodUs);
    channel.hpoint = 0;

    err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "servo channel config failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t InitBuzzer(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kLedcMode;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = kBuzzerTimer;
    timer.freq_hz = 2700;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel = {};
    channel.gpio_num = kBuzzer;
    channel.speed_mode = kLedcMode;
    channel.channel = kBuzzerChannel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kBuzzerTimer;
    channel.duty = 0;
    channel.hpoint = 0;

    err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer channel config failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

} // namespace

esp_err_t actuator_manager_init(void)
{
    esp_err_t err = InitServo();
    if (err != ESP_OK) {
        return err;
    }
    err = InitBuzzer();
    if (err != ESP_OK) {
        return err;
    }

    // Relay is active-low: high = valve closed (safe default).
    err = InitOutput(kValveRelay, 1);
    if (err != ESP_OK) {
        return err;
    }
    err = InitOutput(kFanIna, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = InitOutput(kStatusLed, 0);
    if (err != ESP_OK) {
        return err;
    }

    actuator_manager_set_window(0);
    actuator_manager_set_fan(false);
    actuator_manager_set_valve(false);
    actuator_manager_set_buzzer(false);

    ESP_LOGI(TAG, "ready: servo=GPIO%d relay=GPIO%d fan=GPIO%d buzzer=GPIO%d led=GPIO%d",
             kWindowServo, kValveRelay, kFanIna, kBuzzer, kStatusLed);
    return ESP_OK;
}

void actuator_manager_set_led(bool on)
{
    gpio_set_level(kStatusLed, on ? 1 : 0);
}

void actuator_manager_set_window(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }

    const uint32_t pulse_us =
        kServoMinUs + (kServoMaxUs - kServoMinUs) * percent / 100;
    const uint32_t duty = pulse_us * kServoResolution / kServoPeriodUs;
    ledc_set_duty(kLedcMode, kServoChannel, duty);
    ledc_update_duty(kLedcMode, kServoChannel);
    ESP_LOGI(TAG, "window %u%% (pulse=%u us)", percent,
             static_cast<unsigned>(pulse_us));
}

void actuator_manager_set_fan(bool on)
{
    gpio_set_level(kFanIna, on ? 1 : 0);
    ESP_LOGI(TAG, "fan %s", on ? "on" : "off");
}

void actuator_manager_set_valve(bool open)
{
    // Active-low relay module: low = relay on = demo valve open.
    gpio_set_level(kValveRelay, open ? 0 : 1);
    ESP_LOGI(TAG, "demo valve %s", open ? "open" : "closed");
}

void actuator_manager_set_buzzer(bool on)
{
    const uint32_t duty = on ? 512 : 0;  // 50 % of 10-bit
    ledc_set_duty(kLedcMode, kBuzzerChannel, duty);
    ledc_update_duty(kLedcMode, kBuzzerChannel);
    ESP_LOGI(TAG, "buzzer %s", on ? "on" : "off");
}
