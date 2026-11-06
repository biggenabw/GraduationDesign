#include "radar_manager.h"

#include <cstdlib>
#include <string>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr const char* TAG = "RadarManager";

constexpr gpio_num_t kRadarIo = GPIO_NUM_10;
constexpr uart_port_t kRadarUart = UART_NUM_1;
constexpr gpio_num_t kRadarUartRx = GPIO_NUM_47;

constexpr uint32_t kUartBaud = 115200;
constexpr uint32_t kUartOfflineMs = 3000;
constexpr uint32_t kStartupValidMs = 5000;
constexpr int kHumanDebounceSamples = 3;   // 300 ms at 100 ms poll
constexpr int kNoHumanDebounceSamples = 10; // 1 s at 100 ms poll

bool s_ready = false;
bool s_uart_ready = false;
bool s_human = false;
bool s_last_raw = false;
int s_debounce_count = 0;
uint16_t s_distance_cm = 0;
uint32_t s_start_ms = 0;
uint32_t s_last_uart_ms = 0;
std::string s_line_buffer;

std::string Trim(const std::string& value)
{
    const char* whitespace = " \t\r\n";
    const size_t begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return "";
    }
    const size_t end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

void ProcessLine(const std::string& raw_line)
{
    const std::string line = Trim(raw_line);
    if (line.empty()) {
        return;
    }

    int distance = -1;
    if (line == "OFF") {
        distance = 0;
    } else {
        const size_t pos = line.find("distance:");
        if (pos != std::string::npos) {
            distance = std::atoi(line.c_str() + pos + 9);
        }
    }

    if (distance >= 0) {
        s_distance_cm = static_cast<uint16_t>(distance);
        s_last_uart_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }
}

} // namespace

esp_err_t radar_manager_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << static_cast<uint32_t>(kRadarIo);
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radar IO GPIO%d config failed: %s", kRadarIo,
                 esp_err_to_name(err));
        return err;
    }

    if (!s_uart_ready) {
        uart_config_t uart_config = {};
        uart_config.baud_rate = kUartBaud;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;

        err = uart_param_config(kRadarUart, &uart_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "radar UART param config failed: %s",
                     esp_err_to_name(err));
            return err;
        }

        // TX is not used by the radar; keep it unassigned.
        err = uart_set_pin(kRadarUart, UART_PIN_NO_CHANGE, kRadarUartRx,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "radar UART pin config failed: %s",
                     esp_err_to_name(err));
            return err;
        }

        err = uart_driver_install(kRadarUart, 1024, 0, 0, nullptr, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "radar UART driver install failed: %s",
                     esp_err_to_name(err));
            return err;
        }
        uart_flush_input(kRadarUart);
        s_uart_ready = true;
    }

    s_ready = true;
    s_start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    s_last_uart_ms = 0;
    s_human = gpio_get_level(kRadarIo) == 1;
    s_last_raw = s_human;
    s_debounce_count = 0;
    s_distance_cm = 0;
    s_line_buffer.clear();

    ESP_LOGI(TAG, "LD2402 ready: IO=GPIO%d UART%d RX=GPIO%d %lu 8N1",
             kRadarIo, kRadarUart, kRadarUartRx,
             static_cast<unsigned long>(kUartBaud));
    return ESP_OK;
}

void radar_manager_poll(void)
{
    if (!s_ready) {
        return;
    }

    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    const bool raw_human = gpio_get_level(kRadarIo) == 1;
    if (raw_human != s_last_raw) {
        s_last_raw = raw_human;
        s_debounce_count = 0;
    } else if (s_debounce_count < kNoHumanDebounceSamples) {
        ++s_debounce_count;
    }

    const int threshold = raw_human ? kHumanDebounceSamples : kNoHumanDebounceSamples;
    if (s_debounce_count >= threshold && s_human != raw_human) {
        s_human = raw_human;
        ESP_LOGI(TAG, "human %s", s_human ? "present" : "absent");
    }

    uint8_t buffer[64];
    const int length = uart_read_bytes(kRadarUart, buffer, sizeof(buffer), 0);
    if (length > 0) {
        s_last_uart_ms = now_ms;
        for (int i = 0; i < length; ++i) {
            const char ch = static_cast<char>(buffer[i]);
            if (ch == '\n' || ch == '\r') {
                ProcessLine(s_line_buffer);
                s_line_buffer.clear();
            } else if (s_line_buffer.size() < 120) {
                s_line_buffer.push_back(ch);
            }
        }
    }

    if (s_last_uart_ms == 0) {
        s_last_uart_ms = now_ms;
    }
}

void radar_manager_get(radar_manager_data_t* out_data)
{
    if (out_data == nullptr) {
        return;
    }

    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    const bool online = s_ready && (now_ms - s_last_uart_ms) <= kUartOfflineMs;
    const bool valid = online && (now_ms - s_start_ms) >= kStartupValidMs;

    out_data->human_present = s_human;
    out_data->human_valid = valid;
    out_data->radar_online = online;
    out_data->distance_cm = online ? s_distance_cm : 0;
}
