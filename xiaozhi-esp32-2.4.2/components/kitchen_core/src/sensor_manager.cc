#include "sensor_manager.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "SensorManager";
constexpr i2c_port_num_t kSensorI2cPort = I2C_NUM_1;
constexpr gpio_num_t kSensorSda = GPIO_NUM_8;
constexpr gpio_num_t kSensorScl = GPIO_NUM_9;
constexpr uint8_t kSht30Address = 0x44;
constexpr uint32_t kSht30SpeedHz = 100000;

i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_sht30 = nullptr;

uint8_t Sht30Crc(const uint8_t* data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

} // namespace

esp_err_t sensor_manager_init(void)
{
    if (s_sht30 != nullptr) {
        return ESP_OK;
    }

    if (s_bus == nullptr) {
        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = kSensorI2cPort;
        bus_config.sda_io_num = kSensorSda;
        bus_config.scl_io_num = kSensorScl;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.intr_priority = 0;
        bus_config.trans_queue_depth = 0;
        bus_config.flags.enable_internal_pullup = 1;

        esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            s_bus = nullptr;
            return err;
        }
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = kSht30Address;
    dev_config.scl_speed_hz = kSht30SpeedHz;

    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_config, &s_sht30);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add SHT30 failed: %s", esp_err_to_name(err));
        s_sht30 = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "SHT30 ready on I2C1 (SDA=%d, SCL=%d, addr=0x%02X)",
             kSensorSda, kSensorScl, kSht30Address);
    return ESP_OK;
}

bool sensor_manager_is_ready(void)
{
    return s_sht30 != nullptr;
}

esp_err_t sensor_manager_read(float* temperature_c, float* humidity_rh)
{
    if (temperature_c == nullptr || humidity_rh == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_sht30 == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // Single-shot, high repeatability, no clock stretching.
    const uint8_t command[2] = {0x2C, 0x06};
    esp_err_t err = i2c_master_transmit(s_sht30, command, sizeof(command), 100);
    if (err != ESP_OK) {
        return err;
    }

    // SHT30 conversion takes about 15 ms at high repeatability.
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t data[6] = {};
    err = i2c_master_receive(s_sht30, data, sizeof(data), 100);
    if (err != ESP_OK) {
        return err;
    }

    if (data[2] != Sht30Crc(data, 2) || data[5] != Sht30Crc(data + 3, 2)) {
        ESP_LOGW(TAG, "SHT30 CRC mismatch");
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t raw_temp = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t raw_humi = (static_cast<uint16_t>(data[3]) << 8) | data[4];

    *temperature_c = -45.0f + 175.0f * static_cast<float>(raw_temp) / 65535.0f;
    *humidity_rh = 100.0f * static_cast<float>(raw_humi) / 65535.0f;
    return ESP_OK;
}
