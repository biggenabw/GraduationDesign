#include "sensor_manager.h"

#include <algorithm>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "SensorManager";

constexpr i2c_port_num_t kSensorI2cPort = I2C_NUM_1;
constexpr gpio_num_t kSensorSda = GPIO_NUM_8;
constexpr gpio_num_t kSensorScl = GPIO_NUM_9;
constexpr uint8_t kSht30Address = 0x44;
constexpr uint8_t kMlxAddress = 0x5A;
constexpr uint32_t kSensorSpeedHz = 100000;

constexpr adc_channel_t kMq2AdcChannel = ADC_CHANNEL_0;  // GPIO1
constexpr adc_channel_t kMq4AdcChannel = ADC_CHANNEL_1;  // GPIO2

// MQ sensors need a long heater warm-up. Baseline is captured only after the
// ADC readings are non-zero and stable, otherwise ratio is meaningless.
constexpr uint32_t kMqWarmupMs = 180000;
constexpr uint32_t kMqBaselineMs = 60000;
constexpr int kMqMinRawMq2 = 100;
constexpr int kMqMinRawMq4 = 50;
constexpr size_t kMqBaselineMaxSamples = 64;
constexpr size_t kMqBaselineMinSamples = 10;

i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_sht30 = nullptr;
i2c_master_dev_handle_t s_mlx = nullptr;

adc_oneshot_unit_handle_t s_adc = nullptr;
bool s_adc_ready = false;
uint32_t s_mq_start_ms = 0;
uint32_t s_mq_baseline_start_ms = 0;
uint16_t s_mq_samples2[kMqBaselineMaxSamples] = {};
uint16_t s_mq_samples4[kMqBaselineMaxSamples] = {};
size_t s_mq_sample_count = 0;
float s_mq_base2 = 0.0f;
float s_mq_base4 = 0.0f;
bool s_mq_baseline_ready = false;
bool s_mq_warmup_logged = false;
bool s_mq_warmup_done_logged = false;
bool s_mq_low_raw_logged = false;
bool s_mq_baseline_capture_logged = false;
bool s_mq_baseline_logged = false;
bool s_mq_baseline_reject_logged = false;
bool s_mq_fault_logged = false;

void ResetMqBaselineCapture(void)
{
    s_mq_baseline_start_ms = 0;
    s_mq_sample_count = 0;
}

uint16_t MedianUint16(uint16_t* values, size_t count)
{
    if (count == 0) {
        return 0;
    }
    std::sort(values, values + count);
    return values[count / 2];
}

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

bool InitSht30(void)
{
    if (s_sht30 != nullptr) {
        return true;
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
            return false;
        }
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = kSht30Address;
    dev_config.scl_speed_hz = kSensorSpeedHz;

    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_config, &s_sht30);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add SHT30 failed: %s", esp_err_to_name(err));
        s_sht30 = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "SHT30 ready on I2C1 (SDA=%d, SCL=%d, addr=0x%02X)",
             kSensorSda, kSensorScl, kSht30Address);
    return true;
}

bool InitMlx(void)
{
    if (s_mlx != nullptr) {
        return true;
    }

    if (s_bus == nullptr) {
        InitSht30();
    }
    if (s_bus == nullptr) {
        ESP_LOGE(TAG, "MLX init skipped: no sensor I2C bus");
        return false;
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = kMlxAddress;
    dev_config.scl_speed_hz = kSensorSpeedHz;

    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_config, &s_mlx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add MLX90614 failed: %s", esp_err_to_name(err));
        s_mlx = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "MLX90614 ready on I2C1 (addr=0x%02X)", kMlxAddress);
    return true;
}

bool InitMqAdc(void)
{
    if (s_adc_ready) {
        return true;
    }

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    init_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t err = adc_oneshot_new_unit(&init_config, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        s_adc = nullptr;
        return false;
    }

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = ADC_ATTEN_DB_12;
    chan_config.bitwidth = ADC_BITWIDTH_12;

    err = adc_oneshot_config_channel(s_adc, kMq2AdcChannel, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC MQ-2 channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = nullptr;
        return false;
    }

    err = adc_oneshot_config_channel(s_adc, kMq4AdcChannel, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC MQ-4 channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = nullptr;
        return false;
    }

    s_adc_ready = true;
    s_mq_start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    ResetMqBaselineCapture();
    s_mq_base2 = 0.0f;
    s_mq_base4 = 0.0f;
    s_mq_baseline_ready = false;
    s_mq_warmup_logged = false;
    s_mq_warmup_done_logged = false;
    s_mq_low_raw_logged = false;
    s_mq_baseline_capture_logged = false;
    s_mq_baseline_logged = false;
    s_mq_baseline_reject_logged = false;
    s_mq_fault_logged = false;

    ESP_LOGI(TAG,
             "MQ-2/MQ-4 ADC ready on ADC1 CH0/CH1 (warm-up %us, baseline %us, min raw %d/%d)",
             static_cast<unsigned>(kMqWarmupMs / 1000),
             static_cast<unsigned>(kMqBaselineMs / 1000),
             kMqMinRawMq2, kMqMinRawMq4);
    return true;
}

} // namespace

esp_err_t sensor_manager_init(void)
{
    const bool sht30_ok = InitSht30();
    const bool mlx_ok = InitMlx();
    const bool adc_ok = InitMqAdc();

    if (!sht30_ok) {
        ESP_LOGW(TAG, "SHT30 not available");
    }
    if (!mlx_ok) {
        ESP_LOGW(TAG, "MLX90614 not available");
    }
    if (!adc_ok) {
        ESP_LOGW(TAG, "MQ ADC not available");
    }
    return (sht30_ok || mlx_ok || adc_ok) ? ESP_OK : ESP_ERR_NOT_FOUND;
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

esp_err_t sensor_manager_read_mlx(float* object_c)
{
    if (object_c == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mlx == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // RAM register 0x07: object temperature, 16-bit little-endian, 0.02 K/LSB.
    const uint8_t register_address = 0x07;
    esp_err_t err = i2c_master_transmit(s_mlx, &register_address, 1, 100);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t data[3] = {};
    err = i2c_master_receive(s_mlx, data, sizeof(data), 100);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw = static_cast<uint16_t>(data[0]) |
                         (static_cast<uint16_t>(data[1]) << 8);
    *object_c = static_cast<float>(raw) * 0.02f - 273.15f;
    return ESP_OK;
}

esp_err_t sensor_manager_read_mq(uint16_t* adc_mq2, uint16_t* adc_mq4,
                                 float* ratio_mq2, float* ratio_mq4,
                                 bool* gas_valid)
{
    if (adc_mq2 == nullptr || adc_mq4 == nullptr || ratio_mq2 == nullptr ||
        ratio_mq4 == nullptr || gas_valid == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_adc_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw2 = 0;
    int raw4 = 0;
    esp_err_t err = adc_oneshot_read(s_adc, kMq2AdcChannel, &raw2);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_oneshot_read(s_adc, kMq4AdcChannel, &raw4);
    if (err != ESP_OK) {
        return err;
    }

    *adc_mq2 = static_cast<uint16_t>(raw2);
    *adc_mq4 = static_cast<uint16_t>(raw4);
    *ratio_mq2 = 0.0f;
    *ratio_mq4 = 0.0f;
    *gas_valid = false;

    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    const uint32_t elapsed_ms = now_ms - s_mq_start_ms;

    if (elapsed_ms < kMqWarmupMs) {
        if (!s_mq_warmup_logged) {
            ESP_LOGI(TAG, "MQ warm-up started: %u s",
                     static_cast<unsigned>(kMqWarmupMs / 1000));
            s_mq_warmup_logged = true;
        }
        return ESP_OK;
    }

    if (!s_mq_warmup_done_logged) {
        ESP_LOGI(TAG, "MQ warm-up done; waiting for stable ADC before baseline");
        s_mq_warmup_done_logged = true;
    }

    const bool raw_too_low =
        raw2 < kMqMinRawMq2 || raw4 < kMqMinRawMq4;

    if (!s_mq_baseline_ready) {
        if (raw_too_low) {
            if (!s_mq_low_raw_logged) {
                ESP_LOGW(TAG,
                         "MQ ADC too low for baseline: MQ2=%d MQ4=%d; waiting",
                         raw2, raw4);
                s_mq_low_raw_logged = true;
            }
            ResetMqBaselineCapture();
            s_mq_baseline_capture_logged = false;
            return ESP_OK;
        }

        if (s_mq_low_raw_logged) {
            ESP_LOGI(TAG, "MQ ADC stable: MQ2=%d MQ4=%d; baseline capture reset",
                     raw2, raw4);
            s_mq_low_raw_logged = false;
        }

        if (s_mq_baseline_start_ms == 0) {
            s_mq_baseline_start_ms = now_ms;
            s_mq_sample_count = 0;
            if (!s_mq_baseline_capture_logged) {
                ESP_LOGI(TAG, "MQ baseline capture started: %u s",
                         static_cast<unsigned>(kMqBaselineMs / 1000));
                s_mq_baseline_capture_logged = true;
            }
        }

        if ((now_ms - s_mq_baseline_start_ms) < kMqBaselineMs) {
            if (s_mq_sample_count < kMqBaselineMaxSamples) {
                s_mq_samples2[s_mq_sample_count] = static_cast<uint16_t>(raw2);
                s_mq_samples4[s_mq_sample_count] = static_cast<uint16_t>(raw4);
                ++s_mq_sample_count;
            }
            return ESP_OK;
        }

        if (s_mq_sample_count < kMqBaselineMinSamples) {
            ResetMqBaselineCapture();
            s_mq_baseline_capture_logged = false;
            return ESP_OK;
        }

        const uint16_t median2 =
            MedianUint16(s_mq_samples2, s_mq_sample_count);
        const uint16_t median4 =
            MedianUint16(s_mq_samples4, s_mq_sample_count);

        if (median2 < kMqMinRawMq2 || median4 < kMqMinRawMq4) {
            if (!s_mq_baseline_reject_logged) {
                ESP_LOGW(TAG,
                         "MQ baseline rejected: MQ2=%d MQ4=%d; retrying",
                         static_cast<int>(median2), static_cast<int>(median4));
                s_mq_baseline_reject_logged = true;
            }
            ResetMqBaselineCapture();
            s_mq_baseline_capture_logged = false;
            return ESP_OK;
        }

        s_mq_base2 = static_cast<float>(median2);
        s_mq_base4 = static_cast<float>(median4);
        s_mq_baseline_ready = true;

        if (!s_mq_baseline_logged) {
            ESP_LOGI(TAG, "MQ baseline ready: MQ2=%d MQ4=%d (%u samples)",
                     static_cast<int>(s_mq_base2), static_cast<int>(s_mq_base4),
                     static_cast<unsigned>(s_mq_sample_count));
            s_mq_baseline_logged = true;
        }
    }

    if (raw_too_low) {
        if (!s_mq_fault_logged) {
            ESP_LOGW(TAG, "MQ sensor fault: raw ADC too low MQ2=%d MQ4=%d",
                     raw2, raw4);
            s_mq_fault_logged = true;
        }
        return ESP_OK;
    }

    if (s_mq_fault_logged) {
        ESP_LOGI(TAG, "MQ ADC recovered: MQ2=%d MQ4=%d", raw2, raw4);
        s_mq_fault_logged = false;
    }

    if (s_mq_base2 > 0.0f && s_mq_base4 > 0.0f) {
        *ratio_mq2 = static_cast<float>(raw2) / s_mq_base2;
        *ratio_mq4 = static_cast<float>(raw4) / s_mq_base4;
        *gas_valid = true;
    }
    return ESP_OK;
}
