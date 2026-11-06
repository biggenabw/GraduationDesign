#include "kitchen_core.h"

#include <cstdlib>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "actuator_manager.h"
#include "key_manager.h"
#include "sensor_manager.h"

namespace {

constexpr const char *TAG = "KitchenCore";
constexpr UBaseType_t kMessageQueueLength = 8;
constexpr uint32_t kSafetyPeriodMs = 100;
constexpr int kSafetyTaskPriority = 24;
constexpr int kControlTaskPriority = 22;
constexpr int kSensorTaskPriority = 20;
constexpr int kKeyTaskPriority = 21;
constexpr uint32_t kSafetyTaskStack = 4096;
constexpr uint32_t kControlTaskStack = 4096;
constexpr uint32_t kSensorTaskStack = 4096;
constexpr uint32_t kKeyTaskStack = 3072;

enum class MessageType : uint8_t {
    Command,
    Key,
};

struct Message {
    MessageType type;
    kitchen_action_t action;
    int value;
    kitchen_key_t key;
    bool long_press;
};

struct Context {
    SemaphoreHandle_t lock = nullptr;
    QueueHandle_t queue = nullptr;
    TaskHandle_t safety_task = nullptr;
    TaskHandle_t control_task = nullptr;
    TaskHandle_t sensor_task = nullptr;
    TaskHandle_t key_task = nullptr;
    kitchen_snapshot_t snapshot = {};
    kitchen_event_cb_t event_cb = nullptr;
    void *event_ctx = nullptr;
    bool initialized = false;
    bool started = false;
};

Context s_ctx;

void emit_event(kitchen_event_type_t type, kitchen_alarm_level_t level,
                kitchen_mode_t mode, int32_t value, const char *message)
{
    kitchen_event_cb_t callback = nullptr;
    void *user_ctx = nullptr;

    if (xSemaphoreTake(s_ctx.lock, portMAX_DELAY) == pdTRUE) {
        callback = s_ctx.event_cb;
        user_ctx = s_ctx.event_ctx;
        xSemaphoreGive(s_ctx.lock);
    }

    if (callback == nullptr) {
        return;
    }

    kitchen_event_t event = {};
    event.type = type;
    event.alarm_level = level;
    event.mode = mode;
    event.value = value;
    if (message != nullptr) {
        std::strncpy(event.message, message, sizeof(event.message) - 1);
    }
    callback(&event, user_ctx);
}

void safety_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "safety task started");

    while (true) {
        if (xSemaphoreTake(s_ctx.lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_ctx.snapshot.uptime_ms =
                static_cast<uint32_t>(esp_timer_get_time() / 1000);
            xSemaphoreGive(s_ctx.lock);
        }
        vTaskDelay(pdMS_TO_TICKS(kSafetyPeriodMs));
    }
}

void handle_command(const Message &message)
{
    kitchen_alarm_level_t level = KITCHEN_ALARM_NONE;
    kitchen_mode_t mode = KITCHEN_MODE_AUTO;
    bool valve_open = false;

    if (xSemaphoreTake(s_ctx.lock, portMAX_DELAY) != pdTRUE) {
        return;
    }

    switch (message.action) {
        case KITCHEN_ACTION_SET_MODE:
            s_ctx.snapshot.mode =
                (message.value == 0) ? KITCHEN_MODE_AUTO : KITCHEN_MODE_MANUAL;
            break;
        case KITCHEN_ACTION_SET_WINDOW:
            s_ctx.snapshot.window_position_pct =
                static_cast<uint8_t>(message.value);
            break;
        case KITCHEN_ACTION_SET_FAN:
            s_ctx.snapshot.fan_on = (message.value != 0);
            break;
        case KITCHEN_ACTION_MUTE_ALARM:
            break;
        case KITCHEN_ACTION_CLOSE_VALVE:
            s_ctx.snapshot.valve_open = false;
            break;
        case KITCHEN_ACTION_RESET_ALARM:
            s_ctx.snapshot.alarm_level = KITCHEN_ALARM_NONE;
            break;
        default:
            break;
    }

    level = s_ctx.snapshot.alarm_level;
    mode = s_ctx.snapshot.mode;
    valve_open = s_ctx.snapshot.valve_open;
    xSemaphoreGive(s_ctx.lock);

    switch (message.action) {
        case KITCHEN_ACTION_SET_MODE:
            emit_event(KITCHEN_EVENT_MODE_CHANGED, level, mode, message.value,
                       "mode changed");
            break;
        case KITCHEN_ACTION_CLOSE_VALVE:
            emit_event(KITCHEN_EVENT_VALVE_CHANGED, level, mode,
                       valve_open ? 1 : 0, "valve closed");
            break;
        case KITCHEN_ACTION_RESET_ALARM:
            emit_event(KITCHEN_EVENT_ALARM_CHANGED, KITCHEN_ALARM_NONE, mode, 0,
                       "alarm reset");
            break;
        default:
            break;
    }
}

void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "control task started");

    Message message = {};
    while (true) {
        if (xQueueReceive(s_ctx.queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (message.type == MessageType::Command) {
            handle_command(message);
        } else {
            kitchen_mode_t mode = KITCHEN_MODE_AUTO;
            if (xSemaphoreTake(s_ctx.lock, portMAX_DELAY) == pdTRUE) {
                mode = s_ctx.snapshot.mode;
                xSemaphoreGive(s_ctx.lock);
            }
            ESP_LOGI(TAG, "key %d %s", static_cast<int>(message.key),
                     message.long_press ? "long" : "short");
            emit_event(KITCHEN_EVENT_KEY, KITCHEN_ALARM_NONE, mode,
                       static_cast<int32_t>(message.key), "key");
        }
    }
}

void sensor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "sensor task started");

    TickType_t last_warn = 0;
    TickType_t last_mq_warn = 0;
    TickType_t last_mlx_warn = 0;
    uint32_t last_log_ms = 0;
    uint32_t last_mq_log_ms = 0;
    uint32_t last_mlx_log_ms = 0;
    while (true) {
        if (!sensor_manager_is_ready()) {
            sensor_manager_init();
        }

        float temperature_c = 0.0f;
        float humidity_rh = 0.0f;
        esp_err_t err = sensor_manager_read(&temperature_c, &humidity_rh);

        float pot_temperature_c = 0.0f;
        esp_err_t mlx_err = sensor_manager_read_mlx(&pot_temperature_c);

        uint16_t adc_mq2 = 0;
        uint16_t adc_mq4 = 0;
        float ratio_mq2 = 0.0f;
        float ratio_mq4 = 0.0f;
        bool gas_valid = false;
        esp_err_t mq_err = sensor_manager_read_mq(
            &adc_mq2, &adc_mq4, &ratio_mq2, &ratio_mq4, &gas_valid);

        if (xSemaphoreTake(s_ctx.lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (err == ESP_OK) {
                s_ctx.snapshot.temperature_c = temperature_c;
                s_ctx.snapshot.humidity_rh = humidity_rh;
                s_ctx.snapshot.valid = true;
            } else {
                s_ctx.snapshot.valid = false;
            }

            if (mlx_err == ESP_OK) {
                s_ctx.snapshot.pot_temperature_c = pot_temperature_c;
                s_ctx.snapshot.pot_valid = true;
            } else {
                s_ctx.snapshot.pot_valid = false;
            }

            if (mq_err == ESP_OK) {
                s_ctx.snapshot.gas_adc_mq2 = adc_mq2;
                s_ctx.snapshot.gas_adc_mq4 = adc_mq4;
                s_ctx.snapshot.gas_ratio_mq2 = ratio_mq2;
                s_ctx.snapshot.gas_ratio_mq4 = ratio_mq4;
                s_ctx.snapshot.gas_valid = gas_valid;
            } else {
                s_ctx.snapshot.gas_valid = false;
            }
            xSemaphoreGive(s_ctx.lock);
        }

        // LED is a simple visual indicator: on when the last SHT30 read is valid.
        actuator_manager_set_led(err == ESP_OK);

        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000);

        if (err == ESP_OK) {
            if (last_log_ms == 0 || (now_ms - last_log_ms) >= 5000) {
                const int temp_x100 = static_cast<int>(temperature_c * 100.0f);
                const int humi_x100 = static_cast<int>(humidity_rh * 100.0f);
                ESP_LOGI(TAG, "SHT30 temp=%d.%02dC humi=%d.%02d%%RH",
                         temp_x100 / 100, std::abs(temp_x100 % 100),
                         humi_x100 / 100, std::abs(humi_x100 % 100));
                last_log_ms = now_ms;
            }
        } else {
            TickType_t now = xTaskGetTickCount();
            if (last_warn == 0 || (now - last_warn) >= pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "SHT30 read failed: %s", esp_err_to_name(err));
                last_warn = now;
            }
        }

        if (mlx_err == ESP_OK) {
            if (last_mlx_log_ms == 0 || (now_ms - last_mlx_log_ms) >= 5000) {
                const int pot_x100 = static_cast<int>(pot_temperature_c * 100.0f);
                ESP_LOGI(TAG, "MLX pot=%d.%02dC",
                         pot_x100 / 100, std::abs(pot_x100 % 100));
                last_mlx_log_ms = now_ms;
            }
        } else {
            TickType_t now = xTaskGetTickCount();
            if (last_mlx_warn == 0 || (now - last_mlx_warn) >= pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "MLX read failed: %s", esp_err_to_name(mlx_err));
                last_mlx_warn = now;
            }
        }

        if (mq_err == ESP_OK) {
            if (last_mq_log_ms == 0 || (now_ms - last_mq_log_ms) >= 5000) {
                const int ratio2_x100 = static_cast<int>(ratio_mq2 * 100.0f);
                const int ratio4_x100 = static_cast<int>(ratio_mq4 * 100.0f);
                ESP_LOGI(TAG,
                         "MQ2 adc=%u ratio=%d.%02d MQ4 adc=%u ratio=%d.%02d gas_valid=%d",
                         static_cast<unsigned>(adc_mq2),
                         ratio2_x100 / 100, std::abs(ratio2_x100 % 100),
                         static_cast<unsigned>(adc_mq4),
                         ratio4_x100 / 100, std::abs(ratio4_x100 % 100),
                         gas_valid);
                last_mq_log_ms = now_ms;
            }
        } else {
            TickType_t now = xTaskGetTickCount();
            if (last_mq_warn == 0 || (now - last_mq_warn) >= pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "MQ read failed: %s", esp_err_to_name(mq_err));
                last_mq_warn = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void key_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "key task started");

    while (true) {
        key_manager_poll();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

extern "C" esp_err_t kitchen_core_init(void)
{
    if (s_ctx.initialized) {
        return ESP_OK;
    }

    s_ctx.lock = xSemaphoreCreateMutex();
    if (s_ctx.lock == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    s_ctx.queue = xQueueCreate(kMessageQueueLength, sizeof(Message));
    if (s_ctx.queue == nullptr) {
        vSemaphoreDelete(s_ctx.lock);
        s_ctx.lock = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_ctx.snapshot = {};
    s_ctx.snapshot.mode = KITCHEN_MODE_AUTO;
    s_ctx.snapshot.alarm_level = KITCHEN_ALARM_NONE;
    s_ctx.snapshot.safety_index = 100;
    s_ctx.snapshot.valid = false;
    s_ctx.initialized = true;

    if (sensor_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "SHT30 not ready; continuing without sensor");
    }
    if (actuator_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "status LED init failed; continuing");
    }
    if (key_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "key init failed; continuing");
    }

    ESP_LOGI(TAG, "initialized (v%s)", kitchen_core_version());
    return ESP_OK;
}

extern "C" esp_err_t kitchen_core_start(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.started) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        safety_task, "kitchen_safety", kSafetyTaskStack, nullptr,
        kSafetyTaskPriority, &s_ctx.safety_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create safety task");
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(
        control_task, "kitchen_ctrl", kControlTaskStack, nullptr,
        kControlTaskPriority, &s_ctx.control_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create control task");
        vTaskDelete(s_ctx.safety_task);
        s_ctx.safety_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(
        sensor_task, "kitchen_sensor", kSensorTaskStack, nullptr,
        kSensorTaskPriority, &s_ctx.sensor_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create sensor task");
        vTaskDelete(s_ctx.control_task);
        vTaskDelete(s_ctx.safety_task);
        s_ctx.control_task = nullptr;
        s_ctx.safety_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(
        key_task, "kitchen_key", kKeyTaskStack, nullptr,
        kKeyTaskPriority, &s_ctx.key_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create key task");
        vTaskDelete(s_ctx.sensor_task);
        vTaskDelete(s_ctx.control_task);
        vTaskDelete(s_ctx.safety_task);
        s_ctx.sensor_task = nullptr;
        s_ctx.control_task = nullptr;
        s_ctx.safety_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_ctx.started = true;
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

extern "C" esp_err_t kitchen_core_get_snapshot(kitchen_snapshot_t *out_snapshot)
{
    if (out_snapshot == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_ctx.lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *out_snapshot = s_ctx.snapshot;
    xSemaphoreGive(s_ctx.lock);
    return ESP_OK;
}

extern "C" esp_err_t kitchen_core_request(kitchen_action_t action, int value)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (action < KITCHEN_ACTION_SET_MODE || action > KITCHEN_ACTION_RESET_ALARM) {
        return ESP_ERR_INVALID_ARG;
    }
    if (action == KITCHEN_ACTION_SET_WINDOW && (value < 0 || value > 100)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((action == KITCHEN_ACTION_SET_MODE || action == KITCHEN_ACTION_SET_FAN) &&
        value != 0 && value != 1) {
        return ESP_ERR_INVALID_ARG;
    }

    Message message = {};
    message.type = MessageType::Command;
    message.action = action;
    message.value = value;

    if (xQueueSend(s_ctx.queue, &message, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

extern "C" esp_err_t kitchen_core_post_key(kitchen_key_t key, bool long_press)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (key < KITCHEN_KEY_UP || key > KITCHEN_KEY_BACK) {
        return ESP_ERR_INVALID_ARG;
    }

    Message message = {};
    message.type = MessageType::Key;
    message.key = key;
    message.long_press = long_press;

    if (xQueueSend(s_ctx.queue, &message, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

extern "C" void kitchen_core_set_event_callback(kitchen_event_cb_t callback,
                                                 void *user_ctx)
{
    if (s_ctx.lock == nullptr) {
        s_ctx.event_cb = callback;
        s_ctx.event_ctx = user_ctx;
        return;
    }

    if (xSemaphoreTake(s_ctx.lock, portMAX_DELAY) == pdTRUE) {
        s_ctx.event_cb = callback;
        s_ctx.event_ctx = user_ctx;
        xSemaphoreGive(s_ctx.lock);
    }
}

extern "C" const char *kitchen_core_version(void)
{
    return "0.1.0-skeleton";
}
