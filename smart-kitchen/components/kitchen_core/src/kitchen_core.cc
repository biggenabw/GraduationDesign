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
#include "radar_manager.h"
#include "safety_monitor.h"
#include "sensor_manager.h"

namespace {

constexpr const char *TAG = "KitchenCore";
constexpr UBaseType_t kMessageQueueLength = 8;
constexpr uint32_t kSafetyPeriodMs = 100;
constexpr int kSafetyTaskPriority = 24;
constexpr int kControlTaskPriority = 22;
constexpr int kSensorTaskPriority = 20;
constexpr int kKeyTaskPriority = 21;
constexpr int kRadarTaskPriority = 19;
constexpr uint32_t kSafetyTaskStack = 4096;
constexpr uint32_t kControlTaskStack = 4096;
constexpr uint32_t kSensorTaskStack = 4096;
constexpr uint32_t kKeyTaskStack = 3072;
constexpr uint32_t kRadarTaskStack = 4096;
constexpr uint32_t kRadarPeriodMs = 100;
constexpr uint32_t kAlarmMuteMs = 30000;

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
    TaskHandle_t radar_task = nullptr;
    kitchen_snapshot_t snapshot = {};
    kitchen_event_cb_t event_cb = nullptr;
    void *event_ctx = nullptr;
    uint32_t mute_until_ms = 0;
    bool initialized = false;
    bool started = false;
};

Context s_ctx;

bool OutputsDiffer(const safety_monitor::SafetyDecision& a,
                   const safety_monitor::SafetyDecision& b)
{
    return a.active != b.active ||
           a.fan_on != b.fan_on ||
           a.valve_open != b.valve_open ||
           a.window_position_pct != b.window_position_pct ||
           a.buzzer_on != b.buzzer_on ||
           a.status_led_on != b.status_led_on ||
           a.status_led_blink_hz != b.status_led_blink_hz;
}

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

    safety_monitor::SafetyDecision last_decision = {};
    last_decision.valve_open = true;
    last_decision.safety_index = 100;
    kitchen_alarm_level_t last_alarm_level = KITCHEN_ALARM_NONE;
    bool last_valve_open = true;
    uint16_t last_alarm_flags = 0;
    bool last_led_on = false;
    uint32_t last_log_ms = 0;
    uint32_t last_apply_ms = 0;

    while (true) {
        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000);

        kitchen_snapshot_t snapshot = {};
        uint32_t mute_until_ms = 0;
        bool have_snapshot = false;
        if (xSemaphoreTake(s_ctx.lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_ctx.snapshot.uptime_ms = now_ms;
            snapshot = s_ctx.snapshot;
            mute_until_ms = s_ctx.mute_until_ms;
            have_snapshot = true;
            xSemaphoreGive(s_ctx.lock);
        }

        if (!have_snapshot) {
            vTaskDelay(pdMS_TO_TICKS(kSafetyPeriodMs));
            continue;
        }

        safety_monitor::SafetyInput input = {};
        input.now_ms = now_ms;
        input.sht30_valid = snapshot.valid;
        input.temperature_c = snapshot.temperature_c;
        input.humidity_rh = snapshot.humidity_rh;
        input.gas2_valid = snapshot.gas2_valid;
        input.gas4_valid = snapshot.gas4_valid;
        input.gas_ratio_mq2 = snapshot.gas_ratio_mq2;
        input.gas_ratio_mq4 = snapshot.gas_ratio_mq4;
        input.human_present = snapshot.human_present;
        input.human_valid = snapshot.human_valid;
        input.radar_online = snapshot.radar_online;
        input.pot_valid = snapshot.pot_valid;
        input.pot_temperature_c = snapshot.pot_temperature_c;
        input.manual_mode = (snapshot.mode == KITCHEN_MODE_MANUAL);

        safety_monitor::SafetyDecision decision = {};
        safety_monitor::evaluate(&input, &decision);

        const bool muted = now_ms < mute_until_ms;
        bool outputs_changed = OutputsDiffer(decision, last_decision);
        if (decision.active &&
            (now_ms < last_apply_ms || now_ms - last_apply_ms >= 1000)) {
            outputs_changed = true;
            last_apply_ms = now_ms;
        }
        const bool drive_outputs = decision.active || last_decision.active ||
                                   decision.alarm_level >= KITCHEN_ALARM_L3;

        if (xSemaphoreTake(s_ctx.lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_ctx.snapshot.alarm_level = decision.alarm_level;
            s_ctx.snapshot.alarm_flags = decision.alarm_flags;
            s_ctx.snapshot.safety_index = decision.safety_index;
            s_ctx.snapshot.stove_on = decision.stove_on;
            if (drive_outputs) {
                s_ctx.snapshot.fan_on = decision.fan_on;
                s_ctx.snapshot.valve_open = decision.valve_open;
                s_ctx.snapshot.window_position_pct =
                    decision.window_position_pct;
            }
            xSemaphoreGive(s_ctx.lock);
        }

        if (outputs_changed && drive_outputs) {
            actuator_manager_set_window(decision.window_position_pct);
            actuator_manager_set_fan(decision.fan_on);
            actuator_manager_set_valve(decision.valve_open);
        }

        const bool desired_buzzer = decision.buzzer_on && !muted;
        actuator_manager_set_buzzer(desired_buzzer);

        bool led_on = decision.status_led_on;
        if (decision.status_led_blink_hz > 0) {
            const uint32_t half_period_ms =
                500U / decision.status_led_blink_hz;
            led_on = ((now_ms / half_period_ms) % 2U) == 0;
        }
        if (led_on != last_led_on) {
            actuator_manager_set_led(led_on);
            last_led_on = led_on;
        }

        if (decision.alarm_level != last_alarm_level) {
            emit_event(KITCHEN_EVENT_ALARM_CHANGED, decision.alarm_level,
                       snapshot.mode,
                       static_cast<int32_t>(decision.alarm_level),
                       decision.alarm_level == KITCHEN_ALARM_NONE
                           ? "alarm cleared"
                           : "alarm changed");
            last_alarm_level = decision.alarm_level;
        }

        if (drive_outputs && decision.valve_open != last_valve_open) {
            emit_event(KITCHEN_EVENT_VALVE_CHANGED, decision.alarm_level,
                       snapshot.mode, decision.valve_open ? 1 : 0,
                       decision.valve_open ? "valve open" : "valve closed");
            last_valve_open = decision.valve_open;
        }

        const bool fault_now = (decision.alarm_flags &
                                safety_monitor::kAlarmFlagFault) != 0;
        const bool fault_before = (last_alarm_flags &
                                   safety_monitor::kAlarmFlagFault) != 0;
        if (fault_now != fault_before) {
            emit_event(KITCHEN_EVENT_SENSOR_FAULT, decision.alarm_level,
                       snapshot.mode,
                       static_cast<int32_t>(decision.alarm_flags),
                       fault_now ? "sensor fault" : "sensor fault cleared");
        }
        last_alarm_flags = decision.alarm_flags;

        if ((now_ms - last_log_ms) >= 5000 &&
            (decision.active || decision.alarm_flags != 0)) {
            ESP_LOGI(TAG,
                     "safety level=%d flags=%u active=%d window=%u fan=%d valve=%d buzzer=%d index=%u",
                     static_cast<int>(decision.alarm_level),
                     static_cast<unsigned>(decision.alarm_flags),
                     decision.active,
                     static_cast<unsigned>(decision.window_position_pct),
                     decision.fan_on,
                     decision.valve_open,
                     desired_buzzer,
                     static_cast<unsigned>(decision.safety_index));
            last_log_ms = now_ms;
        }

        last_decision = decision;
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

    const bool safety_override =
        (s_ctx.snapshot.alarm_flags &
         (safety_monitor::kAlarmFlagGas |
          safety_monitor::kAlarmFlagStove |
          safety_monitor::kAlarmFlagHumidity)) != 0;
    const bool manual_actuator_command =
        message.action == KITCHEN_ACTION_SET_WINDOW ||
        message.action == KITCHEN_ACTION_SET_FAN ||
        message.action == KITCHEN_ACTION_SET_VALVE;
    if (safety_override && manual_actuator_command) {
        xSemaphoreGive(s_ctx.lock);
        ESP_LOGW(TAG, "manual actuator command blocked by active safety rule");
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
            s_ctx.mute_until_ms =
                static_cast<uint32_t>(esp_timer_get_time() / 1000) +
                kAlarmMuteMs;
            break;
        case KITCHEN_ACTION_CLOSE_VALVE:
            s_ctx.snapshot.valve_open = false;
            break;
        case KITCHEN_ACTION_SET_VALVE:
            s_ctx.snapshot.valve_open = (message.value != 0);
            break;
        case KITCHEN_ACTION_RESET_ALARM:
            s_ctx.mute_until_ms = 0;
            safety_monitor::request_reset();
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
        case KITCHEN_ACTION_SET_WINDOW:
            actuator_manager_set_window(static_cast<uint8_t>(message.value));
            break;
        case KITCHEN_ACTION_SET_FAN:
            actuator_manager_set_fan(message.value != 0);
            break;
        case KITCHEN_ACTION_MUTE_ALARM:
            actuator_manager_set_buzzer(false);
            break;
        case KITCHEN_ACTION_CLOSE_VALVE:
            actuator_manager_set_valve(false);
            emit_event(KITCHEN_EVENT_VALVE_CHANGED, level, mode,
                       valve_open ? 1 : 0, "valve closed");
            break;
        case KITCHEN_ACTION_SET_VALVE:
            actuator_manager_set_valve(valve_open);
            emit_event(KITCHEN_EVENT_VALVE_CHANGED, level, mode,
                       valve_open ? 1 : 0,
                       valve_open ? "valve open" : "valve closed");
            break;
        case KITCHEN_ACTION_RESET_ALARM:
            actuator_manager_set_buzzer(false);
            emit_event(KITCHEN_EVENT_ALARM_CHANGED, level, mode, 0,
                       "alarm reset requested");
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
        bool gas2_valid = false;
        bool gas4_valid = false;
        esp_err_t mq_err = sensor_manager_read_mq(
            &adc_mq2, &adc_mq4, &ratio_mq2, &ratio_mq4, &gas_valid,
            &gas2_valid, &gas4_valid);

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
                s_ctx.snapshot.gas2_valid = gas2_valid;
                s_ctx.snapshot.gas4_valid = gas4_valid;
            } else {
                s_ctx.snapshot.gas_valid = false;
                s_ctx.snapshot.gas2_valid = false;
                s_ctx.snapshot.gas4_valid = false;
            }
            xSemaphoreGive(s_ctx.lock);
        }

        // Status LED is owned by safety_task: steady when SHT30 is valid,
        // blinking on L2/L3 alarms.

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
                         "MQ2 adc=%u ratio=%d.%02d valid=%d MQ4 adc=%u ratio=%d.%02d valid=%d gas_valid=%d",
                         static_cast<unsigned>(adc_mq2),
                         ratio2_x100 / 100, std::abs(ratio2_x100 % 100),
                         gas2_valid,
                         static_cast<unsigned>(adc_mq4),
                         ratio4_x100 / 100, std::abs(ratio4_x100 % 100),
                         gas4_valid, gas_valid);
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

void radar_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "radar task started");

    while (true) {
        radar_manager_poll();

        radar_manager_data_t data = {};
        radar_manager_get(&data);

        if (xSemaphoreTake(s_ctx.lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_ctx.snapshot.human_present = data.human_present;
            s_ctx.snapshot.human_valid = data.human_valid;
            s_ctx.snapshot.radar_online = data.radar_online;
            s_ctx.snapshot.human_distance_cm = data.distance_cm;
            xSemaphoreGive(s_ctx.lock);
        }

        vTaskDelay(pdMS_TO_TICKS(kRadarPeriodMs));
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
    s_ctx.snapshot.valve_open = actuator_manager_valve_default_open();
    s_ctx.snapshot.safety_index = 100;
    s_ctx.snapshot.valid = false;
    s_ctx.initialized = true;

    safety_monitor::init();

    if (sensor_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "SHT30 not ready; continuing without sensor");
    }
    if (actuator_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "status LED init failed; continuing");
    }
    if (key_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "key init failed; continuing");
    }
    if (radar_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "LD2402 init failed; continuing");
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

    ok = xTaskCreatePinnedToCore(
        radar_task, "kitchen_radar", kRadarTaskStack, nullptr,
        kRadarTaskPriority, &s_ctx.radar_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create radar task");
        vTaskDelete(s_ctx.key_task);
        vTaskDelete(s_ctx.sensor_task);
        vTaskDelete(s_ctx.control_task);
        vTaskDelete(s_ctx.safety_task);
        s_ctx.key_task = nullptr;
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
    if (action < KITCHEN_ACTION_SET_MODE || action > KITCHEN_ACTION_SET_VALVE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (action == KITCHEN_ACTION_SET_WINDOW && (value < 0 || value > 100)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((action == KITCHEN_ACTION_SET_MODE || action == KITCHEN_ACTION_SET_FAN ||
         action == KITCHEN_ACTION_SET_VALVE) &&
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
