#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KITCHEN_CORE_VERSION_MAJOR 0
#define KITCHEN_CORE_VERSION_MINOR 1
#define KITCHEN_CORE_VERSION_PATCH 0

typedef enum {
    KITCHEN_MODE_AUTO = 0,
    KITCHEN_MODE_MANUAL = 1,
} kitchen_mode_t;

typedef enum {
    KITCHEN_ALARM_NONE = 0,
    KITCHEN_ALARM_L1 = 1,
    KITCHEN_ALARM_L2 = 2,
    KITCHEN_ALARM_L3 = 3,
} kitchen_alarm_level_t;

typedef enum {
    KITCHEN_ACTION_SET_MODE = 0,
    KITCHEN_ACTION_SET_WINDOW = 1,
    KITCHEN_ACTION_SET_FAN = 2,
    KITCHEN_ACTION_MUTE_ALARM = 3,
    KITCHEN_ACTION_CLOSE_VALVE = 4,
    KITCHEN_ACTION_RESET_ALARM = 5,
} kitchen_action_t;

typedef enum {
    KITCHEN_KEY_UP = 0,
    KITCHEN_KEY_DOWN = 1,
    KITCHEN_KEY_OK = 2,
    KITCHEN_KEY_BACK = 3,
} kitchen_key_t;

typedef enum {
    KITCHEN_EVENT_ALARM_CHANGED = 0,
    KITCHEN_EVENT_MODE_CHANGED = 1,
    KITCHEN_EVENT_VALVE_CHANGED = 2,
    KITCHEN_EVENT_SENSOR_FAULT = 3,
    KITCHEN_EVENT_KEY = 4,
} kitchen_event_type_t;

typedef struct {
    bool valid;
    float temperature_c;
    float humidity_rh;
    float pot_temperature_c;
    float gas_ratio_mq2;
    float gas_ratio_mq4;
    bool human_present;
    bool human_valid;
    bool radar_online;
    uint16_t human_distance_cm;
    bool stove_on;
    bool fan_on;
    bool valve_open;
    uint8_t window_position_pct;
    kitchen_alarm_level_t alarm_level;
    kitchen_mode_t mode;
    uint32_t safety_index;
    uint32_t uptime_ms;
} kitchen_snapshot_t;

typedef struct {
    kitchen_event_type_t type;
    kitchen_alarm_level_t alarm_level;
    kitchen_mode_t mode;
    int32_t value;
    char message[64];
} kitchen_event_t;

typedef void (*kitchen_event_cb_t)(const kitchen_event_t *event, void *user_ctx);

/**
 * Initialize internal queues, mutex and default snapshot.
 * Does not touch sensors/actuators yet.
 */
esp_err_t kitchen_core_init(void);

/**
 * Start the safety and control tasks. Call after kitchen_core_init().
 */
esp_err_t kitchen_core_start(void);

/**
 * Copy the latest snapshot. Safe to call from any task.
 */
esp_err_t kitchen_core_get_snapshot(kitchen_snapshot_t *out_snapshot);

/**
 * Post a control request from MCP/cloud/UI. Non-blocking.
 */
esp_err_t kitchen_core_request(kitchen_action_t action, int value);

/**
 * Post a normalized key event from the board button adapter. Non-blocking.
 */
esp_err_t kitchen_core_post_key(kitchen_key_t key, bool long_press);

/**
 * Register a single event callback. The callback runs in the kitchen control
 * task context; keep it short and never block on audio/network.
 */
void kitchen_core_set_event_callback(kitchen_event_cb_t callback, void *user_ctx);

const char *kitchen_core_version(void);

#ifdef __cplusplus
}
#endif
