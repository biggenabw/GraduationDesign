#pragma once

#include <cstdint>

#include "kitchen_core.h"

namespace safety_monitor {

constexpr uint16_t kAlarmFlagGas = 1U << 0;
constexpr uint16_t kAlarmFlagStove = 1U << 1;
constexpr uint16_t kAlarmFlagHumidity = 1U << 2;
constexpr uint16_t kAlarmFlagFault = 1U << 3;

struct SafetyInput {
    uint32_t now_ms;
    bool sht30_valid;
    float temperature_c;
    float humidity_rh;
    bool gas_valid;
    float gas_ratio_mq2;
    float gas_ratio_mq4;
    bool human_present;
    bool human_valid;
    bool radar_online;
    bool pot_valid;
    float pot_temperature_c;
    bool manual_mode;
};

struct SafetyDecision {
    bool active;
    kitchen_alarm_level_t alarm_level;
    uint16_t alarm_flags;
    uint32_t safety_index;
    bool stove_on;
    bool fan_on;
    bool valve_open;
    uint8_t window_position_pct;
    bool buzzer_on;
    bool status_led_on;
    uint8_t status_led_blink_hz;
};

void init(void);
void evaluate(const SafetyInput* input, SafetyDecision* decision);
void request_reset(void);

} // namespace safety_monitor
