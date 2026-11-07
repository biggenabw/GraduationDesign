#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t actuator_manager_init(void);
bool actuator_manager_valve_default_open(void);

void actuator_manager_set_leds(bool red, bool yellow, bool green);
void actuator_manager_set_window(uint8_t percent);
void actuator_manager_set_fan(bool on);
void actuator_manager_set_valve(bool open);
void actuator_manager_set_buzzer(bool on);
