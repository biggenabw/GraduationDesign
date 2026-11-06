#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t actuator_manager_init(void);
void actuator_manager_set_led(bool on);
