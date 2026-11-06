#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t sensor_manager_init(void);
bool sensor_manager_is_ready(void);
esp_err_t sensor_manager_read(float* temperature_c, float* humidity_rh);
