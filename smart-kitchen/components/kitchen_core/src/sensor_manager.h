#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t sensor_manager_init(void);
bool sensor_manager_is_ready(void);
esp_err_t sensor_manager_read(float* temperature_c, float* humidity_rh);
esp_err_t sensor_manager_read_mlx(float* object_c);
esp_err_t sensor_manager_read_mq(uint16_t* adc_mq2, uint16_t* adc_mq4,
                                 float* ratio_mq2, float* ratio_mq4,
                                 bool* gas_valid, bool* gas2_valid,
                                 bool* gas4_valid);
