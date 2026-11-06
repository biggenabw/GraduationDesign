#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool human_present;
    bool human_valid;
    bool radar_online;
    uint16_t distance_cm;
} radar_manager_data_t;

esp_err_t radar_manager_init(void);
void radar_manager_poll(void);
void radar_manager_get(radar_manager_data_t* out_data);
