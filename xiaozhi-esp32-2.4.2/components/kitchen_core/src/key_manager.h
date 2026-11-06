#pragma once

#include "esp_err.h"

esp_err_t key_manager_init(void);
void key_manager_poll(void);
