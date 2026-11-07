option(KITCHEN_BUZZER_ACTIVE_HIGH
       "Buzzer sounds on GPIO14 high; keep OFF for low-level trigger modules" OFF)

option(KITCHEN_DEMO_VALVE_DEFAULT_OPEN
       "Demo mode: energize relay and open valve at boot; turn OFF for real gas" ON)

option(KITCHEN_DEMO_ALARM_STAGING
       "Demo mode: keep L2 visible for 5 s before L3; turn OFF for real gas" ON)

idf_component_register(
    SRCS
        "src/kitchen_core.cc"
        "src/sensor_manager.cc"
        "src/actuator_manager.cc"
        "src/key_manager.cc"
        "src/radar_manager.cc"
        "src/safety_monitor.cc"
        "src/params.cc"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        driver
        esp_adc
        freertos
        esp_timer
        log
)

if(KITCHEN_BUZZER_ACTIVE_HIGH)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_BUZZER_ACTIVE_HIGH=1)
endif()

if(KITCHEN_DEMO_VALVE_DEFAULT_OPEN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_DEMO_VALVE_DEFAULT_OPEN=1)
endif()

if(KITCHEN_DEMO_ALARM_STAGING)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_DEMO_ALARM_STAGING=1)
endif()
