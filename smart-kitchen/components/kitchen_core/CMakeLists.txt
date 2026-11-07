option(KITCHEN_BUZZER_ACTIVE_HIGH
       "Buzzer sounds on GPIO14 high; keep OFF for low-level trigger modules" OFF)

option(KITCHEN_DEMO_VALVE_DEFAULT_OPEN
       "Demo mode: energize relay and open valve at boot; turn OFF for real gas" ON)

option(KITCHEN_DEMO_ALARM_STAGING
       "Demo mode: keep L2 visible for 5 s before L3; turn OFF for real gas" ON)

option(KITCHEN_DEMO_AUTO_CLEAR
       "Demo mode: auto-clear L3 after 10 s below ratio 1.6; turn OFF for real gas" ON)

option(KITCHEN_MQ2_CUT_VALVE
       "Let MQ2 smoke L3 close the demo valve; keep OFF for cooking smoke" OFF)

option(KITCHEN_MQ2_HUMAN_GATING
       "MQ2 smoke L3 only when the radar confirms nobody; human present stays L2" ON)

option(KITCHEN_DEMO_FAST_HUMIDITY
       "Demo mode: use 5 s humidity hold instead of 30 s" ON)

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

if(KITCHEN_DEMO_AUTO_CLEAR)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_DEMO_AUTO_CLEAR=1)
endif()

if(KITCHEN_MQ2_CUT_VALVE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_MQ2_CUT_VALVE=1)
endif()

if(KITCHEN_MQ2_HUMAN_GATING)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_MQ2_HUMAN_GATING=1)
endif()

if(KITCHEN_DEMO_FAST_HUMIDITY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE KITCHEN_DEMO_FAST_HUMIDITY=1)
endif()
