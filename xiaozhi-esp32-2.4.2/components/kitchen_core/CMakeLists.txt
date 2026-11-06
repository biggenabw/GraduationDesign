idf_component_register(
    SRCS
        "src/kitchen_core.cc"
        "src/sensor_manager.cc"
        "src/actuator_manager.cc"
        "src/safety_monitor.cc"
        "src/params.cc"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        freertos
        esp_timer
        log
)
