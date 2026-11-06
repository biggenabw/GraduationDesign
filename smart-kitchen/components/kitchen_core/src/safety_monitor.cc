// Safety monitor skeleton.
//
// Planned responsibilities:
// - Gas P0/P1 grading and valve/fan/window interlock
// - Stove unattended timeout (radar + MLX90614)
// - Humidity window hysteresis
// - Sensor fault degradation (MQ warm-up, radar heartbeat, I2C errors)
// - Alarm level arbitration and latch/reset policy
//
// Pure decision logic should be host-testable; hardware access stays in
// sensor_manager/actuator_manager.
