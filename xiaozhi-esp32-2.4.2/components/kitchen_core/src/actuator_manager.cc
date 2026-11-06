// Actuator manager skeleton.
//
// Planned responsibilities:
// - SG90 window servo (GPIO11, 50 Hz PWM)
// - Demo gas valve via relay (GPIO12, active-low module)
// - L9110 fan (GPIO13, INB tied to GND)
// - Piezo buzzer (GPIO14, LEDC 2.7 kHz)
// - Red/yellow/green LEDs (GPIO17/18/21)
//
// All actuator commands must come from kitchen_core control_task; never
// block or delay inside an MCP callback.
