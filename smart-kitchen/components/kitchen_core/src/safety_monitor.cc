#include "safety_monitor.h"

#include <cmath>

namespace safety_monitor {
namespace {

#if defined(KITCHEN_DEMO_AUTO_CLEAR)
// Demo profile: align L2 with the auto-clear ceiling so outputs stop as soon
// as the ratio falls below 1.6.
constexpr float kGasL2Ratio = 1.6f;
#elif defined(KITCHEN_DEMO_ALARM_STAGING)
constexpr float kGasL2Ratio = 1.3f;
#else
constexpr float kGasL2Ratio = 1.5f;
#endif

#if defined(KITCHEN_DEMO_ALARM_STAGING)
constexpr float kGasL3Ratio = 2.5f;
constexpr uint32_t kGasConfirmMs = 2000;
constexpr uint32_t kSmokeConfirmMs = 2000;
constexpr uint32_t kAlarmStageMs = 5000;
#else
constexpr float kGasL3Ratio = 2.0f;
constexpr uint32_t kGasConfirmMs = 2000;
constexpr uint32_t kSmokeConfirmMs = 3000;
constexpr uint32_t kAlarmStageMs = 0;
#endif

#if defined(KITCHEN_DEMO_AUTO_CLEAR)
constexpr float kGasResetRatio = 1.6f;
constexpr uint32_t kGasResetHoldMs = 10000;
#else
constexpr float kGasResetRatio = 1.3f;
constexpr uint32_t kGasResetHoldMs = 60000;
#endif

constexpr float kHumidityHighRh = 75.0f;
constexpr float kHumidityLowRh = 65.0f;
constexpr float kHumidityRateMaxRhPerSec = 15.0f;
#if defined(KITCHEN_DEMO_FAST_HUMIDITY)
constexpr uint32_t kHumidityHoldMs = 5000;
#else
constexpr uint32_t kHumidityHoldMs = 30000;
#endif
constexpr uint32_t kHumiditySuspectMs = 30000;

// High temperature only drives the fan; it does not open the window.
constexpr float kFanHighTempC = 35.0f;
constexpr float kFanLowTempC = 33.0f;

constexpr float kPotOnC = 60.0f;
constexpr float kPotOffC = 50.0f;
constexpr uint32_t kPresenceAbsentMs = 60000;
constexpr uint32_t kUnattendedMs = 600000;
constexpr uint32_t kAlarmToValveMs = 780000;
constexpr uint32_t kStoveMaxOnMs = 1800000;
constexpr uint32_t kPotOffHoldMs = 30000;

struct State {
    bool initialized = false;

    bool gas2_latched = false;
    bool gas4_latched = false;
    bool gas2_l2_active = false;
    bool gas4_l2_active = false;
    uint32_t gas2_l2_since_ms = 0;
    uint32_t gas4_l2_since_ms = 0;
    uint32_t gas2_l3_since_ms = 0;
    uint32_t gas4_l3_since_ms = 0;
    uint32_t gas_reset_since_ms = 0;
    bool gas_reset_ready = false;

    bool humidity_window_active = false;
    uint32_t humidity_high_since_ms = 0;
    uint32_t humidity_low_since_ms = 0;
    uint32_t humidity_suspect_until_ms = 0;
    float last_humidity_rh = 0.0f;
    uint32_t last_humidity_ms = 0;

    bool high_temp_fan_active = false;

    bool stove_latched = false;
    bool stove_valve_cut = false;
    bool stove_absolute = false;
    uint32_t absent_since_ms = 0;
    uint32_t unattended_since_ms = 0;
    uint32_t pot_on_since_ms = 0;
    uint32_t pot_off_since_ms = 0;
};

State s_state;
volatile bool s_reset_requested = false;

float Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

float MaxFloat(float a, float b)
{
    return a > b ? a : b;
}

uint32_t ElapsedSince(uint32_t since_ms, uint32_t now_ms)
{
    if (since_ms == 0 || now_ms < since_ms) {
        return 0;
    }
    return now_ms - since_ms;
}

void MarkSince(uint32_t& since_ms, uint32_t now_ms)
{
    if (since_ms == 0 || now_ms < since_ms) {
        since_ms = now_ms;
    }
}

} // namespace

void init(void)
{
    s_state = State{};
    s_state.initialized = true;
    s_reset_requested = false;
}

void request_reset(void)
{
    s_reset_requested = true;
}

void evaluate(const SafetyInput* input, SafetyDecision* decision)
{
    if (input == nullptr || decision == nullptr) {
        return;
    }
    if (!s_state.initialized) {
        init();
    }

    const SafetyInput& in = *input;
    SafetyDecision out = {};
    out.alarm_level = KITCHEN_ALARM_NONE;
    out.safety_index = 100;
    out.valve_open = true;
    out.status_led_on = in.sht30_valid;

    const bool radar_usable = in.radar_online && in.human_valid;
    const bool human_present_fail_safe = radar_usable ? in.human_present : true;

    uint16_t flags = 0;
    if (!in.sht30_valid) {
        flags |= kAlarmFlagFault;
    }
    if (!in.pot_valid) {
        flags |= kAlarmFlagFault;
    }
    if (!in.radar_online) {
        flags |= kAlarmFlagFault;
    }

    const float ratio2 = in.gas2_valid ? in.gas_ratio_mq2 : 0.0f;
    const float ratio4 = in.gas4_valid ? in.gas_ratio_mq4 : 0.0f;
    const float max_gas_ratio = MaxFloat(ratio2, ratio4);

    // Gas channels are independent:
    // - MQ4: methane/combustible gas, 2 s confirmation
    // - MQ2: smoke/broad-spectrum gas, 3 s confirmation, never downgraded by
    //   human presence (smoke is dangerous whether someone is there or not).
    const bool mq2_l2_condition = in.gas2_valid && ratio2 >= kGasL2Ratio;
#if defined(KITCHEN_MQ2_HUMAN_GATING)
    // Cooking smoke with a confirmed person present stays at L2. Only an
    // explicitly absent person allows MQ2 smoke to escalate to L3.
    const bool mq2_l3_human_ok = !human_present_fail_safe;
#else
    const bool mq2_l3_human_ok = true;
#endif
    const bool mq2_l3_condition =
        in.gas2_valid && ratio2 >= kGasL3Ratio && mq2_l3_human_ok;
    const bool mq4_l2_condition = in.gas4_valid && ratio4 >= kGasL2Ratio;
    const bool mq4_l3_condition = in.gas4_valid && ratio4 >= kGasL3Ratio;

    if (mq2_l2_condition) {
        MarkSince(s_state.gas2_l2_since_ms, in.now_ms);
        s_state.gas2_l2_active =
            ElapsedSince(s_state.gas2_l2_since_ms, in.now_ms) >=
            kSmokeConfirmMs;
    } else {
        s_state.gas2_l2_since_ms = 0;
        s_state.gas2_l2_active = false;
    }

    if (mq4_l2_condition) {
        MarkSince(s_state.gas4_l2_since_ms, in.now_ms);
        s_state.gas4_l2_active =
            ElapsedSince(s_state.gas4_l2_since_ms, in.now_ms) >=
            kGasConfirmMs;
    } else {
        s_state.gas4_l2_since_ms = 0;
        s_state.gas4_l2_active = false;
    }

#if defined(KITCHEN_DEMO_ALARM_STAGING)
    const bool gas2_stage_ready =
        s_state.gas2_l2_since_ms != 0 &&
        ElapsedSince(s_state.gas2_l2_since_ms, in.now_ms) >=
            (kSmokeConfirmMs + kAlarmStageMs);
    const bool gas4_stage_ready =
        s_state.gas4_l2_since_ms != 0 &&
        ElapsedSince(s_state.gas4_l2_since_ms, in.now_ms) >=
            (kGasConfirmMs + kAlarmStageMs);
#else
    const bool gas2_stage_ready = true;
    const bool gas4_stage_ready = true;
#endif

    if (mq2_l3_condition) {
        MarkSince(s_state.gas2_l3_since_ms, in.now_ms);
        if (gas2_stage_ready &&
            ElapsedSince(s_state.gas2_l3_since_ms, in.now_ms) >=
                kSmokeConfirmMs) {
            s_state.gas2_latched = true;
        }
    } else {
        s_state.gas2_l3_since_ms = 0;
    }

    if (mq4_l3_condition) {
        MarkSince(s_state.gas4_l3_since_ms, in.now_ms);
        if (gas4_stage_ready &&
            ElapsedSince(s_state.gas4_l3_since_ms, in.now_ms) >=
                kGasConfirmMs) {
            s_state.gas4_latched = true;
        }
    } else {
        s_state.gas4_l3_since_ms = 0;
    }

    const bool gas_latched = s_state.gas2_latched || s_state.gas4_latched;
    if (gas_latched) {
        const bool gas_any_valid = in.gas2_valid || in.gas4_valid;
        const bool gas_all_valid_low =
            (!in.gas2_valid || in.gas_ratio_mq2 < kGasResetRatio) &&
            (!in.gas4_valid || in.gas_ratio_mq4 < kGasResetRatio);

        if (gas_any_valid && gas_all_valid_low) {
            MarkSince(s_state.gas_reset_since_ms, in.now_ms);
            if (ElapsedSince(s_state.gas_reset_since_ms, in.now_ms) >=
                kGasResetHoldMs) {
                s_state.gas_reset_ready = true;
            }
        } else {
            s_state.gas_reset_since_ms = 0;
            s_state.gas_reset_ready = false;
        }
    }

    // Humidity: reject very fast jumps (breath/steam), then 30 s hysteresis.
    if (in.sht30_valid) {
        if (s_state.last_humidity_ms == 0 ||
            in.now_ms < s_state.last_humidity_ms) {
            s_state.last_humidity_rh = in.humidity_rh;
            s_state.last_humidity_ms = in.now_ms;
        } else if (in.now_ms - s_state.last_humidity_ms >= 1000) {
            const float delta =
                std::fabs(in.humidity_rh - s_state.last_humidity_rh);
            if (delta > kHumidityRateMaxRhPerSec) {
                s_state.humidity_suspect_until_ms =
                    in.now_ms + kHumiditySuspectMs;
            }
            s_state.last_humidity_rh = in.humidity_rh;
            s_state.last_humidity_ms = in.now_ms;
        }
    } else {
        s_state.last_humidity_ms = 0;
        s_state.humidity_high_since_ms = 0;
        s_state.humidity_low_since_ms = 0;
        s_state.humidity_window_active = false;
    }

    const bool humidity_suspect =
        in.now_ms < s_state.humidity_suspect_until_ms;

    if (!in.manual_mode && in.sht30_valid && !humidity_suspect &&
        in.humidity_rh >= kHumidityHighRh) {
        MarkSince(s_state.humidity_high_since_ms, in.now_ms);
        if (ElapsedSince(s_state.humidity_high_since_ms, in.now_ms) >=
            kHumidityHoldMs) {
            s_state.humidity_window_active = true;
        }
    } else {
        s_state.humidity_high_since_ms = 0;
    }

    if (in.sht30_valid && !humidity_suspect &&
        in.humidity_rh <= kHumidityLowRh) {
        MarkSince(s_state.humidity_low_since_ms, in.now_ms);
        if (ElapsedSince(s_state.humidity_low_since_ms, in.now_ms) >=
            kHumidityHoldMs) {
            s_state.humidity_window_active = false;
        }
    } else {
        s_state.humidity_low_since_ms = 0;
    }

    // High temperature is a fan-only condition; humidity alone never starts
    // the fan. 33 C release prevents chatter around the 35 C threshold.
    if (in.sht30_valid) {
        if (in.temperature_c >= kFanHighTempC) {
            s_state.high_temp_fan_active = true;
        } else if (in.temperature_c <= kFanLowTempC) {
            s_state.high_temp_fan_active = false;
        }
    } else {
        s_state.high_temp_fan_active = false;
    }

    // Stove: MLX90614 is optional; when absent this logic stays idle.
    const bool pot_on = in.pot_valid && in.pot_temperature_c >= kPotOnC;
    out.stove_on = pot_on;

    if (pot_on) {
        MarkSince(s_state.pot_on_since_ms, in.now_ms);
        s_state.pot_off_since_ms = 0;
        if (ElapsedSince(s_state.pot_on_since_ms, in.now_ms) >=
            kStoveMaxOnMs) {
            s_state.stove_absolute = true;
        }
    } else {
        s_state.pot_on_since_ms = 0;
        s_state.unattended_since_ms = 0;
        if (!in.pot_valid || in.pot_temperature_c <= kPotOffC) {
            MarkSince(s_state.pot_off_since_ms, in.now_ms);
        } else {
            s_state.pot_off_since_ms = 0;
        }
    }

    const bool absent_now = radar_usable && !in.human_present;
    if (absent_now) {
        MarkSince(s_state.absent_since_ms, in.now_ms);
    } else {
        s_state.absent_since_ms = 0;
    }
    const bool confirmed_absent =
        s_state.absent_since_ms != 0 &&
        ElapsedSince(s_state.absent_since_ms, in.now_ms) >= kPresenceAbsentMs;

    if (pot_on && confirmed_absent) {
        MarkSince(s_state.unattended_since_ms, in.now_ms);
    } else if (!pot_on || !confirmed_absent) {
        s_state.unattended_since_ms = 0;
    }

    const uint32_t unattended_ms =
        ElapsedSince(s_state.unattended_since_ms, in.now_ms);
    if (unattended_ms >= kUnattendedMs) {
        s_state.stove_latched = true;
    }
    if (unattended_ms >= kAlarmToValveMs || s_state.stove_absolute) {
        s_state.stove_valve_cut = true;
    }

    if (!pot_on && human_present_fail_safe &&
        ElapsedSince(s_state.pot_off_since_ms, in.now_ms) >= kPotOffHoldMs) {
        s_state.stove_latched = false;
        s_state.stove_absolute = false;
    }

#if defined(KITCHEN_DEMO_AUTO_CLEAR)
    // Demo convenience: no BACK press needed once all valid channels have
    // stayed below the reset ceiling for the hold time.
    if (s_state.gas_reset_ready) {
        s_reset_requested = true;
    }
#endif

    // A reset request only clears latches when the documented conditions hold.
    if (s_reset_requested) {
        if ((s_state.gas2_latched || s_state.gas4_latched) &&
            s_state.gas_reset_ready) {
            s_state.gas2_latched = false;
            s_state.gas4_latched = false;
            s_state.gas2_l2_active = false;
            s_state.gas4_l2_active = false;
            s_state.gas2_l2_since_ms = 0;
            s_state.gas4_l2_since_ms = 0;
            s_state.gas2_l3_since_ms = 0;
            s_state.gas4_l3_since_ms = 0;
            s_state.gas_reset_ready = false;
            s_state.gas_reset_since_ms = 0;
        }
        if (s_state.stove_valve_cut && !pot_on && human_present_fail_safe) {
            s_state.stove_valve_cut = false;
            s_state.stove_latched = false;
            s_state.stove_absolute = false;
        }
        s_reset_requested = false;
    }

    const bool gas2_active =
        s_state.gas2_latched || s_state.gas2_l2_active;
    const bool gas4_active =
        s_state.gas4_latched || s_state.gas4_l2_active;
    const bool gas_active = gas2_active || gas4_active;
    const bool stove_active =
        s_state.stove_latched || s_state.stove_absolute ||
        s_state.stove_valve_cut;
    const bool humidity_active = s_state.humidity_window_active;

    if (gas4_active) {
        flags |= kAlarmFlagGas;
    }
    if (gas2_active) {
        flags |= kAlarmFlagSmoke;
    }
    if (stove_active) {
        flags |= kAlarmFlagStove;
    }
    if (humidity_active) {
        flags |= kAlarmFlagHumidity;
    }

    out.active = gas_active || stove_active || humidity_active;
    out.alarm_flags = flags;
    out.alarm_level = KITCHEN_ALARM_NONE;
    if (s_state.gas2_latched || s_state.gas4_latched ||
        s_state.stove_latched || s_state.stove_absolute ||
        s_state.stove_valve_cut) {
        out.alarm_level = KITCHEN_ALARM_L3;
    } else if (s_state.gas2_l2_active || s_state.gas4_l2_active) {
        out.alarm_level = KITCHEN_ALARM_L2;
    }

    out.window_position_pct = out.active ? 100 : 0;
    // Humidity opens the window but does not start the fan. The fan runs for
    // gas/stove safety or when the kitchen temperature is above 35 C.
    out.fan_on = gas_active || stove_active ||
                 s_state.high_temp_fan_active;

    bool gas_valve_cut = s_state.gas4_latched;
#if defined(KITCHEN_MQ2_CUT_VALVE)
    gas_valve_cut = gas_valve_cut || s_state.gas2_latched;
#endif
    out.valve_open =
        !(gas_valve_cut || s_state.stove_valve_cut ||
          s_state.stove_absolute);
    out.buzzer_on = (out.alarm_level >= KITCHEN_ALARM_L2);
    out.status_led_blink_hz =
        out.alarm_level == KITCHEN_ALARM_L3
            ? 2
            : (out.alarm_level == KITCHEN_ALARM_L2 ? 1 : 0);

    // Safety index: 100 is best, penalties lower it.
    float gas_pen = 0.0f;
    if (in.gas2_valid || in.gas4_valid) {
        gas_pen = 50.0f * Clamp((max_gas_ratio - 1.0f) /
                                    (kGasL3Ratio - 1.0f),
                                0.0f, 1.0f);
    }
    float pot_pen = 0.0f;
    if (in.pot_valid) {
        const float pot_factor = human_present_fail_safe ? 0.4f : 1.0f;
        pot_pen = 25.0f * Clamp((in.pot_temperature_c - 50.0f) / 40.0f,
                                0.0f, 1.0f) *
                  pot_factor;
    }
    float env_pen = 0.0f;
    if (in.sht30_valid) {
        env_pen = 10.0f * Clamp((in.humidity_rh - 70.0f) / 30.0f, 0.0f, 1.0f) +
                  5.0f * Clamp((in.temperature_c - 32.0f) / 8.0f, 0.0f, 1.0f);
    }
    float sys_pen = (flags & kAlarmFlagFault) ? 4.0f : 0.0f;
    if (!radar_usable) {
        sys_pen += 3.0f;
    }
    if (in.manual_mode) {
        sys_pen += 2.0f;
    }
    const float index =
        Clamp(100.0f - gas_pen - pot_pen - env_pen - sys_pen, 0.0f, 100.0f);
    out.safety_index = static_cast<uint32_t>(index + 0.5f);

    *decision = out;
}

} // namespace safety_monitor
