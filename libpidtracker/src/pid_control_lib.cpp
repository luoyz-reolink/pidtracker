/*** 
 * @Author: lkyezi
 * @Date: 2025-10-30 18:20:20
 * @LastEditTime: 2025-11-06 11:41:39
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */
#include "pid_control_lib.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>

struct PIDController {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float min_output = 0.0f;
    float max_output = 0.0f;
    float output = 0.0f;
    float integral_min = 0.0f;
    float integral_max = 0.0f;
    float derivative_filtered = 0.0f;
    float deadband = 0.0f;
    float d_filter_alpha = 0.0f;
    bool d_initialized = false;
    bool d_filter_enabled = true;
};

struct MotorControl {
    AxisType_E axis = AXIS_HORIZONTAL;
    uint8_t enabled = 0U;
    uint8_t level = 0U;
    uint8_t direction = 1U;
    uint32_t min_deg = 0U;
    uint32_t max_deg = 0U;
    MotorSetSpeedCallback set_speed = nullptr;
    MotorDisableCallback disable_motor = nullptr;
};

struct TargetPredictionState {
    float current_deg = 0.0f;
    float previous_deg = 0.0f;
    float filtered_velocity_deg_per_sec = 0.0f;
    bool has_previous_sample = false;
    float guard_ratio = 0.0f;
    uint32_t guard_attempts = 0U;
    uint32_t guard_rejects = 0U;
    bool prediction_enabled = true;
    bool guard_enabled = true;
    bool adaptive_guard_enabled = true;
};

struct PIDControlLib {
    PIDControlConfig config{};
    PIDController pid_controllers[AXIS_TOTAL]{};
    MotorControl motor_controllers[AXIS_TOTAL]{};
    TargetPredictionState prediction_states[AXIS_TOTAL]{};
    HardwareInterface hw_interface{};
    PIDControlStats stats{};
    uint64_t last_control_pts = 0U;
    bool initialized = false;
    bool running = false;
};

namespace {
constexpr float kMinDtSeconds = 0.01f;
constexpr float kMaxDtSeconds = 0.25f;

inline float AbsF(float value) {
    return std::fabs(value);
}

inline float ClampF(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

inline float PixelsToHorizontalDeg(float pixel_x, const SystemParameters& sys) {
    const float centered = pixel_x - (sys.frame_width_pixels * 0.5f);
    const float deg_per_pixel = sys.horizontal_fov_deg / sys.frame_width_pixels;
    return centered * deg_per_pixel;
}

inline float PixelsToVerticalDeg(float pixel_y, const SystemParameters& sys) {
    const float centered = pixel_y - (sys.frame_height_pixels * 0.5f);
    const float deg_per_pixel = sys.vertical_fov_deg / sys.frame_height_pixels;
    return centered * deg_per_pixel;
}

inline void UpdatePidFromConfig(PIDController& pid,
                                const PIDParameters& pid_config,
                                const PIDAxisParameters& axis_config,
                                float max_output,
                                float guard_epsilon) {
    pid.kp = axis_config.kp;
    pid.ki = axis_config.ki;
    pid.kd = axis_config.kd;
    pid.deadband = pid_config.deadband_deg;
    pid.d_filter_alpha = pid_config.d_filter_alpha;
    pid.max_output = max_output;
    pid.min_output = -max_output;

    if (pid.ki > guard_epsilon) {
        const float limit = (pid.max_output * pid_config.integral_ratio) / pid.ki;
        pid.integral_max = limit;
        pid.integral_min = -limit;
    } else {
        pid.integral_max = 0.0f;
        pid.integral_min = 0.0f;
    }
}

inline void UpdatePredictionFromConfig(TargetPredictionState& predict,
                                       const PredictionParameters& pred_config,
                                       const FeatureSwitches& features) {
    predict.guard_ratio = pred_config.guard_ratio;
    predict.prediction_enabled = features.enable_prediction;
    predict.guard_enabled = features.enable_prediction_guard;
    predict.adaptive_guard_enabled = features.enable_prediction_adaptive_guard;
}

inline void PredictionReset(TargetPredictionState& state, const PredictionParameters& config) {
    state.current_deg = 0.0f;
    state.previous_deg = 0.0f;
    state.filtered_velocity_deg_per_sec = 0.0f;
    state.has_previous_sample = false;
    state.guard_ratio = config.guard_ratio;
    state.guard_attempts = 0U;
    state.guard_rejects = 0U;
}

inline void MotorControlInit(MotorControl& motor, AxisType_E axis) {
    motor.axis = axis;
    motor.enabled = 0U;
    motor.level = 0U;
    motor.direction = 1U;
}

inline void MotorControlDesiredSpeedToLevel(MotorControl& motor,
                                            const SystemParameters& sys,
                                            float desired_speed) {
    uint8_t direction = (desired_speed > 0.0f) ? 0U : 1U;
    uint8_t level = 0U;
    const float abs_speed = AbsF(desired_speed);

    for (; level < sys.max_speed_level; ++level) 
    {
        if (abs_speed < sys.speed_levels[level]) {
            break;
        }
    }

    motor.level = level;
    motor.direction = direction;
    motor.enabled = (level > 0U) ? 1U : 0U;
}

inline void MotorControlSetSpeed(const MotorControl& motor) {
    if (motor.set_speed) {
        motor.set_speed(motor.axis, motor.level, motor.direction);
    }
}

inline void MotorControlDisable(const MotorControl& motor) {
    if (motor.disable_motor) {
        motor.disable_motor(motor.axis);
    }
}

float PredictionApply(TargetPredictionState& state,
                      float measured_deg,
                      float range_limit_deg,
                      float dt,
                      const PredictionParameters& config) {
    if (!state.prediction_enabled) {
        return ClampF(measured_deg, -range_limit_deg, range_limit_deg);
    }

    const float safe_dt = ClampF(dt, kMinDtSeconds, kMaxDtSeconds);
    const float previous_current = state.current_deg;

    if (!state.has_previous_sample) {
        state.previous_deg = measured_deg;
        state.current_deg = measured_deg;
        state.filtered_velocity_deg_per_sec = 0.0f;
        state.has_previous_sample = true;
        return ClampF(measured_deg, -range_limit_deg, range_limit_deg);
    }

    const float raw_velocity = (measured_deg - previous_current) / safe_dt;
    state.previous_deg = previous_current;
    state.current_deg = measured_deg;
    state.filtered_velocity_deg_per_sec = (config.velocity_alpha * raw_velocity) +
                                          ((1.0f - config.velocity_alpha) * state.filtered_velocity_deg_per_sec);

    const float lookahead_time = safe_dt;
    float predicted_shift = state.filtered_velocity_deg_per_sec * lookahead_time;
    const float max_shift = range_limit_deg * config.max_shift_factor;
    predicted_shift = ClampF(predicted_shift, -max_shift, max_shift);

    const float predicted = measured_deg + predicted_shift;

    if (state.guard_enabled) {
        float base_magnitude = AbsF(measured_deg);
        if (base_magnitude < config.center_tolerance_deg) {
            base_magnitude = config.center_tolerance_deg;
        }
        const float shift_magnitude = AbsF(predicted_shift);
        const float ratio = shift_magnitude / base_magnitude;
        const bool out_of_bounds = (predicted < -range_limit_deg) || (predicted > range_limit_deg);
        const bool excessive_shift = (ratio > state.guard_ratio);
        const bool reject_prediction = out_of_bounds || excessive_shift;

        state.guard_attempts++;
        if (reject_prediction) {
            state.guard_rejects++;
        }

        if (state.adaptive_guard_enabled && state.guard_attempts >= config.guard_adapt_window) {
            const float rejection_rate = static_cast<float>(state.guard_rejects) /
                                         static_cast<float>(state.guard_attempts);
            const float span = config.guard_ratio_max - config.guard_ratio_min;
            const float dynamic_ratio = config.guard_ratio_max - (rejection_rate * span);
            state.guard_ratio = ClampF(dynamic_ratio, config.guard_ratio_min, config.guard_ratio_max);
            state.guard_attempts = 0U;
            state.guard_rejects = 0U;
        }

        if (reject_prediction) {
            return ClampF(measured_deg, -range_limit_deg, range_limit_deg);
        }
    }

    return ClampF(predicted, -range_limit_deg, range_limit_deg);
}

float ComputePid(PIDController& pid,
                 float target,
                 float feedback,
                 float dt,
                 float guard_epsilon) {
    float safe_dt = dt;
    if (safe_dt <= 0.0f) {
        safe_dt = kMinDtSeconds;
    } else if (safe_dt > kMaxDtSeconds) {
        safe_dt = kMaxDtSeconds;
    }

    float error = target - feedback;
    if (AbsF(error) < pid.deadband) {
        error = 0.0f;
    }

    const bool allow_integral = (pid.ki > guard_epsilon);
    const bool saturating = (pid.output >= pid.max_output - guard_epsilon && error > 0.0f) ||
                            (pid.output <= pid.min_output + guard_epsilon && error < 0.0f);

    if (allow_integral && !saturating) {
        pid.integral += (error + pid.prev_error) * 0.5f * safe_dt;
        pid.integral = ClampF(pid.integral, pid.integral_min, pid.integral_max);
    }

    const float derivative_raw = (error - pid.prev_error) / safe_dt;
    float derivative_term;

    if (pid.d_filter_enabled) {
        if (pid.d_initialized) {
            pid.derivative_filtered = (pid.d_filter_alpha * derivative_raw) +
                                      ((1.0f - pid.d_filter_alpha) * pid.derivative_filtered);
        } else {
            pid.derivative_filtered = derivative_raw;
            pid.d_initialized = true;
        }
        derivative_term = pid.kd * pid.derivative_filtered;
    } else {
        derivative_term = pid.kd * derivative_raw;
    }

    const float proportional_term = pid.kp * error;
    const float integral_term = pid.ki * pid.integral;
    const float output = proportional_term + integral_term + derivative_term;

    pid.prev_error = error;
    pid.output = ClampF(output, pid.min_output, pid.max_output);

    printf("PID Compute: Error=%.3f, prev_Error=%.3f, dt=%.3f, P=%.3f, I=%.3f, D=%.3f, Output=%.3f\n",
           error,
           pid.prev_error,
           safe_dt,
           proportional_term,
           integral_term,
           derivative_term,
           pid.output);

    return pid.output;
}

}  // namespace

extern "C" {

PIDControlLib* pid_control_create(void) {
    PIDControlLib* lib = new (std::nothrow) PIDControlLib();
    if (!lib) {
        return nullptr;
    }

    pid_config_set_defaults(&lib->config);
    lib->initialized = false;
    lib->running = false;
    return lib;
}

void pid_control_destroy(PIDControlLib* lib) {
    delete lib;
}

bool pid_control_init_from_xml(PIDControlLib* lib,
                               const char* config_file,
                               const HardwareInterface* hw_interface) {
    if (!lib || !config_file || !hw_interface) {
        return false;
    }

    if (!pid_config_load_from_xml(config_file, &lib->config)) {
        return false;
    }

    lib->hw_interface = *hw_interface;

    const uint32_t max_level_index = (lib->config.system.max_speed_level > 0U)
                                         ? static_cast<uint32_t>(lib->config.system.max_speed_level - 1U)
                                         : 0U;
    const float max_output = lib->config.system.speed_levels[max_level_index];

    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        MotorControlInit(lib->motor_controllers[axis], static_cast<AxisType_E>(axis));
        lib->motor_controllers[axis].set_speed = hw_interface->motor_set_speed;
        lib->motor_controllers[axis].disable_motor = hw_interface->motor_disable;

        PredictionReset(lib->prediction_states[axis], lib->config.prediction);
        UpdatePredictionFromConfig(lib->prediction_states[axis],
                                   lib->config.prediction,
                                   lib->config.features);
    }

    UpdatePidFromConfig(lib->pid_controllers[AXIS_HORIZONTAL],
                        lib->config.pid,
                        lib->config.pid.horizontal,
                        max_output,
                        lib->config.pid.guard_epsilon);
    UpdatePidFromConfig(lib->pid_controllers[AXIS_VERTICAL],
                        lib->config.pid,
                        lib->config.pid.vertical,
                        max_output,
                        lib->config.pid.guard_epsilon);

    std::memset(&lib->stats, 0, sizeof(lib->stats));

    lib->initialized = true;
    return true;
}

bool pid_control_init_default(PIDControlLib* lib, const HardwareInterface* hw_interface) {
    if (!lib || !hw_interface) {
        return false;
    }

    pid_config_set_defaults(&lib->config);
    lib->hw_interface = *hw_interface;

    const uint32_t max_level_index = (lib->config.system.max_speed_level > 0U)
                                         ? static_cast<uint32_t>(lib->config.system.max_speed_level - 1U)
                                         : 0U;
    const float max_output = lib->config.system.speed_levels[max_level_index];

    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        MotorControlInit(lib->motor_controllers[axis], static_cast<AxisType_E>(axis));
        lib->motor_controllers[axis].set_speed = hw_interface->motor_set_speed;
        lib->motor_controllers[axis].disable_motor = hw_interface->motor_disable;

        PredictionReset(lib->prediction_states[axis], lib->config.prediction);
        UpdatePredictionFromConfig(lib->prediction_states[axis],
                                   lib->config.prediction,
                                   lib->config.features);
    }

    UpdatePidFromConfig(lib->pid_controllers[AXIS_HORIZONTAL],
                        lib->config.pid,
                        lib->config.pid.horizontal,
                        max_output,
                        lib->config.pid.guard_epsilon);
    UpdatePidFromConfig(lib->pid_controllers[AXIS_VERTICAL],
                        lib->config.pid,
                        lib->config.pid.vertical,
                        max_output,
                        lib->config.pid.guard_epsilon);

    std::memset(&lib->stats, 0, sizeof(lib->stats));

    lib->initialized = true;
    return true;
}

const PIDControlConfig* pid_control_get_config(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return nullptr;
    }
    return &lib->config;
}

bool pid_control_reload_config(PIDControlLib* lib, const char* config_file) {
    if (!lib || !lib->initialized || !config_file) {
        return false;
    }

    PIDControlConfig new_config;
    pid_config_set_defaults(&new_config);
    if (!pid_config_load_from_xml(config_file, &new_config)) {
        return false;
    }

    lib->config = new_config;

    const uint32_t max_level_index = (lib->config.system.max_speed_level > 0U)
                                         ? static_cast<uint32_t>(lib->config.system.max_speed_level - 1U)
                                         : 0U;
    const float max_output = lib->config.system.speed_levels[max_level_index];

    UpdatePidFromConfig(lib->pid_controllers[AXIS_HORIZONTAL],
                        lib->config.pid,
                        lib->config.pid.horizontal,
                        max_output,
                        lib->config.pid.guard_epsilon);
    UpdatePidFromConfig(lib->pid_controllers[AXIS_VERTICAL],
                        lib->config.pid,
                        lib->config.pid.vertical,
                        max_output,
                        lib->config.pid.guard_epsilon);

    UpdatePredictionFromConfig(lib->prediction_states[AXIS_HORIZONTAL],
                               lib->config.prediction,
                               lib->config.features);
    UpdatePredictionFromConfig(lib->prediction_states[AXIS_VERTICAL],
                               lib->config.prediction,
                               lib->config.features);

    lib->pid_controllers[AXIS_HORIZONTAL].d_filter_enabled = lib->config.features.enable_d_filter;
    lib->pid_controllers[AXIS_VERTICAL].d_filter_enabled = lib->config.features.enable_d_filter;

    return true;
}

void pid_control_step(PIDControlLib* lib) {
    if (!lib || !lib->initialized || !lib->running) {
        return;
    }

    if (!lib->hw_interface.get_target_pixel_position) {
        return;
    }

    float target_px_x = 0.0f;
    float target_px_y = 0.0f;
    uint64_t pts = 0U;
    if (!lib->hw_interface.get_target_pixel_position(&target_px_x, &target_px_y, &pts)) {
        lib->motor_controllers[AXIS_HORIZONTAL].enabled = 0U;
        lib->motor_controllers[AXIS_VERTICAL].enabled = 0U;
        MotorControlDisable(lib->motor_controllers[AXIS_HORIZONTAL]);
        MotorControlDisable(lib->motor_controllers[AXIS_VERTICAL]);
        PredictionReset(lib->prediction_states[AXIS_HORIZONTAL], lib->config.prediction);
        PredictionReset(lib->prediction_states[AXIS_VERTICAL], lib->config.prediction);
        lib->stats.target_lost_count++;
        return;
    }

    lib->stats.control_cycles++;

    const float frame_period = pts - lib->last_control_pts > 0U
                                   ? static_cast<float>(pts - lib->last_control_pts) / 1e6f
                                   : 0.0f;
    lib->last_control_pts = pts;
    const float max_horizontal_range = lib->config.system.horizontal_fov_deg * 0.5f;
    const float max_vertical_range = lib->config.system.vertical_fov_deg * 0.5f;

    const float target_horizontal_deg = PixelsToHorizontalDeg(target_px_x, lib->config.system);
    const float target_vertical_deg = PixelsToVerticalDeg(target_px_y, lib->config.system);

    const float predicted_horizontal_deg = PredictionApply(lib->prediction_states[AXIS_HORIZONTAL],
                                                           target_horizontal_deg,
                                                           max_horizontal_range,
                                                           frame_period,
                                                           lib->config.prediction);
    const float predicted_vertical_deg = PredictionApply(lib->prediction_states[AXIS_VERTICAL],
                                                         target_vertical_deg,
                                                         max_vertical_range,
                                                         frame_period,
                                                         lib->config.prediction);

    float actual_horizontal_deg = 0.0f;
    float actual_vertical_deg = 0.0f;
    if (lib->hw_interface.read_angle_deg) {
        actual_horizontal_deg = lib->hw_interface.read_angle_deg(AXIS_HORIZONTAL);
        actual_vertical_deg = lib->hw_interface.read_angle_deg(AXIS_VERTICAL);
    }

    const float horizontal_command = ComputePid(lib->pid_controllers[AXIS_HORIZONTAL],
                                                predicted_horizontal_deg,
                                                actual_horizontal_deg,
                                                frame_period,
                                                lib->config.pid.guard_epsilon);
    const float vertical_command = ComputePid(lib->pid_controllers[AXIS_VERTICAL],
                                              predicted_vertical_deg,
                                              actual_vertical_deg,
                                              frame_period,
                                              lib->config.pid.guard_epsilon);

    MotorControlDesiredSpeedToLevel(lib->motor_controllers[AXIS_HORIZONTAL], lib->config.system, horizontal_command);
    MotorControlDesiredSpeedToLevel(lib->motor_controllers[AXIS_VERTICAL], lib->config.system, vertical_command);

    // printf("Horizontal Target: %.2f deg, Predicted: %.2f deg, Actual: %.2f deg, Command: %.2f deg/s, Level: %u, Direction: %u\n",
    //        target_horizontal_deg,
    //        predicted_horizontal_deg,
    //        actual_horizontal_deg,
    //        horizontal_command,
    //        lib->motor_controllers[AXIS_HORIZONTAL].level,
    //        lib->motor_controllers[AXIS_HORIZONTAL].direction);

    // printf("Vertical   Target: %.2f deg, Predicted: %.2f deg, Actual: %.2f deg, Command: %.2f deg/s, Level: %u, Direction: %u\n",
    //        target_vertical_deg,
    //        predicted_vertical_deg,
    //        actual_vertical_deg,
    //        vertical_command,
    //        lib->motor_controllers[AXIS_VERTICAL].level,
    //        lib->motor_controllers[AXIS_VERTICAL].direction);

    MotorControlSetSpeed(lib->motor_controllers[AXIS_HORIZONTAL]);
    MotorControlSetSpeed(lib->motor_controllers[AXIS_VERTICAL]);

    lib->stats.prediction_attempts += lib->prediction_states[AXIS_HORIZONTAL].guard_attempts +
                                      lib->prediction_states[AXIS_VERTICAL].guard_attempts;
    lib->stats.prediction_rejects += lib->prediction_states[AXIS_HORIZONTAL].guard_rejects +
                                     lib->prediction_states[AXIS_VERTICAL].guard_rejects;

    lib->stats.average_error_horizontal += AbsF(predicted_horizontal_deg - actual_horizontal_deg);
    lib->stats.average_error_vertical += AbsF(predicted_vertical_deg - actual_vertical_deg);
}

void pid_control_start(PIDControlLib* lib) {
    if (lib && lib->initialized) {
        lib->running = true;
    }
}

void pid_control_stop(PIDControlLib* lib) {
    if (lib && lib->initialized) {
        lib->running = false;
        lib->motor_controllers[AXIS_HORIZONTAL].enabled = 0U;
        lib->motor_controllers[AXIS_VERTICAL].enabled = 0U;
    MotorControlDisable(lib->motor_controllers[AXIS_HORIZONTAL]);
    MotorControlDisable(lib->motor_controllers[AXIS_VERTICAL]);
    }
}

void pid_control_reset(PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return;
    }

    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        PIDController& pid = lib->pid_controllers[axis];
        pid.integral = 0.0f;
        pid.prev_error = 0.0f;
        pid.output = 0.0f;
        pid.derivative_filtered = 0.0f;
        pid.d_initialized = false;

        PredictionReset(lib->prediction_states[axis], lib->config.prediction);
    }

    std::memset(&lib->stats, 0, sizeof(lib->stats));
}

bool pid_control_set_pid_params(PIDControlLib* lib, AxisType_E axis, float kp, float ki, float kd) {
    if (!lib || !lib->initialized || axis >= AXIS_TOTAL) {
        return false;
    }

    PIDController& pid = lib->pid_controllers[axis];
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;

    if (pid.ki > lib->config.pid.guard_epsilon) {
        const float limit = (pid.max_output * lib->config.pid.integral_ratio) / pid.ki;
        pid.integral_max = limit;
        pid.integral_min = -limit;
    } else {
        pid.integral_max = 0.0f;
        pid.integral_min = 0.0f;
    }

    if (axis == AXIS_HORIZONTAL) {
        lib->config.pid.horizontal.kp = kp;
        lib->config.pid.horizontal.ki = ki;
        lib->config.pid.horizontal.kd = kd;
    } else {
        lib->config.pid.vertical.kp = kp;
        lib->config.pid.vertical.ki = ki;
        lib->config.pid.vertical.kd = kd;
    }

    printf("!!!PID params updated for axis %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", axis, kp, ki, kd);

    return true;
}

bool pid_control_get_pid_params(const PIDControlLib* lib,
                                AxisType_E axis,
                                float* kp,
                                float* ki,
                                float* kd) {
    if (!lib || !lib->initialized || axis >= AXIS_TOTAL || !kp || !ki || !kd) {
        return false;
    }

    const PIDController& pid = lib->pid_controllers[axis];
    *kp = pid.kp;
    *ki = pid.ki;
    *kd = pid.kd;
    return true;
}

bool pid_control_set_prediction_enabled(PIDControlLib* lib, bool enabled) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.features.enable_prediction = enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        lib->prediction_states[axis].prediction_enabled = enabled;
    }
    return true;
}

bool pid_control_set_d_filter_enabled(PIDControlLib* lib, bool enabled) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.features.enable_d_filter = enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        lib->pid_controllers[axis].d_filter_enabled = enabled;
    }
    return true;
}

bool pid_control_set_prediction_guard_enabled(PIDControlLib* lib, bool enabled) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.features.enable_prediction_guard = enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        lib->prediction_states[axis].guard_enabled = enabled;
    }
    return true;
}

bool pid_control_set_adaptive_guard_enabled(PIDControlLib* lib, bool enabled) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.features.enable_prediction_adaptive_guard = enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        lib->prediction_states[axis].adaptive_guard_enabled = enabled;
    }
    return true;
}

bool pid_control_get_prediction_enabled(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return false;
    }
    return lib->config.features.enable_prediction;
}

bool pid_control_get_d_filter_enabled(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return false;
    }
    return lib->config.features.enable_d_filter;
}

bool pid_control_get_prediction_guard_enabled(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return false;
    }
    return lib->config.features.enable_prediction_guard;
}

bool pid_control_get_adaptive_guard_enabled(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        return false;
    }
    return lib->config.features.enable_prediction_adaptive_guard;
}

bool pid_control_set_prediction_params(PIDControlLib* lib,
                                       float velocity_alpha,
                                       float max_shift_factor,
                                       float guard_ratio) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.prediction.velocity_alpha = velocity_alpha;
    lib->config.prediction.max_shift_factor = max_shift_factor;
    lib->config.prediction.guard_ratio = guard_ratio;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        lib->prediction_states[axis].guard_ratio = guard_ratio;
    }
    return true;
}

bool pid_control_get_prediction_params(const PIDControlLib* lib,
                                       float* velocity_alpha,
                                       float* max_shift_factor,
                                       float* guard_ratio) {
    if (!lib || !lib->initialized || !velocity_alpha || !max_shift_factor || !guard_ratio) {
        return false;
    }

    *velocity_alpha = lib->config.prediction.velocity_alpha;
    *max_shift_factor = lib->config.prediction.max_shift_factor;
    *guard_ratio = lib->config.prediction.guard_ratio;
    return true;
}

bool pid_control_set_pid_advanced_params(PIDControlLib* lib,
                                         float deadband_deg,
                                         float d_filter_alpha,
                                         float integral_ratio) {
    if (!lib || !lib->initialized) {
        return false;
    }

    lib->config.pid.deadband_deg = deadband_deg;
    lib->config.pid.d_filter_alpha = d_filter_alpha;
    lib->config.pid.integral_ratio = integral_ratio;

    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        PIDController& pid = lib->pid_controllers[axis];
        pid.deadband = deadband_deg;
        pid.d_filter_alpha = d_filter_alpha;
        if (pid.ki > lib->config.pid.guard_epsilon) {
            const float limit = (pid.max_output * integral_ratio) / pid.ki;
            pid.integral_max = limit;
            pid.integral_min = -limit;
        }
    }

    return true;
}

bool pid_control_get_pid_advanced_params(const PIDControlLib* lib,
                                         float* deadband_deg,
                                         float* d_filter_alpha,
                                         float* integral_ratio) {
    if (!lib || !lib->initialized || !deadband_deg || !d_filter_alpha || !integral_ratio) {
        return false;
    }

    *deadband_deg = lib->config.pid.deadband_deg;
    *d_filter_alpha = lib->config.pid.d_filter_alpha;
    *integral_ratio = lib->config.pid.integral_ratio;
    return true;
}

bool pid_control_get_pid_output(const PIDControlLib* lib, AxisType_E axis, float* output) {
    if (!lib || !lib->initialized || axis >= AXIS_TOTAL || !output) {
        return false;
    }

    *output = lib->pid_controllers[axis].output;
    return true;
}

bool pid_control_get_motor_level(const PIDControlLib* lib, AxisType_E axis, uint8_t* level) {
    if (!lib || !lib->initialized || axis >= AXIS_TOTAL || !level) {
        return false;
    }

    *level = lib->motor_controllers[axis].level;
    return true;
}

bool pid_control_get_prediction_state(const PIDControlLib* lib,
                                      AxisType_E axis,
                                      float* current_deg,
                                      float* velocity_deg_per_sec) {
    if (!lib || !lib->initialized || axis >= AXIS_TOTAL || !current_deg || !velocity_deg_per_sec) {
        return false;
    }

    *current_deg = lib->prediction_states[axis].current_deg;
    *velocity_deg_per_sec = lib->prediction_states[axis].filtered_velocity_deg_per_sec;
    return true;
}

bool pid_control_get_stats(const PIDControlLib* lib, PIDControlStats* stats) {
    if (!lib || !lib->initialized || !stats) {
        return false;
    }

    *stats = lib->stats;
    if (stats->control_cycles > 0U) {
        const float cycles = static_cast<float>(stats->control_cycles);
        stats->average_error_horizontal /= cycles;
        stats->average_error_vertical /= cycles;
    }

    return true;
}

void pid_control_reset_stats(PIDControlLib* lib) {
    if (lib && lib->initialized) {
        std::memset(&lib->stats, 0, sizeof(lib->stats));
    }
}

void pid_control_print_config(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        std::printf("Library not initialized\n");
        return;
    }

    pid_config_print(&lib->config);
}

void pid_control_print_status(const PIDControlLib* lib) {
    if (!lib || !lib->initialized) {
        std::printf("Library not initialized\n");
        return;
    }

    std::printf("=== PID Control Status ===\n");
    std::printf("Running: %s\n", lib->running ? "YES" : "NO");
    std::printf("Control Cycles: %u\n", lib->stats.control_cycles);
    std::printf("Target Lost Count: %u\n", lib->stats.target_lost_count);

    std::printf("\nHorizontal Axis:\n");
    std::printf("  PID Output: %.2f deg/s\n", lib->pid_controllers[AXIS_HORIZONTAL].output);
    std::printf("  Motor Level: %u\n", lib->motor_controllers[AXIS_HORIZONTAL].level);
    std::printf("  Motor Enabled: %s\n", lib->motor_controllers[AXIS_HORIZONTAL].enabled ? "YES" : "NO");
    std::printf("  Current Angle: %.2f deg\n", lib->prediction_states[AXIS_HORIZONTAL].current_deg);
    std::printf("  Velocity: %.2f deg/s\n", lib->prediction_states[AXIS_HORIZONTAL].filtered_velocity_deg_per_sec);

    std::printf("\nVertical Axis:\n");
    std::printf("  PID Output: %.2f deg/s\n", lib->pid_controllers[AXIS_VERTICAL].output);
    std::printf("  Motor Level: %u\n", lib->motor_controllers[AXIS_VERTICAL].level);
    std::printf("  Motor Enabled: %s\n", lib->motor_controllers[AXIS_VERTICAL].enabled ? "YES" : "NO");
    std::printf("  Current Angle: %.2f deg\n", lib->prediction_states[AXIS_VERTICAL].current_deg);
    std::printf("  Velocity: %.2f deg/s\n", lib->prediction_states[AXIS_VERTICAL].filtered_velocity_deg_per_sec);

    std::printf("\nPrediction Stats:\n");
    std::printf("  Total Attempts: %u\n", lib->stats.prediction_attempts);
    std::printf("  Total Rejects: %u\n", lib->stats.prediction_rejects);
    if (lib->stats.prediction_attempts > 0U) {
        const float success_rate = 1.0f - (static_cast<float>(lib->stats.prediction_rejects) /
                                           static_cast<float>(lib->stats.prediction_attempts));
        std::printf("  Success Rate: %.1f%%\n", success_rate * 100.0f);
    }

    std::printf("==========================\n");
}

bool pid_control_save_config(const PIDControlLib* lib, const char* filename) {
    if (!lib || !lib->initialized || !filename) {
        return false;
    }

    return pid_config_save_to_xml(filename, &lib->config);
}

}  // extern "C"
