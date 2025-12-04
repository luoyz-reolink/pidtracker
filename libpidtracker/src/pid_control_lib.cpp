/*** 
 * @Author: lkyezi
 * @Date: 2025-10-30 18:20:20
 * @LastEditTime: 2025-12-03 09:11:18
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
#include <vector>

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
    uint64_t last_visible_pts = 0U;
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

inline float DegToRad(float degrees) {
    return degrees * static_cast<float>(M_PI) / 180.0f;
}

inline float RadToDeg(float radians) {
    return radians * 180.0f / static_cast<float>(M_PI);
}

// 归一化向量
inline std::vector<float> NormalizeVector(float& x, float& y, float& z) {
    const float length = std::sqrt(x * x + y * y + z * z);
    if (length > 0.0f) {
        x /= length;
        y /= length;
        z /= length;
    }
    return {x, y, z};
}

// 计算向量夹角
inline float VectorAngleDeg(const std::vector<float>& v1, const std::vector<float>& v2) {
    float dot_product = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    dot_product = ClampF(dot_product, -1.0f, 1.0f);
    return RadToDeg(std::acos(dot_product));
}

// 计算2个2维向量的夹角，返回角度值
inline float Vector2DAngleDeg(const std::vector<float>& v1, const std::vector<float>& v2) {
    float cross = v1[0] * v2[1] - v1[1] * v2[0];
    float dot = v1[0] * v2[0] + v1[1] * v2[1];
    return RadToDeg(std::atan2(cross, dot));
}

// 这里一致使用角度单位,不用弧度制
/*
大致思路如下:
所有坐标计算均基于IPC的摄像头坐标系,所有距离均根据像空间内距离计算,不考虑实际物理距离:
设置IPC摄像头坐标系为右手系,


*/



/**
 * @brief 将像素坐标转换为云台角度
 *大致思路如下:
 *  所有坐标计算均基于IPC的摄像头坐标系,所有距离均根据像空间内距离计算,不考虑实际物理距离:
 * 
 * @param pixel_x 
 * @param pixel_y 
 * @param pitch_rad 
 * @param out_horizontal_deg 
 * @param out_vertical_deg 
 * @param sys 
 */
void PixelToDeg(float pixel_x, float pixel_y, float pitch_rad,
                   float& out_horizontal_deg,
                   float& out_vertical_deg,
                   const SystemParameters& sys) {
    const float offset_x = pixel_x - sys.frame_width_pixels * 0.5f;
    const float offset_y = pixel_y - sys.frame_height_pixels * 0.5f;

    // 先获取focal_length
    // 这里添加获取途径,先假设是固定值
    const float fov_max = sys.vertical_fov_deg / 2.0f;
    const float fov_ratio = 1.0f; // 先假设是1.0f
    const float focal_length = (sys.frame_height_pixels / 2.0f) / std::tan(DegToRad(fov_max));

    // 先在三维空间内计算出水平方向需要转动的角度,这个角度在后面可能会根据俯仰角进行修正

    // IPC的画面中心点在三维空间内的坐标
    const float cur_center_x = focal_length * std::cos(pitch_rad);
    const float cur_center_y = 0.0f;
    const float cur_center_z = focal_length * std::sin(pitch_rad);

    // 目标点的坐标
    const float target_x = focal_length * std::cos(pitch_rad) + offset_y * std::sin(pitch_rad);
    const float target_y = -offset_x;
    const float target_z = focal_length * std::sin(pitch_rad) - offset_y * std::cos(pitch_rad);

    // IPC追踪的目标要保证在水平方向旋转后,目标点和新的中心点在XY平面上位于同一条过0的直线上
    // 所以水平方向旋转的角度就是这2个向量在XY平面上的投影的夹角
    // 且由于IPC中心点的向量的XY平面分量在X轴上，所以计算方式很简单
    float out_horizontal_rad = std::atan2(target_y, target_x);
    // 但由于三维空间的Y轴和平面的X轴是相反的，所以这里需要取反
    out_horizontal_rad = -out_horizontal_rad;
    
    // 目标点的X坐标若是负值，说明目标点在ipc的后侧，需要将水平角度取反
    // if (target_x < 0.0f) {
    //     printf("Target x is negative, flip horizontal angle.\n");
    //     out_horizontal_rad += static_cast<float>(M_PI);
    //     out_horizontal_rad = WrapAngle(out_horizontal_rad);
    // }
    out_horizontal_deg = RadToDeg(out_horizontal_rad);

    // 假设ipc已经水平转了这么多角度后，可以认为新的中心点和目标点在同一个包含Z轴的平面内，所以将sqrt(x^2+y^2)作为新的x轴坐标来计算垂直方向的角度
    const float new_center_x = focal_length * std::cos(pitch_rad) * std::cos(out_horizontal_rad);
    const float new_center_y = -focal_length * std::cos(pitch_rad) * std::sin(out_horizontal_rad);
    const float new_center_z = focal_length * std::sin(pitch_rad);

    if(new_center_z * target_z < 0.0f)
    {
        out_vertical_deg = -RadToDeg(pitch_rad);
    }
    else
    {
        std::vector<float> vec_ipc_to_center_new_dimension = {
            std::sqrt(new_center_x * new_center_x + new_center_y * new_center_y),
            new_center_z
        };

        std::vector<float> vec_ipc_to_target_new_dimension = {
            std::sqrt(target_x * target_x + target_y * target_y),
            target_z
        };
        out_vertical_deg = Vector2DAngleDeg(vec_ipc_to_center_new_dimension, vec_ipc_to_target_new_dimension);
    }
    // 这里假设在flip = 1的情况下，水平方向向右是正数,垂直方向向下是正数
    printf("PixelToDeg: offset=(%.2f, %.2f) pitch(%.2f) center(%.2f, %.2f, %.2f) next(%.2f, %.2f, %.2f) deg=(%.2f, %.2f)\n",
           offset_x, offset_y, RadToDeg(pitch_rad), cur_center_x, cur_center_y, cur_center_z, target_x, target_y, target_z, out_horizontal_deg, out_vertical_deg);
}

/**
 * @brief 将水平像素坐标转换为水平角度，已废弃，请使用PixelToDeg
 * 
 * @param pixel_x 
 * @param sys 
 * @return float 
 */
inline float PixelsToHorizontalDeg(float pixel_x, const SystemParameters& sys) {
    const float centered = pixel_x - (sys.frame_width_pixels * 0.5f);
    const float deg_per_pixel = sys.horizontal_fov_deg / sys.frame_width_pixels;
    return centered * deg_per_pixel;
}

/**
 * @brief 将垂直像素坐标转换为垂直角度，已废弃，请使用PixelToDeg
 * 
 * @param pixel_y 
 * @param sys 
 * @return float 
 */
inline float PixelsToVerticalDeg(float pixel_y, const SystemParameters& sys) {
    const float centered = pixel_y - (sys.frame_height_pixels * 0.5f);
    const float deg_per_pixel = sys.vertical_fov_deg / sys.frame_height_pixels;
    return centered * deg_per_pixel;
}

/**
 * @brief 更新PID控制器参数
 * 
 * @param pid 
 * @param pid_config 
 * @param axis_config 
 * @param max_output 
 * @param guard_epsilon 
 */
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

/**
 * @brief 更新目标预测状态
 * 
 * @param predict 
 * @param pred_config 
 * @param features 
 */
inline void UpdatePredictionFromConfig(TargetPredictionState& predict,
                                       const PredictionParameters& pred_config,
                                       const FeatureSwitches& features) {
    predict.guard_ratio = pred_config.guard_ratio;
    predict.prediction_enabled = features.enable_prediction;
    predict.guard_enabled = features.enable_prediction_guard;
    predict.adaptive_guard_enabled = features.enable_prediction_adaptive_guard;
}

/**
 * @brief 重置目标预测状态
 * 
 * @param state 
 * @param config 
 */
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

inline float smooth_and_limit(float raw, float &prev, float dt,
                              float alpha, float max_rate) {
    float filtered = alpha * raw + (1.0f - alpha) * prev;
    float max_delta = max_rate * dt;
    float delta = filtered - prev;
    filtered = ClampF(delta, -max_delta, max_delta);
    prev = filtered;
    return filtered;
}

inline bool MotorControlDesiredSpeedToLevel(MotorControl& motor,
                                            const SystemParameters& sys,
                                            float desired_speed) {
    // direction在serial_mcu里面：1：步数增加方向，0：步数减少方向
    // 而这里获取的速度，其实仅根据物体在画面里的坐标决定
    // 而物体的坐标的逻辑是：从左到右坐标增大，从上到下坐标增大
    // 所以这里的direction就代表：0：相对于画面需要向右/下转动，1：相对于画面需要向左/上转动
    // 之后的motor_set_speed回调会根据画面的mirror、flip情况，决定最终电机的转动方向
    uint8_t direction = (desired_speed > 0.0f) ? 0U : 1U;
    uint8_t level = 0U;
    uint8_t max_level = sys.max_speed_level;
    bool speed_changed = false;
    float abs_speed = AbsF(desired_speed);
    // 速度小于等级1时不需要考虑（这个范围和死区作用类似）
    if(abs_speed < sys.speed_levels[0])
    {
        abs_speed = 0.0f;
    }
    
    // 若需要运动的方向和之前的不同就先直接停下电机
    if(motor.direction != direction)
    {
        level = 0U;
    }
    // 速度小于最小值，直接停下电机
    else if(abs_speed <= sys.motor_vmin_deg_per_sec) 
    {
        // if(abs_speed == 0.0f || motor.level != 0)
        // {
        //     level = 0U;
        // }
        level = 0U;
    }
    // 速度大于等于最大值，直接上到最高挡位
    else if(abs_speed >= sys.speed_levels[max_level - 1U])
    {
        level = max_level;
    }
    else 
    {
        // 直接定位改成递增递减判断试试
        // for (; level < max_level; ++level) 
        // {
            // if (abs_speed < sys.speed_levels[level]) {
            //     level = level + 1U;
            //     break;
            // }
        // }
        // 若当前速度是0，先加速至对应挡位（之前采取逐级增加速度，但对于运动速度较快的物体表现是跟不上）
        if(motor.level == 0U)
        {
            for (; level < max_level; ++level) 
            {
                if (abs_speed < sys.speed_levels[level]) {
                    level = level + 1U;
                    break;
                }
            }
        }
        // 否则的话根据当前挡位和速度，决定是加速还是减速
        else
        {
            // for (; level < max_level; ++level) 
            // {
            //     if (abs_speed < sys.speed_levels[level]) {
            //         level = level + 1U;
            //         break;
            //     }
            // }

            

            // 速度大于当前挡位，逐级加速。要是速度是最大速度则直接加到最大挡位,防止没来得及追导致速度跟不上(其实前面有做处理,这个分支不会走进去)
            if(abs_speed > sys.speed_levels[motor.level - 1U])
            {
                if(motor.level < max_level) 
                {
                    if(abs_speed > (sys.speed_levels[motor.level] + sys.speed_levels[motor.level - 1U]) / 2.0f)
                    {
                        level = motor.level + 1U;
                    }
                    else 
                    {
                        level = motor.level;
                    }
                } 
                else
                {
                    level = max_level;
                }
                
            }
            // 速度小于当前挡位，逐级减速
            else 
            {
                if(motor.level > 1U) 
                {
                    if(abs_speed < (sys.speed_levels[motor.level - 1U] + sys.speed_levels[motor.level - 2U]) / 2.0f)
                    {
                        level = motor.level - 1U;
                    }
                    else 
                    {
                        level = motor.level;
                    }
                }
                else 
                {
                    level = 1U;
                }
            }
        }
    }

    speed_changed = (motor.level != level) || (motor.direction != direction);
    motor.level = level;
    motor.direction = direction;
    motor.enabled = (level > 0U) ? 1U : 0U;
    return speed_changed;
}

inline void MotorControlSetSpeed(const MotorControl& motor, const HardwareInterface& hw) {
    if (motor.set_speed) {
        motor.set_speed(hw.user_ctx, motor.axis, motor.level, motor.direction);
    }
}

inline void MotorControlDisable(const MotorControl& motor, const HardwareInterface& hw) {
    if (motor.disable_motor) {
        motor.disable_motor(hw.user_ctx, motor.axis);
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

    pid.output = ClampF(output, pid.min_output, pid.max_output);
    // printf("PID Compute: Error=%.3f, prev_Error=%.3f, dt=%.3f, P=%.3f, I=%.3f, D=%.3f, Output=%.3f\n",
    //        error,
    //        pid.prev_error,
    //        safe_dt,
    //        proportional_term,
    //        integral_term,
    //        derivative_term,
    //        pid.output);

    pid.prev_error = error;

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
    pid_control_print_config(lib);
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
    static float last_target_px_x = -1.0f;
    static float last_target_px_y = -1.0f;
    uint64_t pts = 0U;
    if (!lib->hw_interface.get_target_pixel_position(lib->hw_interface.user_ctx, &target_px_x, &target_px_y, &pts)) {
        // 目标丢失，使用上次位置和速度进行估计
        float time = static_cast<float>(pts - lib->last_visible_pts) / 1e6f;
        int level_x = lib->motor_controllers[AXIS_HORIZONTAL].level - 1;
        int level_y = lib->motor_controllers[AXIS_VERTICAL].level - 1;
        level_x = (level_x < 0) ? 0 : level_x;
        level_y = (level_y < 0) ? 0 : level_y;
        float speed_x = lib->config.system.speed_levels[level_x] / 
                            (lib->config.system.horizontal_fov_deg / lib->config.system.frame_width_pixels);
        float speed_y = lib->config.system.speed_levels[level_y] / 
                            (lib->config.system.vertical_fov_deg / lib->config.system.frame_height_pixels);
        int sign_x = (lib->motor_controllers[AXIS_HORIZONTAL].direction) ? 1 : -1;
        int sign_y = (lib->motor_controllers[AXIS_VERTICAL].direction) ? 1 : -1;
        if(level_x == 0)
        {
            last_target_px_x = lib->config.system.frame_width_pixels / 2.0f;
        }
        else
        {
            last_target_px_x += sign_x * speed_x * time;

        }
        if(level_y == 0)
        {
            last_target_px_y = lib->config.system.frame_height_pixels / 2.0f;
        }
        else
        {
            last_target_px_y += sign_y * speed_y * time;
        }

        // printf("Target lost, estimating position to (%.2f, %.2f), direction(%d, %d), level(%d, %d), speed(%.2f, %.2f), time(%.3f)\n", last_target_px_x, last_target_px_y, sign_x, sign_y, level_x, level_y, speed_x, speed_y, time);
        // lib->motor_controllers[AXIS_HORIZONTAL].enabled = 0U;
        // lib->motor_controllers[AXIS_VERTICAL].enabled = 0U;
        // MotorControlDisable(lib->motor_controllers[AXIS_HORIZONTAL]);
        // MotorControlDisable(lib->motor_controllers[AXIS_VERTICAL]);
        // PredictionReset(lib->prediction_states[AXIS_HORIZONTAL], lib->config.prediction);
        // PredictionReset(lib->prediction_states[AXIS_VERTICAL], lib->config.prediction);
        // lib->stats.target_lost_count++;
    }
    else
    {
        last_target_px_x = target_px_x;
        last_target_px_y = target_px_y;
    }

    lib->stats.control_cycles++;

    const float frame_period = pts - lib->last_control_pts > 0U
                                   ? static_cast<float>(pts - lib->last_control_pts) / 1e6f
                                   : 0.0f;
    lib->last_control_pts = pts;
    lib->last_visible_pts = pts;
    const float max_horizontal_range = lib->config.system.horizontal_fov_deg * 0.5f;
    const float max_vertical_range = lib->config.system.vertical_fov_deg * 0.5f;

    const float pitch_angle_rad = DegToRad(lib->hw_interface.read_angle(lib->hw_interface.user_ctx, AXIS_VERTICAL));
 
    float target_horizontal_deg ,target_vertical_deg;
    PixelToDeg(last_target_px_x, last_target_px_y, pitch_angle_rad, target_horizontal_deg, target_vertical_deg, lib->config.system);

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

    // const float speed_alpha = 0.3f;
    // const float max_rate = lib->config.system.max_speed_level > 0
    //                     ? lib->config.system.speed_levels[lib->config.system.max_speed_level - 1] * 2.0f   // 每秒允许变化的最大“度/秒”
    //                     : 10.0f;
    // const float smoothed_horizontal_command = smooth_and_limit(horizontal_command,
    //                                           lib->pid_controllers[AXIS_HORIZONTAL].prev_error,
    //                                           frame_period,
    //                                           speed_alpha,
    //                                           max_rate);
    // const float smoothed_vertical_command = smooth_and_limit(vertical_command,
    //                                           lib->pid_controllers[AXIS_VERTICAL].prev_error,
    //                                           frame_period,
    //                                           speed_alpha,
    //                                           max_rate);

    if (MotorControlDesiredSpeedToLevel(lib->motor_controllers[AXIS_HORIZONTAL], lib->config.system, horizontal_command)) {
        MotorControlSetSpeed(lib->motor_controllers[AXIS_HORIZONTAL], lib->hw_interface);
    }

    if (MotorControlDesiredSpeedToLevel(lib->motor_controllers[AXIS_VERTICAL], lib->config.system, vertical_command)) {
        MotorControlSetSpeed(lib->motor_controllers[AXIS_VERTICAL], lib->hw_interface);
    }

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

// 目前只停下了电机，还需要重置一些参数
void pid_control_stop(PIDControlLib* lib) {
    if (lib && lib->initialized) {
        lib->running = false;
        lib->motor_controllers[AXIS_HORIZONTAL].enabled = 0U;
        lib->motor_controllers[AXIS_VERTICAL].enabled = 0U;
        MotorControlDisable(lib->motor_controllers[AXIS_HORIZONTAL], lib->hw_interface);
        MotorControlDisable(lib->motor_controllers[AXIS_VERTICAL], lib->hw_interface);
        PredictionReset(lib->prediction_states[AXIS_HORIZONTAL], lib->config.prediction);
        PredictionReset(lib->prediction_states[AXIS_VERTICAL], lib->config.prediction);

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
