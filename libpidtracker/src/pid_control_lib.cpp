/***
 * @Author: lkyezi
 * @Date: 2025-10-30 18:20:20
 * @LastEditTime: 2025-12-23 11:04:56
 * @LastEditors: lkyezi
 * @Description:
 * @
 */
#include "pid_control_lib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#ifdef PID_LOG_ENABLE
#define PID_LOG(fmt, ...) printf("[PID_Control_Lib] " fmt, ##__VA_ARGS__)
#else
#define PID_LOG(fmt, ...)
#endif

namespace
{
    constexpr float kMinDtSeconds = 0.01f;
    constexpr float kMaxDtSeconds = 0.25f;

    /**
     * @brief 将电机步进位置转换为弧度值。算出当前电机的姿态角
     *
     * @param pos
     * @param axis
     * @param sys
     * @return float
     */
    inline float pos_to_rad(const int32_t pos, axis_type_e axis, const system_parameters_t& sys)
    {
        if (axis == AXIS_HORIZONTAL)
        {
            const float total_steps = static_cast<float>(sys.motor_max_steps[AXIS_HORIZONTAL] - sys.motor_min_steps[AXIS_HORIZONTAL]);
            const float total_deg = static_cast<float>(sys.motor_max_angle[AXIS_HORIZONTAL] - sys.motor_min_angle[AXIS_HORIZONTAL]);
            return deg_to_rad(static_cast<float>(pos - sys.motor_min_steps[AXIS_HORIZONTAL]) * total_deg / total_steps +
                              sys.motor_min_angle[AXIS_HORIZONTAL]);
        }

        // else AXIS_VERTICAL
        const float total_steps = static_cast<float>(sys.motor_max_steps[AXIS_VERTICAL] - sys.motor_min_steps[AXIS_VERTICAL]);
        const float total_deg = static_cast<float>(sys.motor_max_angle[AXIS_VERTICAL] - sys.motor_min_angle[AXIS_VERTICAL]);
        float sign = (pos > 0) ? 1.0f : -1.0f;
        sign = sys.b_hanging_upright ? sign : -sign;
        const float abs_pos = std::fabs(static_cast<float>(pos));
        float offset_steps = sys.motor_orientation[AXIS_VERTICAL] ? static_cast<float>(sys.motor_max_steps[AXIS_VERTICAL]) - abs_pos
                                                       : abs_pos - static_cast<float>(sys.motor_min_steps[AXIS_VERTICAL]);
        return deg_to_rad(sign * (offset_steps * total_deg / total_steps + sys.motor_min_angle[AXIS_VERTICAL]));
    }

    /**
     * @brief 将像素坐标转换为水平和垂直角度偏移值
     *
     * @param pixel_x
     * @param pixel_y
     * @param pitch_rad
     * @param out_horizontal_deg
     * @param out_vertical_deg
     * @param sys
     */
    void pixel_to_deg(float pixel_x, float pixel_y, float pitch_rad, float& out_horizontal_deg, float& out_vertical_deg,
                      const system_parameters_t& sys)
    {
        const float offset_x = pixel_x - sys.frame_width_pixels * 0.5f;
        const float offset_y = pixel_y - sys.frame_height_pixels * 0.5f;

    const float fov_max = sys.fov_deg[AXIS_VERTICAL] / 2.0f;
        const float focal_length = (sys.frame_height_pixels / 2.0f) / std::tan(deg_to_rad(fov_max));

        const float target_x = focal_length * std::cos(pitch_rad) + offset_y * std::sin(pitch_rad);
        const float target_y = -offset_x;
        const float target_z = focal_length * std::sin(pitch_rad) - offset_y * std::cos(pitch_rad);

        float out_horizontal_rad = std::atan2(target_y, target_x);
        out_horizontal_rad = -out_horizontal_rad;
        out_horizontal_deg = rad_to_deg(out_horizontal_rad);

        const float new_center_x = focal_length * std::cos(pitch_rad) * std::cos(out_horizontal_rad);
        const float new_center_y = -focal_length * std::cos(pitch_rad) * std::sin(out_horizontal_rad);
        const float new_center_z = focal_length * std::sin(pitch_rad);

        if (new_center_z * target_z < 0.0f)
        {
            out_vertical_deg = -rad_to_deg(pitch_rad);
        }
        else
        {
            const std::array<float, 2> vec_ipc_to_center_new_dimension = {
                std::sqrt(new_center_x * new_center_x + new_center_y * new_center_y), new_center_z};

            const std::array<float, 2> vec_ipc_to_target_new_dimension = {
                std::sqrt(target_x * target_x + target_y * target_y), target_z};

            out_vertical_deg = vector2d_angle_deg(vec_ipc_to_center_new_dimension, vec_ipc_to_target_new_dimension);
        }
    }

} // namespace

/**
 * @brief 从配置更新 PID 控制器参数
 *
 * @param pid
 * @param pid_config
 * @param axis_config
 * @param max_output
 * @param guard_epsilon
 */
void pid_control_lib::update_pid_from_config(pid_controller_t& pid, const pid_parameters_t& pid_config,
                                             const pid_axis_parameters_t& axis_config, float max_output,
                                             float guard_epsilon)
{
    pid.kp = axis_config.kp;
    pid.ki = axis_config.ki;
    pid.kd = axis_config.kd;
    pid.deadband = pid_config.deadband_deg;
    pid.d_filter_alpha = pid_config.d_filter_alpha;
    pid.max_output = max_output;
    pid.min_output = -max_output;

    if (pid.ki > guard_epsilon)
    {
        const float limit = (pid.max_output * pid_config.integral_ratio) / pid.ki;
        pid.integral_max = limit;
        pid.integral_min = -limit;
    }
    else
    {
        pid.integral_max = 0.0f;
        pid.integral_min = 0.0f;
    }
}

/**
 * @brief 对计算出的角度进行滤波（只能在增大的时候滤波，不然会导致物体在画面中心来来回回晃动）
 *
 * @param axis
 * @param measured_deg
 * @param alpha
 * @return float
 */
float pid_control_lib::filter_angle_deg(axis_type_e axis, float measured_deg, float alpha)
{
    const float a = clamp_f(alpha, 0.0f, 1.0f);
    if (!m_angle_filter_initialized[axis])
    {
        m_filtered_angles_deg[axis] = measured_deg;
        m_angle_filter_initialized[axis] = true;
    }
    else
    {
        if(measured_deg * m_filtered_angles_deg[axis] < 0.0f || measured_deg == 0.0f)
        {
            m_filtered_angles_deg[axis] = measured_deg;
        }
        else if(std::fabs(measured_deg) < std::fabs(m_filtered_angles_deg[axis]))
        {
            m_filtered_angles_deg[axis] = measured_deg;
        }
        else
        {
            m_filtered_angles_deg[axis] = (a * measured_deg) + ((1.0f - a) * m_filtered_angles_deg[axis]);
        }
    }
    return m_filtered_angles_deg[axis];
}

/**
 * @brief 更新目标预测状态
 * 
 * @param predict 
 * @param pred_config 
 * @param features 
 */
void pid_control_lib::update_prediction_from_config(target_prediction_state_t& predict,
                                                    const prediction_parameters_t& pred_config,
                                                    const feature_switches_t& features)
{
    predict.guard_ratio = pred_config.guard_ratio;
    predict.b_prediction_enabled = features.b_enable_prediction;
    predict.b_guard_enabled = features.b_enable_prediction_guard;
    predict.b_adaptive_guard_enabled = features.b_enable_prediction_adaptive_guard;
}

/**
 * @brief 重置目标预测状态
 * 
 * @param state 
 * @param config 
 */
void pid_control_lib::prediction_reset(target_prediction_state_t& state, const prediction_parameters_t& config)
{
    state.current_deg = 0.0f;
    state.previous_deg = 0.0f;
    state.filtered_velocity_deg_per_sec = 0.0f;
    state.b_has_previous_sample = false;
    state.guard_ratio = config.guard_ratio;
    state.guard_attempts = 0U;
    state.guard_rejects = 0U;
}

/**
 * @brief 初始化电机控制
 * 
 * @param motor 
 * @param axis 
 */
void pid_control_lib::motor_control_init(motor_control_t& motor, axis_type_e axis)
{
    motor.axis = axis;
    motor.enabled = 0U;
    motor.level = 0U;
    motor.direction = 1U;
}

/**
 * @brief 将期望速度转换为电机控制级别
 * 
 * @param motor 
 * @param sys 
 * @param desired_speed 
 * @return true 
 * @return false 
 */
bool pid_control_lib::motor_control_desired_speed_to_level(motor_control_t& motor, const system_parameters_t& sys,
                                                           float desired_speed)
{
    const float* speed_levels = sys.speed_levels[motor.axis];
    uint8_t direction = (desired_speed > 0.0f) ? 0U : 1U;
    uint8_t level = 0U;
    uint8_t max_level = sys.max_speed_level;
    bool speed_changed = false;
    float abs_speed = std::fabs(desired_speed);
    if (abs_speed < speed_levels[0])
    {
        abs_speed = 0.0f;
    }

    if ((motor.direction != direction) || (abs_speed <= sys.motor_vmin_deg_per_sec))
    {
        level = 0U;
    }
    else if (abs_speed >= speed_levels[max_level - 1U])
    {
        level = max_level;
    }
    else
    {
        if (motor.level == 0U)
        {
            for (; level < max_level; ++level)
            {
                if (abs_speed < speed_levels[level])
                {
                    level = level + 1U;
                    break;
                }
            }
        }
        else
        {
            if (abs_speed > speed_levels[motor.level - 1U])
            {
                if (motor.level < max_level)
                {
                    if (abs_speed > (speed_levels[motor.level] + speed_levels[motor.level - 1U]) / 2.0f)
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
            else
            {
                if (motor.level > 1U)
                {
                    if (abs_speed < (speed_levels[motor.level - 1U] + speed_levels[motor.level - 2U]) / 2.0f)
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

void pid_control_lib::motor_control_set_speed(const motor_control_t& motor, const hardware_interface_t& hw)
{
    if (motor.p_set_speed)
    {
        motor.p_set_speed(hw.p_user_ctx, motor.axis, motor.level, motor.direction);
    }
}

void pid_control_lib::motor_control_disable(const motor_control_t& motor, const hardware_interface_t& hw)
{
    if (motor.p_disable_motor)
    {
        motor.p_disable_motor(hw.p_user_ctx, motor.axis);
    }
}

float pid_control_lib::prediction_apply(target_prediction_state_t& state, float measured_deg, float range_limit_deg,
                                        float dt, const prediction_parameters_t& config)
{
    if (!state.b_prediction_enabled)
    {
        return clamp_f(measured_deg, -range_limit_deg, range_limit_deg);
    }

    const float safe_dt = clamp_f(dt, kMinDtSeconds, kMaxDtSeconds);
    const float previous_current = state.current_deg;

    if (!state.b_has_previous_sample)
    {
        state.previous_deg = measured_deg;
        state.current_deg = measured_deg;
        state.filtered_velocity_deg_per_sec = 0.0f;
        state.b_has_previous_sample = true;
        return clamp_f(measured_deg, -range_limit_deg, range_limit_deg);
    }

    const float raw_velocity = (measured_deg - previous_current) / safe_dt;
    state.previous_deg = previous_current;
    state.current_deg = measured_deg;
    state.filtered_velocity_deg_per_sec =
        (config.velocity_alpha * raw_velocity) + ((1.0f - config.velocity_alpha) * state.filtered_velocity_deg_per_sec);

    const float lookahead_time = safe_dt;
    float predicted_shift = state.filtered_velocity_deg_per_sec * lookahead_time;
    const float max_shift = range_limit_deg * config.max_shift_factor;
    predicted_shift = clamp_f(predicted_shift, -max_shift, max_shift);

    const float predicted = measured_deg + predicted_shift;

    if (state.b_guard_enabled)
    {
        float base_magnitude = std::fabs(measured_deg);
        if (base_magnitude < config.center_tolerance_deg)
        {
            base_magnitude = config.center_tolerance_deg;
        }
        const float shift_magnitude = std::fabs(predicted_shift);
        const float ratio = shift_magnitude / base_magnitude;
        const bool out_of_bounds = (predicted < -range_limit_deg) || (predicted > range_limit_deg);
        const bool excessive_shift = (ratio > state.guard_ratio);
        const bool reject_prediction = out_of_bounds || excessive_shift;

        state.guard_attempts++;
        if (reject_prediction)
        {
            state.guard_rejects++;
        }

        if (state.b_adaptive_guard_enabled && state.guard_attempts >= config.guard_adapt_window)
        {
            const float rejection_rate =
                static_cast<float>(state.guard_rejects) / static_cast<float>(state.guard_attempts);
            const float span = config.guard_ratio_max - config.guard_ratio_min;
            const float dynamic_ratio = config.guard_ratio_max - (rejection_rate * span);
            state.guard_ratio = clamp_f(dynamic_ratio, config.guard_ratio_min, config.guard_ratio_max);
            state.guard_attempts = 0U;
            state.guard_rejects = 0U;
        }

        if (reject_prediction)
        {
            return clamp_f(measured_deg, -range_limit_deg, range_limit_deg);
        }
    }

    return clamp_f(predicted, -range_limit_deg, range_limit_deg);
}

float pid_control_lib::compute_pid(pid_controller_t& pid, float target, float feedback, float dt, float guard_epsilon)
{
    float safe_dt = dt;
    if (safe_dt <= 0.0f)
    {
        safe_dt = kMinDtSeconds;
    }
    else if (safe_dt > kMaxDtSeconds)
    {
        safe_dt = kMaxDtSeconds;
    }

    float error = target - feedback;
    if (std::fabs(error) < pid.deadband)
    {
        error = 0.0f;
    }

    const bool allow_integral = (pid.ki > guard_epsilon);
    const bool saturating = (pid.output >= pid.max_output - guard_epsilon && error > 0.0f) ||
                            (pid.output <= pid.min_output + guard_epsilon && error < 0.0f);

    if (allow_integral && !saturating)
    {
        pid.integral += (error + pid.prev_error) * 0.5f * safe_dt;
        pid.integral = clamp_f(pid.integral, pid.integral_min, pid.integral_max);
    }

    const float derivative_raw = (error - pid.prev_error) / safe_dt;
    float derivative_term;

    if (pid.b_d_filter_enabled)
    {
        if (pid.b_d_initialized)
        {
            pid.derivative_filtered =
                (pid.d_filter_alpha * derivative_raw) + ((1.0f - pid.d_filter_alpha) * pid.derivative_filtered);
        }
        else
        {
            pid.derivative_filtered = derivative_raw;
            pid.b_d_initialized = true;
        }
        derivative_term = pid.kd * pid.derivative_filtered;
    }
    else
    {
        derivative_term = pid.kd * derivative_raw;
    }

    const float proportional_term = pid.kp * error;
    const float integral_term = pid.ki * pid.integral;
    const float output = proportional_term + integral_term + derivative_term;

    pid.output = clamp_f(output, pid.min_output, pid.max_output);
    pid.prev_error = error;

    return pid.output;
}

void pid_control_lib::update_tracking_analysis(float target_px_x, float target_px_y, bool has_measurement)
{
    if (!b_initialized)
    {
        return;
    }
    auto& config = m_config;
    auto& tracking_metrics = m_tracking_metrics;
    auto& motor_controllers = m_motor_controllers;
    auto& tracking_log = m_tracking_log;

    const float center_x = config.system.frame_width_pixels * 0.5f;
    const float center_y = config.system.frame_height_pixels * 0.5f;
    const float offset_horizontal_px = target_px_x - center_x;
    const float offset_vertical_px = target_px_y - center_y;
    const float abs_offset_horizontal_px = std::fabs(offset_horizontal_px);
    const float abs_offset_vertical_px = std::fabs(offset_vertical_px);

    tracking_metrics.max_offset_px[AXIS_HORIZONTAL] =
        std::max(tracking_metrics.max_offset_px[AXIS_HORIZONTAL], abs_offset_horizontal_px);
    tracking_metrics.max_offset_px[AXIS_VERTICAL] = std::max(tracking_metrics.max_offset_px[AXIS_VERTICAL], abs_offset_vertical_px);
    tracking_metrics.cumulative_offset_px[AXIS_HORIZONTAL] += static_cast<double>(abs_offset_horizontal_px);
    tracking_metrics.cumulative_offset_px[AXIS_VERTICAL] += static_cast<double>(abs_offset_vertical_px);
    tracking_metrics.sample_count++;

    if (abs_offset_horizontal_px > config.system.frame_width_pixels)
    {
        tracking_metrics.bound_violations[AXIS_HORIZONTAL]++;
    }
    if (abs_offset_vertical_px > config.system.frame_height_pixels)
    {
        tracking_metrics.bound_violations[AXIS_VERTICAL]++;
    }

    pid_tracking_sample_t sample{};
    sample.target_px_x = target_px_x;
    sample.target_px_y = target_px_y;
    sample.target_valid = static_cast<uint8_t>(has_measurement ? 1 : 0);
    sample.motor_level[AXIS_HORIZONTAL] = motor_controllers[AXIS_HORIZONTAL].level;
    sample.motor_direction[AXIS_HORIZONTAL] = motor_controllers[AXIS_HORIZONTAL].direction;
    sample.motor_level[AXIS_VERTICAL] = motor_controllers[AXIS_VERTICAL].level;
    sample.motor_direction[AXIS_VERTICAL] = motor_controllers[AXIS_VERTICAL].direction;
    sample.offset_px[AXIS_HORIZONTAL] = offset_horizontal_px;
    sample.offset_px[AXIS_VERTICAL] = offset_vertical_px;
    tracking_log.emplace_back(sample);
}

bool pid_control_lib::init_from_xml(const char* p_config_file, const hardware_interface_t* p_hw_interface)
{
    if (!p_config_file || !p_hw_interface)
    {
        return false;
    }

    if (!pid_config_load_from_xml(p_config_file, &m_config))
    {
        return false;
    }
    m_hw_interface = *p_hw_interface;
    auto& config = m_config;
    auto& motor_controllers = m_motor_controllers;
    auto& prediction_states = m_prediction_states;
    auto& pid_controllers = m_pid_controllers;

    const uint32_t max_level_index =
        (config.system.max_speed_level > 0U) ? static_cast<uint32_t>(config.system.max_speed_level - 1U) : 0U;
    const float max_output_p = config.system.speed_levels[AXIS_HORIZONTAL][max_level_index];
    const float max_output_t = config.system.speed_levels[AXIS_VERTICAL][max_level_index];

    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        motor_control_init(motor_controllers[axis], static_cast<axis_type_e>(axis));
        m_motor_controllers[axis].p_set_speed = m_hw_interface.p_motor_set_speed;
        m_motor_controllers[axis].p_disable_motor = m_hw_interface.p_motor_disable;
        prediction_reset(prediction_states[axis], config.prediction);
        update_prediction_from_config(prediction_states[axis], config.prediction, config.features);
        m_angle_filter_initialized[axis] = false;
        m_filtered_angles_deg[axis] = 0.0f;
    }

    update_pid_from_config(pid_controllers[AXIS_HORIZONTAL], config.pid, config.pid.axis[AXIS_HORIZONTAL], max_output_p,
                           config.pid.guard_epsilon);
    update_pid_from_config(pid_controllers[AXIS_VERTICAL], config.pid, config.pid.axis[AXIS_VERTICAL], max_output_t,
                           config.pid.guard_epsilon);

    std::memset(&m_stats, 0, sizeof(m_stats));

    b_initialized = true;
    print_config();
    return true;
}

bool pid_control_lib::init_default(const hardware_interface_t* p_hw_interface)
{
    if (!p_hw_interface)
    {
        return false;
    }

    pid_config_set_defaults(&m_config);
    m_hw_interface = *p_hw_interface;
    auto& config = m_config;
    auto& motor_controllers = m_motor_controllers;
    auto& prediction_states = m_prediction_states;
    auto& pid_controllers = m_pid_controllers;

    const uint32_t max_level_index =
        (config.system.max_speed_level > 0U) ? static_cast<uint32_t>(config.system.max_speed_level - 1U) : 0U;
    const float max_output_p = config.system.speed_levels[AXIS_HORIZONTAL][max_level_index];
    const float max_output_t = config.system.speed_levels[AXIS_VERTICAL][max_level_index];

    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        motor_control_init(motor_controllers[axis], static_cast<axis_type_e>(axis));
        m_motor_controllers[axis].p_set_speed = m_hw_interface.p_motor_set_speed;
        m_motor_controllers[axis].p_disable_motor = m_hw_interface.p_motor_disable;
        prediction_reset(prediction_states[axis], config.prediction);
        update_prediction_from_config(prediction_states[axis], config.prediction, config.features);
        m_angle_filter_initialized[axis] = false;
        m_filtered_angles_deg[axis] = 0.0f;
    }

    update_pid_from_config(pid_controllers[AXIS_HORIZONTAL], config.pid, config.pid.axis[AXIS_HORIZONTAL], max_output_p,
                           config.pid.guard_epsilon);
    update_pid_from_config(pid_controllers[AXIS_VERTICAL], config.pid, config.pid.axis[AXIS_VERTICAL], max_output_t,
                           config.pid.guard_epsilon);

    std::memset(&m_stats, 0, sizeof(m_stats));

    b_initialized = true;
    return true;
}

const pid_control_config_t* pid_control_lib::get_config() const
{
    if (!b_initialized)
    {
        return nullptr;
    }
    return &m_config;
}

bool pid_control_lib::reload_config(const char* p_config_file)
{
    if (!b_initialized || !p_config_file)
    {
        return false;
    }

    pid_control_config_t new_config{};
    pid_config_set_defaults(&new_config);
    if (!pid_config_load_from_xml(p_config_file, &new_config))
    {
        return false;
    }

    m_config = new_config;
    auto& config = m_config;
    auto& pid_controllers = m_pid_controllers;
    auto& prediction_states = m_prediction_states;

    const uint32_t max_level_index =
        (config.system.max_speed_level > 0U) ? static_cast<uint32_t>(config.system.max_speed_level - 1U) : 0U;
    const float max_output_p = config.system.speed_levels[AXIS_HORIZONTAL][max_level_index];
    const float max_output_t = config.system.speed_levels[AXIS_VERTICAL][max_level_index];

    update_pid_from_config(pid_controllers[AXIS_HORIZONTAL], config.pid, config.pid.axis[AXIS_HORIZONTAL], max_output_p,
                           config.pid.guard_epsilon);
    update_pid_from_config(pid_controllers[AXIS_VERTICAL], config.pid, config.pid.axis[AXIS_VERTICAL], max_output_t,
                           config.pid.guard_epsilon);

    update_prediction_from_config(prediction_states[AXIS_HORIZONTAL], config.prediction, config.features);
    update_prediction_from_config(prediction_states[AXIS_VERTICAL], config.prediction, config.features);

    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_angle_filter_initialized[axis] = false;
        m_filtered_angles_deg[axis] = 0.0f;
    }

    pid_controllers[AXIS_HORIZONTAL].b_d_filter_enabled = config.features.b_enable_d_filter;
    pid_controllers[AXIS_VERTICAL].b_d_filter_enabled = config.features.b_enable_d_filter;

    return true;
}

void pid_control_lib::step()
{
    if (!b_initialized || !b_running)
    {
        return;
    }

    if (!m_hw_interface.p_get_target_pixel_position)
    {
        return;
    }

    auto& config = m_config;

    // 读取当前电机位置，计算当前姿态角
    float filtered_angles_rad[AXIS_TOTAL] = {0.0f, 0.0f};
    if (m_hw_interface.p_read_pos)
    {
        for (int axis = 0; axis < AXIS_TOTAL; ++axis)
        {
            int32_t steps = m_hw_interface.p_read_pos(m_hw_interface.p_user_ctx, static_cast<uint8_t>(axis));
            filtered_angles_rad[axis] = pos_to_rad(steps, static_cast<axis_type_e>(axis), config.system);
            // float deg = rad_to_deg(rad);
            // float smoothed_deg = filter_angle_deg(static_cast<axis_type_e>(axis), deg, config.pid.angle_filter_alpha);
        }
    }

    // 获取目标像素位置，如果目标丢失则进行位置估计
    float target_px_x = 0.0f;
    float target_px_y = 0.0f;
    static float last_target_px_x = -1.0f;
    static float last_target_px_y = -1.0f;
    int level_x = 0;
    int level_y = 0;
    int sign_x = 1;
    int sign_y = 1;
    float speed_x = 0.0f;
    float speed_y = 0.0f;
    float time = 0.0f;
    uint64_t pts = 0U;
    float proportion = 0.0f;
    bool has_target = m_hw_interface.p_get_target_pixel_position(m_hw_interface.p_user_ctx, &target_px_x, &target_px_y,
                                                                 &pts, &proportion);
    b_target_proportion_valid = has_target;

    if (has_target)
    {
        last_target_px_x = target_px_x;
        last_target_px_y = target_px_y;
    }
    else
    {
        // 目标丢失，使用上次位置和速度进行估计
        time = static_cast<float>(pts - m_last_visible_pts) / 1e6f;

        if (m_motor_controllers[AXIS_HORIZONTAL].level > 0)
        {
            level_x = m_motor_controllers[AXIS_HORIZONTAL].level - 1;
            speed_x = m_config.system.speed_levels[AXIS_HORIZONTAL][level_x] / 
                            (m_config.system.fov_deg[AXIS_HORIZONTAL] / m_config.system.frame_width_pixels);
            sign_x = (m_motor_controllers[AXIS_HORIZONTAL].direction) ? 1 : -1;
            last_target_px_x += sign_x * speed_x * time;
        }
        else
        {
            last_target_px_x = m_config.system.frame_width_pixels / 2.0f;
        }

        if (m_motor_controllers[AXIS_VERTICAL].level > 0)
        {
            level_y = m_motor_controllers[AXIS_VERTICAL].level - 1;
            speed_y = m_config.system.speed_levels[AXIS_VERTICAL][level_y] / 
                            (m_config.system.fov_deg[AXIS_VERTICAL] / m_config.system.frame_height_pixels);
            sign_y = (m_motor_controllers[AXIS_VERTICAL].direction) ? -1 : 1;
            last_target_px_y += sign_y * speed_y * time;
        }
        else
        {
            last_target_px_y = m_config.system.frame_height_pixels / 2.0f;
        }

        // printf("Target lost, estimating position to (%.2f, %.2f), level:%d, sign_x:%d, sign_y:%d ,speed_x:%.2f,
        // speed_y:%.2f, time:%.2f\n", last_target_px_x, last_target_px_y, m_motor_controllers[AXIS_HORIZONTAL].level,
        // sign_x, sign_y, speed_x, speed_y, time);
    }

    // update_tracking_analysis(target_px_x, target_px_y, has_target);

    // if (!has_target) {
    //     motor_control_disable(m_motor_controllers[AXIS_HORIZONTAL], m_hw_interface);
    //     motor_control_disable(m_motor_controllers[AXIS_VERTICAL], m_hw_interface);
    //     return;
    // }

    uint64_t dt_us = (m_last_control_pts > 0U) ? (pts - m_last_control_pts) : 0U;
    float dt = (dt_us > 0U) ? (static_cast<float>(dt_us) / 1000000.0f) : kMinDtSeconds;
    m_last_control_pts = pts;
    m_last_visible_pts = pts;
    m_last_target_proportion = proportion;

    float deadband_scale = 1.0f;
    if (has_target && config.pid.proportion_deadband_scale > 0.0f)
    {
        float scaled = 1.0f + proportion * config.pid.proportion_deadband_scale;
        float max_scale = (config.pid.proportion_deadband_max > 1.0f) ? config.pid.proportion_deadband_max : scaled;
        deadband_scale = clamp_f(scaled, 1.0f, max_scale);
    }
    m_pid_controllers[AXIS_HORIZONTAL].deadband = config.pid.deadband_deg * deadband_scale;
    m_pid_controllers[AXIS_VERTICAL].deadband = config.pid.deadband_deg * deadband_scale;

    float pitch_rad = filtered_angles_rad[AXIS_VERTICAL];

    float target_h_deg = 0.0f;
    float target_v_deg = 0.0f;
    pixel_to_deg(last_target_px_x, last_target_px_y, pitch_rad, target_h_deg, target_v_deg, config.system);

    // 对模型输出的角度也做平滑，进一步抑制抖动
    // target_h_deg = filter_angle_deg(AXIS_HORIZONTAL, target_h_deg, config.pid.angle_filter_alpha);
    // target_v_deg = filter_angle_deg(AXIS_VERTICAL, target_v_deg, config.pid.angle_filter_alpha);

    // static bool s_target_filter_init = false;
    // static float s_target_h_deg = 0.0f;
    // static float s_target_v_deg = 0.0f;

    // const float a = clamp_f(config.pid.angle_filter_alpha, 0.0f, 1.0f);
    // if (!s_target_filter_init)
    // {
    //     s_target_h_deg = target_h_deg;
    //     s_target_v_deg = target_v_deg;
    //     s_target_filter_init = true;
    // }
    // else
    // {
    //     s_target_h_deg = a * target_h_deg + (1.0f - a) * s_target_h_deg;
    //     s_target_v_deg = a * target_v_deg + (1.0f - a) * s_target_v_deg;
    // }
    // target_h_deg = s_target_h_deg;
    // target_v_deg = s_target_v_deg;

    float predicted_h_deg = prediction_apply(m_prediction_states[AXIS_HORIZONTAL], target_h_deg,
                                             config.system.motor_max_angle[AXIS_HORIZONTAL], dt, config.prediction);
    float predicted_v_deg = prediction_apply(m_prediction_states[AXIS_VERTICAL], target_v_deg,
                                             config.system.motor_max_angle[AXIS_VERTICAL], dt, config.prediction);

    float output_h = compute_pid(m_pid_controllers[AXIS_HORIZONTAL], predicted_h_deg, 0.0f, dt, config.pid.guard_epsilon);
    float output_v = compute_pid(m_pid_controllers[AXIS_VERTICAL], predicted_v_deg, 0.0f, dt, config.pid.guard_epsilon);

    if (b_target_proportion_valid)
    {
        // todo：添加根据目标占画面比例调整输出速度的功能
    }

    if (motor_control_desired_speed_to_level(m_motor_controllers[AXIS_HORIZONTAL], config.system, output_h))
    {
        motor_control_set_speed(m_motor_controllers[AXIS_HORIZONTAL], m_hw_interface);
    }

    if (motor_control_desired_speed_to_level(m_motor_controllers[AXIS_VERTICAL], config.system, output_v))
    {
        motor_control_set_speed(m_motor_controllers[AXIS_VERTICAL], m_hw_interface);
    }

    m_stats.control_cycles++;
    m_stats.average_error[AXIS_HORIZONTAL] += std::fabs(target_h_deg);
    m_stats.average_error[AXIS_VERTICAL] += std::fabs(target_v_deg);
}

void pid_control_lib::start()
{
    b_running = true;
    m_last_control_pts = 0U;
    m_last_visible_pts = 0U;
}

void pid_control_lib::stop()
{
    b_running = false;
    motor_control_disable(m_motor_controllers[AXIS_HORIZONTAL], m_hw_interface);
    motor_control_disable(m_motor_controllers[AXIS_VERTICAL], m_hw_interface);
}

void pid_control_lib::reset()
{
    std::memset(&m_stats, 0, sizeof(m_stats));
    m_tracking_log.clear();
    m_tracking_metrics = pid_tracking_metrics_t{};

    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        motor_control_init(m_motor_controllers[axis], static_cast<axis_type_e>(axis));
        prediction_reset(m_prediction_states[axis], m_config.prediction);
        m_pid_controllers[axis].integral = 0.0f;
        m_pid_controllers[axis].prev_error = 0.0f;
        m_pid_controllers[axis].output = 0.0f;
        m_pid_controllers[axis].derivative_filtered = 0.0f;
        m_pid_controllers[axis].b_d_initialized = false;
        m_angle_filter_initialized[axis] = false;
        m_filtered_angles_deg[axis] = 0.0f;
    }
}

bool pid_control_lib::set_pid_params(axis_type_e axis, float kp, float ki, float kd)
{
    if (!b_initialized || axis >= AXIS_TOTAL)
    {
        return false;
    }

    pid_controller_t& pid = m_pid_controllers[axis];
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;

    m_config.pid.axis[axis].kp = kp;
    m_config.pid.axis[axis].ki = ki;
    m_config.pid.axis[axis].kd = kd;

    if (pid.ki > m_config.pid.guard_epsilon)
    {
        const float limit = (pid.max_output * m_config.pid.integral_ratio) / pid.ki;
        pid.integral_max = limit;
        pid.integral_min = -limit;
    }
    else
    {
        pid.integral_max = 0.0f;
        pid.integral_min = 0.0f;
    }

    return true;
}

bool pid_control_lib::get_pid_params(axis_type_e axis, float* p_kp, float* p_ki, float* p_kd) const
{
    if (!b_initialized || axis >= AXIS_TOTAL || !p_kp || !p_ki || !p_kd)
    {
        return false;
    }

    const pid_controller_t& pid = m_pid_controllers[axis];
    *p_kp = pid.kp;
    *p_ki = pid.ki;
    *p_kd = pid.kd;
    return true;
}

bool pid_control_lib::set_prediction_enabled(bool b_enabled)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.features.b_enable_prediction = b_enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_prediction_states[axis].b_prediction_enabled = b_enabled;
    }
    return true;
}

bool pid_control_lib::set_d_filter_enabled(bool b_enabled)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.features.b_enable_d_filter = b_enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_pid_controllers[axis].b_d_filter_enabled = b_enabled;
    }
    return true;
}

bool pid_control_lib::set_prediction_guard_enabled(bool b_enabled)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.features.b_enable_prediction_guard = b_enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_prediction_states[axis].b_guard_enabled = b_enabled;
    }
    return true;
}

bool pid_control_lib::set_adaptive_guard_enabled(bool b_enabled)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.features.b_enable_prediction_adaptive_guard = b_enabled;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_prediction_states[axis].b_adaptive_guard_enabled = b_enabled;
    }
    return true;
}

bool pid_control_lib::get_prediction_enabled() const
{
    if (!b_initialized)
    {
        return false;
    }
    return m_config.features.b_enable_prediction;
}

bool pid_control_lib::get_d_filter_enabled() const
{
    if (!b_initialized)
    {
        return false;
    }
    return m_config.features.b_enable_d_filter;
}

bool pid_control_lib::get_prediction_guard_enabled() const
{
    if (!b_initialized)
    {
        return false;
    }
    return m_config.features.b_enable_prediction_guard;
}

bool pid_control_lib::get_adaptive_guard_enabled() const
{
    if (!b_initialized)
    {
        return false;
    }
    return m_config.features.b_enable_prediction_adaptive_guard;
}

bool pid_control_lib::set_prediction_params(float velocity_alpha, float max_shift_factor, float guard_ratio)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.prediction.velocity_alpha = velocity_alpha;
    m_config.prediction.max_shift_factor = max_shift_factor;
    m_config.prediction.guard_ratio = guard_ratio;
    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        m_prediction_states[axis].guard_ratio = guard_ratio;
    }
    return true;
}

bool pid_control_lib::get_prediction_params(float* p_velocity_alpha, float* p_max_shift_factor,
                                            float* p_guard_ratio) const
{
    if (!b_initialized || !p_velocity_alpha || !p_max_shift_factor || !p_guard_ratio)
    {
        return false;
    }

    *p_velocity_alpha = m_config.prediction.velocity_alpha;
    *p_max_shift_factor = m_config.prediction.max_shift_factor;
    *p_guard_ratio = m_config.prediction.guard_ratio;
    return true;
}

bool pid_control_lib::set_pid_advanced_params(float deadband_deg, float d_filter_alpha, float integral_ratio)
{
    if (!b_initialized)
    {
        return false;
    }

    m_config.pid.deadband_deg = deadband_deg;
    m_config.pid.d_filter_alpha = d_filter_alpha;
    m_config.pid.integral_ratio = integral_ratio;

    for (int axis = 0; axis < AXIS_TOTAL; ++axis)
    {
        pid_controller_t& pid = m_pid_controllers[axis];
        pid.deadband = deadband_deg;
        pid.d_filter_alpha = d_filter_alpha;
        if (pid.ki > m_config.pid.guard_epsilon)
        {
            const float limit = (pid.max_output * integral_ratio) / pid.ki;
            pid.integral_max = limit;
            pid.integral_min = -limit;
        }
    }

    return true;
}

bool pid_control_lib::get_pid_advanced_params(float* p_deadband_deg, float* p_d_filter_alpha,
                                              float* p_integral_ratio) const
{
    if (!b_initialized || !p_deadband_deg || !p_d_filter_alpha || !p_integral_ratio)
    {
        return false;
    }

    *p_deadband_deg = m_config.pid.deadband_deg;
    *p_d_filter_alpha = m_config.pid.d_filter_alpha;
    *p_integral_ratio = m_config.pid.integral_ratio;
    return true;
}

bool pid_control_lib::get_pid_output(axis_type_e axis, float* p_output) const
{
    if (!b_initialized || axis >= AXIS_TOTAL || !p_output)
    {
        return false;
    }

    *p_output = m_pid_controllers[axis].output;
    return true;
}

bool pid_control_lib::get_motor_level(axis_type_e axis, uint8_t* p_level) const
{
    if (!b_initialized || axis >= AXIS_TOTAL || !p_level)
    {
        return false;
    }

    *p_level = m_motor_controllers[axis].level;
    return true;
}

bool pid_control_lib::get_prediction_state(axis_type_e axis, float* p_current_deg, float* p_velocity_deg_per_sec) const
{
    if (!b_initialized || axis >= AXIS_TOTAL || !p_current_deg || !p_velocity_deg_per_sec)
    {
        return false;
    }

    *p_current_deg = m_prediction_states[axis].current_deg;
    *p_velocity_deg_per_sec = m_prediction_states[axis].filtered_velocity_deg_per_sec;
    return true;
}

bool pid_control_lib::get_stats(pid_control_stats_t* p_out_stats) const
{
    if (!b_initialized || !p_out_stats)
    {
        return false;
    }

    *p_out_stats = m_stats;
    if (p_out_stats->control_cycles > 0U)
    {
        const float cycles = static_cast<float>(p_out_stats->control_cycles);
        for (int axis = 0; axis < AXIS_TOTAL; ++axis)
        {
            p_out_stats->average_error[axis] /= cycles;
        }
    }

    return true;
}

void pid_control_lib::reset_stats()
{
    if (b_initialized)
    {
        std::memset(&m_stats, 0, sizeof(m_stats));
    }
}

bool pid_control_lib::get_target_proportion(float* p_proportion) const
{
    if (!b_initialized || !p_proportion)
    {
        return false;
    }
    if (!b_target_proportion_valid)
    {
        return false;
    }
    *p_proportion = m_last_target_proportion;
    return true;
}

bool pid_control_lib::get_tracking_metrics(pid_tracking_metrics_t* p_metrics) const
{
    if (!b_initialized || !p_metrics)
    {
        return false;
    }
    *p_metrics = m_tracking_metrics;
    return true;
}

void pid_control_lib::reset_tracking_metrics()
{
    if (b_initialized)
    {
        m_tracking_metrics = pid_tracking_metrics_t{};
    }
}

size_t pid_control_lib::get_tracking_samples(pid_tracking_sample_t* p_out_samples, size_t max_count) const
{
    if (!b_initialized || !p_out_samples || max_count == 0U)
    {
        return 0U;
    }
    const size_t copy_count = std::min(max_count, m_tracking_log.size());
    std::copy_n(m_tracking_log.begin(), copy_count, p_out_samples);
    return copy_count;
}

void pid_control_lib::clear_tracking_samples()
{
    if (b_initialized)
    {
        m_tracking_log.clear();
    }
}

void pid_control_lib::print_config() const
{
    if (!b_initialized)
    {
        PID_LOG("Library not initialized\n");
        return;
    }

    pid_config_print(&m_config);
}

void pid_control_lib::print_status() const
{
    if (!b_initialized)
    {
        PID_LOG("Library not initialized\n");
        return;
    }

    PID_LOG("=== PID Control Status ===\n");
    PID_LOG("Running: %s\n", b_running ? "YES" : "NO");
    PID_LOG("Control Cycles: %u\n", m_stats.control_cycles);
    PID_LOG("Target Lost Count: %u\n", m_stats.target_lost_count);

    PID_LOG("\nHorizontal Axis:\n");
    PID_LOG("  PID Output: %.2f deg/s\n", m_pid_controllers[AXIS_HORIZONTAL].output);
    PID_LOG("  Motor Level: %u\n", m_motor_controllers[AXIS_HORIZONTAL].level);
    PID_LOG("  Motor Enabled: %s\n", m_motor_controllers[AXIS_HORIZONTAL].enabled ? "YES" : "NO");
    PID_LOG("  Current Angle: %.2f deg\n", m_prediction_states[AXIS_HORIZONTAL].current_deg);
    PID_LOG("  Velocity: %.2f deg/s\n", m_prediction_states[AXIS_HORIZONTAL].filtered_velocity_deg_per_sec);

    PID_LOG("\nVertical Axis:\n");
    PID_LOG("  PID Output: %.2f deg/s\n", m_pid_controllers[AXIS_VERTICAL].output);
    PID_LOG("  Motor Level: %u\n", m_motor_controllers[AXIS_VERTICAL].level);
    PID_LOG("  Motor Enabled: %s\n", m_motor_controllers[AXIS_VERTICAL].enabled ? "YES" : "NO");
    PID_LOG("  Current Angle: %.2f deg\n", m_prediction_states[AXIS_VERTICAL].current_deg);
    PID_LOG("  Velocity: %.2f deg/s\n", m_prediction_states[AXIS_VERTICAL].filtered_velocity_deg_per_sec);

    PID_LOG("\nPrediction Stats:\n");
    PID_LOG("  Total Attempts: %u\n", m_stats.prediction_attempts);
    PID_LOG("  Total Rejects: %u\n", m_stats.prediction_rejects);
    if (m_stats.prediction_attempts > 0U)
    {
#ifdef PID_LOG_ENABLE
        const float success_rate =
            1.0f - (static_cast<float>(m_stats.prediction_rejects) / static_cast<float>(m_stats.prediction_attempts));
        PID_LOG("  Success Rate: %.1f%%\n", success_rate * 100.0f);
#endif
    }

    PID_LOG("==========================\n");
}

bool pid_control_lib::save_config(const char* filename) const
{
    if (!b_initialized || !filename)
    {
        return false;
    }

    return pid_config_save_to_xml(filename, &m_config);
}

pid_control_lib* pid_control_lib::instance()
{
    static pid_control_lib* s_ppid;
    if (s_ppid == nullptr)
    {
        s_ppid = new pid_control_lib();
    }
    return s_ppid;
}