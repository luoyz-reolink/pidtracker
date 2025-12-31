#ifndef PID_CONTROL_LIB_H
#define PID_CONTROL_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pid_config_parser.h"
#include "pid_tool.h"


/*
 * PID控制库 - 双轴云台控制
 * 支持XML配置文件加载和运行时参数调整
 */

/*------------------------------ 函数接口定义 ------------------------------*/

/**
 * @brief 获取当前目标像素坐标与时间戳回调。
 */
typedef bool (*get_target_pixel_pts_callback)(void *p_user_ctx, float *p_x_pixel, float *p_y_pixel, uint64_t *p_pts, float *p_proportion);
/**
 * @brief 读取当前角度回调。
 */
typedef int32_t (*read_pos_callback)(void *p_user_ctx, uint8_t axis);
/**
 * @brief 设置电机速度与方向回调。
 */
typedef void (*motor_set_speed_callback)(void *p_user_ctx, uint8_t axis, uint8_t level, uint8_t direction);
/**
 * @brief 停止/关闭电机回调。
 */
typedef void (*motor_disable_callback)(void *p_user_ctx, uint8_t axis);

/* 硬件接口结构体 */
typedef struct hardware_interface_t {
    void *p_user_ctx;
    get_target_pixel_pts_callback p_get_target_pixel_position;
    read_pos_callback p_read_pos;
    motor_set_speed_callback p_motor_set_speed;
    motor_disable_callback p_motor_disable;
} hardware_interface_t;

#ifdef __cplusplus
#include <vector>
#include <algorithm>
#include <cmath>

struct pid_controller_t {
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
    bool b_d_initialized = false;
    bool b_d_filter_enabled = true;
};

struct motor_control_t {
    axis_type_e axis = AXIS_HORIZONTAL;
    uint8_t enabled = 0U;
    uint8_t level = 0U;
    uint8_t direction = 1U;
    uint32_t min_deg = 0U;
    uint32_t max_deg = 0U;
    motor_set_speed_callback p_set_speed = nullptr;
    motor_disable_callback p_disable_motor = nullptr;
};

struct target_prediction_state_t {
    float current_deg = 0.0f;
    float previous_deg = 0.0f;
    float filtered_velocity_deg_per_sec = 0.0f;
    bool b_has_previous_sample = false;
    float guard_ratio = 0.0f;
    uint32_t guard_attempts = 0U;
    uint32_t guard_rejects = 0U;
    bool b_prediction_enabled = true;
    bool b_guard_enabled = true;
    bool b_adaptive_guard_enabled = true;
};

typedef struct pid_control_stats_t {
    uint32_t control_cycles;
    uint32_t target_lost_count;
    uint32_t prediction_attempts;
    uint32_t prediction_rejects;
    float average_error[AXIS_TOTAL];
} pid_control_stats_t;

typedef struct pid_tracking_sample_t {
    float target_px_x;
    float target_px_y;
    uint8_t target_valid;
    uint8_t motor_level[AXIS_TOTAL];
    uint8_t motor_direction[AXIS_TOTAL];
    float offset_px[AXIS_TOTAL];
} pid_tracking_sample_t;

typedef struct pid_tracking_metrics_t {
    float max_offset_px[AXIS_TOTAL];
    double cumulative_offset_px[AXIS_TOTAL];
    uint32_t sample_count;
    uint32_t bound_violations[AXIS_TOTAL];
} pid_tracking_metrics_t;

class pid_control_lib {
public:
    static pid_control_lib* instance();
    pid_control_lib(const pid_control_lib&) = delete;
    pid_control_lib& operator=(const pid_control_lib&) = delete;
    pid_control_lib(pid_control_lib&&) = delete;
    pid_control_lib& operator=(pid_control_lib&&) = delete;

    /*---------------------------------执行接口---------------------------------*/
    bool init_from_xml(const char* p_config_file, const hardware_interface_t* p_hw_interface);
    bool init_default(const hardware_interface_t* p_hw_interface);
    bool reload_config(const char* p_config_file);
    void step();
    void start();
    void stop();
    void reset();

    /*---------------------------------参数接口---------------------------------*/
    bool set_pid_params(axis_type_e axis, float kp, float ki, float kd);
    bool get_pid_params(axis_type_e axis, float* p_kp, float* p_ki, float* p_kd) const;
    bool set_prediction_enabled(bool b_enabled);
    bool set_d_filter_enabled(bool b_enabled);
    bool set_prediction_guard_enabled(bool b_enabled);
    bool set_adaptive_guard_enabled(bool b_enabled);
    bool get_prediction_enabled() const;
    bool get_d_filter_enabled() const;
    bool get_prediction_guard_enabled() const;
    bool get_adaptive_guard_enabled() const;
    bool set_prediction_params(float velocity_alpha, float max_shift_factor, float guard_ratio);
    bool get_prediction_params(float* p_velocity_alpha, float* p_max_shift_factor, float* p_guard_ratio) const;
    bool set_pid_advanced_params(float deadband_deg, float d_filter_alpha, float integral_ratio);
    bool get_pid_advanced_params(float* p_deadband_deg, float* p_d_filter_alpha, float* p_integral_ratio) const;
    bool get_pid_output(axis_type_e axis, float* p_output) const;
    bool get_motor_level(axis_type_e axis, uint8_t* p_level) const;
    bool get_prediction_state(axis_type_e axis, float* p_current_deg, float* p_velocity_deg_per_sec) const;
    bool get_stats(pid_control_stats_t* p_stats) const;
    void reset_stats();
    bool get_target_proportion(float* p_proportion) const;
    bool get_tracking_metrics(pid_tracking_metrics_t* p_metrics) const;
    void reset_tracking_metrics();
    size_t get_tracking_samples(pid_tracking_sample_t* p_out_samples, size_t max_count) const;
    void clear_tracking_samples();
    void print_config() const;
    void print_status() const;
    bool save_config(const char* p_filename) const;
    const pid_control_config_t* get_config() const;

private:
    pid_control_lib() = default;
    ~pid_control_lib();

    pid_control_config_t m_config{};
    pid_controller_t m_pid_controllers[AXIS_TOTAL]{};
    motor_control_t m_motor_controllers[AXIS_TOTAL]{};
    target_prediction_state_t m_prediction_states[AXIS_TOTAL]{};
    hardware_interface_t m_hw_interface{};
    pid_control_stats_t m_stats{};
    pid_tracking_metrics_t m_tracking_metrics{};
    std::vector<pid_tracking_sample_t> m_tracking_log;
    uint64_t m_last_control_pts = 0U;
    uint64_t m_last_visible_pts = 0U;
    float m_last_target_proportion = 0.0f;
    float m_filtered_angles_deg[AXIS_TOTAL] = {0.0f, 0.0f};
    bool m_angle_filter_initialized[AXIS_TOTAL] = {false, false};
    bool b_target_proportion_valid = false;
    bool b_initialized = false;
    bool b_running = false;

    void update_pid_from_config(pid_controller_t& pid, const pid_parameters_t& pid_config, const pid_axis_parameters_t& axis_config, float max_output, float guard_epsilon);
    void update_prediction_from_config(target_prediction_state_t& predict, const prediction_parameters_t& pred_config, const feature_switches_t& features);
    void prediction_reset(target_prediction_state_t& state, const prediction_parameters_t& config);
    void motor_control_init(motor_control_t& motor, axis_type_e axis);
    bool motor_control_desired_speed_to_level(motor_control_t& motor, const system_parameters_t& sys, float desired_speed);
    void motor_control_set_speed(const motor_control_t& motor, const hardware_interface_t& hw);
    void motor_control_disable(const motor_control_t& motor, const hardware_interface_t& hw);
    float prediction_apply(target_prediction_state_t& state, float measured_deg, float range_limit_deg, float dt, const prediction_parameters_t& config);
    float compute_pid(pid_controller_t& pid, float target, float feedback, float dt, float guard_epsilon);
    void update_tracking_analysis(float target_px_x, float target_px_y, bool b_has_measurement);
    float filter_angle_deg(axis_type_e axis, float measured_deg, float alpha);
};

#else
typedef struct pid_controller_t pid_controller_t;
typedef struct motor_control_t motor_control_t;
typedef struct target_prediction_state_t target_prediction_state_t;
typedef struct pid_control_lib pid_control_lib;
typedef struct pid_control_stats_t pid_control_stats_t;
typedef struct pid_tracking_sample_t pid_tracking_sample_t;
typedef struct pid_tracking_metrics_t pid_tracking_metrics_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------ 库管理接口 ------------------------------*/

pid_control_lib* pid_control_create(void);
void pid_control_destroy(pid_control_lib* p_lib);

bool pid_control_init_from_xml(pid_control_lib* p_lib, const char* p_config_file, const hardware_interface_t* p_hw_interface);
bool pid_control_init_default(pid_control_lib* p_lib, const hardware_interface_t* p_hw_interface);
const pid_control_config_t* pid_control_get_config(const pid_control_lib* p_lib);
bool pid_control_reload_config(pid_control_lib* p_lib, const char* p_config_file);

/*------------------------------ 运行时控制接口 ------------------------------*/

void pid_control_step(pid_control_lib* p_lib);
void pid_control_start(pid_control_lib* p_lib);
void pid_control_stop(pid_control_lib* p_lib);
void pid_control_reset(pid_control_lib* p_lib);

/*------------------------------ 参数调整接口 ------------------------------*/

bool pid_control_set_pid_params(pid_control_lib* p_lib, axis_type_e axis, float kp, float ki, float kd);
bool pid_control_get_pid_params(const pid_control_lib* p_lib, axis_type_e axis, float* p_kp, float* p_ki, float* p_kd);
bool pid_control_set_prediction_enabled(pid_control_lib* p_lib, bool b_enabled);
bool pid_control_set_d_filter_enabled(pid_control_lib* p_lib, bool b_enabled);
bool pid_control_set_prediction_guard_enabled(pid_control_lib* p_lib, bool b_enabled);
bool pid_control_set_adaptive_guard_enabled(pid_control_lib* p_lib, bool b_enabled);
bool pid_control_get_prediction_enabled(const pid_control_lib* p_lib);
bool pid_control_get_d_filter_enabled(const pid_control_lib* p_lib);
bool pid_control_get_prediction_guard_enabled(const pid_control_lib* p_lib);
bool pid_control_get_adaptive_guard_enabled(const pid_control_lib* p_lib);
bool pid_control_set_prediction_params(pid_control_lib* p_lib, float velocity_alpha, float max_shift_factor, float guard_ratio);
bool pid_control_get_prediction_params(const pid_control_lib* p_lib, float* p_velocity_alpha, float* p_max_shift_factor, float* p_guard_ratio);
bool pid_control_set_pid_advanced_params(pid_control_lib* p_lib, float deadband_deg, float d_filter_alpha, float integral_ratio);
bool pid_control_get_pid_advanced_params(const pid_control_lib* p_lib, float* p_deadband_deg, float* p_d_filter_alpha, float* p_integral_ratio);

/*------------------------------ 状态查询接口 ------------------------------*/

bool pid_control_get_pid_output(const pid_control_lib* p_lib, axis_type_e axis, float* p_output);
bool pid_control_get_motor_level(const pid_control_lib* p_lib, axis_type_e axis, uint8_t* p_level);
bool pid_control_get_prediction_state(const pid_control_lib* p_lib, axis_type_e axis, float* p_current_deg, float* p_velocity_deg_per_sec);
bool pid_control_get_stats(const pid_control_lib* p_lib, pid_control_stats_t* p_stats);
void pid_control_reset_stats(pid_control_lib* p_lib);
bool pid_control_get_target_proportion(const pid_control_lib* p_lib, float* p_proportion);
bool pid_control_get_tracking_metrics(const pid_control_lib* p_lib, pid_tracking_metrics_t* p_metrics);
void pid_control_reset_tracking_metrics(pid_control_lib* p_lib);
size_t pid_control_get_tracking_samples(const pid_control_lib* p_lib, pid_tracking_sample_t* p_out_samples, size_t max_count);
void pid_control_clear_tracking_samples(pid_control_lib* p_lib);

/*------------------------------ 调试接口 ------------------------------*/

void pid_control_print_config(const pid_control_lib* p_lib);
void pid_control_print_status(const pid_control_lib* p_lib);
bool pid_control_save_config(const pid_control_lib* p_lib, const char* p_filename);

/*------------------------------ 兼容性宏定义 ------------------------------*/

#define FRAME_PERIOD_SECONDS(lib) (1.0f / (float)pid_control_get_config(lib)->system.ai_frame_rate_hz)
#define FRAME_PERIOD_MS(lib) (1000U / pid_control_get_config(lib)->system.ai_frame_rate_hz)
#define MOTOR_VMIN_DEG_PER_SEC(lib) (pid_control_get_config(lib)->system.motor_vmin_deg_per_sec)
#define MOTOR_VMAX_DEG_PER_SEC(lib) (pid_control_get_config(lib)->system.motor_vmax_deg_per_sec)
#define SPEED_LEVELS(lib, axis) (pid_control_get_config(lib)->system.speed_levels[(axis)])

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_LIB_H */