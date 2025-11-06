#ifndef PID_CONTROL_LIB_H
#define PID_CONTROL_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pid_config_parser.h"


/*
 * PID控制库 - 双轴云台控制
 * 支持XML配置文件加载和运行时参数调整
 */

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------ 类型定义 ------------------------------*/

/* PID控制器句柄 */
typedef struct PIDController PIDController;

/* 电机控制句柄 */
typedef struct MotorControl MotorControl;

/* 预测器状态句柄 */
typedef struct TargetPredictionState TargetPredictionState;

/* 库实例句柄 */
typedef struct PIDControlLib PIDControlLib;

/* 轴类型 */
typedef enum {
    AXIS_HORIZONTAL = 0,
    AXIS_VERTICAL = 1,
    AXIS_TOTAL = 2,
} AxisType_E;

/* 硬件接口回调函数类型 */
// 获取目标据中心点的偏移像素数量
typedef bool (*GetTargetPixelPtsCallback)(float *x_pixel, float *y_pixel, uint64_t *pts);
// 获取当前云台的姿态角度（单位：度）
typedef float (*ReadAngleCallback)(uint8_t axis);
// 设置电机速度档位
typedef void (*MotorSetSpeedCallback)(uint8_t axis, uint8_t level, uint8_t direction);
// 防止设置电机速度时电机速度梯度下降不能归0，所以直接强制关闭电机
typedef void (*MotorDisableCallback)(uint8_t axis);

/* 硬件接口结构体 */
typedef struct {
    GetTargetPixelPtsCallback get_target_pixel_position;
    ReadAngleCallback read_angle_deg;
    MotorSetSpeedCallback motor_set_speed;
    MotorDisableCallback motor_disable;
} HardwareInterface;

/*------------------------------ 库管理接口 ------------------------------*/

/* 创建库实例 */
PIDControlLib* pid_control_create(void);

/* 销毁库实例 */
void pid_control_destroy(PIDControlLib* lib);

/* 从XML文件初始化库 */
bool pid_control_init_from_xml(PIDControlLib* lib, const char* config_file, const HardwareInterface* hw_interface);

/* 使用默认配置初始化库 */
bool pid_control_init_default(PIDControlLib* lib, const HardwareInterface* hw_interface);

/* 获取当前配置 */
const PIDControlConfig* pid_control_get_config(const PIDControlLib* lib);

/* 重新加载配置文件 */
bool pid_control_reload_config(PIDControlLib* lib, const char* config_file);

/*------------------------------ 运行时控制接口 ------------------------------*/

/* 执行一次控制循环 */
void pid_control_step(PIDControlLib* lib);

/* 启动/停止控制器 */
void pid_control_start(PIDControlLib* lib);
void pid_control_stop(PIDControlLib* lib);

/* 复位控制器状态 */
void pid_control_reset(PIDControlLib* lib);


/*------------------------------ 参数调整接口 ------------------------------*/

/* 设置PID参数 */
bool pid_control_set_pid_params(PIDControlLib* lib, AxisType_E axis, float kp, float ki, float kd);

/* 获取PID参数 */
bool pid_control_get_pid_params(const PIDControlLib* lib, AxisType_E axis, float* kp, float* ki, float* kd);

/* 设置功能开关 */
bool pid_control_set_prediction_enabled(PIDControlLib* lib, bool enabled);
bool pid_control_set_d_filter_enabled(PIDControlLib* lib, bool enabled);
bool pid_control_set_prediction_guard_enabled(PIDControlLib* lib, bool enabled);
bool pid_control_set_adaptive_guard_enabled(PIDControlLib* lib, bool enabled);

/* 获取功能开关状态 */
bool pid_control_get_prediction_enabled(const PIDControlLib* lib);
bool pid_control_get_d_filter_enabled(const PIDControlLib* lib);
bool pid_control_get_prediction_guard_enabled(const PIDControlLib* lib);
bool pid_control_get_adaptive_guard_enabled(const PIDControlLib* lib);

/* 设置预测器参数 */
bool pid_control_set_prediction_params(PIDControlLib* lib, 
                                       float velocity_alpha, 
                                       float max_shift_factor,
                                       float guard_ratio);

/* 获取预测器参数 */
bool pid_control_get_prediction_params(const PIDControlLib* lib, 
                                       float* velocity_alpha, 
                                       float* max_shift_factor,
                                       float* guard_ratio);

/* 设置PID附加参数 */
bool pid_control_set_pid_advanced_params(PIDControlLib* lib,
                                         float deadband_deg,
                                         float d_filter_alpha,
                                         float integral_ratio);

/* 获取PID附加参数 */
bool pid_control_get_pid_advanced_params(const PIDControlLib* lib,
                                         float* deadband_deg,
                                         float* d_filter_alpha,
                                         float* integral_ratio);

/*------------------------------ 状态查询接口 ------------------------------*/

/* 获取当前PID输出速度 */
bool pid_control_get_pid_output(const PIDControlLib* lib, AxisType_E axis, float* output);

/* 获取当前电机速度档位 */
bool pid_control_get_motor_level(const PIDControlLib* lib, AxisType_E axis, uint8_t* level);

/* 获取预测器状态 */
bool pid_control_get_prediction_state(const PIDControlLib* lib, AxisType_E axis, 
                                     float* current_deg, float* velocity_deg_per_sec);

/* 获取统计信息 */
typedef struct {
    uint32_t control_cycles;
    uint32_t target_lost_count;
    uint32_t prediction_attempts;
    uint32_t prediction_rejects;
    float average_error_horizontal;
    float average_error_vertical;
} PIDControlStats;

bool pid_control_get_stats(const PIDControlLib* lib, PIDControlStats* stats);
void pid_control_reset_stats(PIDControlLib* lib);

/*------------------------------ 调试接口 ------------------------------*/

/* 打印当前配置 */
void pid_control_print_config(const PIDControlLib* lib);

/* 打印运行状态 */
void pid_control_print_status(const PIDControlLib* lib);

/* 保存当前配置到文件 */
bool pid_control_save_config(const PIDControlLib* lib, const char* filename);

/*------------------------------ 兼容性宏定义 ------------------------------*/

/* 为了保持与原代码的兼容性，提供一些宏定义 */
#define FRAME_PERIOD_SECONDS(lib) (1.0f / (float)pid_control_get_config(lib)->system.ai_frame_rate_hz)
#define FRAME_PERIOD_MS(lib) (1000U / pid_control_get_config(lib)->system.ai_frame_rate_hz)
#define MOTOR_VMAX_DEG_PER_SEC(lib) (pid_control_get_config(lib)->system.motor_vmax_deg_per_sec)
#define SPEED_LEVELS(lib) (pid_control_get_config(lib)->system.speed_levels)

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_LIB_H */