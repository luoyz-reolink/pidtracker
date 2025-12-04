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

/**
 * @brief PID 控制器内部状态句柄前置声明。
 * @note 具体结构在实现文件中，仅通过指针使用，避免外部直接访问内部成员。
 */
typedef struct PIDController PIDController;

/**
 * @brief 电机控制状态前置声明。
 */
typedef struct MotorControl MotorControl;

/**
 * @brief 目标预测内部状态前置声明。
 */
typedef struct TargetPredictionState TargetPredictionState;

/**
 * @brief PID 控制库实例前置声明。
 */
typedef struct PIDControlLib PIDControlLib;

/**
 * @brief 云台支持的轴类型。
 */
typedef enum {
    AXIS_HORIZONTAL = 0,
    AXIS_VERTICAL = 1,
    AXIS_TOTAL = 2,
} AxisType_E;

/**
 * @brief 获取当前目标像素坐标与时间戳回调。
 * @param user_ctx 外部上下文（透传）。
 * @param x_pixel 输出目标中心 X 像素。
 * @param y_pixel 输出目标中心 Y 像素。
 * @param pts 输出时间戳（微秒或毫秒，依据来源）。
 * @return true 成功；false 失败或当前无目标。
 */
typedef bool (*GetTargetPixelPtsCallback)(void *user_ctx, float *x_pixel, float *y_pixel, uint64_t *pts);
/**
 * @brief 读取当前角度回调。
 * @param user_ctx 外部上下文。
 * @param axis 轴类型（水平/垂直）。
 * @return 当前角度（度）。
 */
typedef float (*ReadAngleCallback)(void *user_ctx, uint8_t axis);
/**
 * @brief 设置电机速度与方向回调。
 * @param user_ctx 外部上下文。
 * @param axis 轴类型。
 * @param level 档位（速度级别）。
 * @param direction 方向（实现自定义，0/1）。
 */
typedef void (*MotorSetSpeedCallback)(void *user_ctx, uint8_t axis, uint8_t level, uint8_t direction);
/**
 * @brief 停止/关闭电机回调。
 */
typedef void (*MotorDisableCallback)(void *user_ctx, uint8_t axis);

/* 硬件接口结构体 */
struct HardwareInterface{
    void *user_ctx; // 由外部 (例如 PID_Tracker 实例) 设置，库仅透传
    GetTargetPixelPtsCallback get_target_pixel_position;
    ReadAngleCallback read_angle;
    MotorSetSpeedCallback motor_set_speed;
    MotorDisableCallback motor_disable;
    HardwareInterface() : user_ctx(nullptr),
                          get_target_pixel_position(nullptr),
                          read_angle(nullptr),
                          motor_set_speed(nullptr),
                          motor_disable(nullptr) {}
};

/*------------------------------ 库管理接口 ------------------------------*/

/**
 * @brief 创建 PID 控制库实例。
 * @return 返回新建指针；失败返回 nullptr。
 */
PIDControlLib* pid_control_create(void);

/**
 * @brief 销毁库实例并释放资源。
 */
void pid_control_destroy(PIDControlLib* lib);

/**
 * @brief 使用 XML 配置初始化库。
 * @param lib 库实例指针。
 * @param config_file XML 文件路径。
 * @param hw_interface 外部硬件接口（回调集合）。
 * @return true 初始化成功；false 参数错误或解析失败。
 */
bool pid_control_init_from_xml(PIDControlLib* lib, const char* config_file, const HardwareInterface* hw_interface);

/**
 * @brief 使用默认配置初始化库。
 * @param lib 库实例指针。
 * @param hw_interface 硬件接口。
 * @return true 成功；false 失败。
 */
bool pid_control_init_default(PIDControlLib* lib, const HardwareInterface* hw_interface);

/**
 * @brief 获取当前配置对象（只读）。
 * @param lib 库实例。
 * @return 配置指针；未初始化返回 nullptr。
 */
const PIDControlConfig* pid_control_get_config(const PIDControlLib* lib);

/**
 * @brief 重新加载 XML 配置并覆盖当前参数。
 * @param lib 库实例（必须已初始化）。
 * @param config_file XML 路径。
 * @return true 成功；false 失败。
 */
bool pid_control_reload_config(PIDControlLib* lib, const char* config_file);

/*------------------------------ 运行时控制接口 ------------------------------*/

/**
 * @brief 执行一次控制循环（读取目标、计算预测与 PID 输出、驱动电机）。
 * @param lib 库实例。
 */
void pid_control_step(PIDControlLib* lib);

/**
 * @brief 启动控制循环（置运行标志）。
 */
void pid_control_start(PIDControlLib* lib);
/**
 * @brief 停止控制循环并关闭电机输出。
 */
void pid_control_stop(PIDControlLib* lib);

/**
 * @brief 重置内部状态（积分、预测、统计、输出等）。
 */
void pid_control_reset(PIDControlLib* lib);


/*------------------------------ 参数调整接口 ------------------------------*/

/**
 * @brief 设置指定轴的基础 PID 参数。
 * @param lib 库实例。
 * @param axis 轴类型。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @return true 成功；false 轴非法或库未就绪。
 */
bool pid_control_set_pid_params(PIDControlLib* lib, AxisType_E axis, float kp, float ki, float kd);

/**
 * @brief 获取指定轴当前 PID 参数。
 * @param lib 库实例。
 * @param axis 轴类型。
 * @param kp 输出比例系数。
 * @param ki 输出积分系数。
 * @param kd 输出微分系数。
 * @return true 成功；false 失败。
 */
bool pid_control_get_pid_params(const PIDControlLib* lib, AxisType_E axis, float* kp, float* ki, float* kd);

/** @brief 开启/关闭预测功能。 */
bool pid_control_set_prediction_enabled(PIDControlLib* lib, bool enabled);
/** @brief 开启/关闭 D 滤波功能。 */
bool pid_control_set_d_filter_enabled(PIDControlLib* lib, bool enabled);
/** @brief 开启/关闭 预测保护功能（目标异常抑制）。 */
bool pid_control_set_prediction_guard_enabled(PIDControlLib* lib, bool enabled);
/** @brief 开启/关闭 自适应保护调节。 */
bool pid_control_set_adaptive_guard_enabled(PIDControlLib* lib, bool enabled);

/** @brief 是否启用预测。 */
bool pid_control_get_prediction_enabled(const PIDControlLib* lib);
/** @brief 是否启用 D 滤波。 */
bool pid_control_get_d_filter_enabled(const PIDControlLib* lib);
/** @brief 是否启用预测保护。 */
bool pid_control_get_prediction_guard_enabled(const PIDControlLib* lib);
/** @brief 是否启用自适应保护。 */
bool pid_control_get_adaptive_guard_enabled(const PIDControlLib* lib);

/**
 * @brief 设置预测相关参数。
 * @param velocity_alpha 速度滤波系数。
 * @param max_shift_factor 目标最大位移系数（归一化）。
 * @param guard_ratio 保护比率中心值。
 */
bool pid_control_set_prediction_params(PIDControlLib* lib, 
                                       float velocity_alpha, 
                                       float max_shift_factor,
                                       float guard_ratio);

/**
 * @brief 获取预测参数。
 */
bool pid_control_get_prediction_params(const PIDControlLib* lib, 
                                       float* velocity_alpha, 
                                       float* max_shift_factor,
                                       float* guard_ratio);

/**
 * @brief 设置高级 PID 参数（死区、滤波、积分限制比例）。
 */
bool pid_control_set_pid_advanced_params(PIDControlLib* lib,
                                         float deadband_deg,
                                         float d_filter_alpha,
                                         float integral_ratio);

/**
 * @brief 获取高级 PID 参数。
 */
bool pid_control_get_pid_advanced_params(const PIDControlLib* lib,
                                         float* deadband_deg,
                                         float* d_filter_alpha,
                                         float* integral_ratio);

/*------------------------------ 状态查询接口 ------------------------------*/

/**
 * @brief 获取某轴当前 PID 输出（电机期望速度）。
 */
bool pid_control_get_pid_output(const PIDControlLib* lib, AxisType_E axis, float* output);

/**
 * @brief 获取某轴当前电机运行档位。
 */
bool pid_control_get_motor_level(const PIDControlLib* lib, AxisType_E axis, uint8_t* level);

/**
 * @brief 获取预测器当前状态（角度与速度）。
 */
bool pid_control_get_prediction_state(const PIDControlLib* lib, AxisType_E axis, 
                                     float* current_deg, float* velocity_deg_per_sec);

/**
 * @brief 获取控制循环统计信息。
 */
typedef struct {
    uint32_t control_cycles;
    uint32_t target_lost_count;
    uint32_t prediction_attempts;
    uint32_t prediction_rejects;
    float average_error_horizontal;
    float average_error_vertical;
} PIDControlStats;

/** @brief 获取统计数据。 */
bool pid_control_get_stats(const PIDControlLib* lib, PIDControlStats* stats);
/** @brief 重置统计数据。 */
void pid_control_reset_stats(PIDControlLib* lib);

/*------------------------------ 调试接口 ------------------------------*/

/** @brief 打印当前 PID 配置。 */
void pid_control_print_config(const PIDControlLib* lib);

/** @brief 打印当前运行状态（调试用）。 */
void pid_control_print_status(const PIDControlLib* lib);

/** @brief 保存当前配置到文件。 */
bool pid_control_save_config(const PIDControlLib* lib, const char* filename);

/*------------------------------ 兼容性宏定义 ------------------------------*/

/**
 * @brief 兼容性宏：计算一帧周期（秒）。
 */
#define FRAME_PERIOD_SECONDS(lib) (1.0f / (float)pid_control_get_config(lib)->system.ai_frame_rate_hz)
/** @brief 兼容性宏：计算一帧周期（毫秒）。 */
#define FRAME_PERIOD_MS(lib) (1000U / pid_control_get_config(lib)->system.ai_frame_rate_hz)
/** @brief 最小电机速度（度/秒）。 */
#define MOTOR_VMIN_DEG_PER_SEC(lib) (pid_control_get_config(lib)->system.motor_vmin_deg_per_sec)
/** @brief 最大电机速度（度/秒）。 */
#define MOTOR_VMAX_DEG_PER_SEC(lib) (pid_control_get_config(lib)->system.motor_vmax_deg_per_sec)
/** @brief 速度档位数组访问。 */
#define SPEED_LEVELS(lib) (pid_control_get_config(lib)->system.speed_levels)

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_LIB_H */