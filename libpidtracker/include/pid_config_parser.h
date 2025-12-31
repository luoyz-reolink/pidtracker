#ifndef PID_CONFIG_PARSER_H
#define PID_CONFIG_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_SPEED_LEVELS 64
#define MAX_FOCUS_LEVELS 64

typedef enum {
    AXIS_HORIZONTAL = 0,
    AXIS_VERTICAL = 1,
    AXIS_TOTAL = 2,
} axis_type_e;

/**
 * @brief 系统相关参数集合。
 * @details 描述图像尺寸、视场角、云台与电机速度/步进等物理与运行限制。所有角度单位为度；速度单位为度/秒。
 */
typedef struct system_parameters_t {
    uint32_t ai_frame_rate_hz; // AI帧率（Hz），仅供默认值参考
    uint32_t max_speed_level; // 最大速度档位数（按轴共用档位数量）
    float speed_levels[AXIS_TOTAL][MAX_SPEED_LEVELS]; // 按轴的速度档位数组，角速度单位 °/s
    float motor_vmin_deg_per_sec; // 开始追踪的最小电机速度（度/秒）
    float motor_vmax_deg_per_sec; // 最大电机速度（度/秒）
    uint32_t motor_min_steps[AXIS_TOTAL];
    uint32_t motor_max_steps[AXIS_TOTAL];
    float motor_min_angle[AXIS_TOTAL];
    float motor_max_angle[AXIS_TOTAL];
    bool motor_orientation[AXIS_TOTAL]; // true: 正向， false: 反向（如需）
    bool b_hanging_upright; // true: 默认正挂， false: 默认倒挂
    uint32_t max_focus_level;
    float focus_levels[MAX_FOCUS_LEVELS];
    float frame_width_pixels;
    float frame_height_pixels;
    float fov_deg[AXIS_TOTAL];
} system_parameters_t;

/**
 * @brief 单轴 PID 参数集合。
 */
typedef struct pid_axis_parameters_t {
    float kp;
    float ki;
    float kd;
} pid_axis_parameters_t;

/**
 * @brief 双轴共享的 PID 参数配置。
 * @param deadband_deg 死区(角度)，小于该误差不输出。
 * @param d_filter_alpha D项一阶低通滤波系数 0~1。
 * @param integral_ratio 积分项限制比例，防止积分过冲。
 * @param guard_epsilon 防守/保护相关微小阈值。
 */
typedef struct pid_parameters_t {
    float deadband_deg;
    float d_filter_alpha;
    float integral_ratio;
    float guard_epsilon;
    float angle_filter_alpha;   // 当前角度平滑滤波系数 0~1，越大越跟随实时值
    float proportion_deadband_scale; // 死区随目标占比的放大系数，effective_deadband = deadband*(1+scale*proportion)
    float proportion_deadband_max;   // 死区放大上限倍数（含1.0，无缩放）
    pid_axis_parameters_t axis[AXIS_TOTAL];
} pid_parameters_t;

/**
 * @brief 目标预测相关参数。
 * @details 控制速度滤波、预测位移限制、保护自适应窗口等。
 */
typedef struct prediction_parameters_t {
    float velocity_alpha;
    float max_shift_factor;
    float guard_ratio;
    float guard_ratio_min;
    float guard_ratio_max;
    uint32_t guard_adapt_window;
    float center_tolerance_deg;
} prediction_parameters_t;

/**
 * @brief 功能开关集合。
 */
typedef struct feature_switches_t {
    bool b_enable_prediction;
    bool b_enable_d_filter;
    bool b_enable_prediction_guard;
    bool b_enable_prediction_adaptive_guard;
} feature_switches_t;

/**
 * @brief PID 控制总配置根结构。
 */
typedef struct pid_control_config_t {
    system_parameters_t system;
    pid_parameters_t pid;
    prediction_parameters_t prediction;
    feature_switches_t features;
} pid_control_config_t;

/**
 * @brief 从 XML 文件加载 PID 配置。
 * @param filename XML 文件路径。
 * @param config 输出配置结构指针，不能为空。
 * @return true 解析成功并填充结构；false 失败（路径/格式问题）。
 */
bool pid_config_load_from_xml(const char *filename, pid_control_config_t *p_config);

/**
 * @brief 将配置保存到 XML 文件。
 * @param filename 目标文件路径。
 * @param config 输入配置结构指针。
 * @return true 保存成功；false 打开/写入失败。
 */
bool pid_config_save_to_xml(const char *filename, const pid_control_config_t *p_config);

/**
 * @brief 写入默认配置到结构体。
 * @param config 待填充的配置指针。
 */
void pid_config_set_defaults(pid_control_config_t *p_config);

/**
 * @brief 打印当前配置（调试用）。
 * @param config 配置指针。
 */
void pid_config_print(const pid_control_config_t *p_config);

#endif /* PID_CONFIG_PARSER_H */