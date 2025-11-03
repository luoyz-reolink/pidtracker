#ifndef PID_CONFIG_PARSER_H
#define PID_CONFIG_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_SPEED_LEVELS 64

/* 配置结构体定义 */
typedef struct {
    uint32_t ai_frame_rate_hz;
    uint32_t max_speed_level;
    // 速度表，角速度单位 °/s
    float speed_levels[MAX_SPEED_LEVELS];
    float frame_width_pixels;
    float frame_height_pixels;
    float horizontal_fov_deg;
    float vertical_fov_deg;
} SystemParameters;

typedef struct {
    float kp;
    float ki;
    float kd;
} PIDAxisParameters;

typedef struct {
    float deadband_deg;
    float d_filter_alpha;
    float integral_ratio;
    float guard_epsilon;
    PIDAxisParameters horizontal;
    PIDAxisParameters vertical;
} PIDParameters;

typedef struct {
    float velocity_alpha;
    float max_shift_factor;
    float guard_ratio;
    float guard_ratio_min;
    float guard_ratio_max;
    uint32_t guard_adapt_window;
    float center_tolerance_deg;
} PredictionParameters;

typedef struct {
    bool enable_prediction;
    bool enable_d_filter;
    bool enable_prediction_guard;
    bool enable_prediction_adaptive_guard;
} FeatureSwitches;

typedef struct {
    SystemParameters system;
    PIDParameters pid;
    PredictionParameters prediction;
    FeatureSwitches features;
} PIDControlConfig;

/* XML解析函数 */
bool pid_config_load_from_xml(const char *filename, PIDControlConfig *config);
bool pid_config_save_to_xml(const char *filename, const PIDControlConfig *config);
void pid_config_set_defaults(PIDControlConfig *config);
void pid_config_print(const PIDControlConfig *config);

#endif /* PID_CONFIG_PARSER_H */