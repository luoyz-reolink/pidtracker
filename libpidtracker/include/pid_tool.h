#ifndef _PID_TOOL_H_
#define _PID_TOOL_H_

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <array>

#ifdef PID_LOG_ENABLE
    #define PID_LOG(fmt, ...) printf("[PID_Control_Lib] " fmt, ##__VA_ARGS__)   
#else
    #define PID_LOG(fmt, ...)
#endif

// ---------------------------------PID所需的工具函数-------------------------------------

inline float clamp_f(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

inline float deg_to_rad(float degrees) {
    return degrees * static_cast<float>(M_PI) / 180.0f;
}

inline float rad_to_deg(float radians) {
    return radians * 180.0f / static_cast<float>(M_PI);
}

// 计算2个2维向量的夹角，返回角度值
inline float vector2d_angle_deg(const std::array<float, 2>& v1, const std::array<float, 2>& v2) {
    float cross = v1[0] * v2[1] - v1[1] * v2[0];
    float dot = v1[0] * v2[0] + v1[1] * v2[1];
    return rad_to_deg(std::atan2(cross, dot));
}

#endif // _PID_TOOL_H_