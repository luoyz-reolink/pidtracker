#include "../include/pid_config_parser.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>
#include <iostream>

#include "tinyxml.h"

namespace {
constexpr uint32_t kDefaultFrameRateHz = 10U;
constexpr uint32_t kDefaultMaxSpeedLevel = 16U;
constexpr float kDefaultMotorVminDegPerSec = 9.0f;
constexpr float kDefaultMotorVmaxDegPerSec = 90.0f;
constexpr int32_t kDefaultMotorPMinSteps = 0;
constexpr int32_t kDefaultMotorPMaxSteps = 2700;
constexpr float kDefaultMotorPMinAngle = 0.0f;
constexpr float kDefaultMotorPMaxAngle = 360.0f;
constexpr int32_t kDefaultMotorTMinSteps = 0;
constexpr int32_t kDefaultMotorTMaxSteps = 1024;
constexpr bool kDefaultMotorTOrientation = true;
constexpr bool kDefaultHangingUpright = false;
constexpr float kDefaultMotorTMinAngle = 0.0f;
constexpr float kDefaultMotorTMaxAngle = 90.0f;
constexpr float kDefaultFrameWidthPixels = 640.0f;
constexpr float kDefaultFrameHeightPixels = 352.0f;
constexpr float kDefaultHorizontalFovDeg = 60.0f;
constexpr float kDefaultVerticalFovDeg = 45.0f;
constexpr float kDefaultDeadbandDeg = 0.02f;
constexpr float kDefaultFilterAlpha = 0.2f;
constexpr float kDefaultIntegralRatio = 0.8f;
constexpr float kDefaultGuardEpsilon = 1.0e-6f;
constexpr float kDefaultAngleFilterAlpha = 0.6f;
constexpr float kDefaultProportionDeadbandScale = 1.0f;
constexpr float kDefaultProportionDeadbandMax = 3.0f;
constexpr float kDefaultPidKp = 0.8f;
constexpr float kDefaultPidKi = 0.15f;
constexpr float kDefaultPidKd = 0.05f;
constexpr float kDefaultVelocityAlpha = 0.5f;
constexpr float kDefaultMaxShiftFactor = 0.3f;
constexpr float kDefaultGuardRatio = 0.5f;
constexpr float kDefaultGuardRatioMin = 0.3f;
constexpr float kDefaultGuardRatioMax = 0.6f;
constexpr uint32_t kDefaultGuardAdaptWindow = 30U;
constexpr float kDefaultCenterToleranceDeg = 0.3f;


template <typename T>
typename std::enable_if<std::is_same<T, bool>::value, bool>::type
parse_attr(TiXmlElement *elem, const char *key, T &out) {
    if (!elem || !key) return false;
    bool tmp = false;
    if (elem->QueryBoolAttribute(key, &tmp) == TIXML_SUCCESS) {
        out = tmp;
        return true;
    }
    std::cout << "Failed to parse bool attribute: " << key << std::endl;
    return false;
}

template <typename T>
typename std::enable_if<std::is_same<T, int32_t>::value, bool>::type
parse_attr(TiXmlElement *elem, const char *key, T &out) {
    if (!elem || !key) return false;
    int tmp = 0;
    if (elem->QueryIntAttribute(key, &tmp) == TIXML_SUCCESS) {
        out = static_cast<T>(tmp);
        return true;
    }
    std::cout << "Failed to parse int attribute: " << key << std::endl;
    return false;
}

template <typename T>
typename std::enable_if<std::is_same<T, uint32_t>::value, bool>::type
parse_attr(TiXmlElement *elem, const char *key, T &out) {
    if (!elem || !key) return false;
    unsigned int tmp = 0;
    if (elem->QueryUnsignedAttribute(key, &tmp) == TIXML_SUCCESS) {
        out = static_cast<T>(tmp);
        return true;
    }
    std::cout << "Failed to parse uint attribute: " << key << std::endl;
    return false;
}

template <typename T>
typename std::enable_if<std::is_same<T, float>::value, bool>::type
parse_attr(TiXmlElement *elem, const char *key, T &out) {
    if (!elem || !key) return false;
    float tmp = 0.0f;
    if (elem->QueryFloatAttribute(key, &tmp) == TIXML_SUCCESS) {
        out = tmp;
        return true;
    }
    std::cout << "Failed to parse float attribute: " << key << std::endl;
    return false;
}

template <typename T>
typename std::enable_if<std::is_same<T, double>::value, bool>::type
parse_attr(TiXmlElement *elem, const char *key, T &out) {
    if (!elem || !key) return false;
    double tmp = 0.0;
    if (elem->QueryDoubleAttribute(key, &tmp) == TIXML_SUCCESS) {
        out = tmp;
        return true;
    }
    std::cout << "Failed to parse double attribute: " << key << std::endl;
    return false;
}

template<typename T>
static bool parse_series(TiXmlElement *elem, const char *prefix, int count, std::vector<T> &dst) {
    if (!elem) return true;
    dst.clear();
    dst.reserve(count);
    for (int i = 0; i < count; ++i) {
        char key[16];
        std::snprintf(key, sizeof(key), "%s%d", prefix, i);
        T v = 0;
        if (!parse_attr(elem, key, v)) return false;
        dst.push_back(v);
    }
    return true;
}

template <typename T>
static bool parse_child_val(TiXmlElement* parent, const char* child_name, T& out) {
    if (!parent || !child_name) return false;
    printf("Parsing child: %s\n", child_name);
    if (TiXmlElement* child = parent->FirstChildElement(child_name)) {
        return parse_attr(child, "val", out);
    }
    return false;
}


inline void FillDefaultSpeedLevels(pid_control_config_t* p_config) {
    if (!p_config) {
        return;
    }

    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        std::fill_n(p_config->system.speed_levels[axis],
                    MAX_SPEED_LEVELS,
                    std::numeric_limits<float>::max());
    }

    const uint32_t capped_max_level = std::min(p_config->system.max_speed_level, static_cast<uint32_t>(MAX_SPEED_LEVELS));
    for (uint32_t i = 0; i < capped_max_level; ++i) {
        float v = static_cast<float>(i + 1U);
        p_config->system.speed_levels[AXIS_HORIZONTAL][i] = v;
        p_config->system.speed_levels[AXIS_VERTICAL][i] = v;
    }
}

inline void FillDefaultFocusLevels(pid_control_config_t* p_config) {
    if (!p_config) {
        return;
    }

    std::fill_n(p_config->system.focus_levels,
                MAX_FOCUS_LEVELS,
                std::numeric_limits<float>::max());

    for (uint32_t i = 0; i < MAX_FOCUS_LEVELS; ++i) {
        p_config->system.focus_levels[i] = static_cast<float>(i);
    }
}
inline bool GetChildFloat(TiXmlElement* parent, const char* child_name, float& dest) {
    if (!parent || !child_name) return false;
    if (TiXmlElement* child = parent->FirstChildElement(child_name)) {
        if (const char* txt = child->GetText()) {
            dest = static_cast<float>(std::strtod(txt, nullptr));
            return true;
        }
    }
    return false;
}

inline bool GetChildUnsigned(TiXmlElement* parent, const char* child_name, uint32_t& dest) {
    if (!parent || !child_name) return false;
    if (TiXmlElement* child = parent->FirstChildElement(child_name)) {
        if (const char* txt = child->GetText()) {
            unsigned long v = std::strtoul(txt, nullptr, 10);
            dest = static_cast<uint32_t>(v);
            return true;
        }
    }
    return false;
}

inline bool GetChildBool(TiXmlElement* parent, const char* child_name, bool& dest) {
    if (!parent || !child_name) return false;
    if (TiXmlElement* child = parent->FirstChildElement(child_name)) {
        if (const char* txt = child->GetText()) {
            std::string s(txt);
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
            if (s == "1" || s == "true") {
                dest = true;
            } else if (s == "0" || s == "false") {
                dest = false;
            }
            return true;
        }
    }
    return false;
}
}  // namespace

void pid_config_set_defaults(pid_control_config_t* p_config) {
    if (!p_config) {
        return;
    }

    p_config->system.ai_frame_rate_hz = kDefaultFrameRateHz;
    p_config->system.max_speed_level = kDefaultMaxSpeedLevel;
    p_config->system.motor_vmin_deg_per_sec = kDefaultMotorVminDegPerSec;
    p_config->system.motor_vmax_deg_per_sec = kDefaultMotorVmaxDegPerSec;
    p_config->system.motor_min_steps[AXIS_HORIZONTAL] = kDefaultMotorPMinSteps;
    p_config->system.motor_max_steps[AXIS_HORIZONTAL] = kDefaultMotorPMaxSteps;
    p_config->system.motor_min_angle[AXIS_HORIZONTAL] = kDefaultMotorPMinAngle;
    p_config->system.motor_max_angle[AXIS_HORIZONTAL] = kDefaultMotorPMaxAngle;
    p_config->system.motor_orientation[AXIS_HORIZONTAL] = true;

    p_config->system.motor_min_steps[AXIS_VERTICAL] = kDefaultMotorTMinSteps;
    p_config->system.motor_max_steps[AXIS_VERTICAL] = kDefaultMotorTMaxSteps;
    p_config->system.motor_min_angle[AXIS_VERTICAL] = kDefaultMotorTMinAngle;
    p_config->system.motor_max_angle[AXIS_VERTICAL] = kDefaultMotorTMaxAngle;
    p_config->system.motor_orientation[AXIS_VERTICAL] = kDefaultMotorTOrientation;

    p_config->system.b_hanging_upright = kDefaultHangingUpright;
    p_config->system.frame_width_pixels = kDefaultFrameWidthPixels;
    p_config->system.frame_height_pixels = kDefaultFrameHeightPixels;
    p_config->system.fov_deg[AXIS_HORIZONTAL] = kDefaultHorizontalFovDeg;
    p_config->system.fov_deg[AXIS_VERTICAL] = kDefaultVerticalFovDeg;
    FillDefaultSpeedLevels(p_config);
    FillDefaultFocusLevels(p_config);
    p_config->pid.deadband_deg = kDefaultDeadbandDeg;
    p_config->pid.d_filter_alpha = kDefaultFilterAlpha;
    p_config->pid.integral_ratio = kDefaultIntegralRatio;
    p_config->pid.guard_epsilon = kDefaultGuardEpsilon;
    p_config->pid.angle_filter_alpha = kDefaultAngleFilterAlpha;
    p_config->pid.proportion_deadband_scale = kDefaultProportionDeadbandScale;
    p_config->pid.proportion_deadband_max = kDefaultProportionDeadbandMax;
    p_config->pid.axis[AXIS_HORIZONTAL].kp = kDefaultPidKp;
    p_config->pid.axis[AXIS_HORIZONTAL].ki = kDefaultPidKi;
    p_config->pid.axis[AXIS_HORIZONTAL].kd = kDefaultPidKd;
    p_config->pid.axis[AXIS_VERTICAL].kp = kDefaultPidKp;
    p_config->pid.axis[AXIS_VERTICAL].ki = kDefaultPidKi;
    p_config->pid.axis[AXIS_VERTICAL].kd = kDefaultPidKd;

    p_config->prediction.velocity_alpha = kDefaultVelocityAlpha;
    p_config->prediction.max_shift_factor = kDefaultMaxShiftFactor;
    p_config->prediction.guard_ratio = kDefaultGuardRatio;
    p_config->prediction.guard_ratio_min = kDefaultGuardRatioMin;
    p_config->prediction.guard_ratio_max = kDefaultGuardRatioMax;
    p_config->prediction.guard_adapt_window = kDefaultGuardAdaptWindow;
    p_config->prediction.center_tolerance_deg = kDefaultCenterToleranceDeg;

    p_config->features.b_enable_prediction = true;
    p_config->features.b_enable_d_filter = true;
    p_config->features.b_enable_prediction_guard = true;
    p_config->features.b_enable_prediction_adaptive_guard = true;
}

bool pid_config_load_from_xml(const char* filename, pid_control_config_t* p_config) {
    if (!filename || !p_config) {
        return false;
    }

    pid_config_set_defaults(p_config);

    TiXmlDocument doc;
    if (!doc.LoadFile(filename)) {
        std::printf("Error: Failed to parse config '%s'\n", filename);
        return false;
    }

    TiXmlElement* proot = doc.FirstChildElement("pid_control_config");
    if (proot == nullptr) {
        std::printf("Error: Root element <pid_control_config> missing in '%s'\n", filename);
        doc.Clear();
        return false;
    }

    if (TiXmlElement* psystem_node = proot->FirstChildElement("system_parameters")) {
        psystem_node->Print(stdout, 1);

        parse_child_val(psystem_node, "ai_frame_rate_hz", p_config->system.ai_frame_rate_hz);

        if (TiXmlElement* element = psystem_node->FirstChildElement("speed_levels_p")) {
            parse_attr(element, "max_speed_level", p_config->system.max_speed_level);
            const uint32_t capped_max_level = std::min(p_config->system.max_speed_level, static_cast<uint32_t>(MAX_SPEED_LEVELS));
            std::vector<float> speed_vec;
            if (parse_series(element, "s", static_cast<int>(capped_max_level), speed_vec)) {
                for (uint32_t i = 0; i < MAX_SPEED_LEVELS; ++i) {
                    p_config->system.speed_levels[AXIS_HORIZONTAL][i] = (i < speed_vec.size()) ? speed_vec[i] : 0.0f;
                }
            }
        }

        if (TiXmlElement* element = psystem_node->FirstChildElement("speed_levels_t")) {
            // 如果未提供，沿用 max_speed_level；如果提供则覆盖
            parse_attr(element, "max_speed_level", p_config->system.max_speed_level);
            const uint32_t capped_max_level = std::min(p_config->system.max_speed_level, static_cast<uint32_t>(MAX_SPEED_LEVELS));
            std::vector<float> speed_vec;
            if (parse_series(element, "s", static_cast<int>(capped_max_level), speed_vec)) {
                for (uint32_t i = 0; i < MAX_SPEED_LEVELS; ++i) {
                    p_config->system.speed_levels[AXIS_VERTICAL][i] = (i < speed_vec.size()) ? speed_vec[i] : 0.0f;
                }
            }
        }

        parse_child_val(psystem_node, "motor_vmin_deg_per_sec", p_config->system.motor_vmin_deg_per_sec);
        parse_child_val(psystem_node, "motor_vmax_deg_per_sec", p_config->system.motor_vmax_deg_per_sec);
    parse_child_val(psystem_node, "motor_p_min_steps", p_config->system.motor_min_steps[AXIS_HORIZONTAL]);
    parse_child_val(psystem_node, "motor_p_max_steps", p_config->system.motor_max_steps[AXIS_HORIZONTAL]);
    parse_child_val(psystem_node, "motor_p_min_angle", p_config->system.motor_min_angle[AXIS_HORIZONTAL]);
    parse_child_val(psystem_node, "motor_p_max_angle", p_config->system.motor_max_angle[AXIS_HORIZONTAL]);
    parse_child_val(psystem_node, "motor_t_min_steps", p_config->system.motor_min_steps[AXIS_VERTICAL]);
    parse_child_val(psystem_node, "motor_t_max_steps", p_config->system.motor_max_steps[AXIS_VERTICAL]);
    parse_child_val(psystem_node, "motor_t_orientation", p_config->system.motor_orientation[AXIS_VERTICAL]);
    parse_child_val(psystem_node, "hanging_upright", p_config->system.b_hanging_upright);
    parse_child_val(psystem_node, "motor_t_min_angle", p_config->system.motor_min_angle[AXIS_VERTICAL]);
    parse_child_val(psystem_node, "motor_t_max_angle", p_config->system.motor_max_angle[AXIS_VERTICAL]);

        if (TiXmlElement* element = psystem_node->FirstChildElement("focus_levels")) {
            parse_attr(element, "max_focus_level", p_config->system.max_focus_level);
            const uint32_t capped_max_focus = std::min(p_config->system.max_focus_level, static_cast<uint32_t>(MAX_FOCUS_LEVELS));
            std::vector<float> focus_vec;
            if (parse_series(element, "f", static_cast<int>(capped_max_focus), focus_vec)) {
                for (uint32_t i = 0; i < MAX_FOCUS_LEVELS; ++i) {
                    p_config->system.focus_levels[i] = (i < focus_vec.size()) ? focus_vec[i] : 0.0f;
                }
            }
        }
        
        parse_child_val(psystem_node, "frame_width_pixels", p_config->system.frame_width_pixels);
        parse_child_val(psystem_node, "frame_height_pixels", p_config->system.frame_height_pixels);
    parse_child_val(psystem_node, "horizontal_fov_deg", p_config->system.fov_deg[AXIS_HORIZONTAL]);
    parse_child_val(psystem_node, "vertical_fov_deg", p_config->system.fov_deg[AXIS_VERTICAL]);
    }

    if (TiXmlElement* ppid_node = proot->FirstChildElement("pid_parameters")) {
        ppid_node->Print(stdout, 1);
        parse_child_val(ppid_node, "deadband_deg", p_config->pid.deadband_deg);
        parse_child_val(ppid_node, "d_filter_alpha", p_config->pid.d_filter_alpha);
        parse_child_val(ppid_node, "integral_ratio", p_config->pid.integral_ratio);
        parse_child_val(ppid_node, "guard_epsilon", p_config->pid.guard_epsilon);
    parse_child_val(ppid_node, "angle_filter_alpha", p_config->pid.angle_filter_alpha);
    parse_child_val(ppid_node, "proportion_deadband_scale", p_config->pid.proportion_deadband_scale);
    parse_child_val(ppid_node, "proportion_deadband_max", p_config->pid.proportion_deadband_max);

        if (TiXmlElement* phorizontal = ppid_node->FirstChildElement("horizontal")) {
            parse_child_val(phorizontal, "kp", p_config->pid.axis[AXIS_HORIZONTAL].kp);
            parse_child_val(phorizontal, "ki", p_config->pid.axis[AXIS_HORIZONTAL].ki);
            parse_child_val(phorizontal, "kd", p_config->pid.axis[AXIS_HORIZONTAL].kd);
        }

        if (TiXmlElement* pvertical = ppid_node->FirstChildElement("vertical")) {
            parse_child_val(pvertical, "kp", p_config->pid.axis[AXIS_VERTICAL].kp);
            parse_child_val(pvertical, "ki", p_config->pid.axis[AXIS_VERTICAL].ki);
            parse_child_val(pvertical, "kd", p_config->pid.axis[AXIS_VERTICAL].kd);
        }
    }
    else 
    {
        std::printf("Warning: <pid_parameters> missing in '%s', using defaults.\n", filename);
    }

    if (TiXmlElement* pprediction_node = proot->FirstChildElement("prediction_parameters")) {
        printf("Found prediction_parameters\n");
        parse_child_val(pprediction_node, "velocity_alpha", p_config->prediction.velocity_alpha);
        parse_child_val(pprediction_node, "max_shift_factor", p_config->prediction.max_shift_factor);
        parse_child_val(pprediction_node, "guard_ratio", p_config->prediction.guard_ratio);
        parse_child_val(pprediction_node, "guard_ratio_min", p_config->prediction.guard_ratio_min);
        parse_child_val(pprediction_node, "guard_ratio_max", p_config->prediction.guard_ratio_max);
        parse_child_val(pprediction_node, "guard_adapt_window", p_config->prediction.guard_adapt_window);
        parse_child_val(pprediction_node, "center_tolerance_deg", p_config->prediction.center_tolerance_deg);
    }
    else 
    {
        std::printf("Warning: <prediction_parameters> missing in '%s', using defaults.\n", filename);
    }

    if (TiXmlElement* pfeatures_node = proot->FirstChildElement("feature_switches")) {
        printf("Found feature_switches\n");
        parse_child_val(pfeatures_node, "enable_prediction", p_config->features.b_enable_prediction);
        parse_child_val(pfeatures_node, "enable_d_filter", p_config->features.b_enable_d_filter);
        parse_child_val(pfeatures_node, "enable_prediction_guard", p_config->features.b_enable_prediction_guard);
        parse_child_val(pfeatures_node, "enable_prediction_adaptive_guard", p_config->features.b_enable_prediction_adaptive_guard);
    }
    else 
    {
        std::printf("Warning: <feature_switches> missing in '%s', using defaults.\n", filename);
    }

    return true;
}

bool pid_config_save_to_xml(const char* filename, const pid_control_config_t* p_config) {
    if (!filename || !p_config) {
        return false;
    }

    FILE* file = std::fopen(filename, "w");
    if (!file) {
        return false;
    }
    return true;
}

void pid_config_print(const pid_control_config_t* p_config) {
    if (!p_config) {
        return;
    }

    std::printf("=== PID Control Configuration ===\n");
    std::printf("System Parameters:\n");
    std::printf("  AI Frame Rate: %u Hz\n", p_config->system.ai_frame_rate_hz);
    for (int axis = 0; axis < AXIS_TOTAL; ++axis) {
        const char* axis_name = (axis == AXIS_HORIZONTAL) ? "P" : "T";
        std::printf("  Speed Levels %s: %u\n", axis_name, p_config->system.max_speed_level);
        for (uint32_t i = 0; i < p_config->system.max_speed_level; ++i) {
            std::printf("    %s Level %u: %.2f deg/s\n", axis_name, i, p_config->system.speed_levels[axis][i]);
        }
    }
    std::printf("  Min Motor Speed: %.2f deg/s\n", p_config->system.motor_vmin_deg_per_sec);
    std::printf("  Max Motor Speed: %.2f deg/s\n", p_config->system.motor_vmax_deg_per_sec);
    std::printf("  Motor P Steps: %u - %u\n", p_config->system.motor_min_steps[AXIS_HORIZONTAL], p_config->system.motor_max_steps[AXIS_HORIZONTAL]);
    std::printf("  Motor P Angle: %.2f° - %.2f°\n", p_config->system.motor_min_angle[AXIS_HORIZONTAL], p_config->system.motor_max_angle[AXIS_HORIZONTAL]);
    std::printf("  Motor T Steps: %u - %u\n", p_config->system.motor_min_steps[AXIS_VERTICAL], p_config->system.motor_max_steps[AXIS_VERTICAL]);
    std::printf("  Motor T Orientation: %s\n", p_config->system.motor_orientation[AXIS_VERTICAL] ? "Positive" : "Negative");
    std::printf("  Hanging Upright: %s\n", p_config->system.b_hanging_upright ? "Yes" : "No");
    std::printf("  Motor T Angle: %.2f° - %.2f°\n", p_config->system.motor_min_angle[AXIS_VERTICAL], p_config->system.motor_max_angle[AXIS_VERTICAL]);
    std::printf("  Focus Levels: %u\n", p_config->system.max_focus_level);
    for (uint32_t i = 0; i < p_config->system.max_focus_level; ++i) {
        std::printf("    Level %u: %.2f\n", i, p_config->system.focus_levels[i]);
    } 
    std::printf("  Frame Size: %.0fx%.0f pixels\n", p_config->system.frame_width_pixels, p_config->system.frame_height_pixels);
    std::printf("  FOV: %.1f°x%.1f°\n", p_config->system.fov_deg[AXIS_HORIZONTAL], p_config->system.fov_deg[AXIS_VERTICAL]);

    std::printf("\nPID Parameters:\n");
    std::printf("  Deadband: %.3f%%\n", p_config->pid.deadband_deg);
    std::printf("  D Filter Alpha: %.2f\n", p_config->pid.d_filter_alpha);
    std::printf("  Integral Ratio: %.2f\n", p_config->pid.integral_ratio);
    std::printf("  Angle Filter Alpha: %.2f\n", p_config->pid.angle_filter_alpha);
    std::printf("  Proportion Deadband Scale: %.2f\n", p_config->pid.proportion_deadband_scale);
    std::printf("  Proportion Deadband Max: %.2f\n", p_config->pid.proportion_deadband_max);
    std::printf("  Horizontal PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                p_config->pid.axis[AXIS_HORIZONTAL].kp, p_config->pid.axis[AXIS_HORIZONTAL].ki, p_config->pid.axis[AXIS_HORIZONTAL].kd);
    std::printf("  Vertical PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                p_config->pid.axis[AXIS_VERTICAL].kp, p_config->pid.axis[AXIS_VERTICAL].ki, p_config->pid.axis[AXIS_VERTICAL].kd);

    std::printf("\nPrediction Parameters:\n");
    std::printf("  Velocity Alpha: %.2f\n", p_config->prediction.velocity_alpha);
    std::printf("  Max Shift Factor: %.2f\n", p_config->prediction.max_shift_factor);
    std::printf("  Guard Ratio: %.2f (%.2f - %.2f)\n",
                p_config->prediction.guard_ratio,
                p_config->prediction.guard_ratio_min,
                p_config->prediction.guard_ratio_max);
    std::printf("  Adapt Window: %u\n", p_config->prediction.guard_adapt_window);
    std::printf("  Center Tolerance: %.2f%%\n", p_config->prediction.center_tolerance_deg);

    std::printf("\nFeature Switches:\n");
    std::printf("  Prediction: %s\n", p_config->features.b_enable_prediction ? "ON" : "OFF");
    std::printf("  D Filter: %s\n", p_config->features.b_enable_d_filter ? "ON" : "OFF");
    std::printf("  Prediction Guard: %s\n", p_config->features.b_enable_prediction_guard ? "ON" : "OFF");
    std::printf("  Adaptive Guard: %s\n", p_config->features.b_enable_prediction_adaptive_guard ? "ON" : "OFF");

    std::printf("==================================\n");
}

