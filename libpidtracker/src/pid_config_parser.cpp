#include "pid_config_parser.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "tinyxml.h"

namespace {
constexpr uint32_t kDefaultFrameRateHz = 10U;
constexpr uint32_t kDefaultMaxSpeedLevel = 16U;
constexpr float kDefaultMotorVminDegPerSec = 9.0f;
constexpr float kDefaultMotorVmaxDegPerSec = 90.0f;
constexpr int32_t kDefaultMotorTMinSteps = 0;
constexpr int32_t kDefaultMotorTMaxSteps = 1024;
constexpr bool kDefaultMotorTOrientation = true;
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
constexpr std::size_t kSpeedAttrNameSize = 16U;
constexpr std::size_t kFocusAttrNameSize = 64U;

inline void FillDefaultSpeedLevels(PIDControlConfig* config) {
    if (!config) {
        return;
    }

    std::fill_n(config->system.speed_levels,
                MAX_SPEED_LEVELS,
                std::numeric_limits<float>::max());

    const uint32_t capped_max_level = std::min(config->system.max_speed_level, static_cast<uint32_t>(MAX_SPEED_LEVELS));
    for (uint32_t i = 0; i < capped_max_level; ++i) {
        config->system.speed_levels[i] = static_cast<float>(i + 1U);
    }
}

inline void FillDefaultFocusLevels(PIDControlConfig* config) {
    if (!config) {
        return;
    }

    std::fill_n(config->system.focus_levels,
                MAX_FOCUS_LEVELS,
                std::numeric_limits<float>::max());

    for (uint32_t i = 0; i < MAX_FOCUS_LEVELS; ++i) {
        config->system.focus_levels[i] = static_cast<float>(i);
    }
}

// Helper: read child element text as float/unsigned/bool (returns true if updated)
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

void pid_config_set_defaults(PIDControlConfig* config) {
    if (!config) {
        return;
    }

    config->system.ai_frame_rate_hz = kDefaultFrameRateHz;
    config->system.max_speed_level = kDefaultMaxSpeedLevel;
    config->system.motor_vmin_deg_per_sec = kDefaultMotorVminDegPerSec;
    config->system.motor_vmax_deg_per_sec = kDefaultMotorVmaxDegPerSec;
    config->system.motor_t_min_steps = kDefaultMotorTMinSteps;
    config->system.motor_t_max_steps = kDefaultMotorTMaxSteps;
    config->system.motor_t_orientation = kDefaultMotorTOrientation;
    config->system.motor_t_min_angle = kDefaultMotorTMinAngle;
    config->system.motor_t_max_angle = kDefaultMotorTMaxAngle;
    config->system.frame_width_pixels = kDefaultFrameWidthPixels;
    config->system.frame_height_pixels = kDefaultFrameHeightPixels;
    config->system.horizontal_fov_deg = kDefaultHorizontalFovDeg;
    config->system.vertical_fov_deg = kDefaultVerticalFovDeg;
    FillDefaultSpeedLevels(config);
    FillDefaultFocusLevels(config);
    config->pid.deadband_deg = kDefaultDeadbandDeg;
    config->pid.d_filter_alpha = kDefaultFilterAlpha;
    config->pid.integral_ratio = kDefaultIntegralRatio;
    config->pid.guard_epsilon = kDefaultGuardEpsilon;
    config->pid.horizontal.kp = kDefaultPidKp;
    config->pid.horizontal.ki = kDefaultPidKi;
    config->pid.horizontal.kd = kDefaultPidKd;
    config->pid.vertical.kp = kDefaultPidKp;
    config->pid.vertical.ki = kDefaultPidKi;
    config->pid.vertical.kd = kDefaultPidKd;

    config->prediction.velocity_alpha = kDefaultVelocityAlpha;
    config->prediction.max_shift_factor = kDefaultMaxShiftFactor;
    config->prediction.guard_ratio = kDefaultGuardRatio;
    config->prediction.guard_ratio_min = kDefaultGuardRatioMin;
    config->prediction.guard_ratio_max = kDefaultGuardRatioMax;
    config->prediction.guard_adapt_window = kDefaultGuardAdaptWindow;
    config->prediction.center_tolerance_deg = kDefaultCenterToleranceDeg;

    config->features.enable_prediction = true;
    config->features.enable_d_filter = true;
    config->features.enable_prediction_guard = true;
    config->features.enable_prediction_adaptive_guard = true;
}

bool pid_config_load_from_xml(const char* filename, PIDControlConfig* config) {
    if (!filename || !config) {
        return false;
    }

    pid_config_set_defaults(config);

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
        GetChildUnsigned(psystem_node, "ai_frame_rate_hz", config->system.ai_frame_rate_hz);
        if (TiXmlElement* element = psystem_node->FirstChildElement("speed_levels")) {
            element->QueryUnsignedAttribute("max_speed_level", &config->system.max_speed_level);
            std::array<char, kSpeedAttrNameSize> attr_name{};
            for (uint32_t i = 0; i < MAX_SPEED_LEVELS; ++i) {
                if (i < config->system.max_speed_level) {
                    std::snprintf(attr_name.data(), attr_name.size(), "s%u", i);
                    element->QueryFloatAttribute(attr_name.data(), &config->system.speed_levels[i]);
                } else {
                    config->system.speed_levels[i] = 0.0f;
                }
            }
        }
        GetChildFloat(psystem_node, "motor_vmin_deg_per_sec", config->system.motor_vmin_deg_per_sec);
        GetChildFloat(psystem_node, "motor_vmax_deg_per_sec", config->system.motor_vmax_deg_per_sec);
        GetChildUnsigned(psystem_node, "motor_t_min_steps", config->system.motor_t_min_steps);
        GetChildUnsigned(psystem_node, "motor_t_max_steps", config->system.motor_t_max_steps);
        GetChildBool(psystem_node, "motor_t_orientation", config->system.motor_t_orientation);
        GetChildFloat(psystem_node, "motor_t_min_angle", config->system.motor_t_min_angle);
        GetChildFloat(psystem_node, "motor_t_max_angle", config->system.motor_t_max_angle);
        if( TiXmlElement* element = psystem_node->FirstChildElement("focus_levels")) {
            element->QueryUnsignedAttribute("max_focus_level", &config->system.max_focus_level);
            for (uint32_t i = 0; i < MAX_FOCUS_LEVELS; ++i) {
                if (i < config->system.max_focus_level) {
                    std::array<char, kFocusAttrNameSize> attr_name{};
                    std::snprintf(attr_name.data(), attr_name.size(), "f%u", i);
                    element->QueryFloatAttribute(attr_name.data(), &config->system.focus_levels[i]);
                } else {
                    config->system.focus_levels[i] = 0.0f;
                }
            }
        }
        
        GetChildFloat(psystem_node, "frame_width_pixels", config->system.frame_width_pixels);
        GetChildFloat(psystem_node, "frame_height_pixels", config->system.frame_height_pixels);
        GetChildFloat(psystem_node, "horizontal_fov_deg", config->system.horizontal_fov_deg);
        GetChildFloat(psystem_node, "vertical_fov_deg", config->system.vertical_fov_deg);
    }

    if (TiXmlElement* ppid_node = proot->FirstChildElement("pid_parameters")) {
        ppid_node->Print(stdout, 1);
        GetChildFloat(ppid_node, "deadband_deg", config->pid.deadband_deg);
        GetChildFloat(ppid_node, "d_filter_alpha", config->pid.d_filter_alpha);
        GetChildFloat(ppid_node, "integral_ratio", config->pid.integral_ratio);
        GetChildFloat(ppid_node, "guard_epsilon", config->pid.guard_epsilon);

        if (TiXmlElement* phorizontal = ppid_node->FirstChildElement("horizontal")) {
            GetChildFloat(phorizontal, "kp", config->pid.horizontal.kp);
            GetChildFloat(phorizontal, "ki", config->pid.horizontal.ki);
            GetChildFloat(phorizontal, "kd", config->pid.horizontal.kd);
        }

        if (TiXmlElement* pvertical = ppid_node->FirstChildElement("vertical")) {
            GetChildFloat(pvertical, "kp", config->pid.vertical.kp);
            GetChildFloat(pvertical, "ki", config->pid.vertical.ki);
            GetChildFloat(pvertical, "kd", config->pid.vertical.kd);
        }
    }
    else 
    {
        std::printf("Warning: <pid_parameters> missing in '%s', using defaults.\n", filename);
    }

    if (TiXmlElement* pprediction_node = proot->FirstChildElement("prediction_parameters")) {
        printf("Found prediction_parameters\n");
        GetChildFloat(pprediction_node, "velocity_alpha", config->prediction.velocity_alpha);
        GetChildFloat(pprediction_node, "max_shift_factor", config->prediction.max_shift_factor);
        GetChildFloat(pprediction_node, "guard_ratio", config->prediction.guard_ratio);
        GetChildFloat(pprediction_node, "guard_ratio_min", config->prediction.guard_ratio_min);
        GetChildFloat(pprediction_node, "guard_ratio_max", config->prediction.guard_ratio_max);
        GetChildUnsigned(pprediction_node, "guard_adapt_window", config->prediction.guard_adapt_window);
        GetChildFloat(pprediction_node, "center_tolerance_deg", config->prediction.center_tolerance_deg);
    }
    else 
    {
        std::printf("Warning: <prediction_parameters> missing in '%s', using defaults.\n", filename);
    }

    if (TiXmlElement* pfeatures_node = proot->FirstChildElement("feature_switches")) {
        printf("Found feature_switches\n");
        GetChildBool(pfeatures_node, "enable_prediction", config->features.enable_prediction);
        GetChildBool(pfeatures_node, "enable_d_filter", config->features.enable_d_filter);
        GetChildBool(pfeatures_node, "enable_prediction_guard", config->features.enable_prediction_guard);
        GetChildBool(pfeatures_node, "enable_prediction_adaptive_guard", config->features.enable_prediction_adaptive_guard);
    }
    else 
    {
        std::printf("Warning: <feature_switches> missing in '%s', using defaults.\n", filename);
    }

    return true;
}

bool pid_config_save_to_xml(const char* filename, const PIDControlConfig* config) {
    if (!filename || !config) {
        return false;
    }

    FILE* file = std::fopen(filename, "w");
    if (!file) {
        return false;
    }
    return true;
}

void pid_config_print(const PIDControlConfig* config) {
    if (!config) {
        return;
    }

    std::printf("=== PID Control Configuration ===\n");
    std::printf("System Parameters:\n");
    std::printf("  AI Frame Rate: %u Hz\n", config->system.ai_frame_rate_hz);
    std::printf("  Speed Levels: %u\n", config->system.max_speed_level);
    for (uint32_t i = 0; i < config->system.max_speed_level; ++i) {
        std::printf("    Level %u: %.2f deg/s\n", i, config->system.speed_levels[i]);
    }
    std::printf("  Min Motor Speed: %.2f deg/s\n", config->system.motor_vmin_deg_per_sec);
    std::printf("  Max Motor Speed: %.2f deg/s\n", config->system.motor_vmax_deg_per_sec);
    std::printf("  Motor T Steps: %u - %u\n", config->system.motor_t_min_steps, config->system.motor_t_max_steps);
    std::printf("  Motor T Orientation: %s\n", config->system.motor_t_orientation ? "Positive" : "Negative");
    std::printf("  Motor T Angle: %.2f° - %.2f°\n", config->system.motor_t_min_angle, config->system.motor_t_max_angle);
    std::printf("  Focus Levels: %u\n", config->system.max_focus_level);
    for (uint32_t i = 0; i < config->system.max_focus_level; ++i) {
        std::printf("    Level %u: %.2f\n", i, config->system.focus_levels[i]);
    } 
    std::printf("  Frame Size: %.0fx%.0f pixels\n", config->system.frame_width_pixels, config->system.frame_height_pixels);
    std::printf("  FOV: %.1f°x%.1f°\n", config->system.horizontal_fov_deg, config->system.vertical_fov_deg);

    std::printf("\nPID Parameters:\n");
    std::printf("  Deadband: %.3f%%\n", config->pid.deadband_deg);
    std::printf("  D Filter Alpha: %.2f\n", config->pid.d_filter_alpha);
    std::printf("  Integral Ratio: %.2f\n", config->pid.integral_ratio);
    std::printf("  Horizontal PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                config->pid.horizontal.kp, config->pid.horizontal.ki, config->pid.horizontal.kd);
    std::printf("  Vertical PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                config->pid.vertical.kp, config->pid.vertical.ki, config->pid.vertical.kd);

    std::printf("\nPrediction Parameters:\n");
    std::printf("  Velocity Alpha: %.2f\n", config->prediction.velocity_alpha);
    std::printf("  Max Shift Factor: %.2f\n", config->prediction.max_shift_factor);
    std::printf("  Guard Ratio: %.2f (%.2f - %.2f)\n",
                config->prediction.guard_ratio,
                config->prediction.guard_ratio_min,
                config->prediction.guard_ratio_max);
    std::printf("  Adapt Window: %u\n", config->prediction.guard_adapt_window);
    std::printf("  Center Tolerance: %.2f%%\n", config->prediction.center_tolerance_deg);

    std::printf("\nFeature Switches:\n");
    std::printf("  Prediction: %s\n", config->features.enable_prediction ? "ON" : "OFF");
    std::printf("  D Filter: %s\n", config->features.enable_d_filter ? "ON" : "OFF");
    std::printf("  Prediction Guard: %s\n", config->features.enable_prediction_guard ? "ON" : "OFF");
    std::printf("  Adaptive Guard: %s\n", config->features.enable_prediction_adaptive_guard ? "ON" : "OFF");

    std::printf("==================================\n");
}

