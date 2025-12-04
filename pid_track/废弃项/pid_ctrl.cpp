#include "pid_ctrl.h"
#include "serial_mcu.h"
#include <string.h>
#include "misc.h"

PIDCtrl* PIDCtrl::s_active_instance_ = nullptr;

inline int get_motor_index(uint8_t axis)
{
    switch(axis) {
        case AXIS_HORIZONTAL:
            return P_MOTOR;
        case AXIS_VERTICAL:
            return T_MOTOR;
        default:
            break;
    }
    
    return -1;
}

PIDCtrl::PIDCtrl()
    : lib_(nullptr),
      target_info_{0.0f, 0.0f, false, 0, -1},
      external_target_callback_(nullptr),
      external_read_angle_callback_(nullptr),
      external_motor_set_speed_callback_(nullptr),
      external_motor_disable_callback_(nullptr),
      initialized_(false),
      running_(false) 
      {
        //   switch(misc_get_board_type())
        //   {
        //         default:
        //             break;
        //   };
      }

PIDCtrl::~PIDCtrl() {
    Shutdown();
}

bool PIDCtrl::Init(const char* config_file, const HardwareInterface* hw_override) {
    if (initialized_) {
        return true;
    }

    if (s_active_instance_ && s_active_instance_ != this) {
        return false;
    }

    lib_ = pid_control_create();
    if (!lib_) {
        return false;
    }

    InstallHardwareInterface(hw_override);

    bool init_ok = false;
    if (config_file && config_file[0] != '\0') {
        init_ok = pid_control_init_from_xml(lib_, config_file, &hw_interface_);
    }

    if (!init_ok) {
        init_ok = pid_control_init_default(lib_, &hw_interface_);
    }

    if (!init_ok) {
        pid_control_destroy(lib_);
        lib_ = nullptr;
        return false;
    }

    initialized_ = true;
    running_ = false;
    s_active_instance_ = this;
    return true;
}

void PIDCtrl::Shutdown() {
    if (!initialized_) {
        return;
    }

    if (running_) {
        pid_control_stop(lib_);
        running_ = false;
    }

    pid_control_destroy(lib_);
    lib_ = nullptr;
    initialized_ = false;

    if (s_active_instance_ == this) {
        s_active_instance_ = nullptr;
    }
}

void PIDCtrl::UpdateTarget(float x_pixel, float y_pixel, bool visible, uint64_t pts, int32_t rid) {
    target_info_.x_pixel = x_pixel;
    target_info_.y_pixel = y_pixel;
    target_info_.visible = visible;
    target_info_.pts = pts;
    target_info_.rid = rid;
}

void PIDCtrl::UpdateTarget(TargetInfo info) {
    target_info_ = info;
    printf("Update Target: (%.2f, %.2f), visible=%d, rid=%d, pts=%llu\n",
           info.x_pixel,
           info.y_pixel,
           info.visible ? 1 : 0,
           info.rid,
           info.pts);
}


void PIDCtrl::Start() {
    if (!initialized_ || running_) {
        return;
    }

    pid_control_start(lib_);
    running_ = true;
}

void PIDCtrl::Stop() {
    if (!initialized_ || !running_) {
        return;
    }

    pid_control_stop(lib_);
    running_ = false;
}

void PIDCtrl::Reset() {
    if (!initialized_) {
        return;
    }
    pid_control_reset(lib_);
}

void PIDCtrl::Step() {
    if (!initialized_ || !running_) {
        return;
    }
    pid_control_step(lib_);
}

bool PIDCtrl::ReloadConfig(const char* config_file) {
    if (!initialized_ || !config_file || config_file[0] == '\0') {
        return false;
    }
    return pid_control_reload_config(lib_, config_file);
}

const PIDControlConfig* PIDCtrl::GetConfig() const {
    if (!initialized_) {
        return nullptr;
    }
    return pid_control_get_config(lib_);
}

bool PIDCtrl::SetPidParams(float kp_h, float ki_h, float kd_h, float kp_v, float ki_v, float kd_v) {
    if (!initialized_) {
        return false;
    }

    bool ok_h = pid_control_set_pid_params(lib_, AXIS_HORIZONTAL, kp_h, ki_h, kd_h);
    bool ok_v = pid_control_set_pid_params(lib_, AXIS_VERTICAL, kp_v, ki_v, kd_v);
    return ok_h && ok_v;
}

bool PIDCtrl::SetFunctionEnable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable) {
    if (!initialized_) {
        return false;
    }

    bool ok1 = pid_control_set_prediction_enabled(lib_, prediction_enable);
    bool ok2 = pid_control_set_d_filter_enabled(lib_, d_filter_enable);
    bool ok3 = pid_control_set_prediction_guard_enabled(lib_, prediction_guard_enable);
    bool ok4 = pid_control_set_adaptive_guard_enabled(lib_, adaptive_guard_enable);
    return ok1 && ok2 && ok3 && ok4;
}


/* ---------- Static Callback Wrappers ---------- */

bool PIDCtrl::get_target_pixel_callback(float* x_pixel, float* y_pixel, uint64_t *pts) {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return false;
    }
    return self->GetTargetPixel(x_pixel, y_pixel, pts);
}

float PIDCtrl::read_angle_callback(uint8_t axis) {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return 0.0f;
    }
    return self->GetAngle(axis);
}

void PIDCtrl::motor_set_speed_callback(uint8_t axis, uint8_t level, uint8_t direction) {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return;
    }
    self->MotorSetSpeed(axis, level, direction);
}

void PIDCtrl::motor_disable_callback(uint8_t axis) {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return;
    }
    self->MotorDisable(axis);
}

/* ---------- Instance Helpers ---------- */

bool PIDCtrl::GetTargetPixel(float* x_pixel, float* y_pixel, uint64_t *pts) const {
    if (external_target_callback_) {
        return external_target_callback_(x_pixel, y_pixel, pts);
    }

    *x_pixel = target_info_.x_pixel;
    *y_pixel = target_info_.y_pixel;
    *pts = target_info_.pts;
    return target_info_.visible;
}

float PIDCtrl::GetAngle(uint8_t axis) const {
    if (external_read_angle_callback_) {
        return external_read_angle_callback_(axis);
    }
    return 0.0f;
}

void PIDCtrl::MotorSetSpeed(uint8_t axis, uint8_t level, uint8_t direction) {
    if (external_motor_set_speed_callback_) {
        external_motor_set_speed_callback_(axis, level, direction);
        return;
    }
    // printf("Set Motor Speed: Axis=%u, Level=%u, Direction=%u\n", axis, level, direction);
    // 根据flip修正direction
    if(!is_flipped() && axis == AXIS_VERTICAL)
    {
        direction = direction ? 0 : 1;
    }
    // 根据level等级做实际曝光处理
    if(level == 16)
    {
        printf("Warning: Motor level is set to max level 16, need exposure locked!\n");
    }

    int motor = get_motor_index(axis);
    serial_mcu_set_speed_level(motor, level, direction);
}

void PIDCtrl::MotorDisable(uint8_t axis) {
    if (external_motor_disable_callback_) {
        external_motor_disable_callback_(axis);
        return;
    }
    int motor = get_motor_index(axis);
    serial_mcu_set_speed_level(motor, 0, 0);
}


void PIDCtrl::InstallHardwareInterface(const HardwareInterface* hw_override) {
    memset(&hw_interface_, 0, sizeof(hw_interface_));

    hw_interface_.get_target_pixel_position = &PIDCtrl::get_target_pixel_callback;
    hw_interface_.read_angle = &PIDCtrl::read_angle_callback;
    hw_interface_.motor_set_speed = &PIDCtrl::motor_set_speed_callback;
    hw_interface_.motor_disable = &PIDCtrl::motor_disable_callback;

    external_target_callback_ = hw_override ? hw_override->get_target_pixel_position : nullptr;
    external_read_angle_callback_ = hw_override ? hw_override->read_angle : nullptr;
    external_motor_set_speed_callback_ = hw_override ? hw_override->motor_set_speed : nullptr;
    external_motor_disable_callback_ = hw_override ? hw_override->motor_disable : nullptr;
}