#include "pid_tracker.h"

#include <cstring>

PIDCtrl* PIDCtrl::s_active_instance_ = nullptr;

PIDCtrl::PIDCtrl()
    : lib_(nullptr),
      target_info_{0.0f, 0.0f, false},
      horizontal_angle_deg_(0.0f),
      vertical_angle_deg_(0.0f),
      external_target_callback_(nullptr),
      external_read_horizontal_callback_(nullptr),
      external_read_vertical_callback_(nullptr),
      external_motor_h_write_callback_(nullptr),
      external_motor_h_enable_callback_(nullptr),
      external_motor_h_dir_callback_(nullptr),
      external_motor_v_write_callback_(nullptr),
      external_motor_v_enable_callback_(nullptr),
      external_motor_v_dir_callback_(nullptr),
      external_delay_callback_(nullptr),
      initialized_(false),
      running_(false) {}

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

void PIDCtrl::UpdateTarget(float x_pixel, float y_pixel, bool visible) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_info_.x_pixel = x_pixel;
    target_info_.y_pixel = y_pixel;
    target_info_.visible = visible;
}

void PIDCtrl::UpdateGimbalAngles(float horizontal_deg, float vertical_deg) {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    horizontal_angle_deg_ = horizontal_deg;
    vertical_angle_deg_ = vertical_deg;
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

/* ---------- Static Callback Wrappers ---------- */

bool PIDCtrl::TargetPixelCallback(float* x_pixel, float* y_pixel) {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return false;
    }
    return self->GetTargetPixel(x_pixel, y_pixel);
}

float PIDCtrl::ReadHorizontalAngleCallback() {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return 0.0f;
    }
    return self->GetHorizontalAngle();
}

float PIDCtrl::ReadVerticalAngleCallback() {
    PIDCtrl* self = s_active_instance_;
    if (!self) {
        return 0.0f;
    }
    return self->GetVerticalAngle();
}
/* ---------- Instance Helpers ---------- */

bool PIDCtrl::GetTargetPixel(float* x_pixel, float* y_pixel) const {
    if (external_target_callback_) {
        return external_target_callback_(x_pixel, y_pixel);
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!target_info_.visible) {
        return false;
    }

    *x_pixel = target_info_.x_pixel;
    *y_pixel = target_info_.y_pixel;
    return true;
}

float PIDCtrl::GetHorizontalAngle() const {
    if (external_read_horizontal_callback_) {
        return external_read_horizontal_callback_();
    }

    std::lock_guard<std::mutex> lock(angle_mutex_);
    return horizontal_angle_deg_;
}

float PIDCtrl::GetVerticalAngle() const {
    if (external_read_vertical_callback_) {
        return external_read_vertical_callback_();
    }

    std::lock_guard<std::mutex> lock(angle_mutex_);
    return vertical_angle_deg_;
}

void PIDCtrl::HandleHorizontalPhase(uint8_t pattern) {
    if (external_motor_h_write_callback_) {
        external_motor_h_write_callback_(pattern);
    }
    (void)pattern;
}

void PIDCtrl::HandleHorizontalDriver(bool enable) {
    if (external_motor_h_enable_callback_) {
        external_motor_h_enable_callback_(enable);
    }
    (void)enable;
}

void PIDCtrl::HandleHorizontalDirection(bool reverse) {
    if (external_motor_h_dir_callback_) {
        external_motor_h_dir_callback_(reverse);
    }
    (void)reverse;
}

void PIDCtrl::HandleVerticalPhase(uint8_t pattern) {
    if (external_motor_v_write_callback_) {
        external_motor_v_write_callback_(pattern);
    }
    (void)pattern;
}

void PIDCtrl::HandleVerticalDriver(bool enable) {
    if (external_motor_v_enable_callback_) {
        external_motor_v_enable_callback_(enable);
    }
    (void)enable;
}

void PIDCtrl::HandleVerticalDirection(bool reverse) {
    if (external_motor_v_dir_callback_) {
        external_motor_v_dir_callback_(reverse);
    }
    (void)reverse;
}

void PIDCtrl::HandleDelay(uint32_t milliseconds) {
    if (external_delay_callback_) {
        external_delay_callback_(milliseconds);
        return;
    }

#if defined(_WIN32)
    ::Sleep(milliseconds);
#else
    usleep(milliseconds * 1000U);
#endif
}

void PIDCtrl::InstallHardwareInterface(const HardwareInterface* hw_override) {
    std::memset(&hw_interface_, 0, sizeof(hw_interface_));

    hw_interface_.get_target_pixel_position = &PIDTracker::TargetPixelCallback;
    hw_interface_.read_horizontal_angle_deg = &PIDTracker::ReadHorizontalAngleCallback;
    hw_interface_.read_vertical_angle_deg = &PIDTracker::ReadVerticalAngleCallback;
    hw_interface_.motor_horizontal_write_phase_pattern = &PIDTracker::MotorHorizontalWritePhase;
    hw_interface_.motor_horizontal_enable_driver = &PIDTracker::MotorHorizontalEnableDriver;
    hw_interface_.motor_horizontal_set_direction = &PIDTracker::MotorHorizontalSetDirection;
    hw_interface_.motor_vertical_write_phase_pattern = &PIDTracker::MotorVerticalWritePhase;
    hw_interface_.motor_vertical_enable_driver = &PIDTracker::MotorVerticalEnableDriver;
    hw_interface_.motor_vertical_set_direction = &PIDTracker::MotorVerticalSetDirection;
    hw_interface_.delay_ms = &PIDTracker::DelayMsCallback;

    external_target_callback_ = hw_override ? hw_override->get_target_pixel_position : nullptr;
    external_read_horizontal_callback_ = hw_override ? hw_override->read_horizontal_angle_deg : nullptr;
    external_read_vertical_callback_ = hw_override ? hw_override->read_vertical_angle_deg : nullptr;
    external_motor_h_write_callback_ = hw_override ? hw_override->motor_horizontal_write_phase_pattern : nullptr;
    external_motor_h_enable_callback_ = hw_override ? hw_override->motor_horizontal_enable_driver : nullptr;
    external_motor_h_dir_callback_ = hw_override ? hw_override->motor_horizontal_set_direction : nullptr;
    external_motor_v_write_callback_ = hw_override ? hw_override->motor_vertical_write_phase_pattern : nullptr;
    external_motor_v_enable_callback_ = hw_override ? hw_override->motor_vertical_enable_driver : nullptr;
    external_motor_v_dir_callback_ = hw_override ? hw_override->motor_vertical_set_direction : nullptr;
    external_delay_callback_ = hw_override ? hw_override->delay_ms : nullptr;
}