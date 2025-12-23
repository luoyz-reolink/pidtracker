#include "pid_control_lib.h"

pid_control_lib* pid_control_create() {
    return pid_control_lib::instance();
}

void pid_control_destroy(pid_control_lib* p_lib) {
    (void)p_lib;
}

bool pid_control_init_from_xml(pid_control_lib* p_lib,
                               const char* p_config_file,
                               const hardware_interface_t* p_hw_interface) {
    if (!p_lib) return false;
    return p_lib->init_from_xml(p_config_file, p_hw_interface);
}

bool pid_control_init_default(pid_control_lib* p_lib, const hardware_interface_t* p_hw_interface) {
    if (!p_lib) return false;
    return p_lib->init_default(p_hw_interface);
}

const pid_control_config_t* pid_control_get_config(const pid_control_lib* p_lib) {
    if (!p_lib) return nullptr;
    return p_lib->get_config();
}

bool pid_control_reload_config(pid_control_lib* p_lib, const char* p_config_file) {
    if (!p_lib) return false;
    return p_lib->reload_config(p_config_file);
}

void pid_control_step(pid_control_lib* p_lib) {
    if (p_lib) p_lib->step();
}

void pid_control_start(pid_control_lib* p_lib) {
    if (p_lib) p_lib->start();
}

void pid_control_stop(pid_control_lib* p_lib) {
    if (p_lib) p_lib->stop();
}

void pid_control_reset(pid_control_lib* p_lib) {
    if (p_lib) p_lib->reset();
}

bool pid_control_set_pid_params(pid_control_lib* p_lib, axis_type_e axis, float kp, float ki, float kd) {
    if (!p_lib) return false;
    return p_lib->set_pid_params(axis, kp, ki, kd);
}

bool pid_control_get_pid_params(const pid_control_lib* p_lib, axis_type_e axis, float* p_kp, float* p_ki, float* p_kd) {
    if (!p_lib) return false;
    return p_lib->get_pid_params(axis, p_kp, p_ki, p_kd);
}

bool pid_control_set_prediction_enabled(pid_control_lib* p_lib, bool b_enabled) {
    if (!p_lib) return false;
    return p_lib->set_prediction_enabled(b_enabled);
}

bool pid_control_set_d_filter_enabled(pid_control_lib* p_lib, bool b_enabled) {
    if (!p_lib) return false;
    return p_lib->set_d_filter_enabled(b_enabled);
}

bool pid_control_set_prediction_guard_enabled(pid_control_lib* p_lib, bool b_enabled) {
    if (!p_lib) return false;
    return p_lib->set_prediction_guard_enabled(b_enabled);
}

bool pid_control_set_adaptive_guard_enabled(pid_control_lib* p_lib, bool b_enabled) {
    if (!p_lib) return false;
    return p_lib->set_adaptive_guard_enabled(b_enabled);
}

bool pid_control_get_prediction_enabled(pid_control_lib* p_lib) {
    if (!p_lib) return false;
    return p_lib->get_prediction_enabled();
}

bool pid_control_get_d_filter_enabled(pid_control_lib* p_lib) {
    if (!p_lib) return false;
    return p_lib->get_d_filter_enabled();
}

bool pid_control_get_prediction_guard_enabled(pid_control_lib* p_lib) {
    if (!p_lib) return false;
    return p_lib->get_prediction_guard_enabled();
}

bool pid_control_get_adaptive_guard_enabled(pid_control_lib* p_lib) {
    if (!p_lib) return false;
    return p_lib->get_adaptive_guard_enabled();
}

bool pid_control_set_prediction_params(pid_control_lib* p_lib, float velocity_alpha, float max_shift_factor, float guard_ratio) {
    if (!p_lib) return false;
    return p_lib->set_prediction_params(velocity_alpha, max_shift_factor, guard_ratio);
}

bool pid_control_get_prediction_params(pid_control_lib* p_lib, float* p_velocity_alpha, float* p_max_shift_factor, float* p_guard_ratio) {
    if (!p_lib) return false;
    return p_lib->get_prediction_params(p_velocity_alpha, p_max_shift_factor, p_guard_ratio);
}

bool pid_control_set_pid_advanced_params(pid_control_lib* p_lib, float deadband_deg, float d_filter_alpha, float integral_ratio) {
    if (!p_lib) return false;
    return p_lib->set_pid_advanced_params(deadband_deg, d_filter_alpha, integral_ratio);
}

bool pid_control_get_pid_advanced_params(pid_control_lib* p_lib, float* p_deadband_deg, float* p_d_filter_alpha, float* p_integral_ratio) {
    if (!p_lib) return false;
    return p_lib->get_pid_advanced_params(p_deadband_deg, p_d_filter_alpha, p_integral_ratio);
}

bool pid_control_get_pid_output(pid_control_lib* p_lib, axis_type_e axis, float* p_output) {
    if (!p_lib) return false;
    return p_lib->get_pid_output(axis, p_output);
}

bool pid_control_get_motor_level(pid_control_lib* p_lib, axis_type_e axis, uint8_t* p_level) {
    if (!p_lib) return false;
    return p_lib->get_motor_level(axis, p_level);
}

bool pid_control_get_prediction_state(pid_control_lib* p_lib, axis_type_e axis, float* p_current_deg, float* p_velocity_deg_per_sec) {
    if (!p_lib) return false;
    return p_lib->get_prediction_state(axis, p_current_deg, p_velocity_deg_per_sec);
}

bool pid_control_get_stats(pid_control_lib* p_lib, pid_control_stats_t* p_out_stats) {
    if (!p_lib) return false;
    return p_lib->get_stats(p_out_stats);
}

void pid_control_reset_stats(pid_control_lib* p_lib) {
    if (p_lib) p_lib->reset_stats();
}

bool pid_control_get_target_proportion(pid_control_lib* p_lib, float* p_proportion) {
    if (!p_lib) return false;
    return p_lib->get_target_proportion(p_proportion);
}

bool pid_control_get_tracking_metrics(pid_control_lib* p_lib, pid_tracking_metrics_t* p_metrics) {
    if (!p_lib) return false;
    return p_lib->get_tracking_metrics(p_metrics);
}

void pid_control_reset_tracking_metrics(pid_control_lib* p_lib) {
    if (p_lib) p_lib->reset_tracking_metrics();
}

size_t pid_control_get_tracking_samples(pid_control_lib* p_lib, pid_tracking_sample_t* p_out_samples, size_t max_count) {
    if (!p_lib) return 0;
    return p_lib->get_tracking_samples(p_out_samples, max_count);
}

void pid_control_clear_tracking_samples(pid_control_lib* p_lib) {
    if (p_lib) p_lib->clear_tracking_samples();
}

void pid_control_print_config(pid_control_lib* p_lib) {
    if (p_lib) p_lib->print_config();
}

void pid_control_print_status(pid_control_lib* p_lib) {
    if (p_lib) p_lib->print_status();
}

bool pid_control_save_config(pid_control_lib* p_lib, const char* p_filename) {
    if (!p_lib) return false;
    return p_lib->save_config(p_filename);
}
