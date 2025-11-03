/*
 * @Author: lkyezi
 * @Date: 2025-10-29 18:40:46
 * @LastEditTime: 2025-10-31 09:28:37
 * @LastEditors: lkyezi
 * @Description: 
 * 
 */
#include "pid_control_lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * 使用示例程序 - 演示PID控制库的使用方法
 * 
 * 本程序展示如何：
 * 1. 初始化库和硬件接口
 * 2. 从XML文件加载配置
 * 3. 运行时调整参数
 * 4. 查询运行状态
 */

/*------------------------------ 模拟硬件接口 ------------------------------*/

/* 模拟目标位置 - 实际应用中这里应该是视觉算法的输出 */
static float sim_target_x = 320.0f;
static float sim_target_y = 240.0f;
static bool sim_target_visible = true;

/* 模拟目标移动 */
static void simulate_moving_target(void) {
    static float time_counter = 0.0f;
    time_counter += 0.1f;
    
    // 模拟圆形轨迹运动
    sim_target_x = 320.0f + 100.0f * cosf(time_counter * 0.5f);
    sim_target_y = 240.0f + 80.0f * sinf(time_counter * 0.3f);
    
    // 偶尔模拟目标丢失
    if ((int)(time_counter * 10) % 100 == 0) {
        sim_target_visible = false;
        printf("[SIM] Target lost temporarily\n");
    } else if ((int)(time_counter * 10) % 100 == 10) {
        sim_target_visible = true;
        printf("[SIM] Target recovered\n");
    }
}

/* 硬件接口回调函数实现 */
static bool get_target_pixel_position(float *x_pixel, float *y_pixel) {
    if (!sim_target_visible) {
        return false;
    }
    
    *x_pixel = sim_target_x;
    *y_pixel = sim_target_y;
    return true;
}

static float read_horizontal_angle_deg(void) {
    // 在实际应用中，这里应该读取云台的实际角度
    return 0.0f;
}

static float read_vertical_angle_deg(void) {
    // 在实际应用中，这里应该读取云台的实际角度
    return 0.0f;
}

static void motor_horizontal_write_phase_pattern(uint8_t pattern) {
    printf("[MOTOR H] Phase: 0x%02X\n", pattern);
}

static void motor_horizontal_enable_driver(bool enable) {
    printf("[MOTOR H] Driver: %s\n", enable ? "ENABLED" : "DISABLED");
}

static void motor_horizontal_set_direction(bool reverse) {
    printf("[MOTOR H] Direction: %s\n", reverse ? "REVERSE" : "FORWARD");
}

static void motor_vertical_write_phase_pattern(uint8_t pattern) {
    printf("[MOTOR V] Phase: 0x%02X\n", pattern);
}

static void motor_vertical_enable_driver(bool enable) {
    printf("[MOTOR V] Driver: %s\n", enable ? "ENABLED" : "DISABLED");
}

static void motor_vertical_set_direction(bool reverse) {
    printf("[MOTOR V] Direction: %s\n", reverse ? "REVERSE" : "FORWARD");
}

static void delay_ms(uint32_t milliseconds) {
    // 在实际应用中，这里应该是真正的延时函数
    printf("[DELAY] %u ms\n", milliseconds);
}

/*------------------------------ 主程序 ------------------------------*/

int main(void) {
    printf("=== PID Control Library Example ===\n\n");
    
    /* 1. 创建库实例 */
    PIDControlLib* lib = pid_control_create();
    if (!lib) {
        printf("Error: Failed to create library instance\n");
        return -1;
    }
    
    /* 2. 设置硬件接口 */
    HardwareInterface hw_interface = {
        .get_target_pixel_position = get_target_pixel_position,
        .read_horizontal_angle_deg = read_horizontal_angle_deg,
        .read_vertical_angle_deg = read_vertical_angle_deg,
        .motor_horizontal_write_phase_pattern = motor_horizontal_write_phase_pattern,
        .motor_horizontal_enable_driver = motor_horizontal_enable_driver,
        .motor_horizontal_set_direction = motor_horizontal_set_direction,
        .motor_vertical_write_phase_pattern = motor_vertical_write_phase_pattern,
        .motor_vertical_enable_driver = motor_vertical_enable_driver,
        .motor_vertical_set_direction = motor_vertical_set_direction,
        .delay_ms = delay_ms
    };
    
    /* 3. 从XML文件初始化库 */
    printf("Loading configuration from 'config.xml'...\n");
    if (!pid_control_init_from_xml(lib, "config.xml", &hw_interface)) {
        printf("Warning: Failed to load XML config, using defaults\n");
        if (!pid_control_init_default(lib, &hw_interface)) {
            printf("Error: Failed to initialize with defaults\n");
            pid_control_destroy(lib);
            return -1;
        }
    }
    
    /* 4. 显示当前配置 */
    printf("\nCurrent Configuration:\n");
    pid_control_print_config(lib);
    
    /* 5. 启动控制器 */
    printf("\nStarting PID controller...\n");
    pid_control_start(lib);
    
    /* 6. 运行控制循环并演示参数调整 */
    printf("\n=== Starting Control Loop ===\n");
    for (int cycle = 0; cycle < 100; cycle++) {
        /* 模拟目标移动 */
        simulate_moving_target();
        
        /* 执行一次控制循环 */
        pid_control_step(lib);
        
        /* 模拟定时器中断（电机步进） */
        for (int tick = 0; tick < 5; tick++) {
            pid_control_timer_interrupt(lib);
        }
        
        /* 每10个周期显示一次状态 */
        if (cycle % 10 == 0) {
            printf("\n--- Control Cycle %d ---\n", cycle);
            printf("Target: (%.1f, %.1f) pixels\n", sim_target_x, sim_target_y);
            
            float h_output, v_output;
            pid_control_get_pid_output(lib, AXIS_HORIZONTAL, &h_output);
            pid_control_get_pid_output(lib, AXIS_VERTICAL, &v_output);
            printf("PID Output: H=%.2f, V=%.2f deg/s\n", h_output, v_output);
            
            uint8_t h_level, v_level;
            pid_control_get_motor_level(lib, AXIS_HORIZONTAL, &h_level);
            pid_control_get_motor_level(lib, AXIS_VERTICAL, &v_level);
            printf("Motor Level: H=%u, V=%u\n", h_level, v_level);
        }
        
        /* 在第30个周期演示参数调整 */
        if (cycle == 30) {
            printf("\n=== Demonstrating Parameter Adjustment ===\n");
            
            /* 调整PID参数 */
            printf("Adjusting horizontal PID parameters...\n");
            pid_control_set_pid_params(lib, AXIS_HORIZONTAL, 1.2f, 0.25f, 0.08f);
            
            /* 关闭预测功能 */
            printf("Disabling prediction...\n");
            pid_control_set_prediction_enabled(lib, false);
            
            /* 调整预测器参数 */
            printf("Adjusting prediction parameters...\n");
            pid_control_set_prediction_params(lib, 0.7f, 0.2f, 0.4f);
            
            /* 显示调整后的参数 */
            float kp, ki, kd;
            pid_control_get_pid_params(lib, AXIS_HORIZONTAL, &kp, &ki, &kd);
            printf("New horizontal PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
            
            printf("Prediction enabled: %s\n", 
                   pid_control_get_prediction_enabled(lib) ? "YES" : "NO");
        }
        
        /* 在第60个周期重新启用预测 */
        if (cycle == 60) {
            printf("\n=== Re-enabling Prediction ===\n");
            pid_control_set_prediction_enabled(lib, true);
            printf("Prediction enabled: %s\n", 
                   pid_control_get_prediction_enabled(lib) ? "YES" : "NO");
        }
        
        /* 在第80个周期演示配置重新加载 */
        if (cycle == 80) {
            printf("\n=== Demonstrating Config Reload ===\n");
            printf("Attempting to reload configuration...\n");
            if (pid_control_reload_config(lib, "config.xml")) {
                printf("Configuration reloaded successfully\n");
            } else {
                printf("Failed to reload configuration\n");
            }
        }
    }
    
    /* 7. 显示最终状态和统计信息 */
    printf("\n=== Final Status ===\n");
    pid_control_print_status(lib);
    
    PIDControlStats stats;
    if (pid_control_get_stats(lib, &stats)) {
        printf("\nStatistics Summary:\n");
        printf("Control Cycles: %u\n", stats.control_cycles);
        printf("Target Lost Count: %u\n", stats.target_lost_count);
        printf("Prediction Attempts: %u\n", stats.prediction_attempts);
        printf("Prediction Rejects: %u\n", stats.prediction_rejects);
        if (stats.prediction_attempts > 0) {
            float success_rate = 1.0f - ((float)stats.prediction_rejects / (float)stats.prediction_attempts);
            printf("Prediction Success Rate: %.1f%%\n", success_rate * 100.0f);
        }
        printf("Average Error H: %.3f deg\n", stats.average_error_horizontal);
        printf("Average Error V: %.3f deg\n", stats.average_error_vertical);
    }
    
    /* 8. 保存当前配置 */
    printf("\nSaving current configuration to 'config_modified.xml'...\n");
    if (pid_control_save_config(lib, "config_modified.xml")) {
        printf("Configuration saved successfully\n");
    } else {
        printf("Failed to save configuration\n");
    }
    
    /* 9. 停止控制器并清理 */
    printf("\nStopping controller and cleaning up...\n");
    pid_control_stop(lib);
    pid_control_destroy(lib);
    
    printf("\n=== Example Complete ===\n");
    return 0;
}

/*
 * 编译命令示例：
 * gcc -o example example.c pid_control_lib.c pid_config_parser.c -lm
 * 
 * 运行前确保 config.xml 文件存在于当前目录
 */