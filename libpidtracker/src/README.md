<!--
 * @Author: lkyezi
 * @Date: 2025-10-29 18:42:11
 * @LastEditTime: 2025-10-29 18:42:17
 * @LastEditors: lkyezi
 * @Description: 
 * 
-->
# PID控制库

这是一个高性能的双轴云台PID控制库，支持XML配置文件和运行时参数调整。

## 功能特性

### 核心功能
- **双轴PID控制**：支持水平和垂直两轴独立控制
- **XML配置**：支持从XML文件加载系统参数、PID参数和预测器参数
- **运行时调参**：支持运行时动态调整PID参数和功能开关
- **预测补偿**：角速度预测补偿AI帧率延迟
- **多级滤波**：微分项一阶滤波抑制噪声
- **自适应保护**：预测器自适应保护机制

### 高级功能
- **16档电机速度控制**：通过定时器中断节拍实现精确分档控制
- **防积分饱和**：梯形积分和防windup机制
- **死区控制**：可配置的误差死区抑制抖动
- **统计监控**：控制性能统计和状态监控
- **硬件抽象**：通过回调接口支持不同硬件平台

## 文件结构

```
pid_control_lib/
├── pid_control_lib.h          # 库主头文件
├── pid_control_lib.c          # 库核心实现
├── pid_config_parser.h        # XML配置解析器头文件
├── pid_config_parser.c        # XML配置解析器实现
├── config.xml                 # 示例配置文件
├── example.c                  # 使用示例程序
├── Makefile                   # 编译脚本
└── README.md                  # 说明文档
```

## 编译和安装

### 编译静态库
```bash
make
```

### 编译示例程序
```bash
make example
```

### 直接编译（不生成静态库）
```bash
make example-direct
```

### 运行示例
```bash
make test
```

### 清理编译文件
```bash
make clean
```

## 使用方法

### 1. 基本使用流程

```c
#include "pid_control_lib.h"

// 1. 创建库实例
PIDControlLib* lib = pid_control_create();

// 2. 设置硬件接口回调
HardwareInterface hw_interface = {
    .get_target_pixel_position = your_vision_callback,
    .motor_horizontal_write_phase_pattern = your_motor_h_callback,
    // ... 其他回调函数
};

// 3. 从XML文件初始化
if (!pid_control_init_from_xml(lib, "config.xml", &hw_interface)) {
    // 初始化失败，使用默认配置
    pid_control_init_default(lib, &hw_interface);
}

// 4. 启动控制器
pid_control_start(lib);

// 5. 主控制循环
while (running) {
    pid_control_step(lib);
    delay_ms(100); // 10Hz控制频率
}

// 6. 清理资源
pid_control_stop(lib);
pid_control_destroy(lib);
```

### 2. 运行时参数调整

```c
// 调整PID参数
pid_control_set_pid_params(lib, AXIS_HORIZONTAL, 1.2f, 0.25f, 0.08f);

// 开关功能
pid_control_set_prediction_enabled(lib, true);
pid_control_set_d_filter_enabled(lib, false);

// 调整预测器参数
pid_control_set_prediction_params(lib, 0.7f, 0.2f, 0.4f);

// 重新加载配置文件
pid_control_reload_config(lib, "new_config.xml");
```

### 3. 状态查询

```c
// 获取PID输出
float output;
pid_control_get_pid_output(lib, AXIS_HORIZONTAL, &output);

// 获取电机档位
uint8_t level;
pid_control_get_motor_level(lib, AXIS_VERTICAL, &level);

// 获取统计信息
PIDControlStats stats;
pid_control_get_stats(lib, &stats);
```

## XML配置文件

配置文件支持以下参数组：

### 系统参数
```xml
<system_parameters>
    <ai_frame_rate_hz>10</ai_frame_rate_hz>
    <speed_levels>16</speed_levels>
    <motor_vmax_deg_per_sec>60.0</motor_vmax_deg_per_sec>
    <frame_width_pixels>640.0</frame_width_pixels>
    <frame_height_pixels>480.0</frame_height_pixels>
    <horizontal_fov_deg>60.0</horizontal_fov_deg>
    <vertical_fov_deg>45.0</vertical_fov_deg>
</system_parameters>
```

### PID参数
```xml
<pid_parameters>
    <deadband_deg>0.02</deadband_deg>
    <d_filter_alpha>0.2</d_filter_alpha>
    <integral_ratio>0.8</integral_ratio>
    <horizontal>
        <kp>0.8</kp>
        <ki>0.15</ki>
        <kd>0.05</kd>
    </horizontal>
    <vertical>
        <kp>0.8</kp>
        <ki>0.15</ki>
        <kd>0.05</kd>
    </vertical>
</pid_parameters>
```

### 预测器参数
```xml
<prediction_parameters>
    <velocity_alpha>0.5</velocity_alpha>
    <max_shift_factor>0.3</max_shift_factor>
    <guard_ratio>0.5</guard_ratio>
    <guard_ratio_min>0.3</guard_ratio_min>
    <guard_ratio_max>0.6</guard_ratio_max>
    <guard_adapt_window>30</guard_adapt_window>
    <center_tolerance_deg>0.3</center_tolerance_deg>
</prediction_parameters>
```

### 功能开关
```xml
<feature_switches>
    <enable_prediction>1</enable_prediction>
    <enable_d_filter>1</enable_d_filter>
    <enable_prediction_guard>1</enable_prediction_guard>
    <enable_prediction_adaptive_guard>1</enable_prediction_adaptive_guard>
</feature_switches>
```

## API参考

### 库管理函数
- `pid_control_create()` - 创建库实例
- `pid_control_destroy()` - 销毁库实例
- `pid_control_init_from_xml()` - 从XML初始化
- `pid_control_init_default()` - 使用默认配置初始化

### 控制函数
- `pid_control_start()` - 启动控制器
- `pid_control_stop()` - 停止控制器
- `pid_control_step()` - 执行控制步骤
- `pid_control_reset()` - 重置控制器状态

### 参数调整函数
- `pid_control_set_pid_params()` - 设置PID参数
- `pid_control_set_prediction_enabled()` - 开关预测功能
- `pid_control_set_d_filter_enabled()` - 开关微分滤波
- `pid_control_set_prediction_params()` - 设置预测器参数

### 状态查询函数
- `pid_control_get_pid_output()` - 获取PID输出
- `pid_control_get_motor_level()` - 获取电机档位
- `pid_control_get_stats()` - 获取统计信息

### 调试函数
- `pid_control_print_config()` - 打印配置信息
- `pid_control_print_status()` - 打印运行状态
- `pid_control_save_config()` - 保存配置到文件

## 硬件接口

需要实现以下回调函数：

```c
typedef struct {
    GetTargetPixelPositionCallback get_target_pixel_position;
    ReadAngleCallback read_horizontal_angle_deg;
    ReadAngleCallback read_vertical_angle_deg;
    MotorWritePhaseCallback motor_horizontal_write_phase_pattern;
    MotorEnableDriverCallback motor_horizontal_enable_driver;
    MotorSetDirectionCallback motor_horizontal_set_direction;
    MotorWritePhaseCallback motor_vertical_write_phase_pattern;
    MotorEnableDriverCallback motor_vertical_enable_driver;
    MotorSetDirectionCallback motor_vertical_set_direction;
    DelayCallback delay_ms;
} HardwareInterface;
```

## 性能特性

- **控制频率**：支持1-100Hz控制频率
- **响应时间**：微秒级PID计算
- **内存占用**：小于2KB静态内存
- **CPU占用**：单次控制循环<1ms（ARM Cortex-M4）

## 调试和调优

### 启用调试输出
```c
pid_control_print_status(lib);  // 打印运行状态
pid_control_print_config(lib);  // 打印配置信息
```

### 性能监控
```c
PIDControlStats stats;
pid_control_get_stats(lib, &stats);
printf("Control cycles: %u\n", stats.control_cycles);
printf("Prediction success rate: %.1f%%\n", 
       (1.0f - (float)stats.prediction_rejects / stats.prediction_attempts) * 100.0f);
```

### 参数调优建议
1. **Kp调整**：从小到大，观察响应速度
2. **Ki调整**：消除稳态误差，防止积分饱和
3. **Kd调整**：抑制超调，注意噪声放大
4. **预测参数**：根据目标运动特性调整
5. **滤波参数**：平衡响应速度和噪声抑制

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 贡献

欢迎提交问题和改进建议！

## 联系方式

如有技术问题，请提交Issue或联系维护者。