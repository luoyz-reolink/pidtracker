#ifndef PRODUCT_MODULES_ENC_SRC_PID_CTRL_H
#define PRODUCT_MODULES_ENC_SRC_PID_CTRL_H

#include "pid_control_lib.h"

#include <mutex>

/**
 * @brief 目标像素信息，作为视觉算法与 PID 控制库之间的数据桥梁。
 */
struct TargetInfo {
    float x_pixel;    ///< 目标中心点相对图像的 X 像素坐标
    float y_pixel;    ///< 目标中心点相对图像的 Y 像素坐标
    bool  visible;    ///< 当前帧是否检测到目标
};

/**
 * @brief PID 控制跟踪器封装。负责：
 * 1. 初始化并管理 `pid_control_lib` 库实例；
 * 2. 维护硬件接口回调；
 * 3. 向库提供实时的目标像素坐标；
 * 4. 对外提供控制循环驱动接口。
 */
class PIDCtrl {
public:
    PIDCtrl();
    ~PIDCtrl();

    PIDCtrl(const PIDCtrl&) = delete;
    PIDCtrl& operator=(const PIDCtrl&) = delete;

    /**
     * @brief 初始化 PID 控制库。
     * @param config_file 配置文件路径；为空时使用默认配置。
     * @param hw_override 允许外部覆盖硬件接口的指针；若为 nullptr，使用内部默认实现。
     * @return 初始化是否成功。
     */
    bool Init(const char* config_file = nullptr, const HardwareInterface* hw_override = nullptr);

    /** @brief 关闭跟踪器并释放底层资源。 */
    void Shutdown();

    /**
     * @brief 更新视觉目标的像素坐标。
     * @param x_pixel 目标中心点 X 像素坐标。
     * @param y_pixel 目标中心点 Y 像素坐标。
     * @param visible 是否检测到目标。
     */
    void UpdateTarget(float x_pixel, float y_pixel, bool visible);

    /**
     * @brief 更新云台当前姿态角（若外部没有覆盖角度回调时使用）。
     */
    void UpdateGimbalAngles(float horizontal_deg, float vertical_deg);

    /** @brief 启动 / 停止 / 复位控制器。 */
    void Start();
    void Stop();
    void Reset();

    /**
     * @brief 推动 PID 控制迭代一次（通常在主循环中调用）。
     */
    void Step();

    /**
     * @brief 刷新配置文件。
     * @param config_file 配置路径。
     * @return 操作是否成功。
     */
    bool ReloadConfig(const char* config_file);

    /** @brief 获取底层配置，便于调试。 */
    const PIDControlConfig* GetConfig() const;

    /** @brief 判断库是否已完成初始化。 */
    bool IsInitialized() const { return initialized_; }

private:
    /* ---------- 底层回调适配 ---------- */
    static bool TargetPixelCallback(float* x_pixel, float* y_pixel);
    static float ReadHorizontalAngleCallback();
    static float ReadVerticalAngleCallback();
    static void MotorSetSpeedCallback(uint8_t axis,uint8_t level, uint8_t direction);

    bool GetTargetPixel(float* x_pixel, float* y_pixel) const;
    float GetHorizontalAngle() const;
    float GetVerticalAngle() const;
    void MotorSetSpeed(uint8_t axis,uint8_t level, uint8_t direction);

    void InstallHardwareInterface(const HardwareInterface* hw_override);

private:
    static PIDCtrl* s_active_instance_;

    PIDControlLib* lib_;
    HardwareInterface hw_interface_;

    TargetInfo target_info_;
    mutable std::mutex target_mutex_;

    float horizontal_angle_deg_;
    float vertical_angle_deg_;
    mutable std::mutex angle_mutex_;

    /* 记录外部覆盖的硬件回调（可选） */
    GetTargetPixelPositionCallback external_target_callback_;
    ReadAngleCallback external_read_horizontal_callback_;
    ReadAngleCallback external_read_vertical_callback_;
    MotorSetSpeedCallback* external_motor_set_speed_callback_;

    bool initialized_;
    bool running_;
};

#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACKER_H

