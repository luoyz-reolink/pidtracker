/*** 
 * @Author: lkyezi
 * @Date: 2025-11-28 17:41:38
 * @LastEditTime: 2025-12-15 20:49:50
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */
/***
 * @Author: merged by Copilot
 * @Date: 2025-11-28
 * @Description: Unified PID + AI Tracking class PID_Tracker replacing PIDCtrl & AITrack.
 *               Old headers pid_ctrl.h / ai_track.h now forward to this file.
 */
#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TRACKER_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TRACKER_H_

#include "pid_control_lib.h"
#include "tracker.h"
#include "pid_timer.h"
#include "media_ext.h"
#include "media_types.h"
#include "media_def.h"
#include <stdint.h>
#include <vector>
#include <unordered_map>
#include <stdio.h>

typedef struct
{
    /* IPC的电机当前位置 */
    int32_t p_pos; 
    int32_t t_pos;

    /* IPC的电机当前的方向、速度 */
    bool horizontal_direction;
    bool vertical_direction;
    uint8_t horizontal_speed_level;
    uint8_t vertical_speed_level;

    /* IPC的AI码流中心点信息 */
    uint16_t horizontal_center; 
    uint16_t vertical_center;

    /* IPC的画面方向信息 */
    bool mirror; // 是否镜像
    bool flip;   // 是否翻转
} pid_motor_pos_t;

typedef struct
{
    bool b_tracking;                      ///< 是否正在追踪
    bool enable_priority;                 ///< 是否启用优先级追踪
    TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT];
    int32_t support_bitmap;               ///< 支持追踪位图
    int32_t cur_track_level;              ///< 当前优先级层索引
    int32_t cur_track_priority;           ///< 当前优先级枚举值
    bool b_day_mode;                      ///< 白天/夜间模式标记
    int32_t ai_disappear_back_time;            ///< AI目标消失后，返回看守点时间(ms)
    int32_t ai_static_back_timeout;        ///< AI目标离开后，停止追踪时间(ms)
} pid_track_ctx_t;


typedef struct
{
    short x_min; short y_min; short x_max; short y_max;
    short rid; char state; short dist_x; short dist_y;
    std::vector<float> curr_feat;
} rid_trace_obj_t;

typedef struct
{
    unsigned short v_w; unsigned short v_h; unsigned long long pts; unsigned int count;
    rid_trace_obj_t objects[AI_ACTION_MAX_OBJ_CNT];
} ai_trace_table_t;


class PID_Tracker {
public:
    PID_Tracker();
    PID_Tracker(const char* config_file, c_media* media); ///< 直接初始化
    ~PID_Tracker();

    // =============== 初始化与配置 (snake_case) ===============
    bool init(const char* config_file, c_media* media, const hardware_interface_t* hw_override = nullptr);
    void shutdown();
    bool reload_config(const char* config_file);
    const pid_control_config_t* get_config() const;
    bool set_pid_params(float kp_h, float ki_h, float kd_h, float kp_v, float ki_v, float kd_v);
    bool set_function_enable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable);
    bool set_cbw(bool cbw) {m_track_ctx.b_day_mode = cbw; return true;}
    bool set_mirror_flip(bool mirror, bool flip) {m_motor_ctx.mirror = mirror;m_motor_ctx.flip = flip;return true;}
    bool set_ai_cfg(TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT], int32_t support_bitmap, int32_t ai_disappear_back_time = 10 * 1000, int32_t ai_static_back_timeout = 20 * 1000);

    // =============== 运行控制 ===============
    void start();
    void stop();
    void reset();
    void step();                 ///< 仅执行一次 PID 迭代（不做检测）
    int  track_step(uint32_t channel, int handle); ///< 完整一步：取AI->选目标->PID
    void track_delay(uint64_t delay_ms);

    // =============== 状态查询 ===============
    bool is_initialized() const { return m_initialized; }
    bool is_running() const { return m_running; }
    bool is_tracking() const { return m_running && m_current_target.rid != -1; }
    bool is_mirrored() const { return m_motor_ctx.mirror; }
    bool is_flipped() const { return m_motor_ctx.flip; }
    std::string get_track_info(uint8_t type); 

    // =============== 目标更新（可直接外部喂入） ===============
    void update_target(float x_pixel, float y_pixel, bool visible, uint64_t pts, int32_t rid, float proportion);
    void update_target(const TargetInfo& info);
    void update_pdata(int32_t type, void* pdata)
    {
        if(mp_tracker)
        {
            MediaTracker_* media_tracker = dynamic_cast<MediaTracker_*>(mp_tracker);
            if(media_tracker)
            {
                media_tracker->get_pdata_by_type(type, pdata);
            }
        }
    }

private:
    // ---- 内部辅助 ----
    int  select_target(); // 生成/更新 m_current_target
    void get_target_info();
    void apply_pid();
    bool system_delay_timeout(timer_ctrl* timer);
    void install_hardware_interface(const hardware_interface_t* hw_override);
    static int  motor_index(uint8_t axis);

    // ---- 回调静态封装 ----
    static bool s_get_target_pixel_callback(void* ctx, float* x_pixel, float* y_pixel, uint64_t* pts, float *proportion);
    static int32_t s_read_pos_callback(void* ctx, uint8_t axis);
    static void s_motor_set_speed_callback(void* ctx, uint8_t axis, uint8_t level, uint8_t direction);
    static void s_motor_disable_callback(void* ctx, uint8_t axis);
    bool   get_target_pixel(float* x_pixel, float* y_pixel, uint64_t* pts, float *proportion) const;
    int32_t  get_pos(uint8_t axis) const;
    void   motor_set_speed(uint8_t axis, uint8_t level, uint8_t direction);
    void   motor_disable(uint8_t axis);

private:
    // ==== PID 底层 ====
    pid_control_lib* m_lib;
    pid_control_config_t* m_pid_config;
    hardware_interface_t m_hw_interface;
    hardware_interface_t* m_external_hw_interface;

    // ==== AI 追踪 ====
    BaseTrackerUnified* mp_tracker;
    TargetInfo m_current_target;
    pid_motor_pos_t m_motor_ctx;
    pid_track_ctx_t m_track_ctx;
    timer_ctrl* m_timer;

    // ==== 其它上下文 ====
    c_media* mp_media;
    uint64_t m_current_pts;

    // ==== 外部覆盖回调（可选）====
    get_target_pixel_pts_callback m_external_target_cb;
    read_pos_callback m_external_read_pos_cb;
    motor_set_speed_callback m_external_motor_speed_cb;
    motor_disable_callback m_external_motor_disable_cb;

    // ==== 状态标志 ====
    bool m_initialized;
    bool m_running;
};

#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TRACKER_H_
