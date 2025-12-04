/*** 
 * @Author: lkyezi
 * @Date: 2025-11-03 10:55:04
 * @LastEditTime: 2025-11-28 17:43:38
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */

#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_

#include "pid_ctrl.h"
#include "tracker.h"
#include "pid_timer.h"
#include "media_ext.h"
#include "media_types.h"
#include "media_def.h"

#define BIT(x) (1U << (x))
#define BIT64(x) (1ULL << (x))


typedef struct
{
    short x_center; // 发现物体时物体的初始位置
    short y_center;
    short x_range;  // 目标允许的最大水平偏移范围
    short y_range;  // 目标允许的最大垂直偏移范围
    bool is_moving;   // 目标当前是否在移动
    alarm_in_t alarm_type; // 目标报警类型
    int32_t rid;      // 目标的唯一ID
    timer_ctrl lost_timer; // 目标丢失计时器
} wander_target_info_t;

typedef struct
{
    int32_t p_max_pos; // P电机位置最大值
    int32_t p_min_pos; // P电机位置最小值
    int32_t t_max_pos; // T电机位置最大值
    int32_t t_min_pos; // T电机位置最小值

    int32_t p_max_angle; // P电机最大角度值
    int32_t p_min_angle; // P电机最小角度值
    int32_t t_max_angle; // T电机最大角度值
    int32_t t_min_angle; // T电机最小角度值

    int32_t p_pos;  // P电机当前位置，用于计算姿态角
    int32_t t_pos;  // T电机当前位置，用于计算姿态角

    uint16_t horizontal_center; // 码流宽度的一半
    uint16_t vertical_center;   // 码流高度的一半

    bool orientation; // 0:水平方向是0位置 1:垂直方向是0位置
    bool mirror; // 是否镜像
    bool flip;   // 是否翻转
} pid_motor_pos_t;

typedef struct
{
    bool b_tracking; // 是否正在追踪
    TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT]; // 追踪优先级数组
    int32_t support_bitmap; // 支持的追踪类型位图(bit位数是该类型在TRACK_TYPE_E中的枚举值)

    int32_t cur_track_level; // 当前追踪等级
    int32_t cur_track_priority; // 当前追踪优先级

    bool b_day_mode; // 是否为白天模式
} pid_track_ctx_t;

typedef struct
{
    short x_min;
    short y_min;
    short x_max;
    short y_max;
    short rid;
    char state;
    short dist_x;
    short dist_y;
    vector<float> curr_feat;
} rid_trace_obj_t;

typedef struct
{
    unsigned short v_w;     /*AI码流宽*/
    unsigned short v_h;     /*AI码流高*/
    unsigned long long pts; /*ai帧pts*/
    unsigned int count;
    rid_trace_obj_t objects[AI_ACTION_MAX_OBJ_CNT];
} ai_trace_table_t;

typedef enum
{
    TRACKER_TIMER_AI_PTS = 0, // AI帧PTS时间
    TRACKER_TIMER_TARGET_LOST = 1, // 目标丢失时间
    TRACKER_TIMER_DELAY = 2, // 延迟追踪时间
    TRACKER_TIMER_BUTT
} TRACKER_TIMER_E;

#ifdef _SUPPORT_YOLOWORLD_TRACK_
using MyTracker = tracker<OVDObject, Bc_OVDMOTracker>;
using MyObject = OVDObject;
#else
using MyTracker = tracker<Object, Bc_MOTracker>;
using MyObject = Object;
#endif

class AITrack {
public:

    explicit AITrack():mp_tracker(nullptr),mp_pid_controller(nullptr),m_track_timer{nullptr},mp_pid_config(nullptr),mp_hw_interface(nullptr),m_current_pts(0),mp_media(nullptr),m_motor_pos{0},m_track_ctx{0},mb_is_initialized(0)
    {
    }

    explicit AITrack(const char* config_file, c_media *media): AITrack()
    {
        mb_is_initialized = Initialize(config_file, media);
    }
    ~AITrack();

    inline void update_hardware_interface();
    bool Initialize(const char* config_file, c_media *media);
    bool reload_config(const char* config_file);
    bool set_pid_params(float kp_h, float ki_h, float kd_h,
                        float kp_v, float ki_v, float kd_v);
    bool set_function_enable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable);
    bool set_mirror_flip(bool mirror, bool flip);
    bool set_ai_cfg(TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT], int32_t support_bitmap);

    void track_delay(uint64_t delay_time);
    void Shutdown();

    int pre_process(c_media_md *media, int chn, int handle, std::vector<Object>& detections);
    int pre_process(c_media_md *media, int chn, int handle, std::vector<OVDObject>& detections);

    int get_target(const std::vector<Object>& detections);
    int get_target(const std::vector<OVDObject>& detections);

    void get_pid_result();
    inline void target_get_data(rid_trace_obj_t obj);
    void update_target_info(bool get_data_success);

    bool is_system_delay_timeout(timer_ctrl *timer = nullptr);

    int track_start();
    int track_stop();
    int track_reset();
    int track_step(c_media_md *media, uint32_t channel, int handle);

    bool is_initialized() const { return mb_is_initialized != 0; }
    bool is_tracking() const { return mp_pid_controller && mp_pid_controller->is_running(); }


    

private:
    // 工具成员
    MyTracker *mp_tracker; // 目标追踪器
    PIDCtrl *mp_pid_controller; // PID控制器
    timer_ctrl *m_track_timer[TRACKER_TIMER_BUTT];
    // 其他成员变量
    PIDControlConfig *mp_pid_config; // PID配置指针
    HardwareInterface* mp_hw_interface; // 硬件接口指针
    uint64_t m_current_pts; // 当前PTS
    c_media *mp_media; // 媒体指针
    // 追踪所需变量
    pid_motor_pos_t m_motor_pos; // 电机位置结构体
    pid_track_ctx_t m_track_ctx; // 追踪上下文结构体
    
    // 状态标志
    uint8_t mb_is_initialized;
};





#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_