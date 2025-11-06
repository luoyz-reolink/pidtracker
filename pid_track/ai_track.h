/*** 
 * @Author: lkyezi
 * @Date: 2025-11-03 10:55:04
 * @LastEditTime: 2025-11-05 17:53:42
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */

#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_

#include "pid_ctrl.h"
#include "tracker.h"
#include "media_types.h"
#include "media_def.h"


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
    TRACKER_TIME_PTS = 0,
    TRACKER_TIME_SYSTEM = 1,
    TRACKER_TIME_BUTT
} TRACKER_TIME_E;

#ifdef _SUPPORT_YOLOWORLD_TRACK_
using MyTracker = tracker<OVDObject, Bc_OVDMOTracker>;
using MyObject = OVDObject;
#else
using MyTracker = tracker<Object, Bc_MOTracker>;
using MyObject = Object;
#endif

class AITrack {
public:

    explicit AITrack():mp_tracker(nullptr),mp_pid_controller(nullptr),m_ai_trace_table(),m_last_times(),m_last_intervals(),mb_is_initialized(0),mb_is_tracking(0),mb_flip(false),mb_mirror(false)
    {
    }

    explicit AITrack(const char* config_file): AITrack()
    {
        mb_is_initialized = Initialize(config_file);
    }
    ~AITrack();

    inline void update_hardware_interface();
    bool Initialize(const char* config_file);
    bool reload_config(const char* config_file);
    bool set_pid_params(float kp_h, float ki_h, float kd_h,
                        float kp_v, float ki_v, float kd_v);
    bool set_function_enable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable);
    bool set_mirror_flip(bool mirror, bool flip);
    void Shutdown();

    int media_process_ai_frame(c_media_md *media, int chn, int handle, std::vector<Object>& detections);
    int media_process_ai_frame(c_media_md *media, int chn, int handle, std::vector<OVDObject>& detections);

    int mot_process_ai_frame(const std::vector<Object>& detections);
    int mot_process_ai_frame(const std::vector<OVDObject>& detections);

    void update_time(TRACKER_TIME_E time_type, uint64_t pts, int32_t count_per_second, bool update_last_time);
    uint64_t get_interval(TRACKER_TIME_E time_type);
    

    void get_pid_result();
    inline void target_get_data(rid_trace_obj_t obj);
    void update_target_info(bool get_data_success);

    int track_step(c_media_md *media, uint32_t channel, int handle);




    

private:
    // 工具成员
    MyTracker *mp_tracker; // 目标追踪器
    PIDCtrl *mp_pid_controller; // PID控制器
    // 其他成员变量
    ai_trace_table_t m_ai_trace_table; // 维护的追踪表
    std::unordered_map<TRACKER_TIME_E, uint64_t> m_last_times; // 上次时间点，以μs为单位
    std::unordered_map<TRACKER_TIME_E, uint64_t> m_last_intervals; // 上次时间间隔
    // 状态标志
    uint8_t mb_is_initialized;
    uint8_t mb_is_tracking;
    bool mb_flip; // 是否翻转坐标
    bool mb_mirror; // 是否正在移动
};





#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_AI_TRACK_H_