/***
 * @Author: lkyezi / merged
 * @Description: Implementation of unified PID_Tracker (snake_case refactor).
 */
#include "pid_tracker.h"
#include "serial_mcu.h"
#include "misc.h"
#include "time_ex.h"
#include <string.h>
#include <algorithm>

typedef struct
{
    size_t table_offset;
    int label;
    ALARM_IN_IDX_E track_type;
    void (Bc_MOTracker::*result_func)(MOTrackingResult *);
} master_key_t;

static std::unordered_map<int, master_key_t> g_master_key_map = {
    {TRACK_PRIORITY_E::TRACK_TYPE_PERSON,
     {.table_offset = offsetof(ai_mot_cache_t, pd_ai_mot_list),
      .label = 2,
      .track_type = ALARM_IN_IDX_E::ALARM_IN_IDX_AI_PEOPLE,
      .result_func = &Bc_MOTracker::Bc_Get_PD_Result}},
    {TRACK_PRIORITY_E::TRACK_TYPE_MOTOR_VEHICLE,
     {.table_offset = offsetof(ai_mot_cache_t, vd_ai_mot_list),
      .label = 3,
      .track_type = ALARM_IN_IDX_E::ALARM_IN_IDX_AI_VEHICLE,
      .result_func = &Bc_MOTracker::Bc_Get_VD_Result}},
    {TRACK_PRIORITY_E::TRACK_TYPE_ANIMAL,
     {.table_offset = offsetof(ai_mot_cache_t, ad_ai_mot_list),
      .label = 1,
      .track_type = ALARM_IN_IDX_E::ALARM_IN_IDX_AI_DOG_CAT,
      .result_func = &Bc_MOTracker::Bc_Get_AD_Result}}};


BaseTrackerUnified* create_tracker_instance(int32_t tracker_type)
{
    switch(tracker_type)
    {
    case 0:
    return (BaseTrackerUnified*)new MoTracker_();
    case 1:
    return (BaseTrackerUnified*)new MediaTracker_();
    case 2:
    return (BaseTrackerUnified*)new OvdTracker_();
    default:
        return nullptr;
    }
}



// ================= 工具函数 =================
int PID_Tracker::motor_index(uint8_t axis)
{
    switch(axis)
    {
        case AXIS_HORIZONTAL:
        {
            return P_MOTOR;
        }
        case AXIS_VERTICAL:
        {
            return T_MOTOR;
        }
        default:
        {
            return -1;
        }
    }
}

// ================= 构造 / 析构 =================
PID_Tracker::PID_Tracker()
    : m_lib(nullptr), m_pid_config(nullptr), m_external_hw_interface(nullptr),
      mp_tracker(nullptr), m_current_target{0.f,0.f,false,0,-1}, mp_media(nullptr), m_current_pts(0),
      m_external_target_cb(nullptr), m_external_read_angle_cb(nullptr), m_external_motor_speed_cb(nullptr), m_external_motor_disable_cb(nullptr),
      m_initialized(false), m_running(false)
{
    memset(&m_hw_interface, 0, sizeof(m_hw_interface));
    memset(&m_motor_ctx, 0, sizeof(m_motor_ctx));
    switch (misc_get_board_type()) {
        case _BT_NT98538A_SD7_:
            {
                m_motor_ctx.t_min_pos = 0;
                m_motor_ctx.t_max_pos = 1024;
                m_motor_ctx.p_min_pos = 0;
                m_motor_ctx.p_max_pos = 2700;

                m_motor_ctx.t_min_angle = 0;
                m_motor_ctx.t_max_angle = 90;
                m_motor_ctx.p_min_angle = 0;
                m_motor_ctx.p_max_angle = 360;

                m_motor_ctx.hanging_upright = false;
                m_motor_ctx.orientation = 0;

                // 其他参数都是0
            }

        default:
            break;
    }
    memset(&m_track_ctx, 0, sizeof(m_track_ctx));
    for(int i=0; i<TRACKER_TIMER_BUTT; i++)
    {
        m_timers[i] = nullptr;
    }
}

PID_Tracker::PID_Tracker(const char* config_file, c_media* media): PID_Tracker()
{
    if(!init(config_file, media, nullptr))
    {
        printf("PID_Tracker initialized failed in constructor.\n");
    }
}

PID_Tracker::~PID_Tracker()
{
    shutdown();
}

// ================= 初始化 =================
bool PID_Tracker::init(const char* config_file, c_media* media, const HardwareInterface* hw_override)
{
    if(m_initialized)
    {
        return true;
    }
    mp_media = media;
    if(!mp_media)
    {
        printf("PID_Tracker init failed: media nullptr\n");
        return false;
    }

    m_lib = pid_control_create();
    if(!m_lib)
    {
        return false;
    }

    install_hardware_interface(hw_override);

    bool ok = false;
    if(config_file && config_file[0] != '\0')
    {
        ok = pid_control_init_from_xml(m_lib, config_file, &m_hw_interface);
    }
    if(!ok)
    {
        ok = pid_control_init_default(m_lib, &m_hw_interface);
    }
    if(!ok)
    {
        pid_control_destroy(m_lib);
        m_lib = nullptr;
        return false;
    }

    m_pid_config = pid_control_get_config(m_lib);
    m_motor_ctx.horizontal_center = m_pid_config->system.frame_width_pixels / 2;
    m_motor_ctx.vertical_center   = m_pid_config->system.frame_height_pixels / 2;

    for(int i=0; i<TRACKER_TIMER_BUTT; i++)
    {
        m_timers[i] = new timer_ctrl();
    }

    mp_tracker = create_tracker_instance(1); // 0: MoTracker_
    m_initialized = mp_tracker->init(m_motor_ctx.horizontal_center, m_motor_ctx.vertical_center);

    m_running = false;

    return m_initialized;
}

void PID_Tracker::shutdown()
{
    if(mp_tracker)
    {
        mp_tracker->reset();
        delete mp_tracker;
        mp_tracker = nullptr;
    }
    for(int i=0; i<TRACKER_TIMER_BUTT; i++)
    {
        if(m_timers[i])
        {
            delete m_timers[i];
            m_timers[i] = nullptr;
        }
    }
    if(m_lib)
    {
        if(m_running)
        {
            pid_control_stop(m_lib);
        }
        pid_control_destroy(m_lib);
        m_lib = nullptr;
    }
    m_initialized = false;
    m_running = false;
}

// ================= 回调安装 =================
void PID_Tracker::install_hardware_interface(const HardwareInterface* hw_override)
{
    memset(&m_hw_interface, 0, sizeof(m_hw_interface));
    m_hw_interface.user_ctx                  = this;
    m_hw_interface.get_target_pixel_position = &PID_Tracker::get_target_pixel_callback;
    m_hw_interface.read_angle                = &PID_Tracker::read_angle_callback;
    m_hw_interface.motor_set_speed           = &PID_Tracker::motor_set_speed_callback;
    m_hw_interface.motor_disable             = &PID_Tracker::motor_disable_callback;

    m_external_hw_interface = const_cast<HardwareInterface*>(hw_override);
    m_external_target_cb        = hw_override ? hw_override->get_target_pixel_position : nullptr;
    m_external_read_angle_cb    = hw_override ? hw_override->read_angle : nullptr;
    m_external_motor_speed_cb   = hw_override ? hw_override->motor_set_speed : nullptr;
    m_external_motor_disable_cb = hw_override ? hw_override->motor_disable : nullptr;
}

// ================= 基本控制 =================
void PID_Tracker::start()
{
    if(m_initialized && !m_running)
    {
        pid_control_start(m_lib);
        m_running = true;
        m_timers[TRACKER_TIMER_TARGET_LOST]->set_delay(500, timer_ctrl::time_unit::milliseconds);
    }
}

void PID_Tracker::stop()
{
    if(m_initialized && m_running)
    {
        pid_control_stop(m_lib);
        m_running = false;
        m_current_target.rid = -1;
        m_track_ctx.cur_track_level = TRACK_TYPE_BUTT;
        m_timers[TRACKER_TIMER_TARGET_LOST]->cancel_delay();
    }
}

void PID_Tracker::reset()
{
    if(m_initialized)
    {
        pid_control_reset(m_lib);
    }
}

void PID_Tracker::step()
{
    if(m_initialized && m_running)
    {
        pid_control_step(m_lib);
    }
}

// ================= 配置相关 =================
bool PID_Tracker::reload_config(const char* config_file)
{
    if(!m_initialized || !config_file || config_file[0] == '\0')
    {
        return false;
    }
    return pid_control_reload_config(m_lib, config_file);
}

const PIDControlConfig* PID_Tracker::get_config() const
{
    if(m_initialized)
    {
        return m_pid_config;
    }
    return nullptr;
}

bool PID_Tracker::set_pid_params(float kp_h, float ki_h, float kd_h, float kp_v, float ki_v, float kd_v)
{
    if(!m_initialized)
    {
        return false;
    }
    bool ok_h = pid_control_set_pid_params(m_lib, AXIS_HORIZONTAL, kp_h, ki_h, kd_h);
    bool ok_v = pid_control_set_pid_params(m_lib, AXIS_VERTICAL,   kp_v, ki_v, kd_v);
    return ok_h && ok_v;
}

bool PID_Tracker::set_function_enable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable)
{
    if(!m_initialized)
    {
        return false;
    }
    bool ok1 = pid_control_set_prediction_enabled(m_lib, prediction_enable);
    bool ok2 = pid_control_set_d_filter_enabled(m_lib, d_filter_enable);
    bool ok3 = pid_control_set_prediction_guard_enabled(m_lib, prediction_guard_enable);
    bool ok4 = pid_control_set_adaptive_guard_enabled(m_lib, adaptive_guard_enable);
    return ok1 && ok2 && ok3 && ok4;
}

bool PID_Tracker::set_ai_cfg(TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT], int32_t support_bitmap)
{
    for(int i=0; i<TRACK_TYPE_BUTT; i++)
    {
        m_track_ctx.track_priority[i] = track_priority[i];
    }
    m_track_ctx.support_bitmap = support_bitmap;
    m_track_ctx.cur_track_level = TRACK_TYPE_BUTT;
    printf("AI Track cfg: bitmap=0x%X\n", support_bitmap);
    return true;
}

// ================= 目标更新 =================
void PID_Tracker::update_target(float x_pixel, float y_pixel, bool visible, uint64_t pts, int32_t rid)
{
    m_current_target.x_pixel = x_pixel;
    m_current_target.y_pixel = y_pixel;
    m_current_target.visible = visible;
    m_current_target.pts = pts;
    m_current_target.rid = rid;
}

void PID_Tracker::update_target(const TargetInfo& info)
{
    m_current_target = info;
}

// ================= AI + PID 一步 =================
int PID_Tracker::track_step(uint32_t channel, int handle)
{
    if(!m_initialized)
    {
        printf("PID_Tracker not initialized!\n");
        return -1;
    }
    m_current_pts = get_tick_count(0);

    if(!system_delay_timeout(m_timers[TRACKER_TIMER_DELAY]))
    {
        stop();
        return 0;
    }

    // 当前帧有可以获取的数据，就进行目标选择处理逻辑
    if(mp_tracker->get_tracker_result(mp_media->get_media_md(), channel, handle, m_track_ctx.support_bitmap))
    {
        select_target();
    }

    // 更新当前目标的时间戳
    m_current_target.pts = mp_tracker->pts();
    // 更新目标丢失定时器状态
    m_timers[TRACKER_TIMER_TARGET_LOST]->update(get_tick_count(0), timer_ctrl::time_unit::milliseconds, m_current_target.visible);

    apply_pid();
    return 0;
}

void PID_Tracker::track_delay(uint64_t delay_ms)
{
    stop();
    if(delay_ms > 0)
    {
        m_timers[TRACKER_TIMER_DELAY]->set_delay(delay_ms, timer_ctrl::time_unit::milliseconds);
    }
}

// ================= 前处理 & 选目标 =================
// bool PID_Tracker::pre_process(c_media_md* media_md, int chn, int handle, std::vector<MyObject>& detections)
// {
//     ai_mot_cache_t ai_mot_cache = {0};
//     int ret = media_md->get_ai_mot_all(chn, &ai_mot_cache);
//     if(ret != _AI_RESULT_GET_ok_)
//     {
//         return false;
//     }
//     m_timers[TRACKER_TIMER_AI_PTS]->update(ai_mot_cache.pd_ai_mot_list.pts, timer_ctrl::time_unit::microseconds, true);
//     for(auto &item : g_master_key_map)
//     {
//         ai_mot_pd_table_t *p_ai_mot_xd_table = NULL;
//         if((BIT(item.first) & m_track_ctx.support_bitmap) == 0)
//         {
//             continue;
//         }
//         p_ai_mot_xd_table = (ai_mot_pd_table_t *)(((char *)&ai_mot_cache) + item.second.table_offset);
//         if(p_ai_mot_xd_table->count > 0)
//         {
//             MyObject cur_obj;
//             for(unsigned int i = 0; i < p_ai_mot_xd_table->count; i++)
//             {
//                 cur_obj.xmin = p_ai_mot_xd_table->objects[i].x_min;
//                 cur_obj.ymin = p_ai_mot_xd_table->objects[i].y_min;
//                 cur_obj.xmax = p_ai_mot_xd_table->objects[i].x_max;
//                 cur_obj.ymax = p_ai_mot_xd_table->objects[i].y_max;
//                 cur_obj.prob = p_ai_mot_xd_table->objects[i].score;
//                 cur_obj.label = item.second.label;
//                 cur_obj.curr_feat.assign(std::begin(p_ai_mot_xd_table->objects[i].reid_feature), std::end(p_ai_mot_xd_table->objects[i].reid_feature));
//                 detections.emplace_back(cur_obj);
//             }
//         }
//     }
//     return true;
// }

int PID_Tracker::select_target()
{
    if(mp_tracker)
    {
        m_current_target.visible = false;
        // 根据优先级高低选择目标
        for(int i=0; i<TRACK_TYPE_BUTT; i++)
        {
            int type = m_track_ctx.track_priority[i];
            // 若不支持该类型，或该类型没有获取到数据，跳过
            if(((BIT(type) & m_track_ctx.support_bitmap) == 0) || !mp_tracker->get_specific_type(type))
            {
                continue;
            }
            // 发现优先级高的目标
            if(m_track_ctx.cur_track_level < i)
            {
                m_track_ctx.cur_track_level = i;
                mp_tracker->get_closest(m_current_target);
            }
            else
            {
                if(m_current_target.rid != -1)
                {
                    mp_tracker->get_specific_rid(m_current_target);
                }
                else
                {
                    mp_tracker->get_closest(m_current_target);
                }
            }
            break;
        }
        m_current_target.pts = m_timers[TRACKER_TIMER_AI_PTS]->current(timer_ctrl::time_unit::microseconds);
    }
    return 0;
}

void PID_Tracker::apply_pid()
{
    static int32_t last_rid = -1;

    // 目标消失超过设定时间，认为目标丢失，重置 rid
    if(m_current_target.rid != -1)
    {
        if(m_timers[TRACKER_TIMER_TARGET_LOST]->delay_active() && m_timers[TRACKER_TIMER_TARGET_LOST]->delay_reached())
        {
            printf("Target lost rid=%d\n", m_current_target.rid);
            m_current_target.rid = -1;
        }
    }

    if(last_rid != m_current_target.rid)
    {
        // rid 变化时重置 PID 控制器
        pid_control_reset(m_lib);
        if(last_rid == -1 && m_current_target.rid != -1)
        {
            start();
        }
        if(m_current_target.rid == -1)
        {
            stop();
        }
        last_rid = m_current_target.rid;
    }

    // 无目标则不执行 PID 步进
    if(m_current_target.rid == -1)
    {
        return;
    }

    step();
}

// ================= 延时判断 =================
bool PID_Tracker::system_delay_timeout(timer_ctrl* timer)
{
    if(!timer)
    {
        return false;
    }
    timer->update(m_current_pts, timer_ctrl::time_unit::milliseconds, false);
    if((!timer->delay_active()) || timer->delay_reached())
    {
        timer->cancel_delay();
        return true;
    }
    return false;
}

// ================= 回调静态封装 =================
bool PID_Tracker::get_target_pixel_callback(void* ctx, float* x_pixel, float* y_pixel, uint64_t* pts)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    return self ? self->get_target_pixel(x_pixel, y_pixel, pts) : false;
}

float PID_Tracker::read_angle_callback(void* ctx, uint8_t axis)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    return self ? self->get_angle(axis) : 0.0f;
}

void PID_Tracker::motor_set_speed_callback(void* ctx, uint8_t axis, uint8_t level, uint8_t direction)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    if(self)
    {
        self->motor_set_speed(axis, level, direction);
    }
    else
    {
        int motor = motor_index(axis);
        serial_mcu_set_speed_level(motor, level, direction);
    }
}

void PID_Tracker::motor_disable_callback(void* ctx, uint8_t axis)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    if(self)
    {
        self->motor_disable(axis);
    }
    else
    {
        int motor = motor_index(axis);
        serial_mcu_set_speed_level(motor, 0, 0);
    }
}

// ================= 实例层回调实现 =================
bool PID_Tracker::get_target_pixel(float* x_pixel, float* y_pixel, uint64_t* pts) const
{
    if(m_external_target_cb)
    {
        return m_external_target_cb(m_hw_interface.user_ctx, x_pixel, y_pixel, pts);
    }
    *x_pixel = m_current_target.x_pixel;
    *y_pixel = m_current_target.y_pixel;
    *pts     = m_current_target.pts;
    return m_current_target.visible;
}

float PID_Tracker::get_angle(uint8_t axis) const
{
    if(m_external_read_angle_cb)
    {
        return m_external_read_angle_cb(m_hw_interface.user_ctx, axis);
    }

    if(axis == AXIS_VERTICAL)
    {
        int motor = motor_index(axis);
        int32_t steps = serial_mcu_get_pos(motor);
        const float angle_per_step = static_cast<float>(m_motor_ctx.t_max_pos - m_motor_ctx.t_min_pos) / static_cast<float>(m_motor_ctx.t_max_angle - m_motor_ctx.t_min_angle);
        const float angle =  m_motor_ctx.orientation ? static_cast<float>(m_motor_ctx.t_max_pos - steps) / angle_per_step : static_cast<float>(steps - m_motor_ctx.t_min_pos) / angle_per_step;

        // 这里的pitch角度依然以正挂作为正角度
        return (m_motor_ctx.hanging_upright ^ m_motor_ctx.flip) ? angle : -angle;
    }
    else
    {
        INFO(MODULE_ENC, "Motor %d not support get angle.\n", motor_index(axis));
    }

    return 0.0f;
}

void PID_Tracker::motor_set_speed(uint8_t axis, uint8_t level, uint8_t direction)
{
    if(m_external_motor_speed_cb)
    {
        m_external_motor_speed_cb(m_hw_interface.user_ctx, axis, level, direction);
        return;
    }
    static bool enable_limit_max_exposure = false;
    if((m_motor_ctx.hanging_upright ^ m_motor_ctx.mirror) && axis == AXIS_HORIZONTAL)
    {
        direction = direction ? 0 : 1;
    }
    // 默认吊装、水平方向是0点时：
    //                        吊装 不翻转：
    if((m_motor_ctx.hanging_upright ^ !m_motor_ctx.flip) && axis == AXIS_VERTICAL)
    {
        direction = direction ? 0 : 1;
    }
    if(level == 16 || m_track_ctx.b_day_mode == false)
    {
        if(enable_limit_max_exposure == false)
        {
            enable_limit_max_exposure = true;
            printf("speed level 16 warning: need exposure locked!\n");
            mp_media->get_media_input()->enable_limit_max_exposure(1);
        }
    }
    else
    {
        if(enable_limit_max_exposure)
        {
            enable_limit_max_exposure = false;
            printf("speed level <16: release exposure lock\n");
            mp_media->get_media_input()->enable_limit_max_exposure(0);
        }
    }
    int motor = motor_index(axis);
    serial_mcu_set_speed_level(motor, level, direction);
}

void PID_Tracker::motor_disable(uint8_t axis)
{
    if(m_external_motor_disable_cb)
    {
        m_external_motor_disable_cb(m_hw_interface.user_ctx, axis);
        return;
    }
    int motor = motor_index(axis);
    serial_mcu_set_speed_level(motor, 0, 0);
}
