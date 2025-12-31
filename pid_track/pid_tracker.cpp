/***
 * @Author: lkyezi / merged
 * @Description: Implementation of unified PID_Tracker (snake_case refactor).
 */
#include "pid_tracker.h"
#include "serial_mcu.h"
#include "misc.h"
#include "time_ex.h"
#include <cstring>
#include <algorithm>

#define PID_PRINTF
#ifdef PID_PRINTF
    #define PID_LOG(fmt, ...) printf("[PID_Tracker] " fmt, ##__VA_ARGS__)
#else
    #define PID_LOG(fmt, ...)
#endif 


BaseTrackerUnified* create_tracker_instance(int32_t tracker_type)
{
    switch(tracker_type)
    {
    case 0:
    return (BaseTrackerUnified*)new MoTracker_();
    case 1:
    return (BaseTrackerUnified*)new MediaTracker_();
#ifdef _SUPPORT_YOLOWORLD_TRACK_
    case 2:
    return (BaseTrackerUnified*)new OvdTracker_();
#endif // _SUPPORT_YOLOWORLD_TRACK_
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
      m_external_target_cb(nullptr), m_external_read_pos_cb(nullptr), m_external_motor_speed_cb(nullptr), m_external_motor_disable_cb(nullptr),
      m_initialized(false), m_running(false)
{
    m_hw_interface = hardware_interface_t();
    memset(&m_motor_ctx, 0, sizeof(m_motor_ctx));
    memset(&m_track_ctx, 0, sizeof(m_track_ctx));
}

PID_Tracker::PID_Tracker(const char* config_file, c_media* media): PID_Tracker()
{
    if(!init(config_file, media, nullptr))
    {
        PID_LOG("PID_Tracker initialized failed in constructor.\n");
    }
}

PID_Tracker::~PID_Tracker()
{
    shutdown();
}

// ================= 初始化 =================
bool PID_Tracker::init(const char* config_file, c_media* media, const hardware_interface_t* hw_override)
{
    if(m_initialized)
    {
        return true;
    }
    mp_media = media;
    if(!mp_media)
    {
        PID_LOG("PID_Tracker init failed: media nullptr\n");
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
        PID_LOG("PID Control Lib init from xml %s : %d\n", config_file, ok);
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

    m_timer = new timer_ctrl();

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
   if(m_timer)
   {
       delete m_timer;
       m_timer = nullptr;
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
void PID_Tracker::install_hardware_interface(const hardware_interface_t* hw_override)
{
    memset(&m_hw_interface, 0, sizeof(m_hw_interface));
    m_hw_interface.p_user_ctx                  = this;
    m_hw_interface.p_get_target_pixel_position = &PID_Tracker::s_get_target_pixel_callback;
    m_hw_interface.p_read_pos                   = &PID_Tracker::s_read_pos_callback;
    m_hw_interface.p_motor_set_speed           = &PID_Tracker::s_motor_set_speed_callback;
    m_hw_interface.p_motor_disable             = &PID_Tracker::s_motor_disable_callback;

    m_external_hw_interface = const_cast<hardware_interface_t*>(hw_override);
    m_external_target_cb        = hw_override ? hw_override->p_get_target_pixel_position : nullptr;
    m_external_read_pos_cb      = hw_override ? hw_override->p_read_pos : nullptr;
    m_external_motor_speed_cb   = hw_override ? hw_override->p_motor_set_speed : nullptr;
    m_external_motor_disable_cb = hw_override ? hw_override->p_motor_disable : nullptr;
}

// ================= 基本控制 =================
void PID_Tracker::start()
{
    if(m_initialized && !m_running)
    {
        PID_LOG("PID_Tracker start called.\n");
        pid_control_start(m_lib);
        m_running = true;
        // 目标消失超过设定时间后返回看守点
        m_timer->set_delay(m_track_ctx.ai_disappear_back_time, timer_ctrl::time_unit::milliseconds, TRACKER_TIMER_E::TRACKER_TIMER_TARGET_DISAPPEAR);
        // 目标静止超过设定时间后返回看守点
        m_timer->set_delay(m_track_ctx.ai_static_back_timeout, timer_ctrl::time_unit::milliseconds, TRACKER_TIMER_E::TRACKER_TIMER_TARGET_STATIC);
        // 目标消失超过1s后就可以去追其他目标了
        m_timer->set_delay(1000, timer_ctrl::time_unit::milliseconds, TRACKER_TIMER_E::TRACKER_TIMER_TARGET_SWITCH);
    }
}

void PID_Tracker::stop()
{
    if(m_initialized && m_running)
    {
        PID_LOG("PID_Tracker stop called.\n");
        pid_control_stop(m_lib);
        m_running = false;
        m_current_target.rid = -1;
        m_track_ctx.cur_track_level = TRACK_PRIORITY_E::TRACK_TYPE_BUTT;
        m_timer->reset();
    }
}

void PID_Tracker::reset()
{
    if(m_initialized)
    {
        PID_LOG("PID_Tracker reset called.\n");
        pid_control_reset(m_lib);
        m_timer->reset();
    }
}

void PID_Tracker::step()
{
    if(m_initialized && m_running)
    {
        // PID_LOG("PID_Tracker step called, obj visible: %d\n", m_current_target.visible);
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

const pid_control_config_t* PID_Tracker::get_config() const
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

bool PID_Tracker::set_ai_cfg(TRACK_PRIORITY_E track_priority[TRACK_PRIORITY_E::TRACK_TYPE_BUTT], int32_t support_bitmap, int32_t ai_disappear_back_time, int32_t ai_static_back_timeout)
{
    for(int i=0; i<TRACK_PRIORITY_E::TRACK_TYPE_BUTT; i++)
    {
        m_track_ctx.track_priority[i] = track_priority[i];
    }
    m_track_ctx.support_bitmap = support_bitmap;
    m_track_ctx.cur_track_level = TRACK_PRIORITY_E::TRACK_TYPE_BUTT;
    m_track_ctx.ai_disappear_back_time = ai_disappear_back_time;
    m_track_ctx.ai_static_back_timeout = ai_static_back_timeout;
    PID_LOG("AI Track cfg: bitmap=0x%X\n", support_bitmap);
    return true;
}

std::string PID_Tracker::get_track_info(uint8_t type)
{
    std::string osd = "";
    switch(type)
    {
        // 显示在时间上
        case 0:
            {
                static bool s_track_running = false;
                static int32_t s_last_rid = -1;
                if(s_track_running != m_running || s_last_rid != m_current_target.rid)
                {
                    s_track_running = m_running;
                    s_last_rid = m_current_target.rid;                
                    osd = "s:" + std::to_string(is_running()) + " r:" + std::to_string(m_current_target.rid);
                }
                // osd = "s:" + std::to_string(is_running()) + " r:" + std::to_string(m_current_target.rid) + " v:" + std::to_string(m_current_target.visible) + " x:" + std::to_string(static_cast<int>(m_current_target.x_pixel - m_motor_ctx.horizontal_center)) + " y:" + std::to_string(static_cast<int>(m_current_target.y_pixel - m_motor_ctx.vertical_center));
            }
            break;
        // 显示在名字上
        case 1:
            {
                return osd;
                int p_move_step;
                int t_move_step;
                if(serial_mcu_get_move_steps(P_MOTOR, &p_move_step) < 0)
                {
                    BC_LOG("fail\n");
                    BC_LOG("serail_mcu_set_auto_calibration_steps\n");
                }
                if(serial_mcu_get_move_steps(T_MOTOR, &t_move_step) < 0)
                {
                    BC_LOG("fail\n");
                    BC_LOG("serail_mcu_set_auto_calibration_steps\n");
                }
                app_optocoupler_pos_table_t app_date;
                if(serial_mcu_get_optocoupler_pos(P_MOTOR, &app_date) < 0)
                {
                    BC_LOG("fail\n");
                    BC_LOG("serial_mcu_get_optocoupler_pos\n");
                }

                osd = (m_motor_ctx.horizontal_direction ? "l: " : "r: ") + std::to_string(m_motor_ctx.horizontal_speed_level) + " " + (m_motor_ctx.vertical_direction ? "d: " : "u: ") + std::to_string(m_motor_ctx.vertical_speed_level) + " p:" + std::to_string(p_move_step) + " t:" + std::to_string(t_move_step) + " pc:" + std::to_string(app_date.optocouplers[1].cur_pos) + " pt:" + std::to_string(app_date.optocouplers[1].opt_count);
            }
        default:
            break;
    };
    return osd;
}

// ================= 目标更新 =================
void PID_Tracker::update_target(float x_pixel, float y_pixel, bool visible, uint64_t pts, int32_t rid, float proportion)
{
    m_current_target.x_pixel = x_pixel;
    m_current_target.y_pixel = y_pixel;
    m_current_target.visible = visible;
    m_current_target.pts = pts;
    m_current_target.proportion = proportion;
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
        PID_LOG("PID_Tracker not initialized!\n");
        return -1;
    }
    m_current_pts = get_tick_count(0);
    // 更新目标丢失定时器状态
    m_timer->update(m_current_pts, timer_ctrl::time_unit::milliseconds);

    // 手动操作超过15s后才可以进行追踪
    if(!m_running && m_timer->delay_active(TRACKER_TIMER_E::TRACKER_TIMER_DELAY) && !m_timer->delay_reached(TRACKER_TIMER_E::TRACKER_TIMER_DELAY))
    {
        PID_LOG("PID_Tracker in delay state, skipping track_step.\n");
        stop();
        return 0;
    }

    // 先假设目标不可见，之后的逻辑会尝试更新可见状态
    m_current_target.visible = false;

    // 当前帧有可以获取的数据，就进行目标选择处理逻辑
    if(mp_tracker->get_tracker_result(mp_media->get_media_md(), channel, handle, m_track_ctx.support_bitmap))
    {
        select_target();
    }

    // 更新当前目标的时间戳
    m_current_target.pts = mp_tracker->pts();

    apply_pid();
    return 0;
}

void PID_Tracker::track_delay(uint64_t delay_ms)
{
    stop();
    if(delay_ms > 0)
    {
        PID_LOG("PID_Tracker entering delay state for %llu ms.\n", delay_ms);
        m_timer->update(get_tick_count(0), timer_ctrl::time_unit::milliseconds);
        m_timer->set_delay(delay_ms, timer_ctrl::time_unit::milliseconds, TRACKER_TIMER_E::TRACKER_TIMER_DELAY);
    }
}


int PID_Tracker::select_target()
{
    if(mp_tracker)
    {
        // 根据优先级高低选择目标
        for(int i=0; i<TRACK_PRIORITY_E::TRACK_TYPE_BUTT; i++)
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
                    if(m_current_target.visible)
                    {
                        // 仍然看到当前目标，更新更换目标计时器
                        m_timer->update_timer(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_SWITCH);
                    }
                    else
                    {
                        // 当前目标看不见超过1s，尝试选择其他目标
                        if(m_timer->delay_reached(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_SWITCH))
                        {
                            // 由于进到这个分支就证明有其他目标了，所以直接切换即可
                            PID_LOG("Current target RID %d lost, switching target.\n", m_current_target.rid);
                            mp_tracker->get_closest(m_current_target);
                            m_timer->update_timer(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_SWITCH);
                        }
                    }
                }
                else
                {
                    mp_tracker->get_closest(m_current_target);
                }
            }
            break;
        }
    }
    return 0;
}

void PID_Tracker::apply_pid()
{
    static int32_t last_rid = -1;

    if(m_running)
    {
        // 目标静止超过设定时间，认为目标丢失，重置 rid
        // if(m_motor_ctx.vertical_speed_level != 0 || m_motor_ctx.horizontal_speed_level != 0)
        // {
        //     m_timer->update_timer(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_STATIC);
        // }
        // if(m_timer->delay_reached(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_STATIC))
        // {
        //     PID_LOG("Target RID %d considered lost due to static timeout.\n", m_current_target.rid);
        //     m_current_target.rid = -1;
        // }

        // 目标消失超过设定时间，认为目标丢失，重置 rid
        if(m_current_target.visible)
        {
            m_timer->update_timer(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_DISAPPEAR);
        }
        if(m_timer->delay_reached(TRACKER_TIMER_E::TRACKER_TIMER_TARGET_DISAPPEAR))
        {
            PID_LOG("Target RID %d considered lost due to lost timeout.\n", m_current_target.rid);
            m_current_target.rid = -1;
        }
    }


    if(last_rid != m_current_target.rid)
    {
        // rid 变化时重置 PID 控制器
        pid_control_reset(m_lib);
        if(m_current_target.rid != -1)
        {
            start();
        }
        if(m_current_target.rid == -1)
        {
            stop();
        }
        PID_LOG("Target RID changed from %d to %d, PID reset.\n", last_rid, m_current_target.rid);
        last_rid = m_current_target.rid;
    }

    // 无目标则不执行 PID 步进
    if(m_current_target.rid == -1)
    {
        return;
    }

    step();
}


// ================= 回调静态封装 =================
bool PID_Tracker::s_get_target_pixel_callback(void* ctx, float* x_pixel, float* y_pixel, uint64_t* pts, float *proportion)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    return self ? self->get_target_pixel(x_pixel, y_pixel, pts, proportion) : false;
}

int32_t PID_Tracker::s_read_pos_callback(void* ctx, uint8_t axis)
{
    PID_Tracker* self = reinterpret_cast<PID_Tracker*>(ctx);
    return self ? self->get_pos(axis) : 0;
}

void PID_Tracker::s_motor_set_speed_callback(void* ctx, uint8_t axis, uint8_t level, uint8_t direction)
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

void PID_Tracker::s_motor_disable_callback(void* ctx, uint8_t axis)
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
bool PID_Tracker::get_target_pixel(float* x_pixel, float* y_pixel, uint64_t* pts, float *proportion) const
{
    if(m_external_target_cb)
    {
        return m_external_target_cb(m_hw_interface.p_user_ctx, x_pixel, y_pixel, pts, proportion);
    }
    *x_pixel = m_current_target.x_pixel;
    *y_pixel = m_current_target.y_pixel;
    *pts     = m_current_target.pts;
    *proportion = m_current_target.proportion;
    return m_current_target.visible;
}

int32_t PID_Tracker::get_pos(uint8_t axis) const
{
    if(m_external_read_pos_cb)
    {
        return m_external_read_pos_cb(m_hw_interface.p_user_ctx, axis);
    }

    int32_t pos = serial_mcu_get_pos(motor_index(axis));

    if(axis == AXIS_VERTICAL)
    {
        pos = is_flipped() ? - pos : pos;
    }

    return pos;
}

void PID_Tracker::motor_set_speed(uint8_t axis, uint8_t level, uint8_t direction)
{
    if(m_external_motor_speed_cb)
    {
        m_external_motor_speed_cb(m_hw_interface.p_user_ctx, axis, level, direction);
        return;
    }

    // 先按画面中物体方向赋个值
    if(axis == AXIS_VERTICAL)
    {
        m_motor_ctx.vertical_direction = direction;
        m_motor_ctx.vertical_speed_level = level;
    }
    else if(axis == AXIS_HORIZONTAL)
    {
        m_motor_ctx.horizontal_direction = direction;
        m_motor_ctx.horizontal_speed_level = level;
    }

    static bool enable_limit_max_exposure = false;
    if(m_motor_ctx.mirror && axis == AXIS_HORIZONTAL)
    {
        direction = direction ? 0 : 1;
    }
    // 默认吊装、水平方向是0点时：
    //                        吊装 不翻转：
    if( !m_motor_ctx.flip && axis == AXIS_VERTICAL)
    {
        direction = direction ? 0 : 1;
    }
    if(level == 16 || m_track_ctx.b_day_mode == false)
    {
        if(enable_limit_max_exposure == false)
        {
            enable_limit_max_exposure = true;
            PID_LOG("speed level 16 warning: need exposure locked!\n");
            mp_media->get_media_input()->enable_limit_max_exposure(1);
        }
    }
    else
    {
        if(enable_limit_max_exposure)
        {
            enable_limit_max_exposure = false;
            PID_LOG("speed level <16: release exposure lock\n");
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
        m_external_motor_disable_cb(m_hw_interface.p_user_ctx, axis);
        return;
    }
    int motor = motor_index(axis);
    serial_mcu_set_speed_level(motor, 0, 0);
}
