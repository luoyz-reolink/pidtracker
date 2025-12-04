/*** 
 * @Author: lkyezi
 * @Date: 2025-11-03 10:55:10
 * @LastEditTime: 2025-11-28 17:22:45
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */

 #include "ai_track.h"
 #include "time_ex.h"
 #include <limits>
#include "serial_mcu.h"

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

TargetInfo g_target_info = {-1.0f, -1.0f, false, 0, -1};


static void gen_ai_trace_table(ai_trace_table_t *pdata, MOTrackingResult *result_table)
{
    pdata->count = result_table->count;
	for (unsigned int i = 0; i < result_table->count; i++)
	{
		pdata->objects[i].x_min = result_table->objects[i].x_min;
		pdata->objects[i].y_min = result_table->objects[i].y_min;
		pdata->objects[i].x_max = result_table->objects[i].x_max;
		pdata->objects[i].y_max = result_table->objects[i].y_max;
		pdata->objects[i].rid = result_table->objects[i].rid;
		pdata->objects[i].state = result_table->objects[i].state;
		pdata->objects[i].dist_x = result_table->objects[i].dist_x;
		pdata->objects[i].dist_y = result_table->objects[i].dist_y;
        // 先不赋值 reid 特征
		// pdata->objects[i].curr_feat = result_table->objects[i].curr_feat;
	}

	stable_sort(pdata->objects, pdata->objects + pdata->count, [](const rid_trace_obj_t &a, const rid_trace_obj_t &b) {
		return a.dist_x * a.dist_x + a.dist_y * a.dist_y < b.dist_x * b.dist_x + b.dist_y * b.dist_y;
	});
}

inline int get_motor_index(uint8_t axis)
{
    switch(axis) {
        case AXIS_HORIZONTAL:
            return P_MOTOR;
        case AXIS_VERTICAL:
            return T_MOTOR;
        default:
            break;
    }
    
    return -1;
}


AITrack::~AITrack()
{
}

void AITrack::update_hardware_interface()
{
    // Update hardware interface callbacks if needed
}

 bool AITrack::Initialize(const char* config_file, c_media *media)
 {
    mp_media = media;
    if(mp_media == nullptr)
    {
        printf("AITrack Initialize failed: media is nullptr\n");
        return false;
    }

    mp_hw_interface = new HardwareInterface();
    mp_hw_interface->read_angle = [this](uint8_t axis) -> float {
        printf("Read Angle: Axis=%u\n", axis);
        int motor = get_motor_index(axis);
        int32_t steps = serial_mcu_get_pos(motor);
        if(axis == AXIS_VERTICAL)
        {
            const float angle_per_step = static_cast<float>(m_motor_pos.t_max_angle - m_motor_pos.t_min_angle) / static_cast<float>(m_motor_pos.t_max_pos - m_motor_pos.t_min_pos);
            float angle = m_motor_pos.orientation ? (steps - m_motor_pos.t_min_pos) * angle_per_step + m_motor_pos.t_min_angle
                                                  : (m_motor_pos.t_max_pos - steps) * angle_per_step + m_motor_pos.t_min_angle;
            
            // 这里按正挂是姿态角为正，反挂为负
            if(!m_motor_pos.flip)
            {
                angle = -angle;
            }
            return angle;
        }
        else
        {
            printf("Unsupported axis for angle reading: %d\n", axis);
        }
        return 0.0f;
    };
    mp_hw_interface->motor_set_speed = [this](uint8_t axis, uint8_t level, uint8_t direction) {
        printf("Set Motor Speed: Axis=%u, Level=%u, Direction=%u\n", axis, level, direction);
        static bool enable_limit_max_exposure = false;
        int motor = get_motor_index(axis);
        if(m_motor_pos.flip && axis == AXIS_VERTICAL)
        {
            direction = direction ? 0 : 1;
        }
        bool need_exposure_lock = (level == 16) || (m_track_ctx.b_day_mode == false);
        // 根据level等级做实际曝光处理
        if(need_exposure_lock && !enable_limit_max_exposure)
        {
            enable_limit_max_exposure = true;
            mp_media->get_media_input()->enable_limit_max_exposure(1);
        }
        else if(!need_exposure_lock && enable_limit_max_exposure)
        {
            enable_limit_max_exposure = false;
            mp_media->get_media_input()->enable_limit_max_exposure(0);
        }
        
        serial_mcu_set_speed_level(motor, level, direction);
    };

    mp_pid_controller = new PIDCtrl();
    if(mp_pid_controller->IsInitialized() == false)
    {
        // 要是需要外部实现接口的话可以在这里传入对应的函数指针覆盖掉默认实现
        mp_pid_controller->Init(config_file, mp_hw_interface);
    }

    mp_pid_config = mp_pid_controller->GetConfig();
    m_motor_pos.horizontal_center = mp_pid_config->system.frame_width_pixels / 2;
    m_motor_pos.vertical_center = mp_pid_config->system.frame_height_pixels / 2;
    //  初始化目标追踪器
    mp_tracker = new MyTracker(30, m_motor_pos.horizontal_center * 2, m_motor_pos.vertical_center * 2);



    // 初始化计时器
    for(int i = 0; i < TRACKER_TIMER_BUTT; i++)
    {
        m_track_timer[i] = new timer_ctrl();
    }

    return true;
 }

bool AITrack::reload_config(const char* config_file)
{
    if(mp_pid_controller == nullptr)
    {
        return false;
    }
    return mp_pid_controller->ReloadConfig(config_file);
}

bool AITrack::set_pid_params(float kp_h, float ki_h, float kd_h, float kp_v, float ki_v, float kd_v)
{
    if (mp_pid_controller == nullptr) {
        return false;
    }
    return mp_pid_controller->SetPidParams(kp_h, ki_h, kd_h, kp_v, ki_v, kd_v);
}

bool AITrack::set_function_enable(bool prediction_enable, bool d_filter_enable, bool prediction_guard_enable, bool adaptive_guard_enable)
{
    if (mp_pid_controller == nullptr) {
        return false;
    }
    return mp_pid_controller->SetFunctionEnable(prediction_enable, d_filter_enable, prediction_guard_enable, adaptive_guard_enable);
}

bool AITrack::set_mirror_flip(bool mirror, bool flip)
{
    m_motor_pos.mirror = mirror;
    m_motor_pos.flip = flip;
    mp_pid_controller->SetFlipped(flip);
    printf("Set mirror: %d, flip: %d\n", m_motor_pos.mirror, m_motor_pos.flip);
    return true;
}

bool AITrack::set_ai_cfg(TRACK_PRIORITY_E track_priority[TRACK_TYPE_BUTT], int32_t support_bitmap)
{
    for(int i = 0; i < TRACK_TYPE_BUTT; i++)
    {
        m_track_ctx.track_priority[i] = track_priority[i];
    }
    m_track_ctx.support_bitmap = support_bitmap;
    m_track_ctx.cur_track_level = TRACK_TYPE_BUTT; // 初始为无追踪状态
    printf("AI Track config set: support_bitmap=0x%X\n", support_bitmap);
    return true;
}

void AITrack::track_delay(uint64_t delay_time)
{
    if (mp_pid_controller != nullptr) {
        mp_pid_controller->Stop();
    }
    // 延时停止后可以做一些其他处理
    if (delay_time > 0) {
        m_track_timer[TRACKER_TIMER_DELAY]->set_delay(delay_time, timer_ctrl::time_unit::milliseconds);
    }
}

 void AITrack::Shutdown()
 {
     if (mp_tracker != nullptr) {
         mp_tracker->reset();
     }

     if (mp_pid_controller) {
         mp_pid_controller->Shutdown();
     }
 }


int AITrack::pre_process(c_media_md *media, int chn, int handle, std::vector<Object>& detections)
{
    ai_mot_cache_t ai_mot_cache = {0};
    int query_ret = media->get_ai_mot_all(chn, &ai_mot_cache);

    if(query_ret != _AI_RESULT_GET_ok_)
    {
        return query_ret;
    }

    m_track_timer[TRACKER_TIMER_AI_PTS]->update(ai_mot_cache.pd_ai_mot_list.pts, timer_ctrl::time_unit::microseconds, true);

    for(auto &item : g_master_key_map)
    {
        ai_mot_pd_table_t *p_ai_mot_xd_table = NULL;
        if((BIT(item.first) & m_track_ctx.support_bitmap) == 0)
        {
            continue;
        }
        p_ai_mot_xd_table = (ai_mot_pd_table_t *)(((char *)&ai_mot_cache) + item.second.table_offset);
        if(p_ai_mot_xd_table->count > 0)
        {
            MyObject cur_obj;
            for(unsigned int i = 0; i < p_ai_mot_xd_table->count; i++)
            {
                cur_obj.xmin = p_ai_mot_xd_table->objects[i].x_min;
                cur_obj.ymin = p_ai_mot_xd_table->objects[i].y_min;
                cur_obj.xmax = p_ai_mot_xd_table->objects[i].x_max;
                cur_obj.ymax = p_ai_mot_xd_table->objects[i].y_max;
                cur_obj.prob = p_ai_mot_xd_table->objects[i].score;
                cur_obj.label = item.second.label;
                cur_obj.curr_feat.assign(std::begin(p_ai_mot_xd_table->objects[i].reid_feature), std::end(p_ai_mot_xd_table->objects[i].reid_feature));
                detections.emplace_back(cur_obj);
            }
        }
    }

    return query_ret;
}

int AITrack::pre_process(c_media_md *media, int chn, int handle, std::vector<OVDObject>& detections)
{
    // Implementation for processing AI frame with OVDObject detections
    return 0;
}

int AITrack::get_target(const std::vector<Object>& detections)
{
    bool get_target_success = false;
    g_target_info.visible = false;

    // 先不考虑运动补偿
    mp_tracker->process(detections, false, 0, 0);

    // 先不筛选获取结果
    MyTracker::result_type track_results;
    MyTracker::result_type specific_result;
    mp_tracker->collect_results(track_results);

    for(int i = 0; i < TRACK_TYPE_BUTT; i++)
    {
        int type = m_track_ctx.track_priority[i];
        if((BIT(type) & m_track_ctx.support_bitmap) == 0 || mp_tracker->get_specific_type(track_results, specific_result, type) == false)
        {
            continue;
        }

        // 识别到更高优先级的物体了
        if(m_track_ctx.cur_track_level < i)
        {
            m_track_ctx.cur_track_level = i;
            // 选择距离中心点最近的目标
            mp_tracker->get_closest(specific_result, g_target_info, m_motor_pos.horizontal_center, m_motor_pos.vertical_center);
            break;
        }
        else
        {
            // 继续追踪当前目标
            if(g_target_info.rid != -1)
            {
                mp_tracker->get_specific_rid(specific_result, g_target_info);
                break;
            }
            else
            {
                mp_tracker->get_closest(specific_result, g_target_info, m_motor_pos.horizontal_center, m_motor_pos.vertical_center);
                break;
            }
        }
    }
    // 更新目标时间
    g_target_info.pts = m_track_timer[TRACKER_TIMER_AI_PTS]->current(timer_ctrl::time_unit::microseconds);

    // 根据mirror和flip调整坐标
    // 更新：这里不应该修改坐标，因为从AI获取的数据里已经调整过了，将设置的东西放到PID_CTRL里处理即可

    return 0;
}

int AITrack::get_target(const std::vector<OVDObject>& detections)
{
    // Implementation for MOT processing with OVDObject detections
    return 0;
}


inline void AITrack::target_get_data(rid_trace_obj_t obj)
{
}

void AITrack::update_target_info(bool get_data_success)
{
}

void AITrack::get_pid_result()
{
    static int32_t last_rid = -1;

    if(g_target_info.rid != -1)
    {
        if(m_track_timer[TRACKER_TIMER_TARGET_LOST]->delay_active() && m_track_timer[TRACKER_TIMER_TARGET_LOST]->delay_reached())
        {
            // 超过0.5秒未见目标，重置rid
            printf("Target lost for rid: %d\n", g_target_info.rid);
            g_target_info.rid = -1;
        }
    }


    if(last_rid != g_target_info.rid)
    {
        // 新目标，重置PID
        mp_pid_controller->Reset();
        if(last_rid == -1)
        {
            // 需要开启PID
            track_start();
        }
        last_rid = g_target_info.rid;
        if(g_target_info.rid == -1)
        {
            // 需要关闭PID
            track_stop();
            return;
        }
    }

    if(g_target_info.rid == -1)
    {
        // 当前没有在追踪目标，直接返回
        return;
    }

    
    mp_pid_controller->UpdateTarget(g_target_info);
    mp_pid_controller->Step();
}

bool AITrack::is_system_delay_timeout(timer_ctrl *timer)
{
    if(timer == nullptr)
    {
        return false;
    }
    timer->update(m_current_pts, timer_ctrl::time_unit::milliseconds, false);
    if(!timer->delay_active() || timer->delay_reached())
    {
        timer->cancel_delay();
        return true;
    }
    return false;
}

int AITrack::track_start()
{
    if(mb_is_initialized == 0)
    {
        printf("AITrack not initialized!\n");
        return -1; // Not initialized
    }

    if(mp_pid_controller != nullptr && !mp_pid_controller->is_running())
    {
        mp_pid_controller->Start();
        m_track_timer[TRACKER_TIMER_TARGET_LOST]->set_delay(500, timer_ctrl::time_unit::milliseconds); // 目标丢失计时器
    }

    return 0;
}

int AITrack::track_stop()
{
    if(mb_is_initialized == 0)
    {
        printf("AITrack not initialized!\n");
        return -1; // Not initialized
    }

    if(mp_pid_controller != nullptr && mp_pid_controller->is_running())
    {
        printf("AITrack stopping PID controller.\n");
        mp_pid_controller->Stop();
        m_track_timer[TRACKER_TIMER_TARGET_LOST]->cancel_delay();
    }

    return 0;
}

int AITrack::track_reset()
{
    if(mb_is_initialized == 0)
    {
        printf("AITrack not initialized!\n");
        return -1; // Not initialized
    }

    if(mp_pid_controller != nullptr)
    {
        mp_pid_controller->Reset();
        m_track_timer[TRACKER_TIMER_TARGET_LOST]->cancel_delay();
    }

    return 0;
}

int AITrack::track_step(c_media_md *media, uint32_t channel, int handle)
{
    // Perform a tracking step and update PID controller
    if(mb_is_initialized == 0)
    {
        printf("AITrack not initialized!\n");
        return -1; // Not initialized
    }

    m_current_pts = get_tick_count(0);

    // 检查是否在追踪延时停止期间
    if(!is_system_delay_timeout(m_track_timer[TRACKER_TIMER_DELAY]))
    {
        track_stop();
        return 0;
    }

    std::vector<MyObject> tracked_object;

    if(pre_process(media, channel, handle, tracked_object) == _AI_RESULT_GET_ok_)
    {
        get_target(tracked_object);
    }
    else 
    {
        return -1;
    }

    // 更新目标丢失计时器状态
    m_track_timer[TRACKER_TIMER_TARGET_LOST]->update(get_tick_count(0), timer_ctrl::time_unit::milliseconds, g_target_info.visible);

    get_pid_result();

    return 0;
}

   