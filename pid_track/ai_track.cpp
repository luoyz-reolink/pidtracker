/*** 
 * @Author: lkyezi
 * @Date: 2025-11-03 10:55:10
 * @LastEditTime: 2025-11-06 11:39:42
 * @LastEditors: lkyezi
 * @Description: 
 * @
 */

 #include "ai_track.h"
 #include <algorithm>
 #include "time_ex.h"

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
		return a.state > b.state;
	});
}


 AITrack::~AITrack()
 {
 }

void AITrack::update_hardware_interface()
{
    // Update hardware interface callbacks if needed
}

 bool AITrack::Initialize(const char* config_file)
 {
    
    mp_pid_controller = new PIDCtrl();
    if(mp_pid_controller->IsInitialized() == false)
    {
        // 要是需要外部实现接口的话可以在这里传入对应的函数指针覆盖掉默认实现
        mp_pid_controller->Init(config_file, nullptr);
    }

    const PIDControlConfig *pconfig = mp_pid_controller->GetConfig();
    m_ai_trace_table.v_w = pconfig->system.frame_width_pixels;
    m_ai_trace_table.v_h = pconfig->system.frame_height_pixels;
    //  初始化目标追踪器
    mp_tracker = new MyTracker(30, m_ai_trace_table.v_w, m_ai_trace_table.v_h);

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
    mb_mirror = mirror;
    mb_flip = flip;
    printf("Set mirror: %d, flip: %d\n", mb_mirror, mb_flip);
    return true;
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


int AITrack::media_process_ai_frame(c_media_md *media, int chn, int handle, std::vector<Object>& detections)
{
    ai_mot_cache_t ai_mot_cache = {0};
    int query_ret = media->get_ai_mot_all(chn, &ai_mot_cache);

    if(query_ret != _AI_RESULT_GET_ok_)
    {
        return query_ret;
    }
    else 
    {
        update_time(TRACKER_TIME_PTS, ai_mot_cache.pd_ai_mot_list.pts, 1000000, true);
    }

    // TODO: 类型筛选
    
    // 先只追动物
    ai_mot_ad_table_t *p_ad_table = &ai_mot_cache.ad_ai_mot_list;
    for(int i = 0; i < p_ad_table->count; i++)
    {
        MyObject obj;
        obj.xmin = p_ad_table->objects[i].x_min;
        obj.ymin = p_ad_table->objects[i].y_min;
        obj.xmax = p_ad_table->objects[i].x_max;
        obj.ymax = p_ad_table->objects[i].y_max;
        obj.prob = p_ad_table->objects[i].score;
        obj.label = 1; // 先都看作是动物
        obj.curr_feat.assign(std::begin(p_ad_table->objects[i].reid_feature), std::end(p_ad_table->objects[i].reid_feature));
        detections.push_back(obj);
    }

    return query_ret;
}

int AITrack::media_process_ai_frame(c_media_md *media, int chn, int handle, std::vector<OVDObject>& detections)
{
    // Implementation for processing AI frame with OVDObject detections
    return 0;
}

int AITrack::mot_process_ai_frame(const std::vector<Object>& detections)
{
    m_ai_trace_table.count = 0;
    // 先不考虑运动补偿
    mp_tracker->process(detections, false, 0, 0);

    // 先不筛选获取结果
    MyTracker::result_type track_results;
    mp_tracker->collect_results(track_results);

    MOTrackingResult *ad_result = &track_results.ad_result;
    gen_ai_trace_table(&m_ai_trace_table, ad_result);

    return 0;
}

int AITrack::mot_process_ai_frame(const std::vector<OVDObject>& detections)
{
    // Implementation for MOT processing with OVDObject detections
    return 0;
}

void AITrack::update_time(TRACKER_TIME_E time_type, uint64_t pts, int32_t count_per_second, bool update_last_time)
{
    // 将pts转换为微秒
    uint64_t time_us = pts * 1000000 / count_per_second;
    
    if(m_last_times.find(time_type) == m_last_times.end() || m_last_times[time_type] == 0 || time_us == 0)
    {
        m_last_times[time_type] = time_us;
        m_last_intervals[time_type] = 0;
    }
    else
    {
        m_last_intervals[time_type] = time_us < m_last_times[time_type] ? 0 : time_us - m_last_times[time_type];
        if(update_last_time)
        {
            m_last_times[time_type] = time_us;
        }
    }
}

uint64_t AITrack::get_interval(TRACKER_TIME_E time_type)
{
    if(m_last_intervals.find(time_type) != m_last_intervals.end())
    {
        return m_last_intervals[time_type];
    }
    return 0;
}

inline void AITrack::target_get_data(rid_trace_obj_t obj)
{
    if(mb_mirror)
    {

    }

    g_target_info.x_pixel = mb_mirror ? static_cast<float>(m_ai_trace_table.v_w - (obj.x_min + obj.x_max) / 2.0f) : static_cast<float>(obj.x_min + obj.x_max) / 2.0f;
    g_target_info.y_pixel = mb_flip ? static_cast<float>(m_ai_trace_table.v_h - (obj.y_min + obj.y_max) / 2.0f) : static_cast<float>(obj.y_min + obj.y_max) / 2.0f;
    g_target_info.visible = true;
    g_target_info.pts = m_last_times[TRACKER_TIME_PTS];
    g_target_info.rid = obj.rid;
}

void AITrack::update_target_info(bool get_data_success)
{
    if(get_data_success)
    {
        if(g_target_info.rid == -1)
        {
            // 未指定目标，选择第一个稳定目标
            if(m_ai_trace_table.count > 0)
            {
                target_get_data(m_ai_trace_table.objects[0]);
                printf("Selected target rid: %d x: %.2f y: %.2f\n", g_target_info.rid, g_target_info.x_pixel, g_target_info.y_pixel);
            }
            else
            {
                g_target_info.visible = false;
            }
        }
        else
        {
            // 指定目标，查找对应rid
            bool found = false;
            for(unsigned int i = 0; i < m_ai_trace_table.count; i++)
            {
                if(m_ai_trace_table.objects[i].rid == g_target_info.rid)
                {
                    target_get_data(m_ai_trace_table.objects[i]);
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                g_target_info.visible = false;
            }
        }
    }
    else
    {
        g_target_info.visible = false;
    }

    if(g_target_info.visible == false)
    {
        if(get_interval(TRACKER_TIME_SYSTEM) > 2000000)
        {
            // 超过2秒未见目标，重置rid
            g_target_info.rid = -1;
        }
    }

}

void AITrack::get_pid_result()
{
    static int32_t last_rid = -1;
    if(last_rid != g_target_info.rid)
    {
        // 新目标，重置PID
        mp_pid_controller->Reset();
        if(last_rid == -1)
        {
            // 需要开启PID
            printf("Starting PID for new target rid: %d\n", g_target_info.rid);
            mp_pid_controller->Start();
        }
        last_rid = g_target_info.rid;
        if(g_target_info.rid == -1)
        {
            // 需要关闭PID
            printf("Stopping PID as target lost for rid: %d\n", last_rid);
            mp_pid_controller->Stop();
            return;
        }
    }

    if(g_target_info.visible == false)
    {
        // 目标不可见，跳过PID更新
        return;
    }
    
    mp_pid_controller->UpdateTarget(g_target_info);
    mp_pid_controller->Step();
}

int AITrack::track_step(c_media_md *media, uint32_t channel, int handle)
{
    // Perform a tracking step and update PID controller
    if(mb_is_initialized == 0)
    {
        printf("AITrack not initialized!\n");
        return -1; // Not initialized
    }

    std::vector<MyObject> tracked_object;
    bool get_data_success = true;

    if(media_process_ai_frame(media, channel, handle, tracked_object) != _AI_RESULT_GET_ok_)
    {
        get_data_success = false;
    }
    else 
    {
        mot_process_ai_frame(tracked_object);
    }

    update_time(TRACKER_TIME_SYSTEM, get_tick_count(1), 1000, g_target_info.visible);
    update_target_info(get_data_success);
    

    get_pid_result();

    return 0;
}

   