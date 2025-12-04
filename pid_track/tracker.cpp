#include "tracker.h"
#include <limits>

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

MoTracker_::~MoTracker_()
{
    delete mp_mo_tracker;
    mp_mo_tracker = nullptr;
}

bool MoTracker_::get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const
{
    (void)support_bitmap;
    ai_mot_cache_t ai_mot_cache{};
    m_detections.clear();
    int ret = md->get_ai_mot_all(chn, &ai_mot_cache);
    if(ret == _AI_RESULT_GET_ok_)
    {
        for(auto &item : g_master_key_map)
        {
            ai_mot_pd_table_t *p_ai_mot_xd_table = NULL;
 
            p_ai_mot_xd_table = (ai_mot_pd_table_t *)(((char *)&ai_mot_cache) + item.second.table_offset);
            if(p_ai_mot_xd_table->count > 0)
            {
                Object cur_obj;
                for(unsigned int i = 0; i < p_ai_mot_xd_table->count; i++)
                {
                    cur_obj.xmin = p_ai_mot_xd_table->objects[i].x_min;
                    cur_obj.ymin = p_ai_mot_xd_table->objects[i].y_min;
                    cur_obj.xmax = p_ai_mot_xd_table->objects[i].x_max;
                    cur_obj.ymax = p_ai_mot_xd_table->objects[i].y_max;
                    cur_obj.prob = p_ai_mot_xd_table->objects[i].score;
                    cur_obj.label = item.second.label;
                    {
                        // 使用显式指针区间，避免对 std::begin/std::end 的依赖导致模板约束失败
                        auto *feat_begin = p_ai_mot_xd_table->objects[i].reid_feature;
                        auto *feat_end   = p_ai_mot_xd_table->objects[i].reid_feature + (sizeof(p_ai_mot_xd_table->objects[i].reid_feature)/sizeof(p_ai_mot_xd_table->objects[i].reid_feature[0]));
                        cur_obj.curr_feat.assign(feat_begin, feat_end);
                    }
                    m_detections.emplace_back(cur_obj);
                }
            }
        }

        if(mp_mo_tracker)
        {
            mp_mo_tracker->update(m_detections);
        }
    }

    // 更新当前PTS
    m_pts = ai_mot_cache.pd_ai_mot_list.pts;

    // MoTracker的结果获取比较特殊，可以直接根据类型获取对应的结果，所以这里就不先获取全部结果缓存了
    return ret == _AI_RESULT_GET_ok_ && m_detections.size() > 0;
}


bool MoTracker_::get_specific_type(int32_t type) const
{
    // 不在注册的类型里就不处理
    if(g_master_key_map.find(type) == g_master_key_map.end())
    {
        return false;
    }

    (mp_mo_tracker->*g_master_key_map.at(type).result_func)(&xd_result);
    return xd_result.count > 0;
}

bool MoTracker_::get_specific_rid(TargetInfo& out_info) const
{
    int rid = out_info.rid;
    for(unsigned int i = 0; i < xd_result.count; i++)
    {
        if(xd_result.objects[i].rid == rid)
        {
            out_info.x_pixel = (static_cast<float>(xd_result.objects[i].x_min) + static_cast<float>(xd_result.objects[i].x_max)) * 0.5f;
            out_info.y_pixel = (static_cast<float>(xd_result.objects[i].y_min) + static_cast<float>(xd_result.objects[i].y_max)) * 0.5f;
            out_info.visible = true;
            out_info.rid = rid;
            return true;
        }
    }
    return false;
}

bool MoTracker_::get_closest(TargetInfo& out_info) const
{
    if(xd_result.count <= 0)
    {
        return false;
    }

    // 线性扫描找到距离中心最近的目标，避免对不可移动/不可赋值类型进行排序移动
    unsigned int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < xd_result.count; ++i)
    {
        const MOTrackingST &o = xd_result.objects[i];
        float cx = (static_cast<float>(o.x_min) + static_cast<float>(o.x_max)) * 0.5f;
        float cy = (static_cast<float>(o.y_min) + static_cast<float>(o.y_max)) * 0.5f;
        float dx = cx - static_cast<float>(m_v_w_center);
        float dy = cy - static_cast<float>(m_v_h_center);
        float dist = dx*dx + dy*dy;
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    const MOTrackingST &obj = xd_result.objects[best_idx];
    out_info.x_pixel = (static_cast<float>(obj.x_min) + static_cast<float>(obj.x_max)) * 0.5f;
    out_info.y_pixel = (static_cast<float>(obj.y_min) + static_cast<float>(obj.y_max)) * 0.5f;
    out_info.visible = true;
    out_info.rid = obj.rid;
    return true;
}

// ======================= MediaTracker_ 实现 =======================
bool MediaTracker_::get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const
{
    static uint64_t last_pts = 0;
    if(m_last_pd_table->pts == m_last_vd_table->pts  && m_last_pd_table->pts == m_last_ad_table->pts && m_last_pd_table->pts > last_pts)
    {
        m_pts = m_last_pd_table->pts;
        last_pts = m_pts;
        return true;
    }
    return false;
}

bool MediaTracker_::get_specific_type(int32_t type) const
{
    switch (type) {
            case TRACK_PRIORITY_E::TRACK_TYPE_PERSON:
                m_selected_type = m_last_pd_table;
                break;
            case TRACK_PRIORITY_E::TRACK_TYPE_MOTOR_VEHICLE:
                m_selected_type = reinterpret_cast<ai_action_pd_table_t*>(m_last_vd_table);
                break;
            case TRACK_PRIORITY_E::TRACK_TYPE_ANIMAL:
                m_selected_type = reinterpret_cast<ai_action_pd_table_t*>(m_last_ad_table);
                break;
            default:
                m_selected_type = nullptr;
        }
    return m_selected_type != nullptr;
}

bool MediaTracker_::get_specific_rid(TargetInfo& out_info) const
{
    if(m_selected_type == nullptr)
    {
        return false;
    }
    for(unsigned int i = 0; i < m_selected_type->count; i++)
    {
        if(m_selected_type->objects[i].rid == out_info.rid)
        {
            out_info.x_pixel = (static_cast<float>(m_selected_type->objects[i].x_min) + static_cast<float>(m_selected_type->objects[i].x_max)) * 0.5f;
            out_info.y_pixel = (static_cast<float>(m_selected_type->objects[i].y_min) + static_cast<float>(m_selected_type->objects[i].y_max)) * 0.5f;
            out_info.pts = m_pts;
            out_info.visible = true;
            return true;
        }
    }
    return false;
}

bool MediaTracker_::get_closest(TargetInfo& out_info) const
{
    if(m_selected_type == nullptr || m_selected_type->count == 0)
    {
        return false;
    }

    // 线性扫描找到距离中心最近的目标，避免对不可移动/不可赋值类型进行排序移动
    unsigned int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < m_selected_type->count; ++i)
    {
        const auto &o = m_selected_type->objects[i];
        float cx = (static_cast<float>(o.x_min) + static_cast<float>(o.x_max)) * 0.5f;
        float cy = (static_cast<float>(o.y_min) + static_cast<float>(o.y_max)) * 0.5f;
        float dx = cx - static_cast<float>(m_v_w_center);
        float dy = cy - static_cast<float>(m_v_h_center);
        float dist = dx*dx + dy*dy;
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    const auto &obj = m_selected_type->objects[best_idx];
    out_info.x_pixel = (static_cast<float>(obj.x_min) + static_cast<float>(obj.x_max)) * 0.5f;
    out_info.y_pixel = (static_cast<float>(obj.y_min) + static_cast<float>(obj.y_max)) * 0.5f;
    out_info.visible = true;
    out_info.pts = m_pts;
    out_info.rid = obj.rid;

    return true;
}

// ======================= OvdTracker_ 实现（暂用媒体同逻辑） =======================
OvdTracker_::~OvdTracker_() {}

bool OvdTracker_::init(uint16_t v_w_center, uint16_t v_h_center)
{
    m_v_w_center = v_w_center;
    m_v_h_center = v_h_center;
    // 若后续需要使用 OVD 库，可在此处创建实例：mp_ovd_tracker = new Bc_OVDMOTracker(...)
    m_initialized = true;
    return true;
}

bool OvdTracker_::get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const
{
    (void)handle; (void)support_bitmap;
    ai_mot_cache_t cache{};
    int ret = md->get_ai_mot_all(chn, &cache);
    if(ret != _AI_RESULT_GET_ok_)
    {
        return false;
    }
    m_pts = cache.pd_ai_mot_list.pts;
    return true;
}

bool OvdTracker_::get_specific_type(int32_t type) const
{
    m_selected_type = type;

    // switch(type)
    // {
    // case TRACK_PRIORITY_E::TRACK_TYPE_PERSON:
    //     return m_last_cache.pd_ai_mot_list.count > 0;
    // case TRACK_PRIORITY_E::TRACK_TYPE_MOTOR_VEHICLE:
    //     return m_last_cache.vd_ai_mot_list.count > 0;
    // case TRACK_PRIORITY_E::TRACK_TYPE_ANIMAL:
    //     return m_last_cache.ad_ai_mot_list.count > 0;
    // default:
    //     return false;
    // }
    return false;
}

bool OvdTracker_::get_specific_rid(TargetInfo& out_info) const
{
    // 媒体基础表可能不包含稳定的 rid，返回不支持
    (void)out_info;
    return false;
}

bool OvdTracker_::get_closest(TargetInfo& out_info) const
{
    auto compute_dist = [this](int x_min, int x_max, int y_min, int y_max) {
        float cx = (static_cast<float>(x_min) + static_cast<float>(x_max)) * 0.5f;
        float cy = (static_cast<float>(y_min) + static_cast<float>(y_max)) * 0.5f;
    float dx = cx - static_cast<float>(m_v_w_center);
    float dy = cy - static_cast<float>(m_v_h_center);
        return dx*dx + dy*dy;
    };

    float best = std::numeric_limits<float>::max();
    bool found = false;
    int best_rid = -1; float best_x=0.f, best_y=0.f;

    switch(m_selected_type)
    {
    case TRACK_PRIORITY_E::TRACK_TYPE_PERSON:
        for(unsigned int i=0; i<m_last_cache.pd_ai_mot_list.count; ++i)
        {
            const auto &obj = m_last_cache.pd_ai_mot_list.objects[i];
            float d = compute_dist(obj.x_min, obj.x_max, obj.y_min, obj.y_max);
            if(d < best)
            {
                best = d;
                best_rid = -1;
                best_x = (static_cast<float>(obj.x_min) + static_cast<float>(obj.x_max)) * 0.5f;
                best_y = (static_cast<float>(obj.y_min) + static_cast<float>(obj.y_max)) * 0.5f;
                found = true;
            }
        }
        break;
    case TRACK_PRIORITY_E::TRACK_TYPE_MOTOR_VEHICLE:
        for(unsigned int i=0; i<m_last_cache.vd_ai_mot_list.count; ++i)
        {
            const auto &obj = m_last_cache.vd_ai_mot_list.objects[i];
            float d = compute_dist(obj.x_min, obj.x_max, obj.y_min, obj.y_max);
            if(d < best)
            {
                best = d;
                best_rid = -1;
                best_x = (static_cast<float>(obj.x_min) + static_cast<float>(obj.x_max)) * 0.5f;
                best_y = (static_cast<float>(obj.y_min) + static_cast<float>(obj.y_max)) * 0.5f;
                found = true;
            }
        }
        break;
    case TRACK_PRIORITY_E::TRACK_TYPE_ANIMAL:
        for(unsigned int i=0; i<m_last_cache.ad_ai_mot_list.count; ++i)
        {
            const auto &obj = m_last_cache.ad_ai_mot_list.objects[i];
            float d = compute_dist(obj.x_min, obj.x_max, obj.y_min, obj.y_max);
            if(d < best)
            {
                best = d;
                best_rid = -1;
                best_x = (static_cast<float>(obj.x_min) + static_cast<float>(obj.x_max)) * 0.5f;
                best_y = (static_cast<float>(obj.y_min) + static_cast<float>(obj.y_max)) * 0.5f;
                found = true;
            }
        }
        break;
    default:
        break;
    }

    if(!found) return false;
    out_info.x_pixel = best_x;
    out_info.y_pixel = best_y;
    out_info.visible = true;
    out_info.rid = best_rid;
    return true;
}
