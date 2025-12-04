#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_TRACKER_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_TRACKER_H_

#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>

#include "msg_cfg_md_def.h"
#include "md_ext.h"
#include "pid_control_lib.h"
#include "media_common.h"

#include "Bc_MOTracker.h"
#include "Bc_OVDMOTracker.h"


// 前置声明以避免类型未识别问题（若头文件未暴露具体类名）
class Bc_OVDMOTracker;


#ifndef BIT
#define BIT(x) (1U << (x))
#endif
#ifndef BIT64
#define BIT64(x) (1ULL << (x))
#endif

struct TargetInfo {
    float x_pixel;    ///< 目标中心点 X
    float y_pixel;    ///< 目标中心点 Y
    bool  visible;    ///< 是否可见
    uint64_t pts;     ///< 时间戳 (us/ms 视来源而定)
    int32_t rid;      ///< 目标ID
};

// 移除原 tracker_ctx_t，仅保留画面中心坐标作为独立成员变量

class BaseTrackerUnified
{

public:
    BaseTrackerUnified() = default;
    virtual ~BaseTrackerUnified() = default;

    // 初始化：仅设置中心点坐标（原结构体已移除）
    virtual bool init(uint16_t v_w_center, uint16_t v_h_center)
    {
        m_v_w_center = v_w_center;
        m_v_h_center = v_h_center;
        return m_initialized; // 由派生类实际置位
    }
    // 重置(主要针对motracker这种需要保存目标状态的)
    virtual bool reset() {};

    void set_map(std::unordered_map<int32_t, int32_t> classid_type_map) {
        m_classid_type_map = std::move(classid_type_map);
    }

    // 获取当前帧所有目标数据（不同派生类实现各自来源）
    virtual bool get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const { (void)md; (void)chn; (void)handle; (void)support_bitmap; return false; };
    // 获取某种类型的目标数据
    virtual bool get_specific_type(int32_t type) const {(void)type; return false;};
    // 获取某个目标ID的目标数据
    virtual bool get_specific_rid(TargetInfo& out_info) const { (void)out_info; return false; };
    // 获取距离中心最近的目标数据
    virtual bool get_closest(TargetInfo& out_info) const {(void)out_info; return false;};
    uint64_t pts() const { return m_pts; }

protected:
    // 画面中心（原 tracker_ctx_t 中的两个字段）
    uint16_t m_v_w_center{0};
    uint16_t m_v_h_center{0};
    mutable uint64_t m_pts = 0;
    bool m_initialized = false;
    // 类别ID到类型的映射(主要是yoloworld在筛选输送给tracker的数据时会用到，因为yoloworld的重复数据实在是太多了，一个戴眼镜、带帽子、穿衣服的人能被识别成好几个不同的类别ID，所以尽可能筛选一下防止他超过tracker的处理能力也就是50个目标)
    std::unordered_map<int32_t, int32_t> m_classid_type_map;
    // 当前锁定的目标（用于优先维持 RID）
    TargetInfo m_current_target{0.f, 0.f, false, 0ULL, -1};
};

class MoTracker_ : public BaseTrackerUnified
{
public:
    MoTracker_() : BaseTrackerUnified(), mp_mo_tracker(nullptr), m_detections() {}
    ~MoTracker_() override;

    // =======================抽象函数实现========================
    // 初始化（只接受中心点）
    bool init(uint16_t v_w_center, uint16_t v_h_center) override
    {
        m_v_w_center = v_w_center;
        m_v_h_center = v_h_center;
        if(!mp_mo_tracker)
        {
            mp_mo_tracker = new Bc_MOTracker(30, m_v_w_center * 2, m_v_h_center * 2);
        }
        m_initialized = (mp_mo_tracker != nullptr);
        return m_initialized;
    }
    bool reset() override { mp_mo_tracker->Bc_MOTrcaker_Reset(); return true; }

    // 获取当前帧所有目标数据（不同派生类实现各自来源）
    bool get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const override;

    // 获取某种类型的目标数据
    bool get_specific_type(int32_t type) const override;

    // 获取某个目标ID的目标数据
    bool get_specific_rid(TargetInfo& out_info) const override;

    // 获取距离中心最近的目标数据
    bool get_closest(TargetInfo& out_info) const override;

    //=======================工具函数========================
    bool pre_process(c_media_md* media_md, int chn, int handle);
    bool process(bool compensation, uint16_t compensation_x, uint16_t compensation_y);



private:
    Bc_MOTracker *mp_mo_tracker;
    std::vector<Object> m_detections;
    MOTrackingResult xd_result;
};

// media比较特殊，在enc里面已经获取到了ai结果，所以这里直接用媒体的结果
class MediaTracker_ : public BaseTrackerUnified
{
public:
    MediaTracker_() : BaseTrackerUnified() {}
    ~MediaTracker_() override = default;

    bool init(uint16_t v_w_center, uint16_t v_h_center) override {
        m_v_w_center = v_w_center;
        m_v_h_center = v_h_center;
        m_initialized = true;
        return true;
    }

    // 从媒体缓存拉取检测数据
    bool get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const override;

    // 选择某种类型（并缓存以供后续 rid/closest 查询）
    bool get_specific_type(int32_t type) const override;

    // 查询指定 rid 的像素位置
    bool get_specific_rid(TargetInfo& out_info) const override;

    // 获取距离中心最近的目标
    bool get_closest(TargetInfo& out_info) const override;

    void get_pdata_by_type(int32_t type, void* pdata) const {
        switch(type)
        {
            case _AI_TYPE_pd_:
                m_last_pd_table = reinterpret_cast<ai_action_pd_table_t*>(pdata);
                break;
            case _AI_TYPE_vd_:
                m_last_vd_table = reinterpret_cast<ai_action_vd_table_t*>(pdata);
                break;
            case _AI_TYPE_ad_:
                m_last_ad_table = reinterpret_cast<ai_action_ad_table_t*>(pdata);
                break;
            default:
                break;
        }
    }


private:
    // 最近一次的 people AI ACTION 缓存
    mutable ai_action_pd_table_t* m_last_pd_table{nullptr};
    // 最近一次的 vehicle AI ACTION 缓存
    mutable ai_action_vd_table_t* m_last_vd_table{nullptr};
    // 最近一次的 animal AI ACTION 缓存
    mutable ai_action_ad_table_t* m_last_ad_table{nullptr};
    // 最近一次选择的类型
    mutable ai_action_pd_table_t* m_selected_type{nullptr};
};

class OvdTracker_ : public BaseTrackerUnified
{
public:
    OvdTracker_() : BaseTrackerUnified() {}
    ~OvdTracker_() override;

    bool init(uint16_t v_w_center, uint16_t v_h_center) override;

    bool get_tracker_result(c_media_md* md, uint32_t chn, int handle, uint64_t support_bitmap) const override;
    bool get_specific_type(int32_t type) const override;
    bool get_specific_rid(TargetInfo& out_info) const override;
    bool get_closest(TargetInfo& out_info) const override;

private:
    // OVD 跟踪器实例（具体类在 Bc_OVDMOTracker.h 中定义）
    Bc_OVDMOTracker* mp_ovd_tracker{nullptr};
    // 最近一次的 AI MOT 缓存（先与 MediaTracker_ 保持一致，后续接入 OVD）
    mutable ai_mot_cache_t m_last_cache{};
    // 最近一次选择的类型
    mutable int32_t m_selected_type{-1};
};


// ================= 已废弃 ===================================
// ================= 统一设计抽象（模板绑定等） =================
// 保留现有模板绑定与实现（tracker_detail / tracker），用于其他模块复用


namespace tracker_detail
{

template <typename T, typename U>
struct always_false : std::false_type
{
};

template <typename TObject, typename TMotracker>
struct TrackerBinding
{
    static_assert(always_false<TObject, TMotracker>::value,
                  "Unsupported tracker binding. Please provide a TrackerBinding specialization for this pair.");
};

template <>
struct TrackerBinding<nullptr_t, nullptr_t>
{
    struct result_type
    {
        ai_action_ad_table_t ad_result;
        ai_action_pd_table_t pd_result;
        ai_action_vd_table_t vd_result;
    };

    static void process(nullptr_t &impl,
                        const std::vector<nullptr_t> &detections,
                        bool compensation,
                        int delta_x,
                        int delta_y)
    {
        (void)impl; (void)detections; (void)compensation; (void)delta_x; (void)delta_y; // No operation
    }

    static void reset(nullptr_t &impl)
    {
        (void)impl; // No operation
    }

    static void collect_results(nullptr_t &impl, result_type &out)
    {
        (void)impl; (void)out; // No operation
    }
    
};

template <>
struct TrackerBinding<Object, Bc_MOTracker>
{
    struct result_type
    {
        MOTrackingResult ad_result;
        MOTrackingResult pd_result;
        MOTrackingResult vd_result;
    };

    static void process(Bc_MOTracker &impl,
                        const std::vector<Object> &detections,
                        bool compensation,
                        int delta_x,
                        int delta_y)
    {
        if (compensation)
        {
            impl.motion_compensation(delta_x, delta_y);
        }
        impl.update(detections);
    }

    static void reset(Bc_MOTracker &impl)
    {
        impl.Bc_MOTrcaker_Reset();
    }

    static void collect_results(Bc_MOTracker &impl, result_type &out)
    {
        impl.Bc_Get_AD_Result(&out.ad_result);
        impl.Bc_Get_PD_Result(&out.pd_result);
        impl.Bc_Get_VD_Result(&out.vd_result);
    }

    static bool get_specific_type(result_type &out, result_type &specific_result, int32_t type)
    {
        switch(type)
        {
            case TRACK_TYPE_PERSON:
                specific_result.pd_result = out.pd_result;
                break;
            case TRACK_TYPE_MOTOR_VEHICLE:
                specific_result.vd_result = out.vd_result;
                break;
            case TRACK_TYPE_ANIMAL:
                specific_result.ad_result = out.ad_result;
                break;
            default:
                break;  
        }
        return specific_result.pd_result.count > 0 ||
               specific_result.vd_result.count > 0 ||
               specific_result.ad_result.count > 0;
    }

    static bool get_specific_rid(const result_type &out, TargetInfo &target_info)
    {
        MOTrackingResult *result = nullptr;
        if(out.pd_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.pd_result);
        }
        else if(out.vd_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.vd_result);
        }
        else if(out.ad_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.ad_result);
        }
        else 
        {
            return false;
        }
        if(result == nullptr || result->count <= 0)
        {
            return false;
        }

        for(unsigned int i = 0; i < result->count; i++)
        {
            MOTrackingST obj = result->objects[i];
            if(obj.rid == target_info.rid)
            {
                target_info.x_pixel = static_cast<float>( (obj.x_min + obj.x_max) / 2.0f );
                target_info.y_pixel = static_cast<float>( (obj.y_min + obj.y_max) / 2.0f );
                target_info.visible = true;
                return true;
            }
        }
        return false;
    }

    static bool get_closest(result_type &out, TargetInfo &target_info, uint16_t v_w_center, uint16_t v_h_center)
    {
        MOTrackingResult *result = nullptr;
        if(out.pd_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.pd_result);
        }
        else if(out.vd_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.vd_result);
        }
        else if(out.ad_result.count > 0)
        {
            result = const_cast<MOTrackingResult*>(&out.ad_result);
        }
        else 
        {
            return false;
        }
        if(result == nullptr || result->count <= 0)
        {
            return false;
        }

        // 根据物体距离中心点的距离排序
        stable_sort(result->objects, result->objects + result->count, [v_w_center, v_h_center](const MOTrackingST &a, const MOTrackingST &b) {
            float a_dist = ( ( (a.x_min + a.x_max) / 2.0f - v_w_center ) * ( (a.x_min + a.x_max) / 2.0f - v_w_center )
                           + ( (a.y_min + a.y_max) / 2.0f - v_h_center ) * ( (a.y_min + a.y_max) / 2.0f - v_h_center ) );
            float b_dist = ( ( (b.x_min + b.x_max) / 2.0f - v_w_center ) * ( (b.x_min + b.x_max) / 2.0f - v_w_center )
                           + ( (b.y_min + b.y_max) / 2.0f - v_h_center ) * ( (b.y_min + b.y_max) / 2.0f - v_h_center ) );
            return a_dist < b_dist;
        });

        MOTrackingST obj = result->objects[0];
        target_info.x_pixel = static_cast<float>( (obj.x_min + obj.x_max) / 2.0f );
        target_info.y_pixel = static_cast<float>( (obj.y_min + obj.y_max) / 2.0f );
        target_info.visible = true;
        target_info.rid = obj.rid;
        return true;
    }
};

// OVD 专门化暂未实现，原占位已移除。

} // namespace tracker_detail
template <typename TObject, typename TMotracker>
class tracker
{
    using binding = tracker_detail::TrackerBinding<TObject, TMotracker>;

public:
    using object_type = TObject;
    using motracker_type = TMotracker;
    using result_type = typename binding::result_type;

    template <typename... Args>
    explicit tracker(Args &&...args)
        : m_tracker_impl(std::forward<Args>(args)...)
    {
    }

    tracker(const tracker &) = delete;
    tracker &operator=(const tracker &) = delete;

    tracker(tracker &&) = default;
    tracker &operator=(tracker &&) = default;

    ~tracker() = default;

    void process(const std::vector<TObject> &detections,
                 bool compensation = false,
                 int delta_x = 0,
                 int delta_y = 0)
    {
        binding::process(m_tracker_impl, detections, compensation, delta_x, delta_y);
    }

    void reset()
    {
        binding::reset(m_tracker_impl);
    }

    result_type collect_results()
    {
        result_type result{};
        binding::collect_results(m_tracker_impl, result);
        return result;
    }

    void collect_results(result_type &out)
    {
        binding::collect_results(m_tracker_impl, out);
    }

    bool get_specific_type(result_type &out, result_type &specific_result, int32_t type)
    {
        return binding::get_specific_type(out, specific_result, type);
    }

    bool get_specific_rid(const result_type &out, TargetInfo &target_info)
    {
        return binding::get_specific_rid(out, target_info);
    }

    bool get_closest(result_type &out, TargetInfo &target_info, uint16_t v_w_center, uint16_t v_h_center)
    {
        return binding::get_closest(out, target_info, v_w_center, v_h_center);
    }

    TMotracker &native_tracker()
    {
        return m_tracker_impl;
    }

    const TMotracker &native_tracker() const
    {
        return m_tracker_impl;
    }

private:
    TMotracker m_tracker_impl;
};

#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_TRACKER_H_
