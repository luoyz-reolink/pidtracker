#include "pid_control_lib.h"

#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_TRACKER_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_TRACKER_H_

#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include "msg_cfg_md_def.h"
#include "md_ext.h"

#include "Bc_MOTracker.h"
#include "Bc_OVDMOTracker.h"

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
};

template <>
struct TrackerBinding<OVDObject, Bc_OVDTracker>
{
    struct result_type
    {
        OVDTrackingResult tracking_result;
        OVDTrackingResult predicted_result;
    };

    static void process(Bc_OVDTracker &impl,
                        const std::vector<OVDObject> &detections,
                        bool compensation,
                        int delta_x,
                        int delta_y)
    {
        impl.Process(detections, compensation, delta_x, delta_y);
    }

    static void reset(Bc_OVDTracker &impl)
    {
        impl.Reset();
    }

    static void collect_results(Bc_OVDTracker &impl, result_type &out)
    {
        impl.GetResult(&out.tracking_result, &out.predicted_result);
    }
};

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
