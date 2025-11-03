#include "pid_tracker.h"

class tracker
{
public:

    int model_data_to_objects(int chn);
    int objects_to_track_data(int chn);
    int get_target(int chn);
    int track_process();

    PIDTracker tracker;
    c_media_md *mp_media_md;
    uint8_t track_stat;
    // 再加上一些追踪类型、优先级之类的
    // 这些先不搞，先实现最基础的追踪

    tracker() : mp_media_md(nullptr) {}

    
}