#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_

#include <cstdint>
#include <limits>

enum TRACKER_TIMER_E : std::uint8_t
{
    TRACKER_TIMER_TARGET_DISAPPEAR = 0,
    TRACKER_TIMER_TARGET_STATIC = 1,
    TRACKER_TIMER_DELAY = 2,
    TRACKER_TIMER_TARGET_SWITCH = 3,
    TRACKER_TIMER_BUTT
};

struct delay_timer_t
{
    std::uint64_t target_time_microseconds;
    std::uint64_t previous_time_microseconds;
    bool active;
};

class timer_ctrl
{
public:
    enum class time_unit : std::uint32_t
    {
        microseconds = 1,
        milliseconds = 1000,
        seconds = 1000 * 1000,
        minutes = 60 * 1000 * 1000
    };

    timer_ctrl() = default;

    void reset()
    {
        for (auto& timer : m_delay_timers)
        {
            timer.target_time_microseconds = 0;
            timer.previous_time_microseconds = 0;
            timer.active = false;
        }
    }

    void update(std::uint64_t value, time_unit unit = time_unit::microseconds)
    {
        std::uint64_t converted = to_micro(value, unit);
        m_current = converted;
    }

    void update_timer(uint8_t timer_type)
    {
        m_delay_timers[timer_type].previous_time_microseconds = m_current;
    }

    std::uint64_t current(time_unit unit = time_unit::microseconds) const
    {
        return from_micro(m_current, unit);
    }

    std::uint64_t interval(uint8_t timer_type = TRACKER_TIMER_E::TRACKER_TIMER_DELAY, time_unit unit = time_unit::microseconds) const
    {
        if(m_delay_timers[timer_type].previous_time_microseconds == 0)
        {
            return 0;
        }
        std::uint64_t diff = m_current >= m_delay_timers[timer_type].previous_time_microseconds ? (m_current - m_delay_timers[timer_type].previous_time_microseconds) : 0;
        return from_micro(diff, unit);
    }

    void set_delay(std::uint64_t duration, time_unit unit = time_unit::microseconds, uint8_t timer_type = TRACKER_TIMER_E::TRACKER_TIMER_TARGET_DISAPPEAR)
    {
        std::uint64_t converted = to_micro(duration, unit);
        m_delay_timers[timer_type].target_time_microseconds = converted;
        m_delay_timers[timer_type].previous_time_microseconds = m_current;
        m_delay_timers[timer_type].active = true;
    }

    bool delay_active(uint8_t timer_type) const
    {
        return m_delay_timers[timer_type].active;
    }

    bool delay_reached(uint8_t timer_type) const
    {
        if (!delay_active(timer_type))
        {
            return true;
        }
        return interval(timer_type) >= m_delay_timers[timer_type].target_time_microseconds;
    }

    std::uint64_t delay_remaining(uint8_t timer_type, time_unit unit = time_unit::microseconds) const
    {
        if (!delay_active(timer_type))
        {
            return 0;
        }

        if (interval(timer_type) >= m_delay_timers[timer_type].target_time_microseconds)
        {
            return 0;
        }

        std::uint64_t diff = m_delay_timers[timer_type].target_time_microseconds - interval(timer_type);
        return from_micro(diff, unit);
    }

    void cancel_delay(uint8_t timer_type)
    {
        m_delay_timers[timer_type].active = false;
    }

private:
    static std::uint64_t unit_factor(time_unit unit)
    {
        switch (unit)
        {
        case time_unit::microseconds:
            return 1ULL;
        case time_unit::milliseconds:
            return 1000ULL;
        case time_unit::seconds:
            return 1000ULL * 1000ULL;
        case time_unit::minutes:
            return 60ULL * 1000ULL * 1000ULL;
        }
        return 1ULL;
    }

    static std::uint64_t to_micro(std::uint64_t value, time_unit unit)
    {
        const std::uint64_t factor = unit_factor(unit);
        if (factor != 0 && value > std::numeric_limits<std::uint64_t>::max() / factor)
        {
            return std::numeric_limits<std::uint64_t>::max();
        }
        return value * factor;
    }

    static std::uint64_t from_micro(std::uint64_t value, time_unit unit)
    {
        const std::uint64_t factor = unit_factor(unit);
        if (factor == 0)
        {
            return value;
        }
        return value / factor;
    }

    static bool will_overflow(std::uint64_t base, std::uint64_t increment)
    {
        return base > std::numeric_limits<std::uint64_t>::max() - increment;
    }


private:
    std::uint64_t m_current{0};
    delay_timer_t m_delay_timers[TRACKER_TIMER_E::TRACKER_TIMER_BUTT]{0};
};

#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_

