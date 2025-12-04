#ifndef PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_
#define PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_

#include <cstdint>
#include <limits>

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
        m_current = 0;
        m_previous = 0;
        m_delay_target = 0;
        m_delay_active = false;
        cancel_delay();
    }

    void update(std::uint64_t value, time_unit unit = time_unit::microseconds, bool update_previous = true)
    {
        std::uint64_t converted = to_micro(value, unit);
        m_current = converted;
        if (update_previous || m_previous == 0)
        {
            m_previous = m_current;
        }
    }

    std::uint64_t current(time_unit unit = time_unit::microseconds) const
    {
        return from_micro(m_current, unit);
    }

    std::uint64_t interval(time_unit unit = time_unit::microseconds) const
    {
        std::uint64_t diff = m_current >= m_previous ? (m_current - m_previous) : 0;
        return from_micro(diff, unit);
    }

    void set_delay(std::uint64_t duration, time_unit unit = time_unit::microseconds)
    {
        std::uint64_t converted = to_micro(duration, unit);
        m_delay_target = converted;
        m_previous = m_current;
        m_delay_active = true;
    }

    bool delay_active() const
    {
        return m_delay_active;
    }

    bool delay_reached() const
    {
        if (!m_delay_active)
        {
            return true;
        }
        return interval() >= m_delay_target;
    }

    std::uint64_t delay_remaining(time_unit unit = time_unit::microseconds) const
    {
        if (!m_delay_active)
        {
            return 0;
        }

        if (interval() >= m_delay_target)
        {
            return 0;
        }

        std::uint64_t diff = m_delay_target - interval();
        return from_micro(diff, unit);
    }

    void cancel_delay()
    {
        m_delay_active = false;
        m_delay_target = 0;
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
    std::uint64_t m_previous{0};
    std::uint64_t m_delay_target{0};
    bool m_delay_active{false};
};

#endif // PRODUCT_MODULES_ENC_SRC_PID_TRACK_PID_TIMER_H_

