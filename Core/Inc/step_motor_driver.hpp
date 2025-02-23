#ifndef STEP_MOTOR_DRIVER_HPP
#define STEP_MOTOR_DRIVER_HPP

#include "a4988.hpp"
#include "pid.hpp"
#include <cstdint>

namespace A4988 {

    struct StepMotorDriver {
    public:
        using Direction = A4988::Direction;
        using Microstep = A4988::Microstep;
        using PID = Utility::PID<float>;

        void operator()(float const input_speed, float const sampling_time) noexcept;

        A4988 a4988{};
        PID regulator{};

        float degrees_per_step{};
        float degrees_per_second{};

        Direction (*speed_to_direction)(float const);
        std::uint32_t (*speed_to_frequency)(float const);

    private:
        static std::int32_t
        degrees_to_steps(float const degrees, float const degrees_per_step, Microstep const microstep) noexcept;
        static float
        steps_to_degrees(std::int32_t const steps, float const degrees_per_step, Microstep const microstep) noexcept;

        std::uint32_t get_frequency() const noexcept;

        void set_speed(float const control_speed) noexcept;
        void set_frequency(float const control_speed) noexcept;
        void set_microstep(float const control_speed) noexcept;
        void set_direction(float const control_speed) noexcept;

        float get_measured_speed(float const sampling_time) noexcept;
        float get_control_speed(float const error_speed, float const sampling_time) noexcept;
        float get_error_speed(float const input_speed, float const sampling_time) noexcept;
    };

}; // namespace A4988

#endif // STEP_MOTOR_DRIVER_HPP
