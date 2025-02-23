#include "step_motor_driver.hpp"
#include "utility.hpp"
#include <cmath>
#include <numbers>
#include <numeric>

using Direction = A4988::StepMotorDriver::Direction;
using Microstep = A4988::StepMotorDriver::Microstep;

namespace A4988 {

    void StepMotorDriver::operator()(float const input_speed, float const sampling_time) noexcept
    {
        this->set_speed(this->get_control_speed(this->get_error_speed(input_speed, sampling_time), sampling_time));
    }

    std::int32_t StepMotorDriver::degrees_to_steps(float const degrees,
                                                   float const degrees_per_step,
                                                   Microstep const microstep) noexcept
    {
        return static_cast<std::int32_t>(degrees) *
               static_cast<std::int32_t>(360.0F / degrees_per_step * A4988::microstep_to_fraction(microstep));
    }

    float StepMotorDriver::steps_to_degrees(std::int32_t const steps,
                                            float const degrees_per_step,
                                            Microstep const microstep) noexcept
    {
        return static_cast<float>(steps) /
               static_cast<std::int32_t>(360.0F / degrees_per_step * A4988::microstep_to_fraction(microstep));
    }

    std::uint32_t StepMotorDriver::get_frequency() const noexcept
    {
        return this->degrees_per_second / this->degrees_per_step;
    }

    void StepMotorDriver::set_speed(float const control_speed) noexcept
    {
        this->set_direction(control_speed);
        this->set_frequency(control_speed);
        this->set_microstep(control_speed);
    }

    void StepMotorDriver::set_microstep(float const control_speed) noexcept
    {
        this->a4988.set_microstep(Microstep::FULL);
    }

    void StepMotorDriver::set_direction(float const control_speed) noexcept
    {
        this->a4988.set_direction(this->speed_to_direction(control_speed));
    }

    void StepMotorDriver::set_frequency(float const control_speed) noexcept
    {
        this->a4988.set_frequency(this->speed_to_frequency(std::abs(control_speed)));
    }

    float StepMotorDriver::get_measured_speed(float const sampling_time) noexcept
    {
        return 0.0F;
    }

    float StepMotorDriver::get_control_speed(float const error_speed, float const sampling_time) noexcept
    {
        return this->regulator(error_speed, sampling_time);
    }

    float StepMotorDriver::get_error_speed(float const input_speed, float const sampling_time) noexcept
    {
        return input_speed - this->get_measured_speed(sampling_time);
    }

}; // namespace A4988