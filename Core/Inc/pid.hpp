#ifndef PID_HPP
#define PID_HPP

#include <algorithm>
#include <concepts>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace Utility {

    template <typename T>
    struct PID {
        [[nodiscard]] T operator()(T const error, T const sampling_time) noexcept
        {
            return this->get_control(error, sampling_time);
        }

        [[nodiscard]] T get_control(T const error, T const sampling_time) noexcept
        {
            auto control{this->get_proportion(error) + this->get_integral(error, sampling_time) +
                         this->get_derivative(error, sampling_time)};
            auto saturated_control{std::clamp(control, -this->saturation, this->saturation)};
            this->prev_sat_error = std::exchange(this->sat_error, control - saturated_control);
            this->prev_error = error;
            return saturated_control;
        }

        [[nodiscard]] T get_proportion(T const error) noexcept
        {
            return this->proportion_gain * error;
        }

        [[nodiscard]] T get_derivative(T const error, T const sampling_time) noexcept
        {
            this->error_derivative = Utility::differentiate(error,
                                                            this->prev_error,
                                                            sampling_time,
                                                            this->error_derivative,
                                                            this->time_constant);
            return this->derivative_gain * this->error_derivative;
        }

        [[nodiscard]] T get_integral(T const error, T const sampling_time) noexcept
        {
            this->error_integral += Utility::integrate(error, this->prev_error, sampling_time);
            this->sat_error_integral += Utility::integrate(this->sat_error, this->prev_sat_error, sampling_time);
            return this->integral_gain * this->error_integral - this->control_gain * this->sat_error_integral;
        }

        // basic PID parameters
        T proportion_gain{};
        T integral_gain{};
        T derivative_gain{};
        T time_constant{};

        // anti windup parameters
        T control_gain{};
        T saturation{};

        // basic PID internal state
        T prev_error{0.0F};
        T error_integral{0.0F};
        T error_derivative{0.0F};

        // anti windup internal state
        T sat_error{0.0F};
        T prev_sat_error{0.0F};
        T sat_error_integral{0.0F};
    };

}; // namespace Utility

#endif // PID_HPP