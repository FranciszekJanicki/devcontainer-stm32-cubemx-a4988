#include "a4988.hpp"
#include "clock_config.h"
#include "gpio.h"
#include "gpio.hpp"
#include "pwm_device.hpp"
#include "tim.h"
#include "usart.h"
#include <utility>

int main()
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();

    using namespace Utility;

    auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_4, 1U, 3.3F};

    auto a4988 = A4988::A4988{std::move(pwm_device),
                              GPIO::PA0,
                              GPIO::PA1,
                              GPIO::PA4,
                              GPIO::PA5,
                              GPIO::PA6,
                              GPIO::PA7,
                              GPIO::PA8};

    a4988.set_half_microstep();
    a4988.set_forward_direction();

    while (true) {
        a4988.set_backward_direction();
        HAL_Delay(50UL);
        a4988.set_forward_direction();
        HAL_Delay(50UL);
    }

    return 0;
}
