#pragma once

#include <concepts>

template<typename PWM>
concept isPWM = requires(PWM pin) {
    pin.setDuty(0);
    pin.setDuty(PWM::MaxDuty);
};
