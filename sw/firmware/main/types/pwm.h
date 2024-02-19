#pragma once

#include <concepts>

template<typename Pwm>
concept isPwm = requires(Pwm pin) {
    pin.setDuty(0);
    pin.setDuty(std::decay_t<Pwm>::MaxDuty);
};
