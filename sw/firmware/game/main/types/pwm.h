#pragma once

#include <concepts>

#include "../util/concepts.h"

template<typename Pwm>
concept isPwm = requires(Pwm pin) {
    pin.setDuty(0);
}
&& isRatio<typename Pwm::DutyRatio>;
