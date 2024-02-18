#pragma once

#include "types/pwm.h"


namespace driver {


template<isPWM PWM>
class PwmMotor {
    PWM pin;
public:
    static constexpr int MaxPower = PWM::MaxDuty;

    PwmMotor(PWM pin):
        pin(pin)
    {
        pin.setDuty(0);
    }

    void setPower(int power) {
        pin.setDuty(power);
    }
};


} // namespace driver
