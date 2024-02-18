#pragma once

#include <concepts>


template<typename Motor>
concept isMotor = requires(Motor motor) {
    motor.setPower(0);
    motor.setPower(Motor::MaxPower);
};
