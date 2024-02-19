#pragma once

#include <concepts>


template<typename Motor>
concept isMotor = requires(Motor motor) {
    motor.setPower(0);
    motor.setPower(std::decay_t<Motor>::MaxPower);
    motor.setPower(-std::decay_t<Motor>::MaxPower);
};
