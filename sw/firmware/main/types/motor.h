#pragma once

#include <concepts>

#include "util/concepts.h"


template<typename Motor>
concept isMotor = requires(Motor motor) {
    motor.setPower(0);
};

// TODO: fix ratio check
//&& isRatio<typename Motor::PowerRatio>;
