#pragma once

#include <concepts>

#include "../util/concepts.h"


template<typename Motor>
concept isMotor = requires(Motor motor) {
    motor.setSpeed(0);  // ticks per second
    // motor.setPower(0);  // -100 to 100
    motor.moveInfinite();
    motor.moveTime(0);  // milliseconds duration
    motor.moveDistance(0);  // ticks distance
    motor.stop(true);  // brake

    motor.onTarget([](){});  // callback

    { motor.getPosition() } -> std::same_as<int64_t>;  // ticks
};

// TODO: fix ratio check
//&& isRatio<typename Motor::PowerRatio>;
