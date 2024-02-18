#pragma once

#include "motor.h"
#include "lidar.h"
#include "pincerCatcher.h"
#include "tickEncoder.h"


template<typename Robot>
concept isRobot = requires(Robot robot) {
    robot.stop();

    { robot.motorLeft } -> isMotor;
    { robot.motorRight } -> isMotor;

    { robot.encoderLeft } -> isTickEncoder;
    { robot.encoderRight } -> isTickEncoder;

    { robot.pincerCatcher } -> isPincerCatcher;
    { robot.lidar } -> isLidar;
};
