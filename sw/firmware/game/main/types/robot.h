#pragma once

#include "./motor.h"
#include "./lidar.h"
#include "./pincerCatcher.h"


template<typename Robot>
concept isRobot = requires(Robot robot) {
    robot.start();
    robot.stop();

    { robot.motorLeft() } -> isMotor;
    { robot.motorRight() } -> isMotor;

    { robot.pincerCatcher() } -> isPincerCatcher;
    { robot.lidar() } -> isLidar;
};
