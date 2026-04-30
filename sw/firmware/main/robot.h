#pragma once

#include <memory>

#include "./driver/rpLidar.h"

#include "./driver/claws.h"

#include <dcmotor.h>

class Robot {

    std::unique_ptr<DCMotor> _motorLeft;
    std::unique_ptr<DCMotor> _motorRight;

    Claws _claws;
    RpLidar _lidar;

public:

    DCMotor& motorLeft() {
        return *_motorLeft;
    }

    DCMotor& motorRight() {
        return *_motorRight;
    }

    Claws& claws() {
        return _claws;
    }

    RpLidar& lidar() {
        return _lidar;
    }

    Robot(
        std::unique_ptr<DCMotor> motorLeft,
        std::unique_ptr<DCMotor> motorRight,
        Claws claws,
        RpLidar lidar
    ):
        _motorLeft(std::move(motorLeft)),
        _motorRight(std::move(motorRight)),
        _claws(std::move(claws)),
        _lidar(std::move(lidar))
    {}

    Robot(Robot const&) = delete;
    Robot(Robot&& other) = delete;

    void start() {
        motorLeft().startTicker();
        motorRight().startTicker();

        motorLeft().stop(false);
        motorRight().stop(false);

        _lidar.start();
    }

    void stop() {
        motorLeft().stop(false);
        motorRight().stop(false);

        motorLeft().stopTicker();
        motorRight().stopTicker();

        _lidar.stop();
    }
};
