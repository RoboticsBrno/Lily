#pragma once

#include <tuple>

#include "./driver/rpLidar.h"

#include "./driver/esp32/gpio.h"
#include "./driver/esp32/pwm.h"
#include "./driver/esp32/pincerCatcher.h"
#include "./driver/esp32/serial.h"

#include <dcmotor.h>


namespace robot::esp32 {


using Pwm = typename driver::esp32::Pwm;
using Servo = typename driver::esp32::Servo;
using Serial = typename driver::esp32::Serial;
using PincerCatcher = typename driver::esp32::PincerCatcher;
using Motor = DCMotor;

using RpLidar = typename driver::RpLidar<Serial, Pwm>;


class Robot {

    std::unique_ptr<DCMotor> _motorLeft;
    std::unique_ptr<DCMotor> _motorRight;

    PincerCatcher _pincerCatcher;
    RpLidar _lidar;

public:

    Motor& motorLeft() {
        return *_motorLeft;
    }

    Motor& motorRight() {
        return *_motorRight;
    }

    PincerCatcher& pincerCatcher() {
        return _pincerCatcher;
    }

    RpLidar& lidar() {
        return _lidar;
    }

    Robot(
        std::unique_ptr<DCMotor> motorLeft,
        std::unique_ptr<DCMotor> motorRight,
        PincerCatcher pincerCatcher,
        RpLidar lidar
    ):
        _motorLeft(std::move(motorLeft)),
        _motorRight(std::move(motorRight)),
        _pincerCatcher(std::move(pincerCatcher)),
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


} // namespace robot::esp32
