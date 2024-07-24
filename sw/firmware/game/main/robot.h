#pragma once

#include <tuple>

#include "./driver/tickEncoder.h"
#include "./driver/drv8833.h"
#include "./driver/rpLidar.h"

#include "./driver/esp32/gpio.h"
#include "./driver/esp32/pwm.h"
#include "./driver/esp32/pincerCatcher.h"
#include "./driver/esp32/serial.h"


namespace robot::esp32 {


using Pwm = typename driver::esp32::Pwm;
using Gpio = typename driver::esp32::Gpio;
using Servo = typename driver::esp32::Servo;
using Serial = typename driver::esp32::Serial;
using PincerCatcher = typename driver::esp32::PincerCatcher;

using DRV8833 = typename driver::DRV8833<Pwm, Gpio>;
using DRV8833Channel = typename driver::DRV8833Channel<Pwm>;
using TickEncoder = typename driver::TickEncoder<Gpio>;
using RpLidar = typename driver::RpLidar<Serial, Pwm>;


class Robot {
    DRV8833 _motorDriver;

    TickEncoder _encoderLeft;
    TickEncoder _encoderRight;

    PincerCatcher _pincerCatcher;

    RpLidar _lidar;

public:
    DRV8833Channel& motorLeft() {
        return _motorDriver[0];
    }
    DRV8833Channel& motorRight() {
        return _motorDriver[1];
    }

    TickEncoder& encoderLeft() {
        return _encoderLeft;
    }

    TickEncoder& encoderRight() {
        return _encoderRight;
    }

    PincerCatcher& pincerCatcher() {
        return _pincerCatcher;
    }

    RpLidar& lidar() {
        return _lidar;
    }

    Robot(
        DRV8833 motorDriver,
        TickEncoder encoderLeft,
        TickEncoder encoderRight,
        PincerCatcher pincerCatcher,
        RpLidar lidar
    ):
        _motorDriver(std::move(motorDriver)),
        _encoderLeft(std::move(encoderLeft)),
        _encoderRight(std::move(encoderRight)),
        _pincerCatcher(std::move(pincerCatcher)),
        _lidar(std::move(lidar))
    {}

    Robot(Robot const&) = delete;
    Robot(Robot&& other) = delete;

    void start() {
        _motorDriver.start();
        motorLeft().setPower(0);
        motorRight().setPower(0);

        _lidar.start();
    }

    void stop() {
        _motorDriver.sleep();
        motorLeft().setPower(0);
        motorRight().setPower(0);

        _lidar.stop();
    }
};


} // namespace robot::esp32
