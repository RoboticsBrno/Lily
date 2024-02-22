#pragma once

#include <tuple>

#include "driver/tickEncoder.h"
#include "driver/mock/motorEncoder.h"
#include "driver/rpLidar.h"

#include "driver/esp32/gpio.h"
#include "driver/esp32/pwm.h"
#include "driver/esp32/pincerCatcher.h"
#include "driver/esp32/serial.h"


namespace robot::mock {


using Pwm = typename driver::esp32::Pwm;
using Gpio = typename driver::esp32::Gpio;
using Servo = typename driver::esp32::Servo;
using Serial = typename driver::esp32::Serial;
using PincerCatcher = typename driver::esp32::PincerCatcher;

using MotorsEncoders = typename driver::mock::MotorsEncoders<Serial>;
using Motor = typename driver::mock::Motor<MotorsEncoders>;
using Encoder = typename driver::mock::Encoder<MotorsEncoders>;
using RpLidar = typename driver::RpLidar<Serial, Pwm>;


class Robot {
    MotorsEncoders _motorsEncoders;

    PincerCatcher _pincerCatcher;

    RpLidar _lidar;

public:
    Motor motorLeft() {
        return _motorsEncoders.motorLeft();
    }

    Motor motorRight() {
        return _motorsEncoders.motorRight();
    }

    Encoder& encoderLeft() {
        return _motorsEncoders.encoderLeft();
    }

    Encoder& encoderRight() {
        return _motorsEncoders.encoderRight();
    }

    PincerCatcher& pincerCatcher() {
        return _pincerCatcher;
    }

    RpLidar& lidar() {
        return _lidar;
    }

    Robot(
        MotorsEncoders motorsEncoders,
        PincerCatcher pincerCatcher,
        RpLidar lidar
    ):
        _motorsEncoders(std::move(motorsEncoders)),
        _pincerCatcher(std::move(pincerCatcher)),
        _lidar(std::move(lidar))
    {}

    Robot(Robot const&) = delete;
    Robot(Robot&& other) = delete;

    void start() {
        motorLeft().setPower(0);
        motorRight().setPower(0);

        _lidar.start();
    }

    void stop() {
        motorLeft().setPower(0);
        motorRight().setPower(0);

        _lidar.stop();
    }
};


} // namespace robot::esp32
