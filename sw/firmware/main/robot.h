#pragma once

#include <tuple>

#include "driver/tickEncoder.h"
#include "driver/drv8833.h"
#include "driver/rpLidar.h"

#include "driver/esp32/gpio.h"
#include "driver/esp32/pwm.h"
#include "driver/esp32/pincerCatcher.h"
#include "driver/esp32/serial.h"


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


struct Robot {
    DRV8833 motorDriver;

    DRV8833Channel& motorLeft;
    DRV8833Channel& motorRight;

    TickEncoder encoderLeft;
    TickEncoder encoderRight;

    PincerCatcher pincerCatcher;

    RpLidar lidar;

    Robot(
        DRV8833 motorDriver,
        TickEncoder encoderLeft,
        TickEncoder encoderRight,
        PincerCatcher pincerCatcher,
        RpLidar lidar
    ):
        motorDriver(std::move(motorDriver)),
        motorLeft(motorDriver[0]),
        motorRight(motorDriver[1]),
        encoderLeft(std::move(encoderLeft)),
        encoderRight(std::move(encoderRight)),
        pincerCatcher(std::move(pincerCatcher)),
        lidar(std::move(lidar))
    {}

    Robot(Robot const&) = delete;
    Robot(Robot&& other) = delete;

    void start() {
        motorDriver.start();
        motorLeft.setPower(0);
        motorRight.setPower(0);

        lidar.start();
    }

    void stop() {
        motorDriver.sleep();
        motorLeft.setPower(0);
        motorRight.setPower(0);

        lidar.stop();
    }
};


} // namespace robot::esp32
