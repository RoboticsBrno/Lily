#pragma once

#include <tuple>

#include "driver/tickEncoder.h"
#include "driver/pwmMotor.h"

#include "driver/esp32/gpio.h"
#include "driver/esp32/pwm.h"
#include "driver/esp32/pincerCatcher.h"
#include "driver/esp32/lidar.h"


namespace robot::esp32 {


struct Robot {
    driver::PwmMotor<driver::esp32::Pwm> motorLeft;
    driver::PwmMotor<driver::esp32::Pwm> motorRight;

    driver::TickEncoder<driver::esp32::Gpio> encoderLeft;
    driver::TickEncoder<driver::esp32::Gpio> encoderRight;

    driver::esp32::PincerCatcher pincerCatcher;

    driver::esp32::Lidar lidar;

    Robot(
        gpio_num_t pinMotorLeft,
        gpio_num_t pinMotorRight,
        std::pair<gpio_num_t, gpio_num_t> pinsEncoderLeft,
        std::pair<gpio_num_t, gpio_num_t> pinsEncoderRight,
        gpio_num_t pinServoLeft,
        gpio_num_t pinServoRight,
        std::tuple<gpio_num_t, gpio_num_t, gpio_num_t> pinsLidar
    ):
        motorLeft(driver::esp32::Pwm(pinMotorLeft, LEDC_CHANNEL_0, LEDC_TIMER_0)),
        motorRight(driver::esp32::Pwm(pinMotorRight, LEDC_CHANNEL_1, LEDC_TIMER_0)),
        encoderLeft(std::tuple{ pinsEncoderLeft.first }, std::tuple{ pinsEncoderLeft.second }),
        encoderRight(std::tuple{ pinsEncoderRight.first }, std::tuple{ pinsEncoderRight.second }),
        pincerCatcher(
            driver::esp32::Servo(pinServoLeft),
            driver::esp32::Servo(pinServoRight)
        ),
        lidar(pinsLidar)
    {}
};


} // namespace robot::esp32
