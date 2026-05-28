#pragma once

#include <algorithm>

#include "driver/gpio.h"
#include "driver/ledc.h"

class ClawMotor {
    gpio_num_t _pinA;
    gpio_num_t _pinB;
    ledc_channel_t _channelA;
    ledc_channel_t _channelB;

    static constexpr ledc_mode_t SPEED_MODE = LEDC_LOW_SPEED_MODE;
    static constexpr int MAX_DUTY = 1023;

    static void setDuty(ledc_channel_t channel, int duty) {
        ledc_set_duty(SPEED_MODE, channel, duty);
        ledc_update_duty(SPEED_MODE, channel);
    }

public:
    ClawMotor(
        gpio_num_t pinA,
        gpio_num_t pinB,
        ledc_timer_t timer,
        ledc_channel_t channelA,
        ledc_channel_t channelB
    ):
        _pinA(pinA),
        _pinB(pinB),
        _channelA(channelA),
        _channelB(channelB)
    {
        ledc_channel_config_t forwardConfig = {
            .gpio_num = _pinA,
            .speed_mode = SPEED_MODE,
            .channel = _channelA,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0,
            .hpoint = 0,
            .flags = {},
        };
        ledc_channel_config(&forwardConfig);

        ledc_channel_config_t reverseConfig = {
            .gpio_num = _pinB,
            .speed_mode = SPEED_MODE,
            .channel = _channelB,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0,
            .hpoint = 0,
            .flags = {},
        };
        ledc_channel_config(&reverseConfig);
    }

    ClawMotor(ClawMotor const&) = delete;
    ClawMotor(ClawMotor&&) = default;

    void setPower(int power) {
        power = std::clamp(power, -MAX_DUTY, MAX_DUTY);

        if (power > 0) {
            setDuty(_channelA, power);
            setDuty(_channelB, 0);
        }
        else if (power < 0) {
            setDuty(_channelA, 0);
            setDuty(_channelB, -power);
        }
        else {
            stop();
        }
    }

    void stop() {
        setDuty(_channelA, 0);
        setDuty(_channelB, 0);
    }
};

class Claws {
    ClawMotor _left;
    ClawMotor _right;

    static constexpr int CLAW_POWER = 512;

public:
    Claws(
        gpio_num_t leftAPin,
        gpio_num_t leftBPin,
        ledc_timer_t leftTimer,
        ledc_channel_t leftchannelA,
        ledc_channel_t leftchannelB,
        gpio_num_t rightAPin,
        gpio_num_t rightBPin,
        ledc_timer_t rightTimer,
        ledc_channel_t rightchannelA,
        ledc_channel_t rightchannelB
    ):
        _left(leftAPin, leftBPin, leftTimer, leftchannelA, leftchannelB),
        _right(rightAPin, rightBPin, rightTimer, rightchannelA, rightchannelB)
    {}

    Claws(Claws const&) = delete;
    Claws(Claws&&) = default;

    void setPos(int pos) {
        if (pos <= 0) {
            open();
        }
        else {
            close();
        }
    }

    int getPos() {
        return 0;
    }

    void open() {
        _left.setPower(CLAW_POWER);
        _right.setPower(-CLAW_POWER);
    }

    void close() {
        _left.setPower(-CLAW_POWER);
        _right.setPower(CLAW_POWER);
    }

    void stop() {
        _left.stop();
        _right.stop();
    }
};
