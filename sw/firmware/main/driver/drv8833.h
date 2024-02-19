#pragma once

#include "types/pwm.h"
#include "types/gpio.h"


namespace driver {


template<isPwm Pwm>
class DRV8833Channel {
    Pwm pwms[2];
    bool inverted;

public:
    static constexpr int MaxPower = Pwm::MaxDuty;

    DRV8833Channel(Pwm pwmA, Pwm pwmB, bool inverted = false):
        pwms{ std::move(pwmA), std::move(pwmB) },
        inverted(inverted)
    {}

    DRV8833Channel(DRV8833Channel const&) = delete;
    DRV8833Channel(DRV8833Channel&& other):
        pwms{ std::move(other.pwms[0]), std::move(other.pwms[1]) },
        inverted(other.inverted)
    {}

    void setPower(int power) {
        if (inverted) {
            power = -power;
        }

        if (power > 0) {
            pwms[0].setDuty(power);
            pwms[1].setDuty(0);
        }
        else {
            pwms[0].setDuty(0);
            pwms[1].setDuty(-power);
        }
    }
};

template<isPwm Pwm, isGpio Gpio>
class DRV8833 {
    using DRV8833Channel = typename driver::DRV8833Channel<Pwm>;

    DRV8833Channel _channels[2];
    Gpio _sleep;

public:
    DRV8833(DRV8833Channel channel1, DRV8833Channel channel2, Gpio sleep):
        _channels{std::move(channel1), std::move(channel2)},
        _sleep(std::move(sleep))
    {}

    DRV8833Channel& operator[](int i) {
        return _channels[i];
    }

    void start() {
        _sleep.write(true);
    }

    void sleep() {
        _sleep.write(false);
    }
};


} // namespace driver
