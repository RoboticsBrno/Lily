#pragma once

#include <ratio>

#include "driver/ledc.h"
#include "util/concepts.h"


namespace driver::esp32 {


class Pwm {
    ledc_channel_t channel;
    ledc_timer_t timer;

public:
    using DutyRatio = std::ratio<1, 1024>;

    Pwm(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer):
        channel(channel),
        timer(timer)
    {
        ledc_timer_config_t timerConfig {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = timer,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timerConfig);

        ledc_channel_config_t channelConfig {
            .gpio_num = pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0,
            .hpoint = 0,
            .flags = {
                .output_invert = 0
            }
        };
        ledc_channel_config(&channelConfig);
    }

    Pwm(Pwm const&) = delete;
    Pwm(Pwm&& other):
        channel(other.channel),
        timer(other.timer)
    {
        other.channel = LEDC_CHANNEL_MAX;
    }

    ~Pwm() {
        if (channel != LEDC_CHANNEL_MAX) {
            ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
        }
    }

    void setDuty(uint32_t duty) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }
};


} // namespace driver::esp32
