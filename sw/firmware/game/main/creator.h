#pragma once

#include "driver/ledc.h"

#include "robot.h"
#include "mockRobot.h"
#include "types/robot.h"


static inline auto createRobot() {
    using namespace robot::esp32;

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 64000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ledc_timer_config(&timer_conf);

    return Robot(
        std::make_unique<DCMotor>(GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_39, GPIO_NUM_40, 30, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1),
        std::make_unique<DCMotor>(GPIO_NUM_45, GPIO_NUM_13, GPIO_NUM_42, GPIO_NUM_41, 30, LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_CHANNEL_3),
        Gpio(GPIO_NUM_18),
        PincerCatcher(
            Servo(GPIO_NUM_NC),
            Servo(GPIO_NUM_NC)
        ),
        RpLidar(
            Serial(GPIO_NUM_9, GPIO_NUM_17, 115200, UART_NUM_1, 10240, 0),
            Pwm(GPIO_NUM_1, LEDC_CHANNEL_4, LEDC_TIMER_0)
        )
    );
}


static inline robot::mock::Robot createMockRobot() {
    using namespace robot::mock;

    return Robot(
        MotorsEncoders(
            Serial(UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 115200, UART_NUM_0, 10240, 0)
        ),
        PincerCatcher(
            Servo(GPIO_NUM_0),
            Servo(GPIO_NUM_0)
        ),
        RpLidar(
            Serial(UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 115200, UART_NUM_1, 10240, 0),
            Pwm(GPIO_NUM_3, LEDC_CHANNEL_4, LEDC_TIMER_1)
        )
    );
}
