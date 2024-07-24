#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include "robot.h"
#include "mockRobot.h"
#include "types/robot.h"

#include "logic/game.h"
#include "test/hw.h"


auto createRobot() {
    using namespace robot::esp32;

    return Robot(
        DRV8833(
            DRV8833Channel(
                Pwm(GPIO_NUM_11, LEDC_CHANNEL_0, LEDC_TIMER_0),
                Pwm(GPIO_NUM_12, LEDC_CHANNEL_1, LEDC_TIMER_0),
                false
            ),
            DRV8833Channel(
                Pwm(GPIO_NUM_45, LEDC_CHANNEL_2, LEDC_TIMER_0),
                Pwm(GPIO_NUM_13, LEDC_CHANNEL_3, LEDC_TIMER_0),
                true
            ),
            Gpio(GPIO_NUM_NC)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_39),
            Gpio(GPIO_NUM_40)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_42),
            Gpio(GPIO_NUM_41)
        ),
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


robot::mock::Robot createMockRobot() {
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


// auto lily = createRobot();
auto lily = createRobot();

int main() {

}


extern "C" void app_main() {
    uart_config_t config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &config);

    main();
}
