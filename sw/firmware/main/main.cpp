#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include "robot.h"
#include "types/robot.h"


robot::esp32::Robot createRobot() {
    using namespace robot::esp32;

    return Robot(
        DRV8833(
            DRV8833Channel(
                Pwm(GPIO_NUM_0, LEDC_CHANNEL_0, LEDC_TIMER_0),
                Pwm(GPIO_NUM_0, LEDC_CHANNEL_1, LEDC_TIMER_0),
                false
            ),
            DRV8833Channel(
                Pwm(GPIO_NUM_0, LEDC_CHANNEL_2, LEDC_TIMER_0),
                Pwm(GPIO_NUM_0, LEDC_CHANNEL_3, LEDC_TIMER_0),
                true
            ),
            Gpio(GPIO_NUM_0)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_0),
            Gpio(GPIO_NUM_0)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_0),
            Gpio(GPIO_NUM_0)
        ),
        PincerCatcher(
            Servo(GPIO_NUM_0),
            Servo(GPIO_NUM_0)
        ),
        RpLidar(
            Serial(GPIO_NUM_0, GPIO_NUM_0, 115200, UART_NUM_1),
            Pwm(GPIO_NUM_0, LEDC_CHANNEL_4, LEDC_TIMER_1)
        )
    );
}


auto lily = createRobot();


int main() {
    static_assert(isRobot<decltype(lily)>);
}


extern "C" void app_main() {
    main();

    vTaskDelete(NULL);
}
