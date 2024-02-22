#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include "robot.h"
#include "types/robot.h"

#include "logic/game.h"
#include "test/hw.h"


robot::esp32::Robot createRobot() {
    using namespace robot::esp32;

    return Robot(
        DRV8833(
            DRV8833Channel(
                Pwm(GPIO_NUM_4, LEDC_CHANNEL_0, LEDC_TIMER_0),
                Pwm(GPIO_NUM_5, LEDC_CHANNEL_1, LEDC_TIMER_0),
                false
            ),
            DRV8833Channel(
                Pwm(GPIO_NUM_6, LEDC_CHANNEL_2, LEDC_TIMER_0),
                Pwm(GPIO_NUM_7, LEDC_CHANNEL_3, LEDC_TIMER_0),
                true
            ),
            Gpio(GPIO_NUM_15)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_16),
            Gpio(GPIO_NUM_17)
        ),
        TickEncoder(
            Gpio(GPIO_NUM_18),
            Gpio(GPIO_NUM_8)
        ),
        PincerCatcher(
            Servo(GPIO_NUM_0),
            Servo(GPIO_NUM_0)
        ),
        RpLidar(
            Serial(GPIO_NUM_1, GPIO_NUM_2, 115200, UART_NUM_1),
            Pwm(GPIO_NUM_3, LEDC_CHANNEL_4, LEDC_TIMER_1)
        )
    );
}


auto lily = createRobot();


int main() {
    test::robot(lily);
}


extern "C" void app_main() {
    main();

    while (true) {
        using namespace std::chrono_literals;

        std::this_thread::sleep_for(1s);
    }
}
