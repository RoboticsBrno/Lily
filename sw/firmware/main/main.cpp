#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/tickEncoder.h"
#include "driver/pwmMotor.h"
#include "driver/gpio.h"

#include "robot.h"
#include "types/robot.h"

int main() {
    // static_assert(isRobot<robot::esp32::Robot>);
}


extern "C" void app_main() {
    main();

    vTaskDelete(NULL);
}
