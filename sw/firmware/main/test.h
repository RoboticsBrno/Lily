#pragma once

#include <iostream>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "robot.h"

namespace test {

void lidar(RpLidar& lidar) {
    std::cout << "---Testing lidar---" << std::endl;

    lidar.start();

    std::cout << "Measurements:" << std::endl;
    int samples = 30;

    while (samples > 0) {
        auto measurement = lidar.getMeasurement();
        if (!measurement) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        std::cout << "  D: " << measurement->distance
                  << " A: " << measurement->angle
                  << " Q: " << static_cast<int>(measurement->quality)
                  << std::endl;

        --samples;
    }

    lidar.stop();

    while (lidar.getMeasurement()) {
    }

    std::cout << "---Lidar test complete---" << std::endl;
}

void motor(DCMotor& motor) {
    std::cout << "---Testing motor---" << std::endl;

    std::cout << "Stopping motor" << std::endl;
    motor.stop(true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "Setting speed to 300 (forward)" << std::endl;
    motor.moveInfinite();
    motor.setSpeed(300);
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "Setting speed to -300 (backward)" << std::endl;
    motor.setSpeed(-300);
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "Setting speed to 0" << std::endl;
    motor.stop(false);

    std::cout << "---Motor test complete---" << std::endl;
}

void encoder(DCMotor& motor) {
    std::cout << "---Testing encoder---" << std::endl;

    std::cout << "Disabling motor" << std::endl;
    motor.stop(false);

    std::cout << "Reading encoder" << std::endl;

    for (int i = 0; i < 20; ++i) {
        auto ticks = motor.getPosition();
        std::cout << "Ticks: " << ticks << std::endl;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    std::cout << "---Encoder test complete---" << std::endl;
}

void claws(Claws& claws) {
    std::cout << "---Testing claws---" << std::endl;

    std::cout << "Opening claws" << std::endl;
    claws.setPos(0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "Closing claws" << std::endl;
    claws.setPos(100);
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "---Claws test complete---" << std::endl;
}

void robot(Robot& robot) {
    std::cout << "---Testing robot---" << std::endl;

    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "\nMotor left" << std::endl;
    motor(robot.motorLeft());

    std::cout << "\nMotor right" << std::endl;
    motor(robot.motorRight());

    std::cout << "\nEncoder left" << std::endl;
    encoder(robot.motorLeft());

    std::cout << "\nEncoder right" << std::endl;
    encoder(robot.motorRight());

    std::cout << "\nClaws" << std::endl;
    claws(robot.claws());

    std::cout << "\nLidar" << std::endl;
    lidar(robot.lidar());

    std::cout << "\nStopping robot" << std::endl;
    robot.stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    std::cout << "---Robot test complete---" << std::endl;
}

} // namespace test
