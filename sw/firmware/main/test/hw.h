#pragma once

#include <chrono>
#include <thread>
#include <iostream>

#include "types/lidar.h"
#include "types/motor.h"
#include "types/pincerCatcher.h"
#include "types/tickEncoder.h"
#include "types/robot.h"


namespace test {


void lidar(isLidar auto& lidar) {
    using namespace std::chrono_literals;

    std::cout << "---Testing lidar---" << std::endl;

    lidar.start();

    std::cout << "Waiting for data" << std::endl;

    std::this_thread::sleep_for(1s);

    std::cout << "Measurements:" << std::endl;

    while (true) {
        auto measurement = lidar.getMeasurement();
        if (!measurement) {
            break;
        }
        std::cout << "  D: " << measurement->distance << " A: " << measurement->angle << std::endl;
    }

    lidar.stop();

    std::cout << "---Lidar test complete---" << std::endl;
}


void motor(isMotor auto& motor) {
    using namespace std::chrono_literals;

    std::cout << "---Testing motor---" << std::endl;

    std::cout << "Setting power to 0" << std::endl;

    motor.setPower(0);
    std::this_thread::sleep_for(1s);

    std::cout << "Setting power to 100 (forward)" << std::endl;

    motor.setPower(100);
    std::this_thread::sleep_for(1s);

    std::cout << "Setting power to -100 (backward)" << std::endl;

    motor.setPower(-100);
    std::this_thread::sleep_for(1s);

    std::cout << "Setting power to 0" << std::endl;

    motor.setPower(0);

    std::cout << "---Motor test complete---" << std::endl;
}


void pincerCatcher(isPincerCatcher auto& pincerCatcher) {
    using namespace std::chrono_literals;

    std::cout << "---Testing pincerCatcher---" << std::endl;

    std::cout << "Opening pincer" << std::endl;

    pincerCatcher.setPos(100);
    std::this_thread::sleep_for(1s);

    std::cout << "Closing pincer" << std::endl;

    pincerCatcher.setPos(0);
    std::this_thread::sleep_for(1s);

    std::cout << "---PincerCatcher test complete---" << std::endl;
}


void encoder(isTickEncoder auto& encoder) {
    using namespace std::chrono_literals;

    std::cout << "---Testing encoder---" << std::endl;

    std::cout << "Reading encoder" << std::endl;

    for (int i = 0; i < 10; ++i) {
        auto [ ticks, overflow ] = encoder.getTicks();
        std::cout << "Ticks: " << ticks << " OF flag: ";
        switch (overflow) {
            case OverflowFlag::None:
                std::cout << "None";
                break;
            case OverflowFlag::Overflow:
                std::cout << "Overflow";
                break;
            case OverflowFlag::Underflow:
                std::cout << "Underflow";
                break;
        }
        encoder.resetOverflow();
        std::this_thread::sleep_for(200ms);
    }

    std::cout << "---Encoder test complete---" << std::endl;
}


void robot(isRobot auto& robot) {
    using namespace std::chrono_literals;

    std::cout << "---Testing robot---" << std::endl;

    std::cout << "Starting robot" << std::endl;
    robot.start();
    std::this_thread::sleep_for(1s);

    std::cout << "Motor left" << std::endl;
    motor(robot.motorLeft);

    std::cout << "Motor right" << std::endl;
    motor(robot.motorRight);

    std::cout << "Encoder left" << std::endl;
    encoder(robot.encoderLeft);

    std::cout << "Encoder right" << std::endl;
    encoder(robot.encoderRight);

    std::cout << "PincerCatcher" << std::endl;
    pincerCatcher(robot.pincerCatcher);

    std::cout << "Lidar" << std::endl;
    lidar(robot.lidar);

    std::cout << "Stopping robot" << std::endl;
    robot.stop();

    std::this_thread::sleep_for(1s);

    std::cout << "---Robot test complete---" << std::endl;
}


} // namespace test
