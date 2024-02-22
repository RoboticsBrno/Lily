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


void lidar(isLidar auto&& lidar) {
    using namespace std::chrono_literals;

    std::cout << "---Testing lidar---" << std::endl;

    lidar.start();

    std::cout << "Measurements:" << std::endl;
    int samples = 30;

    while (samples > 0) {
        auto measurement = lidar.getMeasurement();
        if (!measurement) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        std::cout << "  D: " << measurement->distance
                  << " A: " << measurement->angle
                  << " Q: " << static_cast<int>(measurement->quality)
                  << std::endl;

        --samples;
    }

    lidar.stop();
    while (lidar.getMeasurement());

    std::cout << "---Lidar test complete---" << std::endl;
}


void motor(isMotor auto&& motor) {
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


void pincerCatcher(isPincerCatcher auto&& pincerCatcher) {
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


void encoder(isTickEncoder auto&& encoder) {
    using namespace std::chrono_literals;

    std::cout << "---Testing encoder---" << std::endl;

    std::cout << "Reading encoder" << std::endl;

    for (int i = 0; i < 20; ++i) {
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
        std::cout << std::endl;
        encoder.resetOverflow();
        std::this_thread::sleep_for(200ms);
    }

    std::cout << "---Encoder test complete---" << std::endl;
}


void robot(isRobot auto&& robot) {
    using namespace std::chrono_literals;

    std::cout << "---Testing robot---" << std::endl;

    std::cout << "\nStarting robot" << std::endl;
    robot.start();
    std::this_thread::sleep_for(1s);

    std::cout << "\nMotor left" << std::endl;
    motor(robot.motorLeft());

    std::cout << "\nMotor right" << std::endl;
    motor(robot.motorRight());

    std::cout << "\nEncoder left" << std::endl;
    encoder(robot.encoderLeft());

    std::cout << "\nEncoder right" << std::endl;
    encoder(robot.encoderRight());

    std::cout << "\nPincerCatcher" << std::endl;
    pincerCatcher(robot.pincerCatcher());

    std::cout << "\nLidar" << std::endl;
    lidar(robot.lidar());

    std::cout << "\nStopping robot" << std::endl;
    robot.stop();

    std::this_thread::sleep_for(1s);

    std::cout << "---Robot test complete---" << std::endl;
}


} // namespace test
