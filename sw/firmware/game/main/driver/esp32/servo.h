#pragma once

#include <iostream>


namespace driver::esp32 {


class Servo {
public:
    Servo(gpio_num_t pin) {
        std::cerr << "Servo::Servo(" << pin << ")" << std::endl;
    }

    Servo(Servo const&) = delete;
    Servo(Servo&& other) {
        std::cerr << "Servo::Servo(Servo&&)" << std::endl;
    }

    void release() {
        std::cerr << "Servo::release()" << std::endl;
    }

    void setPos(int pos) {
        std::cerr << "Servo::setPos(" << pos << ")" << std::endl;
    }
};


} // namespace driver::esp32
