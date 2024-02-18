#pragma once


namespace driver::esp32 {


class Servo {
public:
    Servo(gpio_num_t pin) {
        throw "Not implemented";
    }

    void release() {
        throw "Not implemented";
    }

    void setPos(int pos) {
        throw "Not implemented";
    }
};


} // namespace driver::esp32
