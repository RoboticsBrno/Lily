#pragma once

#include <ratio>

#include "types/lidar.h"


namespace driver::esp32 {


class Lidar {

public:
    using DistanceUnit = std::ratio<1, 64>;
    using AngleUnit = std::ratio<1, 4>;

    Lidar(gpio_num_t pinTx, gpio_num_t pinRx, gpio_num_t pinEnable) {
        throw "Not implemented";
    }

    Lidar(std::tuple<gpio_num_t, gpio_num_t, gpio_num_t> pins):
        Lidar(std::get<0>(pins), std::get<1>(pins), std::get<2>(pins))
    {}

    void start() {
        throw "Not implemented";
    }

    void stop() {
        throw "Not implemented";
    }

    unsigned available() {
        throw "Not implemented";
    }

    Measurement getMeasurement() {
        throw "Not implemented";
    }
};


} // namespace driver::esp32
