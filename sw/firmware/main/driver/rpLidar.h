#pragma once

#include <ratio>
#include <optional>
#include <thread>

#include "types/serial.h"
#include "types/lidar.h"
#include "types/pwm.h"


namespace driver {


template<isSerial Serial, isPwm Pwm>
class RpLidar {
    Serial _serial;
    Pwm _motorControl;

public:
    using DistanceUnit = std::ratio<1, 64>;
    using AngleUnit = std::ratio<1, 4>;


    RpLidar(Serial serial, Pwm motorControl):
        _serial(std::move(serial)),
        _motorControl(std::move(motorControl))
    {}

    RpLidar(RpLidar const&) = delete;
    RpLidar(RpLidar&& other):
        _serial(std::move(other._serial)),
        _motorControl(std::move(other._motorControl))
    {}

    void start() {
        using namespace std::chrono_literals;

        _motorControl.setDuty(100);

        _serial.write(0xA5);  // start packet
        _serial.write(0x25);  // stop
        std::this_thread::sleep_for(50ms);

        _serial.write(0xA5);  // start packet
        _serial.write(0x40);  // reset
        std::this_thread::sleep_for(50ms);

        _serial.write(0xA5);  // start packet
        _serial.write(0x20);  // scan
    }

    void stop() {
        using namespace std::chrono_literals;

        _serial.write(0xA5);  // start packet
        _serial.write(0x25);  // stop
        std::this_thread::sleep_for(50ms);
    }

    std::optional<Measurement> getMeasurement() {
        Measurement measurement;
        while (_serial.available() >= 5) {
            uint8_t data = _serial.read();
            // first byte should contain start bit and its inverse
            if (data & 0x01 == (data >> 1) & 0x01) {
                continue;
            }
            measurement.quality = data >> 2;

            data = _serial.read();
            // last bit of second byte should be 1
            if (data & 0x01 == 0) {
                continue;
            }
            measurement.angle = data >> 1;
            measurement.angle |= _serial.read() << 7;

            measurement.distance = _serial.read() | (_serial.read() << 8);

            return measurement;
        }
    }
};


} // namespace driver
