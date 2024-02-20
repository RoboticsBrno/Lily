#pragma once

#include <utility>

#include "types/serial.h"
#include "types/motor.h"
#include "types/tickEncoder.h"
#include "util/cobs.h"


namespace driver::mock {


class Encoder {
    unsigned _ticks;
    OverflowFlag _overflow;
public:
    Encoder() = default;

    std::pair<unsigned, OverflowFlag> getTicks() {
        return { _ticks, _overflow };
    }

    void resetOverflow() {
        _overflow = OverflowFlag::None;
    }

    void reset() {
        _overflow = OverflowFlag::None;
        _ticks = 0;
    }

    void update(int ticks) {
        unsigned old = _ticks;
        _ticks += ticks;
        if (ticks > 0 && old > _ticks) {
            _overflow = OverflowFlag::Overflow;
        }
        if (ticks < 0 && old < _ticks) {
            _overflow = OverflowFlag::Underflow
        }
    }
};

template<isSerial Serial>
class MotorsEncoders;

template<class MEs>
class Motor {
    MEs& _parent;
    int _id;

public:
    Motor(MEs& me, int id) : _parent(me), id(id) {}

    void setPower(unsigned power);
};



/**
 * The protocol uses COBS encoding and has the following packet structure:
 *
 * | CMD  | [DATA] |
 * | byte | rest   |
 */
template<isSerial Serial>
class MotorsEncoders {
    Serial _serial;
    CobsStreamDecoder _decoder;

    Motor<MotorsEncoders> _motors[2];
    Encoder _encoders[2];

    enum class Cmd : uint8_t {
        TicksLeft = 0,
        TicksRight = 1,
        MotorLeft = 10,
        MotorRight = 11
    };
public:

    MotorsEncoders(Serial serial) : _serial(serial) {}

    void loop() {
        std::vector<uint8_t> packet;
        while (_serial.available()) {
            packet = _decoder.receive(_serial.read());
            if (packet.size() == 0) {
                continue;
            }

            switch (packet[0]) {
                case Cmd::TicksLeft:
                    if (packet.size() != 2) {
                        // XXX: report error
                        break;
                    }
                    _encoders[0].update(packet[1]);
                    break;
                case Cmd::TicksRight:
                    if (packet.size() != 2) {
                        // XXX: report error
                        break;
                    }
                    _encoders[1].update(packet[1]);
                    break;
                default:
                    // ignore - should be only sending
            }
        }
    }

    setMotor(int id, unsigned power) {
        std::array<uint8_t, 3> packet;
        if (id == 0) {
            packet[0] = Cmd::MotorLeft;
        }
        else {
            packet[0] = Cmd::MotorRight;
        }

        packet[1] = power & 0xFF;
        packet[2] = (power >> 8) & 0xFF;
    }
}

template<class MEs>
void Motor<MEs>::setPower(unsigned power) {
    this->_parent.setMotor(this->_id);
}


} // namespace driver::mock
