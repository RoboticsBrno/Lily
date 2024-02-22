#pragma once

#include <utility>

#include "types/serial.h"
#include "types/motor.h"
#include "types/tickEncoder.h"
#include "util/cobs.h"


namespace driver::mock {


template<typename MEs>
class Encoder {
    unsigned _ticks = 0;
    OverflowFlag _overflow = OverflowFlag::None;
    MEs* _parent;
public:
    Encoder(MEs* parent) : _parent(parent) {}

    std::pair<unsigned, OverflowFlag> getTicks() {
        _parent->loop();
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
            _overflow = OverflowFlag::Underflow;
        }
    }

    void setParent(MEs* parent) {
        _parent = parent;
    }
};

template<isSerial Serial>
class MotorsEncoders;

template<class MEs>
class Motor {
    MEs& _parent;
    int _id;

public:
    Motor(MEs& me, int id) : _parent(me), _id(id) {}

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

    Encoder<MotorsEncoders> _encoders[2];

    enum class Cmd : uint8_t {
        TicksLeft = 0,
        TicksRight = 1,
        MotorLeft = 10,
        MotorRight = 11
    };
public:

    MotorsEncoders(Serial serial) : _serial(std::move(serial)), _encoders{this, this} {}

    MotorsEncoders(MotorsEncoders const&) = delete;
    MotorsEncoders(MotorsEncoders&& other):
        _serial(std::move(other._serial)),
        _decoder(std::move(other._decoder)),
        _encoders{std::move(other._encoders[0]), std::move(other._encoders[1])}
    {
        for (int i = 0; i < 2; ++i) {
            _encoders[i].setParent(this);
        }
    }

    void loop() {
        std::vector<uint8_t> packet;
        while (_serial.available()) {
            packet = _decoder.receive(_serial.read());
            if (packet.size() == 0) {
                continue;
            }

            switch (packet[0]) {
                case static_cast<uint8_t>(Cmd::TicksLeft):
                    if (packet.size() != 2) {
                        // XXX: report error
                        break;
                    }
                    _encoders[0].update(packet[1]);
                    break;
                case static_cast<uint8_t>(Cmd::TicksRight):
                    if (packet.size() != 2) {
                        // XXX: report error
                        break;
                    }
                    _encoders[1].update(packet[1]);
                    break;
                default:
                    // ignore - should be only sending
                    break;
            }
        }
    }

    void setMotor(int id, unsigned power) {
        std::array<uint8_t, 5> data;
        std::span<uint8_t> packet(data.data() + 1, data.size() - 1);
        if (id == 0) {
            packet[0] = static_cast<uint8_t>(Cmd::MotorLeft);
        }
        else {
            packet[0] = static_cast<uint8_t>(Cmd::MotorRight);
        }

        packet[1] = power & 0xFF;
        packet[2] = (power >> 8) & 0xFF;

        cobs_encode(std::span(data.data(), data.size()));
        for (uint8_t byte : data) {
            _serial.write(byte);
        }
    }

    Motor<MotorsEncoders> motorLeft() {
        return Motor<MotorsEncoders>(*this, 0);
    }

    Motor<MotorsEncoders> motorRight() {
        return Motor<MotorsEncoders>(*this, 1);
    }

    Encoder<MotorsEncoders>& encoderLeft() {
        return _encoders[0];
    }

    Encoder<MotorsEncoders>& encoderRight() {
        return _encoders[1];
    }
};


template<class MEs>
void Motor<MEs>::setPower(unsigned power) {
    this->_parent.setMotor(this->_id, power);
}


} // namespace driver::mock
