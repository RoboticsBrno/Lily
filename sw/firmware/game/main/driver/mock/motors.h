#pragma once

#include <utility>
#include <functional>

#include "../../types/serial.h"
#include "../../types/motor.h"
#include "../../util/cobs.h"


namespace driver::mock {


template<typename MEs>
class DcMotor {
    MEs* _parent;
    int _id;
    int64_t _position = 0;
public:
    DcMotor(MEs* parent, int id) : _parent(parent), _id(id) {}
    DcMotor(DcMotor const&) = delete;
    DcMotor(DcMotor&& other) = delete;

    void setSpeed(int speed);
    void moveInfinite();
    void moveTime(int duration);
    void moveDistance(int distance);
    void stop(bool brake);
    void onTarget(std::function<void()> callback);
    int64_t getPosition();

    void update(int16_t value) {
        _position += value;
    }
};

enum class MotorCmd : uint8_t {
    Speed = 0,
    Infinite = 1,
    Time = 2,
    Distance = 3,
    Stop = 4,
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

    std::unique_ptr<DcMotor<MotorsEncoders>> _motors[2];
public:
    enum class Cmd : uint8_t {
        TicksLeft = 0,
        TicksRight = 1,
        MotorLeft = 10,
        MotorRight = 11
    };

    MotorsEncoders(Serial serial):
        _serial(std::move(serial)),
        _motors{
            std::make_unique<DcMotor<MotorsEncoders>>(this, 0),
            std::make_unique<DcMotor<MotorsEncoders>>(this, 1)
        }
    {}

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
                    _motors[0].update(packet[1]);
                    break;
                case static_cast<uint8_t>(Cmd::TicksRight):
                    if (packet.size() != 2) {
                        // XXX: report error
                        break;
                    }
                    _motors[1].update(packet[1]);
                    break;
                default:
                    // ignore - should be only sending
                    break;
            }
        }
    }

    void setMotor(int id, int16_t value, MotorCmd command) {
        std::array<uint8_t, 5> data;
        std::span<uint8_t> packet(data.data() + 1, data.size() - 1);
        if (id == 0) {
            packet[0] = static_cast<uint8_t>(Cmd::MotorLeft);
        }
        else {
            packet[0] = static_cast<uint8_t>(Cmd::MotorRight);
        }

        packet[2] = value & 0xFF;
        packet[3] = (value >> 8) & 0xFF;

        cobs_encode(std::span(data.data(), data.size()));
        for (uint8_t byte : data) {
            _serial.write(byte);
        }
    }

    DcMotor<MotorsEncoders>& motorLeft() {
        return *_motors[0];
    }

    DcMotor<MotorsEncoders>& motorRight() {
        return *_motors[1];
    }
};


template<typename MEs>
void DcMotor<MEs>::setSpeed(int speed) {
    _parent->setMotor(_id, speed, MotorCmd::Speed);
}

template<typename MEs>
void DcMotor<MEs>::moveInfinite() {
    _parent->setMotor(_id, 0, MotorCmd::Infinite);
}

template<typename MEs>
void DcMotor<MEs>::moveTime(int duration) {
    _parent->setMotor(_id, duration, MotorCmd::Time);
}

template<typename MEs>
void DcMotor<MEs>::moveDistance(int distance) {
    _parent->setMotor(_id, distance, MotorCmd::Distance);
}

template<typename MEs>
void DcMotor<MEs>::stop(bool brake) {
    _parent->setMotor(_id, brake ? 1 : 0, MotorCmd::Stop);
}

template<typename MEs>
void DcMotor<MEs>::onTarget(std::function<void()> callback) {
    // XXX: not implemented
}

template<typename MEs>
int64_t DcMotor<MEs>::getPosition() {
    // XXX: not implemented
    return 0;
}


} // namespace driver::mock
