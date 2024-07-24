#pragma once

#include "servo.h"


namespace driver::esp32 {


class PincerCatcher {
    Servo servoLeft;
    Servo servoRight;
public:
    PincerCatcher(Servo servoLeft, Servo servoRight):
        servoLeft(std::move(servoLeft)),
        servoRight(std::move(servoRight))
    {}

    PincerCatcher(PincerCatcher const&) = delete;
    PincerCatcher(PincerCatcher&& other):
        servoLeft(std::move(other.servoLeft)),
        servoRight(std::move(other.servoRight))
    {}

    void setPos(int pos) {
        servoLeft.setPos(pos);
        servoRight.setPos(-pos);
    }

    void open() {
        setPos(0);
    }

    int getPos() {
        throw "Not implemented";
    }
};


} // namespace driver::esp32
