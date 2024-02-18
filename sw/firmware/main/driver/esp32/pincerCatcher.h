#pragma once

#include "servo.h"


namespace driver::esp32 {


class PincerCatcher {
    Servo servoLeft;
    Servo servoRight;
public:
    PincerCatcher(Servo servoLeft, Servo servoRight):
        servoLeft(servoLeft),
        servoRight(servoRight)
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
