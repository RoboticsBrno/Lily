#pragma once

#include <utility>
#include <tuple>

#include "../types/tickEncoder.h"
#include "../types/gpio.h"


namespace driver {


template<isGpio Gpio>
class TickEncoder {
    Gpio pinA;
    Gpio pinB;

    unsigned ticks = 0;
    OverflowFlag overflowFlag = OverflowFlag::None;

    void tickUp() {
        ticks++;
        if (ticks == 0) {
            if (overflowFlag == OverflowFlag::Underflow) {
                overflowFlag = OverflowFlag::None;
            }
            else if (overflowFlag == OverflowFlag::None) {
                overflowFlag = OverflowFlag::Overflow;
            }
            else {
                // XXX: report double overflow?
                overflowFlag = OverflowFlag::Overflow;
            }
        }
    }

    void tickDown() {
        ticks--;
        if (ticks == 0) {
            if (overflowFlag == OverflowFlag::Overflow) {
                overflowFlag = OverflowFlag::None;
            }
            else if (overflowFlag == OverflowFlag::None) {
                overflowFlag = OverflowFlag::Underflow;
            }
            else {
                // XXX: report double underflow?
                overflowFlag = OverflowFlag::Underflow;
            }
        }
    }

    void init() {
        // TODO: initialize pins
    }

public:
    TickEncoder(Gpio pinA, Gpio pinB):
        pinA(std::move(pinA)),
        pinB(std::move(pinB))
    {
        init();
    }

    std::pair<unsigned, OverflowFlag> getTicks() {
        return std::make_pair(ticks, overflowFlag);
    }

    void resetOverflow() {
        overflowFlag = OverflowFlag::None;
    }

    void reset() {
        ticks = 0;
        overflowFlag = OverflowFlag::None;
    }
};


} // namespace driver
