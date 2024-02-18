#pragma once

#include <utility>
#include <tuple>

#include "types/tickEncoder.h"
#include "types/gpio.h"


namespace driver {


template<isGPIO GPIO>
class TickEncoder {
    GPIO pinA;
    GPIO pinB;

    unsigned int ticks = 0;
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

    template<typename... Args, size_t... Is>
    TickEncoder(std::tuple<Args...>&& argsA, std::tuple<Args...>&& argsB, std::index_sequence<Is...>):
        pinA(std::get<Is>(std::forward<std::tuple<Args...>>(argsA))...),
        pinB(std::get<Is>(std::forward<std::tuple<Args...>>(argsB))...)
    {
        init();
    }

    void init() {
        // TODO: initialize pins
    }

public:
    TickEncoder(GPIO pinA, GPIO pinB):
        pinA(pinA),
        pinB(pinB)
    {
        init();
    }

    template<typename... Args>
    TickEncoder(std::tuple<Args...>&& argsA, std::tuple<Args...>&& argsB):
        TickEncoder(
            std::forward<std::tuple<Args...>>(argsA),
            std::forward<std::tuple<Args...>>(argsB),
            std::index_sequence_for<Args...>()
        )
    {}

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
