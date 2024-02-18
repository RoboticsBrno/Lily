#pragma once

#include <concepts>
#include <utility>


enum class OverflowFlag {
    None,
    Overflow,
    Underflow
};


template<typename TickEncoder>
concept isTickEncoder = requires(TickEncoder tickEncoder) {
    { tickEncoder.getTicks() } -> std::same_as<std::pair<unsigned, OverflowFlag>>;

    tickEncoder.resetOverflow();
    tickEncoder.reset();
};
