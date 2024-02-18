#pragma once

#include <concepts>

template<typename GPIO>
concept isGPIO = requires(GPIO pin) {
    GPIO::Edge::Rising;
    GPIO::Edge::Falling;
    GPIO::Edge::Both;

    GPIO::Direction::Disable;
    GPIO::Direction::Input;
    GPIO::Direction::Output;

    GPIO::Pull::None;
    GPIO::Pull::Up;
    GPIO::Pull::Down;

    pin.setDirection(GPIO::Direction::Disable);
    pin.setDirection(GPIO::Direction::Input);
    pin.setDirection(GPIO::Direction::Output);

    pin.setPull(GPIO::Pull::None);
    pin.setPull(GPIO::Pull::Up);
    pin.setPull(GPIO::Pull::Down);

    pin.enableInterrupt(GPIO::Edge::Rising);
    pin.enableInterrupt(GPIO::Edge::Falling);
    pin.enableInterrupt(GPIO::Edge::Both);
    pin.disableInterrupt();

    pin.onInterrupt([]() {});

    { pin.read() } -> std::same_as<bool>;
    pin.write(true);
    pin.write(false);
};
