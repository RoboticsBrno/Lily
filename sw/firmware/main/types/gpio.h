#pragma once

#include <concepts>

template<typename Gpio>
concept isGpio = requires(Gpio pin) {
    Gpio::Edge::Rising;
    Gpio::Edge::Falling;
    Gpio::Edge::Both;

    Gpio::Direction::Disable;
    Gpio::Direction::Input;
    Gpio::Direction::Output;

    Gpio::Pull::None;
    Gpio::Pull::Up;
    Gpio::Pull::Down;

    pin.setDirection(Gpio::Direction::Disable);
    pin.setDirection(Gpio::Direction::Input);
    pin.setDirection(Gpio::Direction::Output);

    pin.setPull(Gpio::Pull::None);
    pin.setPull(Gpio::Pull::Up);
    pin.setPull(Gpio::Pull::Down);

    pin.enableInterrupt(Gpio::Edge::Rising);
    pin.enableInterrupt(Gpio::Edge::Falling);
    pin.enableInterrupt(Gpio::Edge::Both);
    pin.disableInterrupt();

    pin.onInterrupt([]() {});

    { pin.read() } -> std::same_as<bool>;
    pin.write(true);
    pin.write(false);
};
