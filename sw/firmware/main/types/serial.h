#pragma once

#include <concepts>


template<typename Serial>
concept isSerial = requires(Serial serial) {
    serial.write(0);
    { serial.read() } -> std::same_as<int>;
    { serial.available() } -> std::same_as<unsigned>;
};
