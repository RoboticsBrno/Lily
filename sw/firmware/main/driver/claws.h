#pragma once

#include <iostream>

class Claws {
public:
    Claws() = default;

    Claws(Claws const&) = delete;
    Claws(Claws&& other) = default;

    void setPos(int pos) {
        std::cout << "Claws: setPos(" << pos << ')' << std::endl;
    }

    int getPos() {
        std::cout << "Claws: getPos() -> 0" << std::endl;
        return 0;
    }
};
