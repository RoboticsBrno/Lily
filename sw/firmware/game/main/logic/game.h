#pragma once

#include "../types/robot.h"

#include "./core.h"

namespace logic {


template<isRobot Robot>
class Game {
    Robot& _robot;
public:
    Game(Robot& robot) : _robot(robot) {}

    void run() {

    }
};


} // namespace logic
