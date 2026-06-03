#pragma once

#include <cstdint>
#include <vector>
#include "../driver/rpLidar.h"


namespace comm {


enum class CommandType {
    Move,
    Claw,
    Arm,
};


struct Command {
    CommandType type = CommandType::Arm;
    int16_t leftSpeed = 0;
    int16_t rightSpeed = 0;
    int16_t clawPwm = 0;
};


using LidarMeasurement = Measurement;


struct EncodersMeasurement {
    int32_t leftTicks = 0;
    int32_t rightTicks = 0;
};


struct Measurements {
    int64_t timestamp = 0;
    std::vector<LidarMeasurement> lidar;
    EncodersMeasurement encoders;
};


} // namespace comm
