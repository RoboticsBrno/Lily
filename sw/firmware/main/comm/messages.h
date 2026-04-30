#pragma once

#include <cstdint>
#include <vector>


namespace comm {


enum class CommandType {
    Move,
    Claw,
    Subscribe,
};


struct Command {
    CommandType type = CommandType::Subscribe;
    float leftPower = 0.0f;
    float rightPower = 0.0f;
    bool clawOpen = false;
};


struct LidarMeasurement {
    uint16_t angle = 0;
    uint16_t distance = 0;
};


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
