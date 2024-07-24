#pragma once

#include <concepts>
#include <optional>
#include <ratio>

#include "../util/concepts.h"


struct Measurement {
    uint16_t distance;
    uint16_t angle;
    uint8_t quality;
};


template<typename Lidar>
concept isLidar = requires(Lidar lidar) {
    lidar.start();
    lidar.stop();

    { lidar.getMeasurement() } -> std::same_as<std::optional<Measurement>>;
}
&& isRatio<typename std::decay_t<Lidar>::DistanceUnit>
&& isRatio<typename std::decay_t<Lidar>::AngleUnit>;
