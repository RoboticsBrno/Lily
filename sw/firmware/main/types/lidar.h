#pragma once

#include <concepts>
#include <optional>
#include <ratio>


namespace detail {

    template<typename T>
    concept isRatio = std::ratio<T::num, T::den>::den != 0;
}


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
&& detail::isRatio<typename std::decay_t<Lidar>::DistanceUnit>
&& detail::isRatio<typename std::decay_t<Lidar>::AngleUnit>;
