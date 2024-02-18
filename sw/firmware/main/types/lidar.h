#pragma once

#include <concepts>
#include <ratio>


namespace detail {

    template<typename T>
    concept isRatio = std::ratio<T::num, T::den>::den != 0;
}


struct Measurement {
    uint16_t distance;
    uint16_t angle;
};


template<typename Lidar>
concept isLidar = requires(Lidar lidar) {
    lidar.start();
    lidar.stop();

    { lidar.available() } -> std::same_as<unsigned>;
    { lidar.getMeasurement() } -> std::same_as<Measurement>;
}
&& detail::isRatio<typename Lidar::DistanceUnit>
&& detail::isRatio<typename Lidar::AngleUnit>;
