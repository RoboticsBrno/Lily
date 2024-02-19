#pragma once

#include <cmath>


namespace lmath {


template <typename T>
    requires requires(T val) { val.sin(); }
auto sin(T val) {
    return val.sin();
}

template <typename T>
auto sin(T val) {
    return std::sin(val);
}


template <typename T>
    requires requires(T val) { val.cos(); }
auto cos(T val) {
    return val.cos();
}

template <typename T>
auto cos(T val) {
    return std::cos(val);
}


template <typename T>
    requires requires(T val) { val.atan2(val); }
auto atan2(T y, T x) {
    return y.atan2(x);
}

template <typename T>
auto atan2(T y, T x) {
    return std::atan2(y, x);
}


template <typename T>
    requires requires(T val) { val.sqrt(); }
auto sqrt(T val) {
    return val.sqrt();
}

template <typename T>
auto sqrt(T val) {
    return std::sqrt(val);
}


template <typename T>
    requires requires(T val) { val.abs(); }
auto abs(T val) {
    return val.abs();
}

template <typename T>
auto abs(T val) {
    return std::abs(val);
}


template <typename T>
static T radians(T degrees) {
    return degrees * M_PI / 180;
}

template <typename T>
static T degrees(T radians) {
    return radians * 180 / M_PI;
}


} // namespace lmath
