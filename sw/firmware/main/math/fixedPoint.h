#pragma once

#include <ratio>
#include <cmath>

#include "util/concepts.h"
#include "lmath.h"


namespace fp {


template<typename T, isRatio Ratio>
class FPNum {
    T _value;
public:
    struct raw_tag_t {};
    struct ratio_tag_t {};

    static constexpr raw_tag_t raw_tag{};
    static constexpr ratio_tag_t convert_tag{};

    FPNum(T value, raw_tag_t) : _value(value) {}
    FPNum(T value, ratio_tag_t) : _value(value * Ratio::den / Ratio::num) {}

    FPNum(FPNum const&) = default;
    FPNum(FPNum&&) = default;

    FPNum& operator=(FPNum const&) = default;
    FPNum& operator=(FPNum&&) = default;

    T value() const {
        return _value;
    }

    FPNum operator+(FPNum const& other) const {
        return FPNum(_value + other._value);
    }

    FPNum operator-(FPNum const& other) const {
        return FPNum(_value - other._value);
    }

    FPNum operator*(FPNum const& other) const {
        return FPNum(_value * other._value / Ratio::den);
    }

    FPNum operator/(FPNum const& other) const {
        return FPNum((_value * Ratio::den) / other._value);
    }

    FPNum operator%(FPNum const& other) const {
        return FPNum(_value % other._value);
    }

    auto operator<=>(FPNum const& other) const = default;

    FPNum operator-() const {
        return FPNum(-_value);
    }

    FPNum operator+() const {
        return *this;
    }

    FPNum sin() const {
        return FPNum(lmath::sin(static_cast<float>(_value) * Ratio::num / Ratio::den));
    }

    FPNum cos() const {
        return FPNum(lmath::cos(static_cast<float>(_value) * Ratio::num / Ratio::den));
    }

    FPNum atan2(FPNum const& other) const {
        return FPNum(lmath::atan2(static_cast<float>(_value) * Ratio::num / Ratio::den, static_cast<float>(other._value) * Ratio::num / Ratio::den));
    }

    FPNum sqrt() const {
        return FPNum(lmath::sqrt(_value * Ratio::num / Ratio::den));
    }

    bool isZero() const {
        return _value == 0;
    }

    template<isRatio R>
    explicit operator FPNum<T, R>() const {
        return FPNum<T, R>(_value * (R::den * Ratio::num) / (R::num * Ratio::den));
    }

    explicit operator float() const {
        return static_cast<float>(_value) * Ratio::num / Ratio::den;
    }

    static FPNum fromFloat(float value) {
        return FPNum(std::llround(value * Ratio::den / Ratio::num), raw_tag);
    }
};


using FPUnit = FPNum<int, std::ratio<1, 1>>;
using FPMilli = FPNum<int, std::milli>;
using FPMicro = FPNum<int, std::micro>;

using FPBinDec = FPNum<int, std::ratio<1, 1024>>;


namespace literals {


FPUnit operator""_fp(unsigned long long value) {
    return FPUnit(value, FPUnit::convert_tag);
}
FPUnit operator""_rfp(unsigned long long value) {
    return FPUnit(value, FPUnit::raw_tag);
}
FPMilli operator""_fp(long double value) {
    return FPMilli::fromFloat(value);
}

FPMilli operator""_rfpm(unsigned long long value) {
    return FPMilli(value, FPMilli::raw_tag);
}
FPMilli operator""_fpm(unsigned long long value) {
    return FPMilli(value, FPMilli::convert_tag);
}
FPMilli operator""_fpm(long double value) {
    return FPMilli::fromFloat(value);
}

FPMicro operator""_rfpu(unsigned long long value) {
    return FPMicro(value, FPMicro::raw_tag);
}
FPMicro operator""_fpu(unsigned long long value) {
    return FPMicro(value, FPMicro::convert_tag);
}
FPMicro operator""_fpu(long double value) {
    return FPMicro::fromFloat(value);
}

FPBinDec operator""_rfpb(unsigned long long value) {
    return FPBinDec(value, FPBinDec::raw_tag);
}

FPBinDec operator""_fpb(unsigned long long value) {
    return FPBinDec(value, FPBinDec::convert_tag);
}

FPBinDec operator""_fpb(long double value) {
    return FPBinDec::fromFloat(value);
}


} // namespace literals


} // namespace fp
