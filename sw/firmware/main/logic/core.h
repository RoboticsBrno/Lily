#pragma once

#include "math/geometry.h"
#include "math/curves.h"
#include "math/fixedPoint.h"
#include "math/lmath.h"


namespace logic {


using num = fp::FPBinDec;

using Point = linalg::Point<num>;
using Vector = linalg::Vector<num>;
using Line = linalg::Line<num>;


num operator""_n(unsigned long long value) {
    return fp::literals::operator""_fpb(value);
}

num operator""_n(long double value) {
    return fp::literals::operator""_fpb(value);
}


} // namespace logic
