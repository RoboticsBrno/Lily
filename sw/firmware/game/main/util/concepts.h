#pragma once

#include <ratio>


template<typename T>
concept isRatio = std::ratio<T::num, T::den>::den != 0;
