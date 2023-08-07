#pragma once

#include <vector>
#include <cstddef>


#define EQUAL_ITERABLE(a, b) { REQUIRE(std::vector(a.begin(), a.end()) == std::vector(b.begin(), b.end())); }

template <typename T>
auto rangeVector(T start, size_t count) {
    static_assert(std::is_integral_v<T>);
    std::vector<T> result;
    while (count > 0) {
        result.push_back(start);
        ++start;
        --count;
    }
    return result;
}
