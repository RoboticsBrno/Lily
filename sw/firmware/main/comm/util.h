#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>
#include <vector>


namespace comm {


template <typename T>
void appendLe(std::vector<uint8_t>& out, T value) {
    static_assert(std::is_trivially_copyable_v<T>);

    std::array<uint8_t, sizeof(T)> bytes {};
    std::memcpy(bytes.data(), &value, sizeof(T));

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes.begin(), bytes.end());
#endif
    out.insert(out.end(), bytes.begin(), bytes.end());
}


template <typename T>
bool readLe(std::span<const uint8_t> data, size_t& offset, T& out) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (offset + sizeof(T) > data.size()) {
        return false;
    }

    std::array<uint8_t, sizeof(T)> bytes {};
    std::memcpy(bytes.data(), data.data() + offset, sizeof(T));

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes.begin(), bytes.end());
#endif
    std::memcpy(&out, bytes.data(), sizeof(T));
    offset += sizeof(T);
    return true;
}


} // namespace comm
