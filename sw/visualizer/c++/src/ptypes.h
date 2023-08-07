#pragma once

#include <span>
#include <cstddef>
#include <cstdint>
#include <string>


enum class ProtocolTypeId : uint8_t {
    NIL = 0x00,
    UINT8 = 0x01,
    INT8 = 0x02,
    UINT16 = 0x03,
    INT16 = 0x04,
    UINT32 = 0x05,
    INT32 = 0x06,
    STRING = 0x07,
    RGB332 = 0x08,
    TYPE = 0x09,
};


template<typename T>
struct ProtocolType {};

struct Nil {};

template<>
struct ProtocolType<Nil> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, Nil value) {
        return 0;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::NIL;
};

template<>
struct ProtocolType<uint8_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint8_t value) {
        bytes[0] = uint8_t(value);
        return 1;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::UINT8;
};

template<>
struct ProtocolType<int8_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, int8_t value) {
        bytes[0] = uint8_t(value);
        return 1;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::INT8;
};

template<>
struct ProtocolType<uint16_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint16_t value) {
        bytes[0] = uint8_t(value >> 8);
        bytes[1] = uint8_t(value);
        return 2;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::UINT16;
};

template<>
struct ProtocolType<int16_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, int16_t value) {
        bytes[0] = uint8_t(value >> 8);
        bytes[1] = uint8_t(value);
        return 2;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::INT16;
};

template<>
struct ProtocolType<uint32_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint32_t value) {
        bytes[0] = uint8_t(value >> 24);
        bytes[1] = uint8_t(value >> 16);
        bytes[2] = uint8_t(value >> 8);
        bytes[3] = uint8_t(value);
        return 4;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::UINT32;
};

template<>
struct ProtocolType<int32_t> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, int32_t value) {
        bytes[0] = uint8_t(value >> 24);
        bytes[1] = uint8_t(value >> 16);
        bytes[2] = uint8_t(value >> 8);
        bytes[3] = uint8_t(value);
        return 4;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::INT32;
};

template<>
struct ProtocolType<std::string> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, const std::string& value) {
        size_t i = 0;
        for (auto c : value) {
            bytes[i++] = uint8_t(c);
        }
        return i;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::STRING;
};

struct Rgb332 {
    uint8_t r : 3;
    uint8_t g : 3;
    uint8_t b : 2;
};

template<>
struct ProtocolType<Rgb332> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, Rgb332 value) {
        bytes[0] = uint8_t(value.r << 5 | value.g << 2 | value.b);
        return 1;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::RGB332;
};


struct Type {
    ProtocolTypeId code;
};

template<>
struct ProtocolType<Type> {
    static constexpr size_t toBytes(std::span<uint8_t> bytes, Type value) {
        bytes[0] = uint8_t(value.code);
        return 1;
    }

    static constexpr ProtocolTypeId id = ProtocolTypeId::TYPE;
};
