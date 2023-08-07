#pragma once

#include <cstdint>
#include <span>
#include <cstddef>

#include "ptypes.h"


enum class CommandIds : uint8_t {
    CLEAR_SCREEN = 0x01,
    CLEAR_LAYER = 0x02,
    DRAW_POINT = 0x10,
    DRAW_LINE = 0x11,
    DRAW_RECTANGLE = 0x12,
    DRAW_CIRCLE = 0x13,
};


class ClearScreen {
public:
    static constexpr CommandIds id = CommandIds::CLEAR_SCREEN;

    static constexpr size_t toBytes(std::span<uint8_t> bytes) {
        bytes[0] = uint8_t(id);
        return 1;
    }
};


class ClearLayer {
public:
    static constexpr CommandIds id = CommandIds::CLEAR_LAYER;

    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint8_t layer) {
        bytes[0] = uint8_t(id);
        size_t size = 1;

        size += ProtocolType<uint8_t>::toBytes(bytes.subspan(size), layer);

        return size;
    }
};


class DrawPoint {
public:
    static constexpr CommandIds id = CommandIds::DRAW_POINT;

    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint16_t x, uint16_t y, uint8_t layer, Rgb332 color, uint16_t radius) {
        bytes[0] = uint8_t(id);
        size_t size = 1;

        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y);
        size += ProtocolType<uint8_t>::toBytes(bytes.subspan(size), layer);
        size += ProtocolType<Rgb332>::toBytes(bytes.subspan(size), color);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), radius);

        return size;
    }
};


class DrawLine {
public:
    static constexpr CommandIds id = CommandIds::DRAW_LINE;

    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t layer, Rgb332 color, uint16_t width) {
        bytes[0] = uint8_t(id);
        size_t size = 1;

        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x1);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y1);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x2);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y2);
        size += ProtocolType<uint8_t>::toBytes(bytes.subspan(size), layer);
        size += ProtocolType<Rgb332>::toBytes(bytes.subspan(size), color);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), width);

        return size;
    }
};


class DrawRectangle {
public:
    static constexpr CommandIds id = CommandIds::DRAW_RECTANGLE;

    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t layer, Rgb332 color, uint16_t width) {
        bytes[0] = uint8_t(id);
        size_t size = 1;

        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x1);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y1);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x2);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y2);
        size += ProtocolType<uint8_t>::toBytes(bytes.subspan(size), layer);
        size += ProtocolType<Rgb332>::toBytes(bytes.subspan(size), color);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), width);

        return size;
    }
};


class DrawCircle {
public:
    static constexpr CommandIds id = CommandIds::DRAW_CIRCLE;

    static constexpr size_t toBytes(std::span<uint8_t> bytes, uint16_t x, uint16_t y, uint16_t r, uint8_t layer, Rgb332 color, uint16_t width) {
        bytes[0] = uint8_t(id);
        size_t size = 1;

        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), x);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), y);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), r);
        size += ProtocolType<uint8_t>::toBytes(bytes.subspan(size), layer);
        size += ProtocolType<Rgb332>::toBytes(bytes.subspan(size), color);
        size += ProtocolType<uint16_t>::toBytes(bytes.subspan(size), width);

        return size;
    }
};
