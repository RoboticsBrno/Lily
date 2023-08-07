#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <span>


/**
 * @brief Encodes data using COBS algorithm in-place
 *
 * @param data A buffer containing data to encode, must be padded with a byte at the start and end
 */
void cobs_encode(std::span<uint8_t> data) {
    data[0] = 0x00;
    data[data.size() - 1] = 0x00;

    int zeroIdx = 0;
    int distance = 1;

    for (int i = 1; i < data.size() - 1; ++i) {
        if (data[i] == 0x00) {
            data[zeroIdx] = distance;
            zeroIdx = i;
            distance = 1;
        } else {
            distance += 1;
        }

        if (distance == 0xFF) {
            throw std::runtime_error("COBS encode error: data too long");
        }
    }

    data[zeroIdx] = distance;
}


/**
 * @brief Decodes data using COBS algorithm in-place
 *
 * @param data A buffer containing data to decode
 */
std::span<uint8_t> cobs_decode(std::span<uint8_t> data) {
    if (data.empty()) {
        return {};
    }

    if (data.back() != 0x00) {
        throw std::runtime_error("COBS decode error: unexpected end of data");
    }

    int zeroIdx = 0;

    for (int i = 0; i < data.size() - 1; ++i) {
        if (i == zeroIdx) {
            zeroIdx += data[i];
            data[i] = 0x00;
        }
    }

    return data.subspan(1, data.size() - 2);
}


class CobsStreamDecoder {
public:
    CobsStreamDecoder() {}

    std::vector<uint8_t> receive(uint8_t byte) {
        _buffer.push_back(byte);

        if (byte == 0x00) {
            auto decoded = cobs_decode(std::span<uint8_t>(_buffer));
            std::vector<uint8_t> result(decoded.begin(), decoded.end());
            _buffer.clear();
            return result;
        }

        return {};
    }

    void reset() {
        _buffer.clear();
    }

private:
    std::vector<uint8_t> _buffer;
};
