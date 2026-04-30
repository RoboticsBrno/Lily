#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "./messages.h"
#include "util.h"
#include "esp_log.h"


namespace comm {


class BinarySerializer {
    static constexpr uint8_t COMMAND_MOVE = 1;
    static constexpr uint8_t COMMAND_CLAW = 2;
    static constexpr uint8_t COMMAND_SUBSCRIBE = 3;

public:
    static std::optional<Command> deserializeCommand(std::span<const uint8_t> data) {
        if (data.empty()) {
            return std::nullopt;
        }

        size_t offset = 1;
        const uint8_t commandType = data[0];

        if (commandType == COMMAND_MOVE) {
            Command command;
            command.type = CommandType::Move;
            if (!readLe(data, offset, command.leftPower)) {
                return std::nullopt;
            }
            if (!readLe(data, offset, command.rightPower)) {
                return std::nullopt;
            }
            if (offset != data.size()) {
                return std::nullopt;
            }
            return command;
        }

        if (commandType == COMMAND_CLAW) {
            uint8_t rawAction = 0;
            if (!readLe(data, offset, rawAction) || offset != data.size()) {
                return std::nullopt;
            }

            Command command;
            command.type = CommandType::Claw;
            command.clawOpen = (rawAction != 0);
            return command;
        }

        if (commandType == COMMAND_SUBSCRIBE) {
            if (offset != data.size()) {
                return std::nullopt;
            }
            Command command;
            command.type = CommandType::Subscribe;
            return command;
        }

        return std::nullopt;
    }

    static std::vector<uint8_t> serializeMeasurements(const Measurements& measurements) {
        std::vector<uint8_t> payload;
        payload.reserve(8 + 2 + measurements.lidar.size() * (2 + 2) + (4 + 4));

        appendLe<int64_t>(payload, measurements.timestamp);
        appendLe<uint16_t>(payload, measurements.lidar.size());
        for (const auto& measurement : measurements.lidar) {
            appendLe<uint16_t>(payload, measurement.angle);
            appendLe<uint16_t>(payload, measurement.distance);
        }

        appendLe<int32_t>(payload, measurements.encoders.leftTicks);
        appendLe<int32_t>(payload, measurements.encoders.rightTicks);

        return payload;
    }
};


} // namespace comm
