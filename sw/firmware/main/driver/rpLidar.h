#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

#define PWM_CONTROL 0


struct Measurement {
    uint16_t distanceQ2;
    uint16_t angleQ6;
};


struct ExpressCabin {
    uint16_t distance1;
    uint16_t distance2;
    int8_t dtheta1Q3;
    int8_t dtheta2Q3;

    static ExpressCabin parse(std::span<const uint8_t> data) {
        uint16_t d1 = (data[0] >> 2) | (static_cast<uint16_t>(data[1]) << 6);
        uint16_t d2 = (data[2] >> 2) | (static_cast<uint16_t>(data[3]) << 6);
        uint8_t rawDt1 = ((data[0] & 0x03) << 4) | (data[4] & 0x0F);
        uint8_t rawDt2 = ((data[2] & 0x03) << 4) | ((data[4] >> 4) & 0x0F);
        int8_t dt1 = static_cast<int8_t>(rawDt1 << 2) >> 2;
        int8_t dt2 = static_cast<int8_t>(rawDt2 << 2) >> 2;
        return {d1, d2, dt1, dt2};
    }
};


struct ParsedExpressPacket {
    uint16_t startAngleQ6;
    std::array<ExpressCabin, 16> cabins;
};


class RpLidar {
    uart_port_t _uart;
    ledc_channel_t _motorChannel;
    gpio_num_t _motorPin;
    std::optional<ParsedExpressPacket> _expressPrevPacket;

    static constexpr int BAUD_RATE = 115200;
    static constexpr int RX_BUFFER_SIZE = 10240;
    static constexpr int TX_BUFFER_SIZE = 0;
    static constexpr int EXPRESS_PACKET_SIZE = 84;
    static constexpr int CABINS_PER_PACKET = 16;
    static constexpr uint16_t FULL_CIRCLE_Q6 = 360 * 64;

public:
    RpLidar(
        uart_port_t uartUnit,
        gpio_num_t tx,
        gpio_num_t rx,
        gpio_num_t motorPin,
        ledc_channel_t motorChannel,
        ledc_timer_t motorTimer
    ):
        _uart(uartUnit),
        _motorChannel(motorChannel),
        _motorPin(motorPin)
    {
        uart_config_t config {
            .baud_rate = BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,
        };
        uart_param_config(_uart, &config);
        uart_set_pin(_uart, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(_uart, RX_BUFFER_SIZE, TX_BUFFER_SIZE, 0, nullptr, 0);

#if PWM_CONTROL
        ledc_channel_config_t channelConfig = {
            .gpio_num = _motorPin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _motorChannel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motorTimer,
            .duty = 0,
            .hpoint = 0,
            .flags = {},
        };
        ledc_channel_config(&channelConfig);
#else
        gpio_set_direction(_motorPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_motorPin, 0);
#endif
    }

    RpLidar(RpLidar const&) = delete;
    RpLidar(RpLidar&& other):
        _uart(other._uart),
        _motorChannel(other._motorChannel),
        _motorPin(other._motorPin),
        _expressPrevPacket(std::move(other._expressPrevPacket))
    {
        other._uart = UART_NUM_MAX;
    }

    void start() {
#if PWM_CONTROL
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannel, 1024);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannel);
#else
        gpio_set_level(_motorPin, 1);
#endif

        std::array<uint8_t, 2> startStop{{0xA5, 0x25}};
        uart_write_bytes(_uart, startStop.data(), startStop.size());
        vTaskDelay(pdMS_TO_TICKS(100));

        std::array<uint8_t, 2> reset{{0xA5, 0x40}};
        uart_write_bytes(_uart, reset.data(), reset.size());
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t data = 0;
        while (uart_read_bytes(_uart, &data, 1, 0) == 1) {}

        std::array<uint8_t, 2> scan{{0xA5, 0x20}};
        uart_write_bytes(_uart, scan.data(), scan.size());
    }

    void stop() {
        _expressPrevPacket.reset();

        std::array<uint8_t, 2> stop{{0xA5, 0x25}};
        uart_write_bytes(_uart, stop.data(), stop.size());

#if PWM_CONTROL
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannel);
#else
        gpio_set_level(_motorPin, 0);
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void startExpress() {
        _expressPrevPacket.reset();

#if PWM_CONTROL
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannel, 1024);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannel);
#else
        gpio_set_level(_motorPin, 1);
#endif

        std::array<uint8_t, 2> stopCmd{{0xA5, 0x25}};
        uart_write_bytes(_uart, stopCmd.data(), stopCmd.size());
        vTaskDelay(pdMS_TO_TICKS(100));

        std::array<uint8_t, 2> reset{{0xA5, 0x40}};
        uart_write_bytes(_uart, reset.data(), reset.size());
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t flushB = 0;
        while (uart_read_bytes(_uart, &flushB, 1, 0) == 1) {}

        std::array<uint8_t, 9> scan{{0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}};
        uart_write_bytes(_uart, scan.data(), scan.size());

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    std::optional<std::vector<Measurement>> getMeasurementsExpress() {
        auto packet = expressReadPacket();
        if (!packet) {
            return std::nullopt;
        }

        if (!_expressPrevPacket) {
            _expressPrevPacket = std::move(packet);
            return std::nullopt;
        }

        auto prevAngleQ6 = _expressPrevPacket->startAngleQ6;
        auto curAngleQ6 = packet->startAngleQ6;

        uint16_t angleDiff;
        if (curAngleQ6 >= prevAngleQ6) {
            angleDiff = curAngleQ6 - prevAngleQ6;
        } else {
            angleDiff = FULL_CIRCLE_Q6 + curAngleQ6 - prevAngleQ6;
        }

        std::vector<Measurement> result;
        result.reserve(CABINS_PER_PACKET * 2);

        float anglePerMeasurement = static_cast<float>(angleDiff) / (CABINS_PER_PACKET * 2);

        for (int i = 0; i < CABINS_PER_PACKET; ++i) {
            auto const& cabin = _expressPrevPacket->cabins[i];

            float angle1 = prevAngleQ6 + anglePerMeasurement * (i * 2) - cabin.dtheta1Q3 * 8.0f;
            float angle2 = prevAngleQ6 + anglePerMeasurement * (i * 2 + 1) - cabin.dtheta2Q3 * 8.0f;

            while (angle1 < 0) angle1 += FULL_CIRCLE_Q6;
            while (angle1 >= FULL_CIRCLE_Q6) angle1 -= FULL_CIRCLE_Q6;
            while (angle2 < 0) angle2 += FULL_CIRCLE_Q6;
            while (angle2 >= FULL_CIRCLE_Q6) angle2 -= FULL_CIRCLE_Q6;

            result.push_back({static_cast<uint16_t>(cabin.distance1 * 4), static_cast<uint16_t>(angle1)});
            result.push_back({static_cast<uint16_t>(cabin.distance2 * 4), static_cast<uint16_t>(angle2)});
        }

        _expressPrevPacket = std::move(packet);
        return result;
    }

    std::optional<std::array<Measurement, 1>> getMeasurement() {
        Measurement measurement;
        size_t available = 0;
        uart_get_buffered_data_len(_uart, &available);
        while (available >= 5) {
            uint8_t data = 0;
            uart_read_bytes(_uart, &data, 1, 0);
            if ((data & 0x01) == ((data >> 1) & 0x01)) {
                uart_get_buffered_data_len(_uart, &available);
                continue;
            }
            // uint8_t quality = data >> 2;

            uart_read_bytes(_uart, &data, 1, 0);
            if ((data & 0x01) == 0) {
                uart_get_buffered_data_len(_uart, &available);
                continue;
            }
            uint16_t angleQ6 = data >> 1;
            std::array<uint8_t, 3> payload{};
            uart_read_bytes(_uart, payload.data(), payload.size(), 0);
            angleQ6 |= static_cast<uint16_t>(payload[0]) << 7;

            uint16_t distanceQ2 = payload[1];
            distanceQ2 |= static_cast<uint16_t>(payload[2]) << 8;

            measurement.angleQ6 = angleQ6;
            measurement.distanceQ2 = distanceQ2;

            return std::array{ measurement };
        }

        return std::nullopt;
    }

    std::string getInfo() {
        // flush uart buffer
        uint8_t flushB = 0;
        while (uart_read_bytes(_uart, &flushB, 1, 0) == 1) {}

        std::array<uint8_t, 2> req{{0xA5, 0x50}};
        uart_write_bytes(_uart, req.data(), req.size());

        vTaskDelay(pdMS_TO_TICKS(50));

        // search for descriptor sync in the response stream
        uint8_t desc[7] = {};
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(200);
        int descIdx = 0;
        while (descIdx < 7 && xTaskGetTickCount() < deadline) {
            if (uart_read_bytes(_uart, desc + descIdx, 1, pdMS_TO_TICKS(10)) == 1) {
                if (descIdx == 0 && desc[0] != 0xA5) continue;
                if (descIdx == 1 && desc[1] != 0x5A) { descIdx = 0; continue; }
                descIdx++;
            }
        }

        if (descIdx < 7) {
            return "Error: no response descriptor";
        }

        uint32_t descHeader = desc[2] | (static_cast<uint32_t>(desc[3]) << 8)
                            | (static_cast<uint32_t>(desc[4]) << 16) | (static_cast<uint32_t>(desc[5]) << 24);
        uint32_t dataLen = descHeader & 0x3FFFFFFF;

        if (dataLen != 20) {
            return "Error: unexpected info length";
        }

        uint8_t info[20] = {};
        for (int i = 0; i < 20; i++) {
            if (uart_read_bytes(_uart, info + i, 1, pdMS_TO_TICKS(50)) != 1) {
                return "Error: incomplete info response";
            }
        }

        uint8_t model = info[0];
        uint8_t fwMinor = info[1];
        uint8_t fwMajor = info[2];
        uint8_t hw = info[3];

        char serialStr[33] = {};
        for (int i = 0; i < 16; i++) {
            snprintf(serialStr + i * 2, 3, "%02X", info[4 + i]);
        }

        char result[128] = {};
        snprintf(result, sizeof(result),
            "Model: %u, Firmware: %u.%u, Hardware: %u, Serial: %s",
            model, fwMajor, fwMinor, hw, serialStr);
        return std::string(result);
    }

    std::optional<ParsedExpressPacket> expressReadPacket() {
        size_t available = 0;
        uart_get_buffered_data_len(_uart, &available);

        while (available >= EXPRESS_PACKET_SIZE) {
            uint8_t sync1 = 0;
            uart_read_bytes(_uart, &sync1, 1, 0);
            if ((sync1 >> 4) != 0xA) {
                uart_get_buffered_data_len(_uart, &available);
                continue;
            }

            uint8_t sync2 = 0;
            uart_read_bytes(_uart, &sync2, 1, 0);
            if ((sync2 >> 4) != 0x5) {
                uart_get_buffered_data_len(_uart, &available);
                continue;
            }

            // ignored
            // uint8_t checksum = ((sync1 & 0x0F) << 4) | (sync2 & 0x0F);

            std::array<uint8_t, EXPRESS_PACKET_SIZE - 2> rest{};
            for (int i = 0; i < EXPRESS_PACKET_SIZE - 2; ++i) {
                while (uart_read_bytes(_uart, &rest[i], 1, pdMS_TO_TICKS(1)) != 1) {}
            }

            uint16_t startAngleQ6 = rest[0] | ((rest[1] & 0x7F) << 8);

            ParsedExpressPacket packet;
            packet.startAngleQ6 = startAngleQ6;

            for (int i = 0; i < CABINS_PER_PACKET; ++i) {
                packet.cabins[i] = ExpressCabin::parse(std::span<const uint8_t>(rest.data() + 2 + i * 5, 5));
            }

            return packet;
        }

        return std::nullopt;
    }
};
