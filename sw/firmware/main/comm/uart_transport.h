#pragma once

#include <functional>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>
#include <array>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_crc.h"

#include "util.h"


namespace comm {

static constexpr const char* UART_TRANSPORT_LOG_TAG = "uart_transport";


class UartTransport {
public:
    using ReceiveCallback = std::function<void(std::span<const uint8_t>)>;

private:
    static constexpr uint8_t FRAME_INIT = 0xA5;
    static constexpr unsigned MAX_PAYLOAD_SIZE = 2048;
    static constexpr unsigned HEADER_SIZE = 6;

    uart_port_t _uart;
    ReceiveCallback _receiveCallback;
    uint8_t _txNonce = 0;

    static uint8_t computeChecksum(std::span<const uint8_t> data) {
        return esp_rom_crc8_be(0, data.data(), data.size());
    }

    bool readExact(uint8_t* buffer, size_t size) {
        size_t offset = 0;
        while (offset < size) {
            const int read = uart_read_bytes(_uart, buffer + offset, size - offset, portMAX_DELAY);
            if (read <= 0) {
                return false;
            }
            offset += read;
        }
        return true;
    }

    void receiveLoop() {
        uint8_t initByte = 0;
        std::array<uint8_t, HEADER_SIZE - 1> header{};

        while (true) {
            if (!readExact(&initByte, 1) || initByte != FRAME_INIT) {
                continue;
            }

            if (!readExact(header.data(), header.size())) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Failed to read frame header");
                continue;
            }

            size_t offset = 0;
            const std::span<const uint8_t> headerSpan(header.data(), header.size());

            uint8_t nonce = 0;
            uint16_t size = 0;
            uint8_t payloadChecksum = 0;
            uint8_t headerChecksum = 0;

            if (!readLe(headerSpan, offset, nonce) ||
                !readLe(headerSpan, offset, size) ||
                !readLe(headerSpan, offset, payloadChecksum) ||
                !readLe(headerSpan, offset, headerChecksum)) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Failed to decode frame header");
                continue;
            }

            std::array<uint8_t, HEADER_SIZE - 1> checksumHeader{ {
                FRAME_INIT,
                nonce,
                static_cast<uint8_t>(size & 0xFF),
                static_cast<uint8_t>((size >> 8) & 0xFF),
                payloadChecksum,
            } };
            if (headerChecksum != computeChecksum(std::span<const uint8_t>(checksumHeader.data(), checksumHeader.size()))) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Header checksum mismatch");
                continue;
            }

            if (size > MAX_PAYLOAD_SIZE) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Dropped oversized payload size=%u max=%u", size, MAX_PAYLOAD_SIZE);
                continue;
            }

            std::vector<uint8_t> payload(size);
            if (size > 0 && !readExact(payload.data(), payload.size())) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Failed to read payload bytes size=%u", size);
                continue;
            }

            if (payloadChecksum != computeChecksum(std::span<const uint8_t>(payload.data(), payload.size()))) {
                ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Payload checksum mismatch");
                continue;
            }

            if (_receiveCallback) {
                _receiveCallback(std::span<const uint8_t>(payload.data(), payload.size()));
            }
        }
    }

public:
    UartTransport(uart_port_t uart, int baudRate, int rxBufferSize, int txBufferSize):
        _uart(uart)
    {
        uart_config_t config = {
            .baud_rate = baudRate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,
        };

        uart_param_config(_uart, &config);
        uart_driver_install(_uart, rxBufferSize, txBufferSize, 0, nullptr, 0);

        xTaskCreate(
            [](void* arg) {
                static_cast<UartTransport*>(arg)->receiveLoop();
            },
            "uart_rx", 4096, this, tskIDLE_PRIORITY + 1, nullptr
        );
    }

    void setReceiveCallback(ReceiveCallback callback) {
        _receiveCallback = std::move(callback);
    }

    void send(std::span<const uint8_t> payload) {
        if (payload.size() > MAX_PAYLOAD_SIZE) {
            ESP_LOGW(UART_TRANSPORT_LOG_TAG, "Send skipped: payload too large size=%u max=%u", payload.size(), MAX_PAYLOAD_SIZE);
            return;
        }

        ESP_LOGI(UART_TRANSPORT_LOG_TAG, "Sending frame size=%u", payload.size());

        std::vector<uint8_t> header;
        header.reserve(HEADER_SIZE);
        appendLe<uint8_t>(header, FRAME_INIT);
        appendLe<uint8_t>(header, _txNonce);
        appendLe<uint16_t>(header, payload.size());
        appendLe<uint8_t>(header, computeChecksum(payload));
        const uint8_t checksum = computeChecksum(std::span<const uint8_t>(header.data(), header.size()));
        appendLe<uint8_t>(header, checksum);

        uart_write_bytes(_uart, reinterpret_cast<const char*>(header.data()), header.size());
        if (!payload.empty()) {
            uart_write_bytes(_uart, reinterpret_cast<const char*>(payload.data()), payload.size());
        }

        _txNonce++;
    }
};


} // namespace comm
