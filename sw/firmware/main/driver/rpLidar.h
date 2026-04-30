#pragma once

#include <cstdint>
#include <optional>
#include <ratio>
#include <array>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"


struct Measurement {
    uint16_t distance;
    uint16_t angle;
    uint8_t quality;
};


class RpLidar {
    uart_port_t _uart;
    ledc_channel_t _motorChannel;
    static constexpr int BAUD_RATE = 115200;
    static constexpr int RX_BUFFER_SIZE = 10240;
    static constexpr int TX_BUFFER_SIZE = 0;

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
        _motorChannel(motorChannel)
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

        ledc_channel_config_t channelConfig = {
            .gpio_num = motorPin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motorChannel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motorTimer,
            .duty = 0,
            .hpoint = 0,
            .flags = {},
        };
        ledc_channel_config(&channelConfig);
    }

    RpLidar(RpLidar const&) = delete;
    RpLidar(RpLidar&& other):
        _uart(other._uart),
        _motorChannel(other._motorChannel)
    {
        other._uart = UART_NUM_MAX;
    }

    ~RpLidar() {
        if (_uart != UART_NUM_MAX) {
            uart_driver_delete(_uart);
        }
    }

    void start() {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannel, 1024);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannel);

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
        std::array<uint8_t, 2> stop{{0xA5, 0x25}};
        uart_write_bytes(_uart, stop.data(), stop.size());
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    std::optional<Measurement> getMeasurement() {
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
            measurement.quality = data >> 2;

            uart_read_bytes(_uart, &data, 1, 0);
            if ((data & 0x01) == 0) {
                uart_get_buffered_data_len(_uart, &available);
                continue;
            }
            uint16_t angle = data >> 1;
            std::array<uint8_t, 3> payload{};
            uart_read_bytes(_uart, payload.data(), payload.size(), 0);
            angle |= static_cast<uint16_t>(payload[0]) << 7;

            uint16_t distance = payload[1];
            distance |= static_cast<uint16_t>(payload[2]) << 8;

            measurement.angle = angle;
            measurement.distance = distance;

            return measurement;
        }

        return std::nullopt;
    }
};
