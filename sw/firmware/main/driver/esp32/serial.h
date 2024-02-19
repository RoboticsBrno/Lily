#pragma once

#include "driver/uart.h"


namespace driver::esp32 {


class Serial {
    uart_port_t _uart;
public:
    Serial(gpio_num_t tx, gpio_num_t rx, int baudRate, uart_port_t uart):
        _uart(uart)
    {
        uart_config_t config {
            .baud_rate = baudRate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,
        };
        uart_param_config(uart, &config);

        uart_set_pin(uart, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart, 1024, 0, 0, nullptr, 0);
    }

    Serial(Serial const&) = delete;
    Serial(Serial&& other):
        _uart(other._uart)
    {
        other._uart = UART_NUM_MAX;
    }

    ~Serial() {
        if (_uart != UART_NUM_MAX) {
            uart_driver_delete(_uart);
        }
    }

    void write(uint8_t data) {
        uart_write_bytes(_uart, &data, 1);
    }

    int read() {
        uint8_t data;
        int read = uart_read_bytes(_uart, &data, 1, 0);
        if (read == 1) {
            return data;
        }
        return -1;
    }

    unsigned available() {
        size_t available;
        uart_get_buffered_data_len(_uart, &available);
        return available;
    }
};


} // namespace driver::esp32
