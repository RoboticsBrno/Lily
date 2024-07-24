#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "creator.h"

#include "logic/game.h"
#include "test/hw.h"


// auto lily = createRobot();
auto lily = createRobot();

int main() {

}


extern "C" void app_main() {
    uart_config_t config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &config);

    main();
}
