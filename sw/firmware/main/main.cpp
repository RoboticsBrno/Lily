#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>
#include <span>

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "./comm/binary_serializer.h"
#include "./comm/uart_transport.h"
#include "robot.h"
#include "test.h"

constexpr const char* LOG_TAG = "robot_cmd";

constexpr RegParams robutekV2Regulator = {
    .kp = 7000,
    .ki = 350,
    .kd = 400,
    .kv = 166,
    .ka = 16000,
    .kc = 72,
    .maxIOut = 1023,
    .unwindFactor = 1,
};

auto lily = []() {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 64000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ledc_timer_config(&timer_conf);

    return Robot(
        std::make_unique<DCMotor>(GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_40, GPIO_NUM_39, robutekV2Regulator, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1),
        std::make_unique<DCMotor>(GPIO_NUM_45, GPIO_NUM_13, GPIO_NUM_41, GPIO_NUM_42, robutekV2Regulator, LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_CHANNEL_3),
        Claws(),
        RpLidar(
            UART_NUM_1,
            GPIO_NUM_9,
            GPIO_NUM_17,
            GPIO_NUM_1,
            LEDC_CHANNEL_4,
            LEDC_TIMER_0
        )
    );
}();

extern "C" void app_main() {
    lily.start();
    // test::robot(lily);
    // return;

    comm::UartTransport transport(UART_NUM_0, 921600, 10240, 10240);

    bool subscribed = false;

    transport.setReceiveCallback([&](std::span<const uint8_t> payload) {
        auto command = comm::BinarySerializer::deserializeCommand(payload);
        if (!command) {
            ESP_LOGW(LOG_TAG, "Failed to parse command payload, size=%u", payload.size());
            return;
        }

        switch (command->type) {
            case comm::CommandType::Move: {
                lily.motorLeft().setSpeed(command->leftSpeed);
                lily.motorRight().setSpeed(command->rightSpeed);

                if (command->leftSpeed == 0) {
                    lily.motorLeft().stop(false);
                }
                else {
                    lily.motorLeft().moveInfinite();
                }

                if (command->rightSpeed == 0) {
                    lily.motorRight().stop(false);
                }
                else {
                    lily.motorRight().moveInfinite();
                }
                break;
            }
            case comm::CommandType::Claw:
                if (command->clawOpen) {
                    lily.claws().setPos(0);
                }
                else {
                    lily.claws().setPos(100);
                }
                break;
            case comm::CommandType::Subscribe:
                ESP_LOGD(LOG_TAG, "Subscribe command received: enabling telemetry stream");
                subscribed = true;
                break;
            default:
                ESP_LOGW(LOG_TAG, "Unhandled command type=%d", static_cast<int>(command->type));
                break;
        }
    });

    int64_t lastMeasurementUs = 0;

    while (true) {
        int64_t nowUs = esp_timer_get_time();
        if (subscribed && nowUs - lastMeasurementUs >= 50 * 1000) {
            lastMeasurementUs = nowUs;

            comm::Measurements measurements;
            measurements.timestamp = nowUs;
            measurements.lidar.reserve(128);

            while (measurements.lidar.size() < measurements.lidar.capacity()) {
                auto lidarMeasurement = lily.lidar().getMeasurement();
                if (!lidarMeasurement.has_value()) {
                    break;
                }

                measurements.lidar.push_back(comm::LidarMeasurement {
                    .angle = lidarMeasurement->angle,
                    .distance = lidarMeasurement->distance,
                });
            }

            measurements.encoders = {
                .leftTicks = static_cast<int32_t>(lily.motorLeft().getPosition()),
                .rightTicks = static_cast<int32_t>(lily.motorRight().getPosition()),
            };

            auto payload = comm::BinarySerializer::serializeMeasurements(measurements);
            transport.send(std::span<const uint8_t>(payload));
        }

        vTaskDelay(1);
    }
}
