#pragma once

#include <functional>
#include <memory>

#include "driver/gpio.h"


namespace driver::esp32 {


class Gpio {
    using InterruptHandler = std::function<void()>;

    gpio_num_t pin;
    std::unique_ptr<InterruptHandler> interruptHandler;

    static void onInterrupt(void* handler) {
        static_cast<InterruptHandler*>(handler)->operator()();
    }

public:
    enum class Edge {
        Rising = GPIO_INTR_POSEDGE,
        Falling = GPIO_INTR_NEGEDGE,
        Both = GPIO_INTR_ANYEDGE
    };

    enum class Direction {
        Disable = GPIO_MODE_DISABLE,
        Input = GPIO_MODE_INPUT,
        Output = GPIO_MODE_OUTPUT
    };

    enum class Pull {
        None = GPIO_FLOATING,
        Up = GPIO_PULLUP_ONLY,
        Down = GPIO_PULLDOWN_ONLY
    };

    Gpio(gpio_num_t pin):
        pin(pin)
    {
        gpio_config_t config;
        config.pin_bit_mask = 1 << pin;
        config.mode = GPIO_MODE_DISABLE;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;

        gpio_config(&config);
    }

    Gpio(const Gpio&) = delete;
    Gpio(Gpio&& other):
        pin(other.pin),
        interruptHandler(std::move(other.interruptHandler))
    {
        other.pin = GPIO_NUM_NC;
    }

    ~Gpio() {
        gpio_reset_pin(pin);
        disableInterrupt();
    }

    void setDirection(Direction direction) {
        if (direction == Direction::Disable) {
            gpio_reset_pin(pin);
        } else {
            gpio_set_direction(pin, static_cast<gpio_mode_t>(direction));
        }
    }

    void setPull(Pull pull) {
        gpio_set_pull_mode(pin, static_cast<gpio_pull_mode_t>(pull));
    }

    void enableInterrupt(Edge edge) {
        gpio_set_intr_type(pin, static_cast<gpio_int_type_t>(edge));
        gpio_isr_handler_add(pin, &onInterrupt, interruptHandler.get());
    }

    void disableInterrupt() {
        gpio_set_intr_type(pin, GPIO_INTR_DISABLE);
        gpio_isr_handler_remove(pin);
    }

    void onInterrupt(std::function<void()> handler) {
        interruptHandler = std::make_unique<InterruptHandler>(std::move(handler));
    }

    bool read() {
        return gpio_get_level(pin);
    }

    void write(bool value) {
        gpio_set_level(pin, value);
    }
};


} // namespace driver::esp32
