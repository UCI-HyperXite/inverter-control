#include <vector>

#include "pico/stdlib.h"

#include "pin_logic.hpp"

void initialize_pins()
{
    const std::vector<unsigned> pins = TEST_CIRCUIT ?
        std::vector<unsigned>{PIN_LOGIC, PIN_ENABLE} :
        std::vector<unsigned>{ pin_H, pin_L };

    for (unsigned pin : pins)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }
}

void set_logic_pin_(bool v)
{
    gpio_put(PIN_LOGIC, v);
    gpio_put(PIN_ENABLE, 1);
}

void set_hilo_pins_(bool v)
{
    // Even though we should not need to manually introduce a delay (deadtime),
    // we should still ensure that the rise always occurs after the fall on the
    // GPIO pins for each phase.
    if (v)
    {
        gpio_put(pin_L, !v);
        gpio_put(pin_H, v);
    }
    else
    {
        gpio_put(pin_H, v);
        gpio_put(pin_L, !v);
    }
}

void set_logic_off_()
{
    gpio_put(PIN_LOGIC, 0);
    gpio_put(PIN_ENABLE, 0);
}

void set_hilo_pins_off_()
{
    gpio_put(pin_H, 0);
    gpio_put(pin_L, 0);
}
