#include "pin_logic.hpp"
#include "hardware/gpio.h"
#include <vector>

// Initialize GPIO pins for the three phases
void initialize_pins() {
    const std::vector<unsigned> pins = {
        PHASE_A_PIN_H, PHASE_A_PIN_L,
        PHASE_B_PIN_H, PHASE_B_PIN_L,
        PHASE_C_PIN_H, PHASE_C_PIN_L
    };

    for (unsigned pin : pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0); // Initially, set each pin to low state
    }
}

// Set the high and low states of each phase’s GPIO pins
void set_inverter_pins_(bool v_A, bool v_B, bool v_C) {
    // Control Phase A
    if (v_A) {
        gpio_put(PHASE_A_PIN_L, 0);  // Ensure the low pin is off first
        gpio_put(PHASE_A_PIN_H, 1);  // Set the high pin
    } else {
        gpio_put(PHASE_A_PIN_H, 0);  // Turn off the high pin
        gpio_put(PHASE_A_PIN_L, 1);  // Set the low pin
    }

    // Control Phase B
    if (v_B) {
        gpio_put(PHASE_B_PIN_L, 0);
        gpio_put(PHASE_B_PIN_H, 1);
    } else {
        gpio_put(PHASE_B_PIN_H, 0);
        gpio_put(PHASE_B_PIN_L, 1);
    }

    // Control Phase C
    if (v_C) {
        gpio_put(PHASE_C_PIN_L, 0);
        gpio_put(PHASE_C_PIN_H, 1);
    } else {
        gpio_put(PHASE_C_PIN_H, 0);
        gpio_put(PHASE_C_PIN_L, 1);
    }
}

// Turn off all GPIO pins for the three phases
void set_inverter_off_() {
    gpio_put(PHASE_A_PIN_H, 0);
    gpio_put(PHASE_A_PIN_L, 0);
    gpio_put(PHASE_B_PIN_H, 0);
    gpio_put(PHASE_B_PIN_L, 0);
    gpio_put(PHASE_C_PIN_H, 0);
    gpio_put(PHASE_C_PIN_L, 0);
}