#ifndef PIN_LOGIC_HPP
#define PIN_LOGIC_HPP

#include "pico/stdlib.h"

// GPIO pin assignments for each phase
// PHASE_X_PIN_H: High-side pin for Phase X
// PHASE_X_PIN_L: Low-side pin for Phase X
const unsigned PHASE_A_PIN_H = 28;
const unsigned PHASE_A_PIN_L = 14;
const unsigned PHASE_B_PIN_H = 27;
const unsigned PHASE_B_PIN_L = 13;
const unsigned PHASE_C_PIN_H = 26;
const unsigned PHASE_C_PIN_L = 12;

// Iinitialize all GPIO pins for three-phase SPDM signals
void initialize_pins();

// Set the high and low states of each phase’s GPIO pins (true for high, false for low): 
void set_inverter_pins_(bool v_A, bool v_B, bool v_C);

// Disable the inverter by setting all GPIO outputs to low
void set_inverter_off_();

#endif