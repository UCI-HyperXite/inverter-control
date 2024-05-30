#ifndef PIN_LOGIC_HPP
#define PIN_LOGIC_HPP

const unsigned PIN_LOGIC = 28;
const unsigned PIN_ENABLE = 14;

const unsigned pin_H = 28;
const unsigned pin_L = 14;

constexpr bool TEST_CIRCUIT = false;

void initialize_pins();

void set_logic_pin_(bool v);

void set_hilo_pins_(bool v);

void set_logic_off_();

void set_hilo_pins_off_();

static constexpr auto& set_inverter_pins_ = TEST_CIRCUIT ? set_logic_pin_ : set_hilo_pins_;
static constexpr auto& set_inverter_off_ = TEST_CIRCUIT ? set_logic_off_ : set_hilo_pins_off_;

#endif