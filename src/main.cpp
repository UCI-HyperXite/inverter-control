#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cmath>
#include <vector>

const unsigned PIN_LOGIC = 28;
const unsigned PIN_ENABLE = 14;

const unsigned pin_H = 28;
const unsigned pin_L = 14;

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

constexpr bool TEST_CIRCUIT = false;

void initialize_pins()
{
	const std::vector<unsigned> pins = TEST_CIRCUIT ? 
		std::vector<unsigned>{PIN_LOGIC, PIN_ENABLE} :
		std::vector<unsigned>{pin_H, pin_L};
	for (unsigned pin : pins)
	{
		gpio_init(pin);
		gpio_set_dir(pin, GPIO_OUT);
	}
}

float calculate_frequency(float velocity, float throttle)
{
	float d_r = 0.01048;     // track thickness (meters)
	float L = 0.55;          // stator length (meters)
	float sigma = 3.03e7;    // track conductance/length (Siemens/meter)
	float g = 0.0305;        // air gap between stators (meters)
	float mu_r = 1.00000037; // relative permeability of air


	// The thrust equation has the form F = Bs / (C + As^2)
	// which has a peak value at s = √(C/A)
	// where C is the square of the magnetic sensitivity
	// and A is the square of the length resistance,
	// so the peak is found by simply dividing the two.

	// Derived from C = πg/µ using µ_0 = 4πe-7
	float magneticSensitivity = 1e7 * g / (4 * mu_r); // amps/tesla
	float lengthResistance = sigma * d_r * L / 2;     // meters/ohm
	float peakThrustSlip = magneticSensitivity / lengthResistance;

	// To provide a proportional throttle, the peak is remapped to 1,
	// and the normalized inverse profile for the stable region is (1 - √(1 - u^2)) / u
	// The denominator is irrationalized to avoid division by zero.
	float proportion = throttle / (1 + std::sqrt(1 - throttle * throttle));
	float slip = proportion * peakThrustSlip;

	return (slip + velocity) * 2 * M_PI / L;
}

void set_logic_pin_(bool v) {
	gpio_put(PIN_LOGIC, v);
	gpio_put(PIN_ENABLE, 1);
}

void set_hilo_pins_(bool v)
{
	// Even though we should not need to manually introduce a delay (deadtime),
	// we should still ensure that the rise always occurs after the fall on the
	// GPIO pins for each phase.
	if (v) {
		gpio_put(pin_L, !v);
		gpio_put(pin_H, v);
	} else {
		gpio_put(pin_H, v);
		gpio_put(pin_L, !v);
	}
}

constexpr auto& set_inverter_pins_ = TEST_CIRCUIT ? set_logic_pin_ : set_hilo_pins_;

void run_inverter_cycle(int N, float amplitude)
{
	float qe = 0.0;
	for (int i = 0; i < N; ++i)
	{
		float s = amplitude * std::sin(2 * M_PI * i / N);
		qe += s;
		bool v = qe > 0;
		int fix = v ? 1 : -1;
		qe -= fix;
		set_inverter_pins_(v);
	}
}

int frequency_to_samples(float frequency) {
	return OPERATING_FREQUENCY / frequency - OFFSET;
}

float get_frequency() {
	return 1;
}	

int main()
{
	initialize_pins();

	while (true)
	{
		float frequency = get_frequency();
		int N = frequency_to_samples(frequency);
		run_inverter_cycle(N, 1);
	}

	return 0;
}
