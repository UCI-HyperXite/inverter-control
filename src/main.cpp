#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cmath>
#include <vector>

const unsigned PIN_LOGIC = 28;
const unsigned PIN_ENABLE = 14;

const unsigned pin_H = 28;
const unsigned pin_L = 14;

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

float calculate_frequency(float velocity)
{
	// Take the equation the propulsion subteam provided, substitute the
	// constants, and simplify. The resulting equation is a quadratic, where
	// x is the frequency and the velocity is a constant. This means that
	// we can use the quadratic formula and solve for the appropriate frequency.
	// Note that since this is a quadratic, there are two solutions. We
	// default to the solution that has the lower frequency for increased
	// stability.

	// Run the quadratic formula on these coefficients to solve for
	// u = ((L*x)/(2*pi)) - v_r and solve for x.

	int N = 183;			  // number of turns per phase
	int I = 54;				  // RMS current through coil (amps)
	float D = 0.1;			  // stator thickness (meters)
	float d_r = 0.01048;	  // track thickness (meters)
	float L = 0.55;			  // stator length (meters)
	float sigma = 3.03e7;	  // track conductance/meter (Siemens/meter)
	float g = 0.02548;		  // air gap between stators (meters)
	float mu = 1.25663753e-6; // permeability of air (Henries per meter)
	int F_thrust = 1200;	  // constant thrust (Newtons)

	float cTerm = F_thrust * (M_PI * g / mu) * (M_PI * g / mu);
	float bCoefficient = -18 * D * d_r * sigma * L * N * N * I * I;
	float aCoefficient = F_thrust * (sigma * d_r * L / 2) * (sigma * d_r * L / 2);

	float u = (-bCoefficient - std::sqrt(bCoefficient * bCoefficient - 4 * aCoefficient * cTerm)) / (2 * aCoefficient);

	return (u + velocity) * 2 * M_PI / L;
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

int main()
{
	initialize_pins();
	int frequency = 1000;

	while (true)
	{
		run_inverter_cycle(frequency, 1);
	}

	return 0;
}
