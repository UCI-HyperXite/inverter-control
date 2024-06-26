#include <cmath>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pin_logic.hpp"
#include "pod_communication.hpp"

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

// Value will be changed by other core, so prevent compiler from optimizing as constant
volatile static LimControlMessage lcm{ 0, 1 };
// Allow only one core at a time to access lcm
static mutex lcmMutex;

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

int frequency_to_samples(float frequency)
{
	return OPERATING_FREQUENCY / frequency - OFFSET;
}

// Secondary program to run on core 1
void monitor_serial()
{
	while (true)
	{
		LimControlMessage message = read_control_message();

		mutex_enter_blocking(&lcmMutex);
		lcm.velocity = message.velocity;
		lcm.throttle = message.throttle;
		mutex_exit(&lcmMutex);
	}
}

// Main program to run on core 0
void run_inverter()
{
	while (true)
	{
		mutex_enter_blocking(&lcmMutex);
		float frequency = calculate_frequency(lcm.velocity, lcm.throttle);
		mutex_exit(&lcmMutex);

		// Set pins to low if frequency is zero
		if (frequency == 0)
		{
			set_inverter_off_();
			continue;
		}

		int N = frequency_to_samples(frequency);
		// TODO: amplitude ratio
		run_inverter_cycle(N, 1);
	}
}

int main()
{
	stdio_init_all();

	// Wait until USB device is connected
	while (!tud_cdc_connected())
		sleep_ms(250);

	initialize_pins();

	mutex_init(&lcmMutex);

	// Run inverter on core 0 while updating control parameters on core 1
	multicore_launch_core1(monitor_serial);
	run_inverter();

	return 0;
}
