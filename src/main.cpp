#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cmath>
#include <vector>

const unsigned pinU_H = 21;
const unsigned pinU_L = 20;
const unsigned pinV_H = 19;
const unsigned pinV_L = 18;
const unsigned pinW_H = 17;
const unsigned pinW_L = 16;

const float sin_120 = 0.866025;

void initialize_pins()
{
	const std::vector<unsigned> pins = {
		pinU_H,
		pinU_L,
		pinV_H,
		pinV_L,
		pinW_H,
		pinW_L};
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

void set_inverter_pins_(bool v, unsigned pin_H, unsigned pin_L)
{
	gpio_put(pin_H, v);
	gpio_put(pin_L, !v);
}

void run_one_inverter(int N)
{
	float qe = 0.0;
	for (int i = 0; i < N; ++i)
	{
		float s = std::sin(2 * M_PI * i / N);
		qe += s;
		bool v = qe > 0;
		int fix = v ? 1 : -1;
		qe -= fix;
		set_inverter_pins_(v, pinU_H, pinU_L);
	}
}

void run_inverter(int N)
{
	float qeU = 0.0;
	float qeV = 0.0;
	float qeW = 0.0;
	for (int i = 0; i < N; ++i)
	{
		/*
		Instead of doing three trigonometric calculations, we can use the sine
		addition formula to need only two trigonometric calculations:

		sin(x + phi) = sin(x)cos(phi) + sin(phi)cos(x)

		Because phi is a constant, we can precompute cos(phi) and sin(phi) so
		that now we only perform two trigonometric calculations.
		*/

		float x = 2 * M_PI * i / N;

		float sU = std::sin(x);
		float cosX = std::cos(x);
		float sinAcosB = sU * -0.5;
		float sV = sinAcosB + sin_120 * cosX;
		float sW = sinAcosB - sin_120 * cosX;

		qeU += sU;
		qeV += sV;
		qeW += sW;

		bool vU = qeU > 0;
		bool vV = qeV > 0;
		bool vW = qeW > 0;

		int fixU = vU ? 1 : -1;
		int fixV = vV ? 1 : -1;
		int fixW = vW ? 1 : -1;

		qeU -= fixU;
		qeV -= fixV;
		qeW -= fixW;

		set_inverter_pins_(vU, pinU_H, pinU_L);
		set_inverter_pins_(vV, pinV_H, pinV_L);
		set_inverter_pins_(vW, pinW_H, pinW_L);
	}
}

int main()
{
	initialize_pins();
	int frequency = 1000;

	while (true)
	{
		run_inverter(frequency);

		// if (time_us_32() % 10000000 > 5000000)
		// {
		// 	frequency = (frequency == 2000) ? 3000 : 2000;
		// }
	}

	return 0;
}
