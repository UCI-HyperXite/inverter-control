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

int velocity = 0;

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

void set_inverter_pins_(bool v, unsigned pin_H, unsigned pin_L)
{
	gpio_put(pin_H, v);
	gpio_put(pin_L, !v);
}

void run_one_inverter(int N)
{
	double qe = 0.0;
	for (int i = 0; i < N; ++i)
	{
		double s = std::sin(2 * M_PI * i / N);
		qe += s;
		bool v = qe > 0;
		int fix = v ? 1 : -1;
		qe -= fix;
		set_inverter_pins_(v, pinU_H, pinU_L);
	}
}

void run_inverter(int N)
{
	double qeU = 0.0;
	double qeV = 0.0;
	double qeW = 0.0;

	for (int i = 0; i < N; ++i)
	{
		double x = 2 * M_PI * i / N;

		double sU = std::sin(x);				// 0 degrees
		double sV = std::sin(x + 2 * M_PI / 3); // 120 degrees
		double sW = std::sin(x + 4 * M_PI / 3); // 240 degrees

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
	int frequency = 2000;

	while (true)
	{
		run_inverter(frequency);

		if (time_us_32() % 10000000 > 5000000)
		{
			frequency = (frequency == 2000) ? 3000 : 2000;
		}
	}

	return 0;
}
