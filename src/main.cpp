#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cmath>
#include <vector>

const unsigned pinU_H = 28;
const unsigned pinU_L = 27;
const unsigned pinV_H = 26;
const unsigned pinV_L = 22;

void initialize_pins()
{
	const std::vector<unsigned> pins = {pinU_H, pinU_L, pinV_H, pinV_L};
	for (unsigned pin : pins)
	{
		gpio_init(pin);
		gpio_set_dir(pin, GPIO_OUT);
	}
}

void set_inverter_pins_(bool v)
{
	gpio_put(pinU_H, v);
	gpio_put(pinU_L, !v);
	gpio_put(pinV_H, !v);
	gpio_put(pinV_L, v);
}

void run_one_inverter(int N)
{
	double qe = 0.0;
	for (int i = 0; i < N; ++i)
	{
		double s = std::sin(i * 2 * M_PI / N);
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
	int frequency = 2000;

	while (true)
	{
		run_one_inverter(frequency);

		if (time_us_32() % 10000000 > 5000000)
		{
			frequency = (frequency == 2000) ? 3000 : 2000;
		}
	}

	return 0;
}
