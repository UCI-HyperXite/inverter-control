#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cmath>
#include <vector>

const unsigned pinU1_H = 28;
const unsigned pinU1_L = 27;
const unsigned pinV1_H = 26;
const unsigned pinV1_L = 22;

const unsigned pinU2_H = 21;
const unsigned pinU2_L = 20;
const unsigned pinV2_H = 19;
const unsigned pinV2_L = 18;

const unsigned pinU3_H = 17;
const unsigned pinU3_L = 16;
const unsigned pinV3_H = 15;
const unsigned pinV3_L = 14;

int velocity = 0;

void initialize_pins()
{
	const std::vector<unsigned> pins = {
		pinU1_H,
		pinU1_L,
		pinV1_H,
		pinV1_L,
		pinU2_H,
		pinU2_L,
		pinV2_H,
		pinV2_L,
		pinU3_H,
		pinU3_L,
		pinV3_H,
		pinV3_L};
	for (unsigned pin : pins)
	{
		gpio_init(pin);
		gpio_set_dir(pin, GPIO_OUT);
	}
}

void set_inverter_pins_(bool v, unsigned pinU_H, unsigned pinU_L, unsigned pinV_H, unsigned pinV_L)
{
	gpio_put(pinU_H, v);
	gpio_put(pinU_L, !v);
	gpio_put(pinV_H, !v);
	gpio_put(pinV_L, v);
}

void run_one_inverter(int N)
{
	double qe = 0.0;
	double qe2 = 0.0;
	double qe3 = 0.0;

	for (int i = 0; i < N; ++i)
	{
		double x = i * 2 * M_PI / N;

		double s = std::sin(x);			// 0 degrees
		double s2 = std::sin(x + 5 * M_PI / 6); // 120 degrees
		double s3 = std::sin(x + 4 * M_PI / 3); // 240 degrees

		qe += s;
		qe2 += s2;
		qe3 += s3;

		bool v = qe > 0;
		bool v2 = qe2 > 0;
		bool v3 = qe3 > 0;

		int fix = v ? 1 : -1;
		int fix2 = v2 ? 1 : -1;
		int fix3 = v3 ? 1 : -1;

		qe -= fix;
		qe2 -= fix2;
		qe3 -= fix3;

		set_inverter_pins_(v, pinU1_H, pinU1_L, pinV1_H, pinV1_L);
		set_inverter_pins_(v2, pinU2_H, pinU2_L, pinV2_H, pinV2_L);
		set_inverter_pins_(v3, pinU3_H, pinU3_L, pinV3_H, pinV3_L);
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
