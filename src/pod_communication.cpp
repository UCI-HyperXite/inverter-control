#include <iostream>
#include "pod_communication.hpp"

LimControlMessage read_control_message()
{
	float velocity;
	float throttle;
	std::cin >> velocity >> throttle;

	return { velocity, throttle, get_absolute_time() };
}
