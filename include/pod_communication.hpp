#ifndef POD_COMMUNICATION_HPP
#define POD_COMMUNICATION_HPP

#include "pico/time.h"

struct LimControlMessage
{
	float velocity;
	float throttle;
	absolute_time_t messageTime;
};

LimControlMessage read_control_message();

#endif
