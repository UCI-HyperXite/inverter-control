#ifndef POD_COMMUNICATION_HPP
#define POD_COMMUNICATION_HPP

struct LimControlMessage
{
    float velocity;
    float throttle;
};

LimControlMessage read_control_message();

#endif