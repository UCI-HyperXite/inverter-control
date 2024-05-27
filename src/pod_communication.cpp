#include <iostream>
#include "pod_communication.hpp"

LimControlMessage read_control_message()
{
    float velocity;
    float throttle;
    std::cin >> velocity >> throttle;

    std::cout << "Received velocity: " << velocity << std::endl;
    std::cout << "Received throttle: " << throttle << std::endl;

    return {velocity, throttle};
}