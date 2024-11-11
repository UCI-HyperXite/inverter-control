#include <cmath>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pin_logic.hpp"
#include "pod_communication.hpp"

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

// Feature flag for indefinite forward motion until peak thrust
const bool INFINITE_FORWARD_MODE = true; 

// Constants for thrust equation
const float N = 110;                // Coil turns/phase
const float I = 20.0;               // RMS current in A
const float D = 0.1;                // Stator thickness in m

const float d_r = 0.01048;          // Track thickness in m
const float L = 0.55;               // Stator length in m
const float sigma = 3.03e7;         // Track conductance per meter in S/m
const float g = 0.0305;             // Air gap between stators in m
const float mu = 4 * M_PI * 1e-7;   // Permeability of air in H/m

// Pod values
const float pod_mass = 160.0;    // Define pod mass in kg
const float delta_t = 0.25;  // Time step in seconds for velocity update
const float target_slip = 0.7; // Slip at peak thrust

// Shared structure for velocity; modified by another core
volatile static LimControlMessage lcm{ 0, 1 };
static mutex lcmMutex;  // Mutex to ensure only one core accesses lcm at a time

// Calculate thrust based on the universal thrust equation
float calculate_thrust(float omega, float velocity) {
    float vs = L * omega / (2 * M_PI);  // Synchronous speed
    float slip_velocity = vs - velocity;

    // Thrust equation 
    float numerator = 18 * D * d_r * sigma * L * N * N * I * I * slip_velocity;
    float denominator = std::pow((M_PI * g / mu), 2) + std::pow(slip_velocity * sigma * d_r * L / 2, 2);

    return numerator / denominator;
}

// Calculate angular frequency based on velocity and slip
float calculate_frequency(float velocity, float slip) {
    if (slip <= 0 || slip >= 1) slip = target_slip;  // Bounds checking for slip value
    float vs = velocity / (1 - slip);  // Synch speed 
    return (2 * M_PI * vs) / L;        
}

// Generate one inverter cycle for all three phases using SPDM
void run_inverter_cycle(int N, float amplitude) {
    float qe_A = 0.0, qe_B = 0.0, qe_C = 0.0;
    float threshold = 0.5;

    for (int i = 0; i < N; ++i) {
        float s_A = amplitude * std::sin(2 * M_PI * i / N);                // Phase A: 0 degrees
        float s_B = amplitude * std::sin(2 * M_PI * i / N - 2 * M_PI / 3); // Phase B: 120 degrees
        float s_C = amplitude * std::sin(2 * M_PI * i / N + 2 * M_PI / 3); // Phase C: 240 degrees

        qe_A += s_A;
        qe_B += s_B;
        qe_C += s_C;

        bool v_A = qe_A > threshold;
        bool v_B = qe_B > threshold;
        bool v_C = qe_C > threshold;

        qe_A -= v_A ? 1 : -1;
        qe_B -= v_B ? 1 : -1;
        qe_C -= v_C ? 1 : -1;

        set_inverter_pins_(v_A, v_B, v_C);
    }
}

int frequency_to_samples(float frequency) {
    return static_cast<int>(OPERATING_FREQUENCY / frequency - OFFSET);
}

// Main inverter control loop, with INFINITE_FORWARD_MODE simulation
void run_inverter() {
    float velocity = 0.0f;  // Initialize velocity
    bool reached_peak_thrust = false;

    while (true) {
        if (INFINITE_FORWARD_MODE && !reached_peak_thrust) {
            // Simulate LIM moving forward indefinitely until peak thrust
            float omega = calculate_frequency(velocity, target_slip);
            float thrust = calculate_thrust(omega, velocity);

            // Update velocity based on thrust and pod mass
            float acceleration = thrust / pod_mass;
            velocity += acceleration * delta_t;

            // Calculate slip to determine if peak thrust is reached
            float vs = L * omega / (2 * M_PI);
            float current_slip = (vs - velocity) / vs;

            // Check if LIM reached peak thrust condition based on slip
            if (std::abs(current_slip - target_slip) < 0.01) {
                reached_peak_thrust = true;
            }

            // Run inverter cycle
            int N = frequency_to_samples(omega);
            run_inverter_cycle(N, 1);

        } else if (!INFINITE_FORWARD_MODE || reached_peak_thrust) {
            // Default operation with external inputs or hold velocity if peak thrust reached
            mutex_enter_blocking(&lcmMutex);
            float slip = target_slip; // Use a fixed slip or implement logic for dynamic adjustment
            float omega = calculate_frequency(lcm.velocity, slip);
            mutex_exit(&lcmMutex);

            if (omega == 0) {
                set_inverter_pins_(false, false, false);
                continue;
            }

            int N = std::max(1, frequency_to_samples(omega));
            run_inverter_cycle(N, 1);
        }
    }
}

int main() {
    stdio_init_all();

    const int max_attempts = 80; // Maximum number of attempts, 40 * 250ms = 20 sec 
    int attempts = 0;

    while (!stdio_usb_connected() && attempts < max_attempts) {
        sleep_ms(250); 
        attempts++;
    }

    if (stdio_usb_connected()) {
        // USB connected, launch monitor_serial on core 1
        multicore_launch_core1(monitor_serial);
    }

    // Initialize GPIO pins and the mutex
    initialize_pins();
    mutex_init(&lcmMutex);

    // Run inverter on the primary core
    run_inverter();

    return 0;
}