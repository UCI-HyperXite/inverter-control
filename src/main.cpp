#include <cmath>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pin_logic.hpp"
#include "pod_communication.hpp"

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

// Shared structure for velocity and throttle, modified by another core
volatile static LimControlMessage lcm{ 0, 1 };
// Mutex to ensure only one core accesses lcm at a time
static mutex lcmMutex;

float calculate_frequency(float velocity, float throttle) {
    float d_r = 0.01048;     // Track thickness (meters)
    float L = 0.55;          // Stator length (meters)
    float sigma = 3.03e7;    // Track conductance per length (Siemens/meter)
    float g = 0.0305;        // Air gap between stators (meters)
    float mu_r = 1.00000037; // Relative permeability of air

    // The thrust equation has the form F = Bs / (C + As^2)
    // which has a peak value at s = √(C/A)
    // where C is the square of the magnetic sensitivity
    // and A is the square of the length resistance,
    // so the peak is found by simply dividing the two.

    float magneticSensitivity = 1e7 * g / (4 * mu_r); // Amps per Tesla
    float lengthResistance = sigma * d_r * L / 2;     // Meters per ohm
    float peakThrustSlip = magneticSensitivity / lengthResistance;

    // To provide a proportional throttle, the peak is remapped to 1,
    // and the normalized inverse profile for the stable region is (1 - √(1 - u^2)) / u
    // The denominator is irrationalized to avoid division by zero.

    // Calculate slip based on throttle
    float proportion = throttle / (1 + std::sqrt(1 - throttle * throttle));
    float slip = proportion * peakThrustSlip;

    return (slip + velocity) * 2 * M_PI / L; // Angular frequency in rad/sec
}

// Generate one inverter cycle for all three phases
void run_inverter_cycle(int N, float amplitude) {
    float qe_A = 0.0, qe_B = 0.0, qe_C = 0.0; // Cumulative quantization errors
    float threshold = 0.5; // EXPERIMENT W/ THRESHOLD FOR CLEANER SIGNAL?? 

    for (int i = 0; i < N; ++i) {
        // Calculate SPDM waveform values for each phase
        float s_A = amplitude * std::sin(2 * M_PI * i / N);                // Phase A: 0 degrees
        float s_B = amplitude * std::sin(2 * M_PI * i / N - 2 * M_PI / 3); // Phase B: 120 degrees
        float s_C = amplitude * std::sin(2 * M_PI * i / N + 2 * M_PI / 3); // Phase C: 240 degrees

        // Update quantization errors for each phase
        qe_A += s_A;
        qe_B += s_B;
        qe_C += s_C;

        // Set pins high or low based on quantization errors and threshold
        bool v_A = qe_A > threshold;
        bool v_B = qe_B > threshold;
        bool v_C = qe_C > threshold;

        // Adjust quantization errors with a smaller step for stability
        qe_A -= v_A ? 0.5 : -0.5;
        qe_B -= v_B ? 0.5 : -0.5;
        qe_C -= v_C ? 0.5 : -0.5;

        // Set inverter pins for all 3 phases
        set_inverter_pins_(v_A, v_B, v_C);
    }
}

int frequency_to_samples(float frequency) {
    return static_cast<int>(OPERATING_FREQUENCY / frequency - OFFSET);
}

// Secondary core program to monitor serial input
void monitor_serial() {
    while (true) {
        LimControlMessage message = read_control_message();

        mutex_enter_blocking(&lcmMutex);
        lcm.velocity = message.velocity;
        lcm.throttle = message.throttle;
        mutex_exit(&lcmMutex);
    }
}

// Main program to run on the primary core
void run_inverter() {
    while (true) {
        mutex_enter_blocking(&lcmMutex);
        float frequency = calculate_frequency(lcm.velocity, lcm.throttle);
        mutex_exit(&lcmMutex);

        // Set pins to low if frequency is zero
        if (frequency == 0) {
            set_inverter_pins_(false, false, false);
            continue;
        }

        int N = std::max(1, frequency_to_samples(frequency));
        run_inverter_cycle(N, 1);
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