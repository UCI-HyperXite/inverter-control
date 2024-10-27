#include <cmath>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pin_logic.hpp"
#include "pod_communication.hpp"

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

// Value will be changed by other core, so prevent compiler from optimizing as constant
volatile static LimControlMessage lcm{ 0, 1 };
// Allow only one core at a time to access lcm
static mutex lcmMutex;

float calculate_frequency(float velocity, float throttle)
{
    float d_r = 0.01048;     // track thickness (meters)
    float L = 0.55;          // stator length (meters)
    float sigma = 3.03e7;    // track conductance/length (Siemens/meter)
    float g = 0.0305;        // air gap between stators (meters)
    float mu_r = 1.00000037; // relative permeability of air

    // Derived from C = πg/µ using µ_0 = 4πe-7
    float magneticSensitivity = 1e7 * g / (4 * mu_r); // amps/tesla
    float lengthResistance = sigma * d_r * L / 2;     // meters/ohm
    float peakThrustSlip = magneticSensitivity / lengthResistance;

    // Proportional throttle for slip calculation
    float proportion = throttle / (1 + std::sqrt(1 - throttle * throttle));
    float slip = proportion * peakThrustSlip;

    return (slip + velocity) * 2 * M_PI / L; // calculates angular freq (rad/sec)
}

// Function to run 1 inverter cycle for all 3 phases
void run_inverter_cycle(int N, float amplitude)
{
    float qe_A = 0.0, qe_B = 0.0, qe_C = 0.0; // cumulative quantization errors, per phase 
    float threshold = 0.0; // threshold to enforce 50% duty cycle

    for (int i = 0; i < N; ++i)
    {
        // Generate SPDM waveform values for all three phases
        float s_A = amplitude * std::sin(2 * M_PI * i / N);                // Phase A: 0 degrees
        float s_B = amplitude * std::sin(2 * M_PI * i / N - 2 * M_PI / 3); // Phase B: 120 degrees shift
        float s_C = amplitude * std::sin(2 * M_PI * i / N + 2 * M_PI / 3); // Phase C: 240 degrees shift

        // Update quantization errors for each phase
        qe_A += s_A;
        qe_B += s_B;
        qe_C += s_C;

        // Set the pins high or low based on quantization errors
        bool v_A = qe_A > threshold;
        bool v_B = qe_B > threshold;
        bool v_C = qe_C > threshold;

        // Adjust quantization errors accordingly
        qe_A -= v_A ? 1 : -1;
        qe_B -= v_B ? 1 : -1;
        qe_C -= v_C ? 1 : -1;

        // Set inverter pins for all 3 phases
        set_inverter_pins_3phase(v_A, v_B, v_C);
    }
}

int frequency_to_samples(float frequency)
{
    return OPERATING_FREQUENCY / frequency - OFFSET;
}

// Secondary program to run on core 1
void monitor_serial()
{
    while (true)
    {
        LimControlMessage message = read_control_message();

        mutex_enter_blocking(&lcmMutex);
        lcm.velocity = message.velocity;
        lcm.throttle = message.throttle;
        mutex_exit(&lcmMutex);
    }
}

// Main program to run on core 0
void run_inverter()
{
    while (true)
    {
        mutex_enter_blocking(&lcmMutex);
        float frequency = calculate_frequency(lcm.velocity, lcm.throttle);
        mutex_exit(&lcmMutex);

        // Set pins to low if frequency is zero
        if (frequency == 0)
        {
            set_inverter_pins_off_3phase();
            continue;
        }

        int N = frequency_to_samples(frequency);
        run_inverter_cycle(N, 1);
    }
}

int main()
{
    stdio_init_all();

    const int max_attempts = 80; // max number of attempts, 40 * 250ms = 20 sec 
    int attempts = 0;

    while (!tud_cdc_connected() && attempts < max_attempts)
    {
        sleep_ms(250); 
        attempts++;
    }

    if (tud_cdc_connected()) 
    {
        // USB connected, launch monitor_serial on core 1
        multicore_launch_core1(monitor_serial);
    }

    // Proceed with initialization and inverter operation regardless of USB status
    initialize_pins();
    mutex_init(&lcmMutex);

    // Run inverter on core 0
    run_inverter();

    return 0;
}