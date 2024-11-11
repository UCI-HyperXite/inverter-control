#include <cmath>
#include <algorithm>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pin_logic.hpp"
#include "pod_communication.hpp"

const float OPERATING_FREQUENCY = 45552.3;
const float OFFSET = 0.0411;

// ADDED CONSTANTS 
const float TARGET_FREQUENCY = 60.0; // Target frequency in Hz
const float FREQUENCY_RAMP_STEP = 0.5; // Frequency increment per cycle for smooth ramp-up

// Value will be changed by other core, so prevent compiler from optimizing as constant
volatile static LimControlMessage lcm{ 0, 1 };
// Allow only one core at a time to access lcm
static mutex lcmMutex;

// calculate frequency () finds the point where the magnetic sensitivity and length resistance 
// are balanced for peak performance
float calculate_frequency(float velocity, float throttle)
{
	float d_r = 0.01048;     // track thickness (meters)
	float L = 0.55;          // stator length (meters)
	float sigma = 3.03e7;    // track conductance/length (Siemens/meter)
	float g = 0.0305;        // air gap between stators (meters)
	float mu_r = 1.00000037; // relative permeability of air

	// The thrust equation has the form F = Bs / (C + As^2)
	// which has a peak value at s = √(C/A)
	// where C is the square of the magnetic sensitivity
	// and A is the square of the length resistance,
	// so the peak is found by simply dividing the two.

	// Derived from C = πg/µ using µ_0 = 4πe-7
	float magneticSensitivity = 1e7 * g / (4 * mu_r); // amps/tesla
	float lengthResistance = sigma * d_r * L / 2;     // meters/ohm
	float peakThrustSlip = magneticSensitivity / lengthResistance;

	// To provide a proportional throttle, the peak is remapped to 1,
	// and the normalized inverse profile for the stable region is (1 - √(1 - u^2)) / u
	// The denominator is irrationalized to avoid division by zero.
	float proportion = throttle / (1 + std::sqrt(1 - throttle * throttle));
	float slip = proportion * peakThrustSlip;

	return (slip + velocity) * 2 * M_PI / L;
}

// pwm_switching_time_calculation() calculates the on-time for each phase in a switching period. 
// On-time is determined by input voltage values and DC bus voltage. 

// Space-vector implementation, primitive attempt at FOC
// Vabc[3]: Array containing the three phase voltages.
// Tswh: Duration of the switching period
// Tga_on, Tgb_on, Tgc_on: Output references for on-times of each phase.
void pwm_switching_time_calculation(float Vabc[3], float vdc, float Tswh, float& Tga_on, float& Tgb_on, float& Tgc_on) {
    // Calculate phase times by caling each phase voltage by the switching time (Tswh) and dividing by the DC voltage (vdc)
    // Gives the amount of time each phase would be on if not modulated
    float Tas = Vabc[0] * Tswh / vdc;
    float Tbs = Vabc[1] * Tswh / vdc;
    float Tcs = Vabc[2] * Tswh / vdc;

    float Tmax = std::max({Tas, Tbs, Tcs});
    float Tmin = std::min({Tas, Tbs, Tcs});
    float Teff = Tmax - Tmin; // Difference b/t highest & lowest on-times; balance switching
    float T0 = Tswh - Teff;

    float Toffset = 0.5 * Tswh; // Time offset centers switching pulses for balanced PWM
    
    // calculate on/off times per phase
    Tga_on = std::max(0.0f, Tswh - (Tas + Toffset));
    Tgb_on = std::max(0.0f, Tswh - (Tbs + Toffset));
    Tgc_on = std::max(0.0f, Tswh - (Tcs + Toffset));
}

// corrects quantization error across cycles
void accumulate_quantization_error(float& qe, float threshold = 1.0f) {
    if (qe >= threshold) {
        qe -= threshold;
    } else if (qe < 0) {
        qe += threshold;
    }
}

// run_inverter_cycle() runs phase voltages (Vabc) and calculating on-times for each phase
void run_inverter_cycle(int N, float amplitude, float phase_shift, float vdc) {
    float qe = 0.0f;
    for (int i = 0; i < N; ++i) {
        float Vabc[3] = {
            amplitude * sinf(2.0f * M_PI * i / N + phase_shift),
            amplitude * sinf(2.0f * M_PI * i / N + phase_shift + 2.0f * M_PI / 3.0f),
            amplitude * sinf(2.0f * M_PI * i / N + phase_shift + 4.0f * M_PI / 3.0f)
        };

        float Tga_on, Tgb_on, Tgc_on;
        pwm_switching_time_calculation(Vabc, vdc, 0.5f * N, Tga_on, Tgb_on, Tgc_on);

        bool phaseA = (qe > Tga_on);
        bool phaseB = (qe > Tgb_on);
        bool phaseC = (qe > Tgc_on);
        set_inverter_pins_(phaseA, phaseB, phaseC);

        qe += 1.0f / N;
        accumulate_quantization_error(qe);
    }
}

// Translates a frequency into an integer sample count, 
// mapping the input frequency to a discrete sample value that the inverter uses. 

// This sets the timing for each PDM cycle in run_inverter_cycle(). 
// The OFFSET adjusts for discrepancies in OPERATING_FREQUENCY vs. target frequency.
int frequency_to_samples(float frequency) {
    return static_cast<int>(OPERATING_FREQUENCY / frequency - OFFSET);
}

// Adjusts frequency toward target value
void adjust_frequency(float& current_frequency, float target_frequency, float step) {
    if (current_frequency < target_frequency) {
        current_frequency += step;
        if (current_frequency > target_frequency) {
            current_frequency = target_frequency;
        }
    } else if (current_frequency > target_frequency) {
        current_frequency -= step;
        if (current_frequency < target_frequency) {
            current_frequency = target_frequency;
        }
    }
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
void run_inverter() {
    float current_frequency = 0;

    while (true) {
        mutex_enter_blocking(&lcmMutex);
        float target_frequency = calculate_frequency(lcm.velocity, lcm.throttle);
        mutex_exit(&lcmMutex);

        adjust_frequency(current_frequency, target_frequency, FREQUENCY_RAMP_STEP);

        if (current_frequency == 0) {
            set_inverter_off_();
            continue;
        }

        int N = frequency_to_samples(current_frequency);

        run_inverter_cycle(N, 1, 0, lcm.throttle);               // Phase A
        run_inverter_cycle(N, 1, 2 * M_PI / 3, lcm.throttle);    // Phase B
        run_inverter_cycle(N, 1, 4 * M_PI / 3, lcm.throttle);    // Phase C
    }
}

int main()
{
    stdio_init_all();

    const int max_attempts = 80; // Max number of attempts, 40 * 250ms = 20 sec
    int attempts = 0;

    // Wait until USB device is connected or max attempts reached
    while (!tud_cdc_connected() && attempts < max_attempts) {
        sleep_ms(250);
        attempts++;
    }

    if (tud_cdc_connected()) {
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