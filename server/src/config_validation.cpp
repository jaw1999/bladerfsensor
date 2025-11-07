#include "config_validation.h"

bool validate_frequency(uint64_t freq) {
    // bladeRF xA9 frequency range 47 MHz to 6 GHz
    return freq >= 47000000 && freq <= 6000000000ULL;
}

bool validate_sample_rate(uint32_t rate) {
    // Valid range 520 kHz to 61.44 MHz
    return rate >= 520000 && rate <= 61440000;
}

bool validate_gain(uint32_t gain) {
    // Valid gain range 0 to 60 dB
    return gain <= 60;
}

bool validate_bandwidth(uint32_t bw) {
    // Valid range 520 kHz to 61.44 MHz (same as sample rate)
    return bw >= 520000 && bw <= 61440000;
}
