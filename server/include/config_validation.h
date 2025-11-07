#ifndef CONFIG_VALIDATION_H
#define CONFIG_VALIDATION_H

#include <cstdint>

// Validate frequency is within supported range
// Returns: true if valid, false otherwise
bool validate_frequency(uint64_t freq);

// Validate sample rate is supported by hardware
// Returns: true if valid, false otherwise
bool validate_sample_rate(uint32_t rate);

// Validate gain is within hardware limits (0-60 dB)
// Returns: true if valid, false otherwise
bool validate_gain(uint32_t gain);

// Validate bandwidth is supported by hardware
// Returns: true if valid, false otherwise
bool validate_bandwidth(uint32_t bw);

#endif // CONFIG_VALIDATION_H
