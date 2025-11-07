#ifndef RECORDING_H
#define RECORDING_H

#include <string>
#include <cstdint>
#include <cstddef>

// Start recording IQ samples to a file
// Args:
//   filename: Path to output file
//   center_freq: Center frequency in Hz
//   sample_rate: Sample rate in Hz
//   bandwidth: Analog bandwidth in Hz
//   gain_rx1: RX1 gain in dB
//   gain_rx2: RX2 gain in dB
// Returns: true on success, false on failure
bool start_recording(const std::string& filename, uint64_t center_freq,
                    uint32_t sample_rate, uint32_t bandwidth,
                    uint32_t gain_rx1, uint32_t gain_rx2);

// Stop active recording and finalize file
void stop_recording();

// Write IQ samples to active recording
// Args:
//   samples: Interleaved I/Q samples (int16_t pairs)
//   num_samples: Number of IQ pairs (not total int16_t count)
void write_samples_to_file(const int16_t* samples, size_t num_samples);

// Check if recording is currently active
bool is_recording();

// Get current recording statistics
// Args:
//   samples_written: Output parameter for number of samples written
// Returns: true if recording is active, false otherwise
bool get_recording_status(uint64_t& samples_written);

#endif // RECORDING_H
