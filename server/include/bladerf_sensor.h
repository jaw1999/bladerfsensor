#ifndef BLADERF_SENSOR_H
#define BLADERF_SENSOR_H

#include <libbladeRF.h>
#include <complex>
#include <vector>
#include <cstdint>

// Configuration constants for RF hardware and signal processing
constexpr uint32_t SAMPLE_RATE = 40000000;      // 40 MHz sample rate
constexpr uint32_t BANDWIDTH = 40000000;        // 40 MHz analog bandwidth
constexpr uint64_t CENTER_FREQ = 915000000;     // Default center frequency 915 MHz
constexpr uint32_t FFT_SIZE = 4096;             // Number of FFT bins
constexpr uint32_t NUM_SAMPLES = FFT_SIZE;      // Samples per processing block
constexpr uint32_t UPDATE_RATE_HZ = 10;         // Spectrum update rate in Hz
constexpr uint32_t GAIN_RX1 = 40;               // Default RX1 gain in dB
constexpr uint32_t GAIN_RX2 = 40;               // Default RX2 gain in dB
constexpr int NUM_BUFFERS = 32;                 // Number of USB transfer buffers
constexpr int BUFFER_SIZE = 8192;               // Samples per USB buffer
constexpr int NUM_TRANSFERS = 16;               // Number of concurrent USB transfers

// Automatic gain control (AGC) configuration parameters
constexpr int AGC_TARGET_LEVEL = 200;           // Target signal level on 0-255 scale
constexpr int AGC_HYSTERESIS = 20;              // Hysteresis zone to prevent oscillation
constexpr float AGC_ATTACK_RATE = 0.1f;         // Attack rate for gain decrease
constexpr float AGC_DECAY_RATE = 0.01f;         // Decay rate for gain increase

// Recording metadata structure
#pragma pack(push, 1)

struct RecordingMetadata {
    uint64_t center_freq;              // Center frequency in Hz
    uint32_t sample_rate;              // Sample rate in Hz
    uint32_t bandwidth;                // Analog filter bandwidth in Hz
    uint32_t gain_rx1;                 // RX1 gain setting in dB
    uint32_t gain_rx2;                 // RX2 gain setting in dB
    uint64_t timestamp_start_sec;      // Recording start time (UNIX seconds)
    uint64_t timestamp_start_nsec;     // Recording start time (nanoseconds)
    uint64_t num_samples;              // Total number of IQ samples recorded
    char notes[256];                   // User notes or description
};

#pragma pack(pop)

// Recording state structure for managing IQ sample recording sessions
struct RecordingState {
    bool active;                       // Recording in progress flag
    FILE* file;                        // File handle for recording output
    uint64_t samples_written;          // Number of samples written so far
    uint64_t start_time_sec;           // Recording start time (UNIX seconds)
    uint64_t start_time_nsec;          // Recording start time (nanoseconds)
    RecordingMetadata metadata;        // Recording metadata header
};

// Function declarations

// Initialize bladeRF device and verify FPGA configuration
int initialize_bladerf(struct bladerf **dev);

// Configure a single RX channel with specified parameters
int configure_channel(struct bladerf *dev, bladerf_channel ch, uint64_t freq,
                     uint32_t gain, uint32_t sample_rate, uint32_t bandwidth);

#endif // BLADERF_SENSOR_H
