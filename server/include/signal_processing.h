#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <fftw3.h>
#include <cstddef>
#include <cstdint>
#include <vector>

// Window function types
constexpr uint32_t WINDOW_RECTANGULAR = 0;
constexpr uint32_t WINDOW_HAMMING = 1;
constexpr uint32_t WINDOW_HANNING = 2;
constexpr uint32_t WINDOW_BLACKMAN = 3;
constexpr uint32_t WINDOW_BLACKMAN_HARRIS = 4;
constexpr uint32_t WINDOW_KAISER = 5;
constexpr uint32_t WINDOW_TUKEY = 6;
constexpr uint32_t WINDOW_GAUSSIAN = 7;

// Generate window function coefficients
void generate_window(uint32_t window_type, size_t length, std::vector<float>& window);

// Apply window function to complex data
void apply_window(fftwf_complex* data, size_t length, const std::vector<float>& window);

// Compute FFT (wrapper around FFTW)
void compute_fft(fftwf_complex *in, fftwf_complex *out, fftwf_plan plan);

// Convert FFT output to magnitude in dB scale (0-255)
void compute_magnitude_db(fftwf_complex *fft_out, uint8_t *mag_out, size_t size);

// Remove DC offset spike at center frequency bin
void remove_dc_offset(uint8_t *magnitude, size_t size);

// Compute cross-correlation between two FFT outputs
void compute_cross_correlation(fftwf_complex *fft_ch1, fftwf_complex *fft_ch2,
                              float *correlation, float *phase_diff, size_t size);

// Initialize FFT averaging buffers
void init_averaging(uint32_t num_frames, size_t fft_size,
                    std::vector<std::vector<uint8_t>>& avg_buffer_ch1,
                    std::vector<std::vector<uint8_t>>& avg_buffer_ch2);

// Apply temporal averaging to FFT magnitudes
void apply_averaging(uint8_t* ch1_mag, uint8_t* ch2_mag, size_t fft_size,
                     uint32_t averaging_frames,
                     std::vector<std::vector<uint8_t>>& avg_buffer_ch1,
                     std::vector<std::vector<uint8_t>>& avg_buffer_ch2,
                     uint32_t& avg_index);

// Automatic Gain Control (AGC) state
struct AGCState {
    bool enabled;                  // AGC enable flag
    float current_level;           // Current signal level (0-255)
    uint32_t current_gain_rx1;     // Current RX1 gain (dB)
    uint32_t current_gain_rx2;     // Current RX2 gain (dB)
    int hysteresis_counter;        // Counter to prevent rapid gain changes
};

// Initialize AGC state
void init_agc(AGCState& agc, uint32_t initial_gain_rx1, uint32_t initial_gain_rx2);

// Update AGC and adjust gains if needed
// Args:
//   agc: AGC state structure
//   ch1_mag, ch2_mag: Magnitude arrays (0-255 scale)
//   len: Array length
//   gain_rx1, gain_rx2: Output gain values (will be modified if AGC adjusts)
//   params_changed: Set to true if gains were changed
void update_agc(AGCState& agc, const uint8_t* ch1_mag, const uint8_t* ch2_mag, size_t len,
                uint32_t& gain_rx1, uint32_t& gain_rx2, bool& params_changed);

#endif // SIGNAL_PROCESSING_H
