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

// Noise floor estimation state
struct NoiseFloorState {
    float noise_floor_ch1;         // Current noise floor estimate for CH1 (0-255 scale)
    float noise_floor_ch2;         // Current noise floor estimate for CH2 (0-255 scale)
    float smoothed_floor_ch1;      // Smoothed noise floor for CH1 (temporal filtering)
    float smoothed_floor_ch2;      // Smoothed noise floor for CH2 (temporal filtering)
    std::vector<uint8_t> sorted_buffer; // Temporary buffer for percentile calculation
    int update_counter;            // Counter for periodic updates
    bool initialized;              // Flag indicating if noise floor has been initialized
};

// DC offset correction state (EWMA-based)
struct DCOffsetState {
    float dc_i_ch1;                // Channel 1 I component DC offset
    float dc_q_ch1;                // Channel 1 Q component DC offset
    float dc_i_ch2;                // Channel 2 I component DC offset
    float dc_q_ch2;                // Channel 2 Q component DC offset
    uint64_t last_freq;            // Last frequency (for reset detection)
    int convergence_counter;       // Convergence tracking counter
};

// Overlap-add state for smoother spectrum
struct OverlapState {
    std::vector<float> overlap_buf_ch1;          // Channel 1 overlap buffer (interleaved I/Q pairs)
    std::vector<float> overlap_buf_ch2;          // Channel 2 overlap buffer (interleaved I/Q pairs)
    std::vector<uint8_t> prev_magnitude_ch1;     // Previous magnitude for averaging
    std::vector<uint8_t> prev_magnitude_ch2;     // Previous magnitude for averaging
    bool has_prev_fft;                           // Flag indicating if previous FFT exists
};

// IQ processing result
struct IQProcessingResult {
    int16_t peak_sample;           // Peak ADC sample value
    bool freq_changed;             // True if frequency changed (triggers resets)
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

// Initialize noise floor estimation state
void init_noise_floor(NoiseFloorState& nf, size_t fft_size);

// Update noise floor estimate using percentile method
// Args:
//   nf: Noise floor state structure
//   ch1_mag, ch2_mag: Magnitude arrays (0-255 scale)
//   len: Array length
//   percentile: Percentile to use for noise floor (typical: 10-25)
//   alpha: Smoothing factor for temporal filtering (0-1, typical: 0.05-0.2)
void update_noise_floor(NoiseFloorState& nf, const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                       size_t len, float percentile = 15.0f, float alpha = 0.1f);

// Get current noise floor estimates
// Returns: pair of (ch1_floor, ch2_floor) in 0-255 scale
void get_noise_floor(const NoiseFloorState& nf, float& ch1_floor, float& ch2_floor);

// Initialize DC offset state
void init_dc_offset(DCOffsetState& dc);

// Initialize overlap-add state
void init_overlap(OverlapState& overlap, size_t fft_size);

// Process IQ samples through complete pipeline: deinterleave, DC removal, window, FFT, magnitude
// Args:
//   iq_buffer: Interleaved IQ samples [I1,Q1,I2,Q2,I3,Q3,I4,Q4,...] (4 channels)
//   buffer_size: Number of IQ sample pairs in buffer (not total int16_t count)
//   fft_size: Size of FFT
//   current_freq: Current center frequency (for DC offset reset detection)
//   fft_in_ch1, fft_in_ch2: FFT input buffers (will be filled)
//   fft_out_ch1, fft_out_ch2: FFT output buffers (will be filled)
//   ch1_mag, ch2_mag: Output magnitude arrays (0-255 scale)
//   dc_state: DC offset correction state
//   overlap_state: Overlap-add state
//   window: Window function coefficients
//   plan_ch1, plan_ch2: FFTW plans for both channels
// Returns: IQProcessingResult with peak sample and frequency change flag
IQProcessingResult process_iq_to_fft(
    const int16_t* iq_buffer,
    size_t buffer_size,
    size_t fft_size,
    uint64_t current_freq,
    fftwf_complex* fft_in_ch1,
    fftwf_complex* fft_in_ch2,
    fftwf_complex* fft_out_ch1,
    fftwf_complex* fft_out_ch2,
    uint8_t* ch1_mag,
    uint8_t* ch2_mag,
    DCOffsetState& dc_state,
    OverlapState& overlap_state,
    const std::vector<float>& window,
    fftwf_plan plan_ch1,
    fftwf_plan plan_ch2
);

#endif // SIGNAL_PROCESSING_H
