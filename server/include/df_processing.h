#ifndef DF_PROCESSING_H
#define DF_PROCESSING_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <fftw3.h>
#include "cfar_detector.h"

// Bin information for direction finding
struct BinInfo {
    size_t index;
    float magnitude;
    float phase_diff;
};

// Direction finding result structure
struct DFResult {
    float azimuth;              // Primary azimuth angle (0-360 degrees)
    float back_azimuth;         // Back azimuth (180° ambiguity)
    float phase_diff_deg;       // Phase difference in degrees
    float phase_std_deg;        // Phase standard deviation (quality metric)
    float confidence;           // Confidence percentage (0-100)
    float snr_db;               // Signal-to-noise ratio estimate (dB)
    float coherence;            // Coherence metric (0-1)
    bool is_holding;            // True if using held bearing (low confidence)
    size_t num_bins;            // Number of bins used for calculation
    size_t num_signals;         // Number of CFAR detected signals
};

// Kalman filter state for bearing smoothing
struct KalmanState {
    float azimuth;           // Estimated azimuth (degrees)
    float velocity;          // Angular velocity (degrees/sec)
    float P[2][2];           // Error covariance matrix
    bool initialized;        // Whether filter has been initialized
    uint64_t last_update_ms; // Timestamp of last update
};

// Last valid DoA state (for bearing hold and Kalman filtering)
struct LastValidDoA {
    bool has_valid;
    float azimuth;
    float back_azimuth;
    float phase_diff_deg;
    float phase_std_deg;
    float confidence;
    float snr_db;
    float coherence;
    uint32_t last_start_bin;  // Track bin range to detect selection changes
    uint32_t last_end_bin;
    KalmanState kalman;       // Kalman filter state for smoothing
};

// Perform complete direction finding analysis on FFT data
// Args:
//   fft_out_ch1, fft_out_ch2: FFT output from both channels
//   ch1_mag, ch2_mag: Magnitude arrays (0-255 scale)
//   fft_size: Size of FFT (typically 4096)
//   bin_start, bin_end: Frequency range to process (0 = full spectrum)
//   center_freq: Current center frequency in Hz (for calibration)
//   last_valid: Last valid DoA state (for bearing hold logic)
//   noise_floor_ch1, noise_floor_ch2: Optional noise floor estimates (< 0 to disable)
// Returns: DFResult with azimuth, confidence, and quality metrics
DFResult compute_direction_finding(
    const fftwf_complex* fft_out_ch1,
    const fftwf_complex* fft_out_ch2,
    const uint8_t* ch1_mag,
    const uint8_t* ch2_mag,
    size_t fft_size,
    size_t bin_start,
    size_t bin_end,
    uint64_t center_freq,
    LastValidDoA& last_valid,
    float noise_floor_ch1 = -1.0f,
    float noise_floor_ch2 = -1.0f
);

#endif // DF_PROCESSING_H
