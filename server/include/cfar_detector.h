#ifndef CFAR_DETECTOR_H
#define CFAR_DETECTOR_H

#include <cstddef>
#include <cstdint>
#include <vector>

// CFAR detection parameters
struct CFARParams {
    size_t training_cells;  // Number of training cells on each side
    size_t guard_cells;     // Number of guard cells on each side
    float threshold_db;     // Detection threshold above noise (dB)
    size_t min_signal_bins; // Minimum contiguous bins for valid signal
};

// Default CFAR parameters (balanced configuration)
constexpr CFARParams DEFAULT_CFAR = {
    .training_cells = 32,
    .guard_cells = 8,
    .threshold_db = 3.0f,
    .min_signal_bins = 5
};

// Signal detection result structure
struct SignalRegion {
    size_t start_bin;       // Starting bin index
    size_t end_bin;         // Ending bin index
    float integrated_power; // Sum of power across signal bins
    float avg_magnitude;    // Average magnitude
    size_t bin_count;       // Number of bins in signal
};

// Compute CFAR threshold for a single bin
float compute_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                             const CFARParams& params, size_t dc_exclusion_start,
                             size_t dc_exclusion_end);

// Detect signal regions using CA-CFAR with bandwidth integration
std::vector<SignalRegion> detect_signals_cfar(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                               size_t fft_size, const CFARParams& params,
                                               size_t bin_start, size_t bin_end);

#endif // CFAR_DETECTOR_H
