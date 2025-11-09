/**
 * @file os_cfar_detector.h
 * @brief Ordered Statistic CFAR (OS-CFAR) Detector
 *
 * OS-CFAR provides better performance than CA-CFAR in multi-target scenarios
 * by using order statistics (median/percentile) instead of averaging.
 * This makes it more robust to interfering targets in the training cells.
 *
 * Performance comparison:
 * - CA-CFAR: Fast, but degrades with multiple targets close together
 * - OS-CFAR: Slower, but maintains performance with interfering targets
 * - GO-CFAR: Greatest-of selection between leading/trailing cells
 * - SO-CFAR: Smallest-of selection (conservative)
 */

#ifndef OS_CFAR_DETECTOR_H
#define OS_CFAR_DETECTOR_H

#include <cstddef>
#include <cstdint>
#include <vector>

// OS-CFAR detection parameters
struct OSCFARParams {
    size_t training_cells;      // Number of training cells on each side
    size_t guard_cells;         // Number of guard cells on each side
    float threshold_db;         // Detection threshold above noise (dB)
    size_t min_signal_bins;     // Minimum contiguous bins for valid signal
    float k_percentile;         // Percentile for order statistic (0-1, typical: 0.75 for 75th percentile)
    bool use_go_cfar;           // Use Greatest-Of CFAR (max of leading/trailing)
    bool use_so_cfar;           // Use Smallest-Of CFAR (min of leading/trailing)
};

// Default OS-CFAR parameters (75th percentile, robust to 25% interferers)
constexpr OSCFARParams DEFAULT_OS_CFAR = {
    .training_cells = 32,
    .guard_cells = 8,
    .threshold_db = 3.0f,
    .min_signal_bins = 5,
    .k_percentile = 0.75f,      // 75th percentile (k = 3/4 * training_cells)
    .use_go_cfar = false,
    .use_so_cfar = false
};

// More aggressive OS-CFAR for high-interference environments
constexpr OSCFARParams AGGRESSIVE_OS_CFAR = {
    .training_cells = 48,       // More training cells
    .guard_cells = 12,          // More guard cells
    .threshold_db = 4.0f,       // Higher threshold
    .min_signal_bins = 7,       // Longer minimum signal
    .k_percentile = 0.90f,      // 90th percentile (very robust to interference)
    .use_go_cfar = true,        // Use GO-CFAR for conservative detection
    .use_so_cfar = false
};

// Signal detection result structure (same as CA-CFAR)
struct SignalRegion {
    size_t start_bin;           // Starting bin index
    size_t end_bin;             // Ending bin index
    float integrated_power;     // Sum of power across signal bins
    float avg_magnitude;        // Average magnitude
    size_t bin_count;           // Number of bins in signal
    float snr_db;               // Estimated SNR in dB
    float peak_magnitude;       // Peak magnitude in region
};

/**
 * @brief Compute OS-CFAR threshold for a single bin
 *
 * Uses order statistics instead of averaging. The k-th order statistic
 * is selected from the sorted training cells.
 *
 * @param magnitude FFT magnitude array (0-255 scale)
 * @param bin_idx Bin index to compute threshold for
 * @param fft_size Total FFT size
 * @param params OS-CFAR parameters
 * @param dc_exclusion_start DC exclusion zone start
 * @param dc_exclusion_end DC exclusion zone end
 * @return Threshold value in 0-255 scale
 */
float compute_os_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end);

/**
 * @brief Compute GO-CFAR threshold (Greatest-Of)
 *
 * Takes the maximum of leading and trailing window estimates.
 * More conservative than CA-CFAR, good for edge detection.
 */
float compute_go_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end);

/**
 * @brief Compute SO-CFAR threshold (Smallest-Of)
 *
 * Takes the minimum of leading and trailing window estimates.
 * Most conservative, good for avoiding false alarms.
 */
float compute_so_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end);

/**
 * @brief Detect signal regions using OS-CFAR with bandwidth integration
 *
 * Improved multi-target detection using order statistics.
 * Better performance than CA-CFAR when multiple targets are present.
 *
 * @param ch1_mag Channel 1 FFT magnitude (0-255 scale)
 * @param ch2_mag Channel 2 FFT magnitude (0-255 scale)
 * @param fft_size Total FFT size
 * @param params OS-CFAR parameters
 * @param bin_start Start bin for detection region
 * @param bin_end End bin for detection region
 * @return Vector of detected signal regions
 */
std::vector<SignalRegion> detect_signals_os_cfar(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                                  size_t fft_size, const OSCFARParams& params,
                                                  size_t bin_start, size_t bin_end);

/**
 * @brief Detect signal regions using OS-CFAR with dynamic noise floor
 *
 * Uses global noise floor estimate to improve threshold calculation.
 * Blends OS-CFAR local estimate with global noise floor.
 *
 * @param ch1_mag Channel 1 FFT magnitude (0-255 scale)
 * @param ch2_mag Channel 2 FFT magnitude (0-255 scale)
 * @param fft_size Total FFT size
 * @param params OS-CFAR parameters
 * @param bin_start Start bin for detection region
 * @param bin_end End bin for detection region
 * @param noise_floor_ch1 Global noise floor for CH1 (0-255 scale, < 0 to disable)
 * @param noise_floor_ch2 Global noise floor for CH2 (0-255 scale, < 0 to disable)
 * @return Vector of detected signal regions
 */
std::vector<SignalRegion> detect_signals_os_cfar_with_floor(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                                             size_t fft_size, const OSCFARParams& params,
                                                             size_t bin_start, size_t bin_end,
                                                             float noise_floor_ch1, float noise_floor_ch2);

/**
 * @brief Estimate SNR for detected signal region
 *
 * @param magnitude FFT magnitude array
 * @param signal Detected signal region
 * @param noise_floor Estimated noise floor (0-255 scale)
 * @return SNR in dB
 */
float estimate_signal_snr(const uint8_t* magnitude, const SignalRegion& signal, float noise_floor);

#endif // OS_CFAR_DETECTOR_H
