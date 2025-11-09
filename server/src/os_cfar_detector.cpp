/**
 * @file os_cfar_detector.cpp
 * @brief OS-CFAR implementation for robust multi-target detection
 */

#include "os_cfar_detector.h"
#include <algorithm>
#include <vector>
#include <cmath>

/**
 * @brief Quick select algorithm for finding k-th order statistic
 * Average O(n) performance, faster than full sort for percentile calculation
 */
static float quickselect_kth(std::vector<uint8_t>& arr, size_t k) {
    if (arr.empty()) return 0.0f;
    if (k >= arr.size()) k = arr.size() - 1;

    // Use std::nth_element for efficient partial sorting
    std::nth_element(arr.begin(), arr.begin() + k, arr.end());
    return static_cast<float>(arr[k]);
}

float compute_os_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end) {
    // Collect training cell samples
    std::vector<uint8_t> training_samples;
    training_samples.reserve(params.training_cells * 2);

    // Left training cells
    const size_t left_start = (bin_idx > params.training_cells + params.guard_cells) ?
                              bin_idx - params.training_cells - params.guard_cells : 0;
    const size_t left_end = (bin_idx > params.guard_cells) ?
                            bin_idx - params.guard_cells : 0;

    for (size_t i = left_start; i < left_end && i < fft_size; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        training_samples.push_back(magnitude[i]);
    }

    // Right training cells
    const size_t right_start = std::min(bin_idx + params.guard_cells + 1, fft_size);
    const size_t right_end = std::min(bin_idx + params.guard_cells + params.training_cells + 1, fft_size);

    for (size_t i = right_start; i < right_end && i < fft_size; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        training_samples.push_back(magnitude[i]);
    }

    if (training_samples.empty()) return 255.0f;  // No valid training cells

    // Calculate k-th order statistic (percentile)
    const size_t k = static_cast<size_t>(params.k_percentile * training_samples.size());
    const float noise_level = quickselect_kth(training_samples, k);

    // Convert noise level from uint8_t scale to dB
    // Magnitudes are in uint8_t scale: 0-255 represents 120 dB range (-100 to +20 dBFS)
    const float noise_db = (noise_level / 255.0f) * 120.0f - 100.0f;

    // Add threshold in dB
    const float threshold_db = noise_db + params.threshold_db;

    // Convert back to uint8_t scale
    const float threshold = (threshold_db + 100.0f) * (255.0f / 120.0f);

    return std::max(0.0f, std::min(255.0f, threshold));
}

float compute_go_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end) {
    // Leading window (left side)
    std::vector<uint8_t> leading_samples;
    leading_samples.reserve(params.training_cells);

    const size_t left_start = (bin_idx > params.training_cells + params.guard_cells) ?
                              bin_idx - params.training_cells - params.guard_cells : 0;
    const size_t left_end = (bin_idx > params.guard_cells) ?
                            bin_idx - params.guard_cells : 0;

    for (size_t i = left_start; i < left_end && i < fft_size; i++) {
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        leading_samples.push_back(magnitude[i]);
    }

    // Trailing window (right side)
    std::vector<uint8_t> trailing_samples;
    trailing_samples.reserve(params.training_cells);

    const size_t right_start = std::min(bin_idx + params.guard_cells + 1, fft_size);
    const size_t right_end = std::min(bin_idx + params.guard_cells + params.training_cells + 1, fft_size);

    for (size_t i = right_start; i < right_end && i < fft_size; i++) {
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        trailing_samples.push_back(magnitude[i]);
    }

    if (leading_samples.empty() && trailing_samples.empty()) return 255.0f;

    // Calculate k-th order statistic for each window
    const size_t k = static_cast<size_t>(params.k_percentile * params.training_cells);

    float leading_noise = 0.0f;
    float trailing_noise = 0.0f;

    if (!leading_samples.empty()) {
        const size_t k_lead = std::min(k, leading_samples.size() - 1);
        leading_noise = quickselect_kth(leading_samples, k_lead);
    }

    if (!trailing_samples.empty()) {
        const size_t k_trail = std::min(k, trailing_samples.size() - 1);
        trailing_noise = quickselect_kth(trailing_samples, k_trail);
    }

    // Take maximum (greatest-of)
    const float noise_level = std::max(leading_noise, trailing_noise);

    // Convert to dB and add threshold
    const float noise_db = (noise_level / 255.0f) * 120.0f - 100.0f;
    const float threshold_db = noise_db + params.threshold_db;
    const float threshold = (threshold_db + 100.0f) * (255.0f / 120.0f);

    return std::max(0.0f, std::min(255.0f, threshold));
}

float compute_so_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                const OSCFARParams& params, size_t dc_exclusion_start,
                                size_t dc_exclusion_end) {
    // Same as GO-CFAR but takes minimum instead of maximum
    std::vector<uint8_t> leading_samples;
    leading_samples.reserve(params.training_cells);

    const size_t left_start = (bin_idx > params.training_cells + params.guard_cells) ?
                              bin_idx - params.training_cells - params.guard_cells : 0;
    const size_t left_end = (bin_idx > params.guard_cells) ?
                            bin_idx - params.guard_cells : 0;

    for (size_t i = left_start; i < left_end && i < fft_size; i++) {
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        leading_samples.push_back(magnitude[i]);
    }

    std::vector<uint8_t> trailing_samples;
    trailing_samples.reserve(params.training_cells);

    const size_t right_start = std::min(bin_idx + params.guard_cells + 1, fft_size);
    const size_t right_end = std::min(bin_idx + params.guard_cells + params.training_cells + 1, fft_size);

    for (size_t i = right_start; i < right_end && i < fft_size; i++) {
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;
        trailing_samples.push_back(magnitude[i]);
    }

    if (leading_samples.empty() && trailing_samples.empty()) return 255.0f;

    const size_t k = static_cast<size_t>(params.k_percentile * params.training_cells);

    float leading_noise = 255.0f;
    float trailing_noise = 255.0f;

    if (!leading_samples.empty()) {
        const size_t k_lead = std::min(k, leading_samples.size() - 1);
        leading_noise = quickselect_kth(leading_samples, k_lead);
    }

    if (!trailing_samples.empty()) {
        const size_t k_trail = std::min(k, trailing_samples.size() - 1);
        trailing_noise = quickselect_kth(trailing_samples, k_trail);
    }

    // Take minimum (smallest-of)
    const float noise_level = std::min(leading_noise, trailing_noise);

    const float noise_db = (noise_level / 255.0f) * 120.0f - 100.0f;
    const float threshold_db = noise_db + params.threshold_db;
    const float threshold = (threshold_db + 100.0f) * (255.0f / 120.0f);

    return std::max(0.0f, std::min(255.0f, threshold));
}

std::vector<SignalRegion> detect_signals_os_cfar(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                                  size_t fft_size, const OSCFARParams& params,
                                                  size_t bin_start, size_t bin_end) {
    std::vector<SignalRegion> signals;

    // DC exclusion zone (center bin ± margin)
    const size_t dc_center = fft_size / 2;
    const size_t dc_margin = 10;
    const size_t dc_exclusion_start = (dc_center > dc_margin) ? dc_center - dc_margin : 0;
    const size_t dc_exclusion_end = std::min(dc_center + dc_margin, fft_size - 1);

    // Detection flags for each bin
    std::vector<bool> detected(fft_size, false);

    // Select CFAR variant
    auto threshold_func = compute_os_cfar_threshold;
    if (params.use_go_cfar) {
        threshold_func = compute_go_cfar_threshold;
    } else if (params.use_so_cfar) {
        threshold_func = compute_so_cfar_threshold;
    }

    // Run OS-CFAR detection on each bin
    for (size_t i = bin_start; i <= bin_end; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) {
            continue;
        }

        // Average magnitude from both channels
        const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;

        // Compute threshold for this bin
        const float threshold = threshold_func(ch1_mag, i, fft_size, params,
                                              dc_exclusion_start, dc_exclusion_end);

        // Detect if magnitude exceeds threshold
        if (avg_mag > threshold) {
            detected[i] = true;
        }
    }

    // Group contiguous detected bins into signal regions
    bool in_signal = false;
    SignalRegion current_signal = {0, 0, 0.0f, 0.0f, 0, 0.0f, 0.0f};

    for (size_t i = bin_start; i <= bin_end; i++) {
        if (detected[i]) {
            if (!in_signal) {
                // Start new signal region
                current_signal.start_bin = i;
                current_signal.integrated_power = 0.0f;
                current_signal.peak_magnitude = 0.0f;
                current_signal.bin_count = 0;
                in_signal = true;
            }

            // Accumulate signal power
            const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;
            current_signal.integrated_power += avg_mag;
            current_signal.bin_count++;
            current_signal.end_bin = i;

            // Track peak magnitude
            if (avg_mag > current_signal.peak_magnitude) {
                current_signal.peak_magnitude = avg_mag;
            }

        } else {
            if (in_signal) {
                // End of signal region
                current_signal.avg_magnitude = current_signal.integrated_power / current_signal.bin_count;

                // Only accept signals with minimum bandwidth
                if (current_signal.bin_count >= params.min_signal_bins) {
                    signals.push_back(current_signal);
                }

                in_signal = false;
            }
        }
    }

    // Handle signal extending to end of range
    if (in_signal) {
        current_signal.avg_magnitude = current_signal.integrated_power / current_signal.bin_count;

        if (current_signal.bin_count >= params.min_signal_bins) {
            signals.push_back(current_signal);
        }
    }

    return signals;
}

std::vector<SignalRegion> detect_signals_os_cfar_with_floor(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                                             size_t fft_size, const OSCFARParams& params,
                                                             size_t bin_start, size_t bin_end,
                                                             float noise_floor_ch1, float noise_floor_ch2) {
    // If no noise floor provided, fall back to standard OS-CFAR
    if (noise_floor_ch1 < 0.0f || noise_floor_ch2 < 0.0f) {
        return detect_signals_os_cfar(ch1_mag, ch2_mag, fft_size, params, bin_start, bin_end);
    }

    // Run standard OS-CFAR detection
    auto signals = detect_signals_os_cfar(ch1_mag, ch2_mag, fft_size, params, bin_start, bin_end);

    // Calculate average noise floor
    const float avg_noise_floor = (noise_floor_ch1 + noise_floor_ch2) / 2.0f;

    // Estimate SNR for each detected signal
    for (auto& signal : signals) {
        signal.snr_db = estimate_signal_snr(ch1_mag, signal, avg_noise_floor);
    }

    return signals;
}

float estimate_signal_snr(const uint8_t* magnitude, const SignalRegion& signal, float noise_floor) {
    (void)magnitude;  // Reserved for future use (could calculate SNR from magnitude array instead of peak)

    // Use peak magnitude for SNR calculation
    const float signal_level = signal.peak_magnitude;

    // Convert to dB scale
    const float signal_db = (signal_level / 255.0f) * 120.0f - 100.0f;
    const float noise_db = (noise_floor / 255.0f) * 120.0f - 100.0f;

    // SNR = Signal - Noise (in dB)
    const float snr = signal_db - noise_db;

    return snr;
}
