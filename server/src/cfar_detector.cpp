#include "cfar_detector.h"
#include <algorithm>
#include <vector>

float compute_cfar_threshold(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                             const CFARParams& params, size_t dc_exclusion_start,
                             size_t dc_exclusion_end) {
    // CA-CFAR: Average power in training cells (excluding guard cells and DC region)

    float noise_sum = 0.0f;
    size_t noise_count = 0;

    // Left training cells
    const size_t left_start = (bin_idx > params.training_cells + params.guard_cells) ?
                              bin_idx - params.training_cells - params.guard_cells : 0;
    const size_t left_end = (bin_idx > params.guard_cells) ?
                            bin_idx - params.guard_cells : 0;

    for (size_t i = left_start; i < left_end && i < fft_size; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;

        noise_sum += magnitude[i];
        noise_count++;
    }

    // Right training cells
    const size_t right_start = std::min(bin_idx + params.guard_cells + 1, fft_size);
    const size_t right_end = std::min(bin_idx + params.guard_cells + params.training_cells + 1, fft_size);

    for (size_t i = right_start; i < right_end && i < fft_size; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) continue;

        noise_sum += magnitude[i];
        noise_count++;
    }

    if (noise_count == 0) return 255.0f;  // No valid training cells

    // Average noise level (in uint8_t scale)
    const float noise_level = noise_sum / noise_count;

    // Convert noise level from uint8_t scale to dB
    // Magnitudes are in uint8_t scale: 0-255 represents 120 dB range (-100 to +20 dBFS)
    const float noise_db = (noise_level / 255.0f) * 120.0f - 100.0f;

    // Add threshold in dB
    const float threshold_db = noise_db + params.threshold_db;

    // Convert back to uint8_t scale
    const float threshold = (threshold_db + 100.0f) * (255.0f / 120.0f);

    return threshold;
}

std::vector<SignalRegion> detect_signals_cfar(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                               size_t fft_size, const CFARParams& params,
                                               size_t bin_start, size_t bin_end) {
    std::vector<SignalRegion> signals;

    // DC exclusion zone (center bin ± margin)
    const size_t dc_center = fft_size / 2;
    const size_t dc_margin = 10;  // Exclude ±10 bins around DC
    const size_t dc_exclusion_start = (dc_center > dc_margin) ? dc_center - dc_margin : 0;
    const size_t dc_exclusion_end = std::min(dc_center + dc_margin, fft_size - 1);

    // Detection flags for each bin
    std::vector<bool> detected(fft_size, false);

    // Run CFAR detection on each bin
    for (size_t i = bin_start; i <= bin_end; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) {
            continue;
        }

        // Average magnitude from both channels
        const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;

        // Compute CFAR threshold for this bin
        const float threshold = compute_cfar_threshold(ch1_mag, i, fft_size, params,
                                                       dc_exclusion_start, dc_exclusion_end);

        // Detect if magnitude exceeds threshold
        if (avg_mag > threshold) {
            detected[i] = true;
        }
    }

    // Group contiguous detected bins into signal regions
    bool in_signal = false;
    SignalRegion current_signal = {0, 0, 0.0f, 0.0f, 0};

    for (size_t i = bin_start; i <= bin_end; i++) {
        if (detected[i]) {
            if (!in_signal) {
                // Start new signal region
                current_signal.start_bin = i;
                current_signal.integrated_power = 0.0f;
                current_signal.bin_count = 0;
                in_signal = true;
            }

            // Accumulate signal power
            const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;
            current_signal.integrated_power += avg_mag;
            current_signal.bin_count++;
            current_signal.end_bin = i;

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

float compute_cfar_threshold_with_floor(const uint8_t* magnitude, size_t bin_idx, size_t fft_size,
                                        const CFARParams& params, size_t dc_exclusion_start,
                                        size_t dc_exclusion_end, float noise_floor) {
    // If noise floor is provided (>= 0), blend it with local CFAR estimate
    // This provides more stable thresholds especially in low SNR conditions

    if (noise_floor < 0.0f) {
        // Fall back to standard CFAR
        return compute_cfar_threshold(magnitude, bin_idx, fft_size, params,
                                     dc_exclusion_start, dc_exclusion_end);
    }

    // Compute local noise estimate using standard CFAR
    float local_threshold = compute_cfar_threshold(magnitude, bin_idx, fft_size, params,
                                                   dc_exclusion_start, dc_exclusion_end);

    // Blend global noise floor with local estimate (70% global, 30% local)
    // This maintains CFAR's adaptability while stabilizing against local variations

    // Convert global noise floor from uint8_t scale to dB and add threshold
    const float noise_floor_db = (noise_floor / 255.0f) * 120.0f - 100.0f;
    const float global_threshold_db = noise_floor_db + params.threshold_db;
    const float global_threshold = (global_threshold_db + 100.0f) * (255.0f / 120.0f);

    return 0.7f * global_threshold + 0.3f * local_threshold;
}

std::vector<SignalRegion> detect_signals_cfar_with_floor(const uint8_t* ch1_mag, const uint8_t* ch2_mag,
                                                          size_t fft_size, const CFARParams& params,
                                                          size_t bin_start, size_t bin_end,
                                                          float noise_floor_ch1, float noise_floor_ch2) {
    std::vector<SignalRegion> signals;

    // DC exclusion zone (center bin ± margin)
    const size_t dc_center = fft_size / 2;
    const size_t dc_margin = 10;  // Exclude ±10 bins around DC
    const size_t dc_exclusion_start = (dc_center > dc_margin) ? dc_center - dc_margin : 0;
    const size_t dc_exclusion_end = std::min(dc_center + dc_margin, fft_size - 1);

    // Detection flags for each bin
    std::vector<bool> detected(fft_size, false);

    // Average noise floor across channels
    const float avg_noise_floor = (noise_floor_ch1 + noise_floor_ch2) / 2.0f;

    // Run CFAR detection on each bin
    for (size_t i = bin_start; i <= bin_end; i++) {
        // Skip DC exclusion zone
        if (i >= dc_exclusion_start && i <= dc_exclusion_end) {
            continue;
        }

        // Average magnitude from both channels
        const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;

        // Compute CFAR threshold with noise floor for this bin
        const float threshold = compute_cfar_threshold_with_floor(ch1_mag, i, fft_size, params,
                                                                  dc_exclusion_start, dc_exclusion_end,
                                                                  avg_noise_floor);

        // Detect if magnitude exceeds threshold
        if (avg_mag > threshold) {
            detected[i] = true;
        }
    }

    // Group contiguous detected bins into signal regions
    bool in_signal = false;
    SignalRegion current_signal = {0, 0, 0.0f, 0.0f, 0};

    for (size_t i = bin_start; i <= bin_end; i++) {
        if (detected[i]) {
            if (!in_signal) {
                // Start new signal region
                current_signal.start_bin = i;
                current_signal.integrated_power = 0.0f;
                current_signal.bin_count = 0;
                in_signal = true;
            }

            // Accumulate signal power
            const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;
            current_signal.integrated_power += avg_mag;
            current_signal.bin_count++;
            current_signal.end_bin = i;

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
