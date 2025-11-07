#include "scanner.h"
#include <algorithm>
#include <cmath>

SpectrumPeak find_spectrum_peak(const uint8_t* magnitude, size_t fft_size) {
    SpectrumPeak peak{0, 0, 0.0f};

    // Find integer bin with maximum magnitude
    for (size_t i = 0; i < fft_size; i++) {
        if (magnitude[i] > peak.magnitude) {
            peak.magnitude = magnitude[i];
            peak.bin = i;
        }
    }

    // Apply parabolic interpolation for sub-bin accuracy
    // Uses neighboring bins to estimate true peak location
    if (peak.bin > 0 && peak.bin < fft_size - 1) {
        const float alpha = magnitude[peak.bin - 1];
        const float beta = magnitude[peak.bin];
        const float gamma = magnitude[peak.bin + 1];

        // Parabolic interpolation formula
        // delta represents fractional bin offset from integer peak
        const float denominator = alpha - 2.0f * beta + gamma;

        if (std::abs(denominator) > 1e-6f) {  // Avoid division by zero
            const float delta = 0.5f * (alpha - gamma) / denominator;

            // Clamp to reasonable range [-0.5, 0.5]
            const float clamped_delta = std::max(-0.5f, std::min(0.5f, delta));

            peak.interpolated_bin = peak.bin + clamped_delta;
        } else {
            // Fall back to integer bin if parabola is degenerate
            peak.interpolated_bin = static_cast<float>(peak.bin);
        }
    } else {
        // Edge bins - no interpolation possible
        peak.interpolated_bin = static_cast<float>(peak.bin);
    }

    return peak;
}

float magnitude_to_dbm(uint8_t magnitude) {
    // Convert 8-bit magnitude to approximate dBm
    // Assuming 0-255 maps to roughly -120 to 0 dBm range
    return (magnitude / 255.0f) * 120.0f - 120.0f;
}

uint64_t bin_to_frequency(size_t bin, size_t fft_size, uint64_t center_freq, uint32_t sample_rate) {
    // Calculate signal frequency (peak bin relative to center)
    const int64_t bin_offset = static_cast<int64_t>(bin) - static_cast<int64_t>(fft_size / 2);
    return center_freq + (bin_offset * sample_rate / fft_size);
}

double interpolated_bin_to_frequency(float bin, size_t fft_size, uint64_t center_freq, uint32_t sample_rate) {
    // Calculate signal frequency with sub-bin accuracy
    const double bin_offset = static_cast<double>(bin) - static_cast<double>(fft_size / 2);
    const double freq_per_bin = static_cast<double>(sample_rate) / static_cast<double>(fft_size);
    return static_cast<double>(center_freq) + (bin_offset * freq_per_bin);
}

float estimate_bandwidth(const uint8_t* magnitude, size_t fft_size, uint8_t peak_mag, uint32_t sample_rate) {
    // Estimate bandwidth (count bins above threshold - 6dB)
    const uint8_t bw_threshold = peak_mag > 12 ? peak_mag - 12 : 0;
    size_t bw_bins = 0;

    for (size_t i = 0; i < fft_size; i++) {
        if (magnitude[i] >= bw_threshold) {
            bw_bins++;
        }
    }

    return (bw_bins * sample_rate) / static_cast<float>(fft_size);
}

SignalCharacteristics analyze_spectrum(
    const uint8_t* magnitude,
    size_t fft_size,
    uint64_t center_freq,
    uint32_t sample_rate
) {
    SignalCharacteristics characteristics;

    // Find peak in spectrum
    SpectrumPeak peak = find_spectrum_peak(magnitude, fft_size);

    // Convert magnitude to dBm
    characteristics.power_dbm = magnitude_to_dbm(peak.magnitude);

    // Calculate signal frequency
    characteristics.frequency = bin_to_frequency(peak.bin, fft_size, center_freq, sample_rate);

    // Estimate bandwidth
    characteristics.bandwidth_hz = estimate_bandwidth(magnitude, fft_size, peak.magnitude, sample_rate);

    return characteristics;
}
