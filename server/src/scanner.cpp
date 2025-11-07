#include "scanner.h"
#include <algorithm>

SpectrumPeak find_spectrum_peak(const uint8_t* magnitude, size_t fft_size) {
    SpectrumPeak peak{0, 0};

    for (size_t i = 0; i < fft_size; i++) {
        if (magnitude[i] > peak.magnitude) {
            peak.magnitude = magnitude[i];
            peak.bin = i;
        }
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
