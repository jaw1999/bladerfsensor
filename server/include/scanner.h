#ifndef SCANNER_H
#define SCANNER_H

#include <cstdint>
#include <cstddef>

// Result of peak finding in spectrum
struct SpectrumPeak {
    uint8_t magnitude;             // Peak magnitude value (0-255)
    size_t bin;                    // FFT bin index of peak (integer)
    float interpolated_bin;        // Interpolated bin location (sub-bin accuracy)
};

// Signal characteristics derived from FFT analysis
struct SignalCharacteristics {
    uint64_t frequency;            // Signal center frequency (Hz)
    float power_dbm;               // Signal power (dBm)
    float bandwidth_hz;            // Estimated bandwidth (Hz)
};

// Find peak magnitude and bin location in spectrum
SpectrumPeak find_spectrum_peak(const uint8_t* magnitude, size_t fft_size);

// Convert 8-bit magnitude to approximate dBm
// Assumes 0-255 maps to roughly -120 to 0 dBm range
float magnitude_to_dbm(uint8_t magnitude);

// Calculate signal frequency from FFT bin offset (integer bin)
uint64_t bin_to_frequency(size_t bin, size_t fft_size, uint64_t center_freq, uint32_t sample_rate);

// Calculate signal frequency from interpolated bin (sub-bin accuracy)
double interpolated_bin_to_frequency(float bin, size_t fft_size, uint64_t center_freq, uint32_t sample_rate);

// Estimate bandwidth by counting bins above threshold (-6dB from peak)
float estimate_bandwidth(const uint8_t* magnitude, size_t fft_size, uint8_t peak_mag, uint32_t sample_rate);

// Analyze spectrum and extract signal characteristics
SignalCharacteristics analyze_spectrum(
    const uint8_t* magnitude,
    size_t fft_size,
    uint64_t center_freq,
    uint32_t sample_rate
);

#endif // SCANNER_H
