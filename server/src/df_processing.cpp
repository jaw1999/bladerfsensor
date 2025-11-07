#include "df_processing.h"
#include "array_calibration.h"
#include <cmath>
#include <algorithm>

DFResult compute_direction_finding(
    const fftwf_complex* fft_out_ch1,
    const fftwf_complex* fft_out_ch2,
    const uint8_t* ch1_mag,
    const uint8_t* ch2_mag,
    size_t fft_size,
    size_t bin_start,
    size_t bin_end,
    uint64_t center_freq,
    LastValidDoA& last_valid
) {
    // Detect selection changes and reset bearing hold
    if (last_valid.has_valid &&
        (last_valid.last_start_bin != static_cast<uint32_t>(bin_start) ||
         last_valid.last_end_bin != static_cast<uint32_t>(bin_end))) {
        last_valid.has_valid = false;
    }

    // Calculate bin count for statistics
    const size_t bin_count = (bin_end >= bin_start) ? (bin_end - bin_start + 1) : 1;

    // ===== CA-CFAR SIGNAL DETECTION =====
    // Use bandwidth-integrated CFAR to detect real signals, reject noise spikes and DC

    // Detect signal regions using CFAR
    std::vector<SignalRegion> detected_signals = detect_signals_cfar(ch1_mag, ch2_mag, fft_size,
                                                                      DEFAULT_CFAR, bin_start, bin_end);

    // Collect all bins from detected signal regions with their phase differences
    std::vector<BinInfo> strong_bins;
    strong_bins.reserve(fft_size / 4);  // Preallocate reasonable size

    for (const auto& signal : detected_signals) {
        // For each detected signal region, extract phase information from all bins
        for (size_t i = signal.start_bin; i <= signal.end_bin; i++) {
            // Average magnitude from both channels
            const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;

            // Compute phase from FFT output (frequency domain)
            const float phase1 = atan2f(fft_out_ch1[i][1], fft_out_ch1[i][0]);
            const float phase2 = atan2f(fft_out_ch2[i][1], fft_out_ch2[i][0]);

            // Phase difference (CH2 - CH1)
            float diff = phase2 - phase1;

            // Wrap to [-π, π] to handle 2π ambiguity
            while (diff > M_PI) diff -= 2.0f * M_PI;
            while (diff < -M_PI) diff += 2.0f * M_PI;

            strong_bins.push_back({i, avg_mag, diff});
        }
    }

    // Calculate statistics for debugging
    uint32_t magnitude_sum = 0;
    uint8_t peak_mag = 0;
    for (size_t i = bin_start; i <= bin_end; i++) {
        const uint8_t avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2;
        magnitude_sum += avg_mag;
        if (avg_mag > peak_mag) peak_mag = avg_mag;
    }
    const uint8_t mean_mag = magnitude_sum / bin_count;

    constexpr size_t min_bins_for_df = 3;  // Need at least 3 bins for reliable DF

    // Default values if no strong signals present
    float avg_phase_diff_rad = 0.0f;
    float avg_phase_diff_deg = 0.0f;
    float std_dev_rad = M_PI;  // Maximum uncertainty
    float std_dev_deg = 180.0f;

    if (strong_bins.size() >= min_bins_for_df) {
        // Compute weighted average phase difference
        // Weight by signal magnitude for better accuracy on strong signals
        float weighted_sum = 0.0f;
        float weight_total = 0.0f;

        for (const auto& bin : strong_bins) {
            const float weight = bin.magnitude;
            weighted_sum += bin.phase_diff * weight;
            weight_total += weight;
        }

        avg_phase_diff_rad = weighted_sum / weight_total;
        avg_phase_diff_deg = avg_phase_diff_rad * 180.0f / M_PI;

        // Apply array calibration correction (frequency-dependent phase error)
        const float phase_correction = get_phase_correction(center_freq);
        avg_phase_diff_deg += phase_correction;
        avg_phase_diff_rad = avg_phase_diff_deg * M_PI / 180.0f;

        // Calculate weighted standard deviation (measure of phase coherence)
        float variance = 0.0f;
        for (const auto& bin : strong_bins) {
            float diff = bin.phase_diff - avg_phase_diff_rad;
            // Handle phase wrapping in variance calculation
            while (diff > M_PI) diff -= 2.0f * M_PI;
            while (diff < -M_PI) diff += 2.0f * M_PI;
            const float weight = bin.magnitude;
            variance += weight * diff * diff;
        }
        std_dev_rad = std::sqrt(variance / weight_total);
        std_dev_deg = std_dev_rad * 180.0f / M_PI;
    }

    // Convert phase difference to angle of arrival
    // Antenna spacing assumption: 0.5 wavelengths (typical for DF)
    // For bladeRF at 915 MHz: λ = c/f = 0.328m, so 0.5λ = 0.164m = 164mm
    constexpr float antenna_spacing_wavelengths = 0.5f;
    constexpr float lambda = 1.0f;  // Normalized (spacing already in wavelengths)

    // Apply interferometer equation: sin(θ) = (Δφ * λ) / (2π * d)
    float sin_theta = (avg_phase_diff_rad * lambda) / (2.0f * M_PI * antenna_spacing_wavelengths);
    sin_theta = std::max(-1.0f, std::min(1.0f, sin_theta));  // Clamp to [-1, 1]

    // Calculate azimuth using atan2 for full [-180°, 180°] range
    // This provides better angle resolution across all quadrants than asin alone
    float cos_theta_sq = 1.0f - sin_theta * sin_theta;
    float cos_theta = std::sqrt(std::max(0.0f, cos_theta_sq));

    // Use atan2(sin, cos) for proper quadrant handling
    // Note: We compute both positive and negative cos_theta solutions
    float azimuth_rad_pos = std::atan2(sin_theta, cos_theta);   // [0°, 180°] solution
    float azimuth_rad_neg = std::atan2(sin_theta, -cos_theta);  // [180°, 360°] solution

    float azimuth_deg_pos = azimuth_rad_pos * 180.0f / M_PI;
    float azimuth_deg_neg = azimuth_rad_neg * 180.0f / M_PI;

    // For display, we present the solution in [0°, 180°] as primary
    // and its 180° complement as the ambiguous back azimuth
    float azimuth_deg = azimuth_deg_pos;
    float back_azimuth_deg = azimuth_deg_neg;

    // Normalize to [0, 360) for display
    float azimuth_norm = azimuth_deg < 0 ? azimuth_deg + 360.0f : azimuth_deg;
    float back_azimuth_norm = back_azimuth_deg < 0 ? back_azimuth_deg + 360.0f : back_azimuth_deg;

    // Ensure both azimuths are in [0, 360)
    azimuth_norm = std::fmod(azimuth_norm + 360.0f, 360.0f);
    back_azimuth_norm = std::fmod(back_azimuth_norm + 360.0f, 360.0f);

    // Calculate SNR estimate from FFT bins (frequency domain)
    // Use the average power of strong signal bins vs noise floor
    float signal_power = 0.0f;
    float noise_power = 0.0f;
    size_t noise_bin_count = 0;

    if (strong_bins.size() >= min_bins_for_df) {
        // Calculate average signal power from strong bins
        for (const auto& bin : strong_bins) {
            // Convert FFT output to power: |FFT[k]|^2 = real^2 + imag^2
            const float real1 = fft_out_ch1[bin.index][0];
            const float imag1 = fft_out_ch1[bin.index][1];
            signal_power += (real1 * real1 + imag1 * imag1);
        }
        signal_power /= strong_bins.size();

        // Estimate noise floor from bins below mean (noise reference)
        // Only use bins within selected range for fair comparison
        for (size_t i = bin_start; i <= bin_end; i++) {
            const uint8_t avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2;
            if (avg_mag <= mean_mag) {  // Use bins at or below mean as noise reference
                const float real1 = fft_out_ch1[i][0];
                const float imag1 = fft_out_ch1[i][1];
                noise_power += (real1 * real1 + imag1 * imag1);
                noise_bin_count++;
            }
        }

        if (noise_bin_count > 0) {
            noise_power /= noise_bin_count;
        }
    }

    // Calculate SNR in dB: 10*log10(signal_power / noise_power)
    const float snr_db = (noise_power > 0.0f && signal_power > 0.0f) ?
                         10.0f * std::log10(signal_power / noise_power) :
                         0.0f;

    // Calculate confidence based on phase stability and SNR
    // Lower std_dev = higher confidence, boosted by SNR
    // Adjusted for realistic wideband scenarios:
    // - Excellent: std < 15° → 80-100%
    // - Good: std < 30° → 60-80%
    // - Fair: std < 60° → 30-60%
    // - Poor: std > 60° → 0-30%
    float phase_confidence = 100.0f * expf(-std_dev_deg / 25.0f);  // Exponential decay

    // SNR boost: good SNR (>20dB) increases confidence
    float snr_boost = (snr_db > 20.0f) ? std::min(1.0f + (snr_db - 20.0f) / 40.0f, 1.3f) : 1.0f;

    // Combined confidence with 10% penalty for 180° ambiguity
    const float confidence = std::max(0.0f, std::min(100.0f, phase_confidence * snr_boost * 0.9f));

    // Calculate coherence metric (exponential decay with std_dev)
    const float coherence = std::exp(-std_dev_deg / 10.0f);

    // Hold last valid result when confidence is too low
    // This prevents jumping to 0° when signal is weak or noisy
    constexpr float MIN_CONFIDENCE_THRESHOLD = 20.0f;  // Minimum confidence to report new bearing

    bool use_current_result = (confidence >= MIN_CONFIDENCE_THRESHOLD &&
                               strong_bins.size() >= min_bins_for_df);

    float final_azimuth = azimuth_norm;
    float final_back_azimuth = back_azimuth_norm;
    float final_phase_diff = avg_phase_diff_deg;
    float final_phase_std = std_dev_deg;
    float final_confidence = confidence;
    float final_snr = snr_db;
    float final_coherence = coherence;
    bool is_holding = false;

    if (use_current_result) {
        // Store this as the new valid result
        last_valid.has_valid = true;
        last_valid.azimuth = azimuth_norm;
        last_valid.back_azimuth = back_azimuth_norm;
        last_valid.phase_diff_deg = avg_phase_diff_deg;
        last_valid.phase_std_deg = std_dev_deg;
        last_valid.confidence = confidence;
        last_valid.snr_db = snr_db;
        last_valid.coherence = coherence;
        last_valid.last_start_bin = static_cast<uint32_t>(bin_start);
        last_valid.last_end_bin = static_cast<uint32_t>(bin_end);
    } else if (last_valid.has_valid) {
        // Use last valid result instead of defaulting to 0
        final_azimuth = last_valid.azimuth;
        final_back_azimuth = last_valid.back_azimuth;
        final_phase_diff = last_valid.phase_diff_deg;
        final_phase_std = last_valid.phase_std_deg;
        final_confidence = last_valid.confidence * 0.8f;  // Decay confidence slightly to indicate stale
        final_snr = last_valid.snr_db;
        final_coherence = last_valid.coherence;
        is_holding = true;
    }
    // else: no valid result yet and current is bad, use defaults (will be 0°)

    return DFResult{
        .azimuth = final_azimuth,
        .back_azimuth = final_back_azimuth,
        .phase_diff_deg = final_phase_diff,
        .phase_std_deg = final_phase_std,
        .confidence = final_confidence,
        .snr_db = final_snr,
        .coherence = final_coherence,
        .is_holding = is_holding,
        .num_bins = strong_bins.size(),
        .num_signals = detected_signals.size()
    };
}
