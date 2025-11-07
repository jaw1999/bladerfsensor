#include "df_processing.h"
#include "array_calibration.h"
#include <cmath>
#include <algorithm>
#include <chrono>

// Helper function: Get current time in milliseconds
static uint64_t get_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Kalman filter prediction step
static void kalman_predict(KalmanState& state, float dt) {
    // State transition: x(k) = x(k-1) + velocity * dt
    state.azimuth += state.velocity * dt;

    // Normalize azimuth to [0, 360)
    state.azimuth = std::fmod(state.azimuth + 360.0f, 360.0f);

    // Process noise covariance
    const float process_noise_azimuth = 0.5f;  // degrees^2
    const float process_noise_velocity = 0.1f; // (degrees/sec)^2

    // Update error covariance: P = F*P*F' + Q
    // State transition matrix F = [[1, dt], [0, 1]]
    float P00 = state.P[0][0] + 2.0f * dt * state.P[0][1] + dt * dt * state.P[1][1] + process_noise_azimuth;
    float P01 = state.P[0][1] + dt * state.P[1][1];
    float P10 = P01;  // Symmetric
    float P11 = state.P[1][1] + process_noise_velocity;

    state.P[0][0] = P00;
    state.P[0][1] = P01;
    state.P[1][0] = P10;
    state.P[1][1] = P11;
}

// Kalman filter update step
static void kalman_update(KalmanState& state, float measurement, float measurement_variance) {
    // Measurement model: z = H * x, where H = [1, 0] (we measure azimuth)
    // Innovation (measurement residual)
    float innovation = measurement - state.azimuth;

    // Handle wraparound: find shortest angular distance
    if (innovation > 180.0f) innovation -= 360.0f;
    if (innovation < -180.0f) innovation += 360.0f;

    // Innovation covariance: S = H*P*H' + R
    float S = state.P[0][0] + measurement_variance;

    // Kalman gain: K = P*H' * inv(S)
    float K0 = state.P[0][0] / S;
    float K1 = state.P[1][0] / S;

    // Update state estimate: x = x + K * innovation
    state.azimuth += K0 * innovation;
    state.velocity += K1 * innovation;

    // Normalize azimuth to [0, 360)
    state.azimuth = std::fmod(state.azimuth + 360.0f, 360.0f);

    // Update error covariance: P = (I - K*H) * P
    float P00 = (1.0f - K0) * state.P[0][0];
    float P01 = (1.0f - K0) * state.P[0][1];
    float P10 = state.P[1][0] - K1 * state.P[0][0];
    float P11 = state.P[1][1] - K1 * state.P[0][1];

    state.P[0][0] = P00;
    state.P[0][1] = P01;
    state.P[1][0] = P10;
    state.P[1][1] = P11;
}

// Initialize Kalman filter with first measurement
static void kalman_initialize(KalmanState& state, float initial_azimuth, float initial_variance) {
    state.azimuth = initial_azimuth;
    state.velocity = 0.0f;

    // Initial error covariance
    state.P[0][0] = initial_variance;
    state.P[0][1] = 0.0f;
    state.P[1][0] = 0.0f;
    state.P[1][1] = 10.0f;  // Initial velocity uncertainty

    state.initialized = true;
    state.last_update_ms = get_time_ms();
}

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
    float noise_floor_ch1,
    float noise_floor_ch2
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

    // Detect signal regions using CFAR with dynamic noise floor if available
    std::vector<SignalRegion> detected_signals;
    if (noise_floor_ch1 >= 0.0f && noise_floor_ch2 >= 0.0f) {
        detected_signals = detect_signals_cfar_with_floor(ch1_mag, ch2_mag, fft_size,
                                                          DEFAULT_CFAR, bin_start, bin_end,
                                                          noise_floor_ch1, noise_floor_ch2);
    } else {
        detected_signals = detect_signals_cfar(ch1_mag, ch2_mag, fft_size,
                                              DEFAULT_CFAR, bin_start, bin_end);
    }

    // Collect all bins from detected signal regions with their phase differences
    std::vector<BinInfo> strong_bins;
    strong_bins.reserve(fft_size / 4);  // Preallocate reasonable size

    for (const auto& signal : detected_signals) {
        // For each detected signal region, extract phase information from all bins
        std::vector<float> raw_phase_diffs;
        raw_phase_diffs.reserve(signal.end_bin - signal.start_bin + 1);

        for (size_t i = signal.start_bin; i <= signal.end_bin; i++) {
            // Compute phase from FFT output (frequency domain)
            const float phase1 = atan2f(fft_out_ch1[i][1], fft_out_ch1[i][0]);
            const float phase2 = atan2f(fft_out_ch2[i][1], fft_out_ch2[i][0]);

            // Phase difference (CH2 - CH1)
            float diff = phase2 - phase1;

            // Initial wrap to [-π, π]
            while (diff > M_PI) diff -= 2.0f * M_PI;
            while (diff < -M_PI) diff += 2.0f * M_PI;

            raw_phase_diffs.push_back(diff);
        }

        // Phase unwrapping for wideband signals (Itoh's method)
        // Corrects phase jumps across frequency bins for signals wider than one wavelength
        if (raw_phase_diffs.size() > 1) {
            for (size_t j = 1; j < raw_phase_diffs.size(); j++) {
                float phase_jump = raw_phase_diffs[j] - raw_phase_diffs[j-1];

                // Detect and correct 2π jumps
                if (phase_jump > M_PI) {
                    // Positive jump - unwrap by subtracting 2π
                    for (size_t k = j; k < raw_phase_diffs.size(); k++) {
                        raw_phase_diffs[k] -= 2.0f * M_PI;
                    }
                } else if (phase_jump < -M_PI) {
                    // Negative jump - unwrap by adding 2π
                    for (size_t k = j; k < raw_phase_diffs.size(); k++) {
                        raw_phase_diffs[k] += 2.0f * M_PI;
                    }
                }
            }
        }

        // Store unwrapped phases with magnitude weighting
        for (size_t j = 0; j < raw_phase_diffs.size(); j++) {
            size_t i = signal.start_bin + j;
            const float avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2.0f;
            strong_bins.push_back({i, avg_mag, raw_phase_diffs[j]});
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

        // Calculate weighted standard deviation using Welford's algorithm (numerically stable)
        float M = 0.0f;  // Running mean
        float S = 0.0f;  // Running sum of squared differences
        float W = 0.0f;  // Running sum of weights

        for (const auto& bin : strong_bins) {
            float diff = bin.phase_diff - avg_phase_diff_rad;
            // Handle phase wrapping
            while (diff > M_PI) diff -= 2.0f * M_PI;
            while (diff < -M_PI) diff += 2.0f * M_PI;

            const float weight = bin.magnitude;
            W += weight;

            // Numerically stable weighted variance update
            const float delta = diff - M;
            M += delta * weight / W;
            S += weight * delta * (diff - M);
        }

        std_dev_rad = std::sqrt(S / W);
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

    if (strong_bins.size() >= min_bins_for_df) {
        // Calculate average signal power from strong bins
        for (const auto& bin : strong_bins) {
            // Convert FFT output to power: |FFT[k]|^2 = real^2 + imag^2
            const float real1 = fft_out_ch1[bin.index][0];
            const float imag1 = fft_out_ch1[bin.index][1];
            signal_power += (real1 * real1 + imag1 * imag1);
        }
        signal_power /= strong_bins.size();

        // Use dynamic noise floor if available, otherwise estimate locally
        if (noise_floor_ch1 >= 0.0f && noise_floor_ch2 >= 0.0f) {
            // Convert noise floor from 0-255 magnitude scale to power
            // Magnitude scale: 0-255 represents -120 to 0 dBm (120 dB range)
            // noise_mag = 0-255, convert to linear power
            const float avg_noise_mag = (noise_floor_ch1 + noise_floor_ch2) / 2.0f;
            // Approximate conversion: magnitude relates to power logarithmically
            // Use empirical scaling based on FFT output characteristics
            const float noise_scale = 1e-6f;  // Scaling factor for FFT power units
            noise_power = noise_scale * avg_noise_mag * avg_noise_mag;
        } else {
            // Fallback: Estimate noise floor from bins below mean (noise reference)
            size_t noise_bin_count = 0;
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

    // Apply Kalman filter for smooth bearing tracking
    // This reduces jitter and provides predictive capability
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

    // Get current time for Kalman filter
    uint64_t current_time_ms = get_time_ms();

    if (use_current_result) {
        // Measurement variance based on phase standard deviation
        // Lower std_dev = more confident measurement
        float measurement_variance = std::max(1.0f, std_dev_deg * std_dev_deg);

        if (!last_valid.kalman.initialized) {
            // Initialize Kalman filter with first good measurement
            kalman_initialize(last_valid.kalman, azimuth_norm, measurement_variance);
            final_azimuth = azimuth_norm;
        } else {
            // Kalman filter predict + update
            float dt = (current_time_ms - last_valid.kalman.last_update_ms) / 1000.0f; // Convert to seconds
            dt = std::max(0.001f, std::min(dt, 1.0f));  // Clamp to reasonable range

            kalman_predict(last_valid.kalman, dt);
            kalman_update(last_valid.kalman, azimuth_norm, measurement_variance);

            // Use Kalman filtered azimuth
            final_azimuth = last_valid.kalman.azimuth;

            // Update back azimuth to maintain 180° offset
            final_back_azimuth = std::fmod(final_azimuth + 180.0f, 360.0f);
        }

        last_valid.kalman.last_update_ms = current_time_ms;

        // Store this as the new valid result
        last_valid.has_valid = true;
        last_valid.azimuth = final_azimuth;
        last_valid.back_azimuth = final_back_azimuth;
        last_valid.phase_diff_deg = avg_phase_diff_deg;
        last_valid.phase_std_deg = std_dev_deg;
        last_valid.confidence = confidence;
        last_valid.snr_db = snr_db;
        last_valid.coherence = coherence;
        last_valid.last_start_bin = static_cast<uint32_t>(bin_start);
        last_valid.last_end_bin = static_cast<uint32_t>(bin_end);

    } else if (last_valid.has_valid && last_valid.kalman.initialized) {
        // No good measurement, but we have Kalman state - use prediction only
        float dt = (current_time_ms - last_valid.kalman.last_update_ms) / 1000.0f;
        dt = std::max(0.001f, std::min(dt, 1.0f));

        kalman_predict(last_valid.kalman, dt);
        last_valid.kalman.last_update_ms = current_time_ms;

        // Use predicted azimuth
        final_azimuth = last_valid.kalman.azimuth;
        final_back_azimuth = std::fmod(final_azimuth + 180.0f, 360.0f);
        final_phase_diff = last_valid.phase_diff_deg;
        final_phase_std = last_valid.phase_std_deg;
        final_confidence = last_valid.confidence * 0.8f;  // Decay confidence to indicate prediction
        final_snr = last_valid.snr_db;
        final_coherence = last_valid.coherence;
        is_holding = true;

    } else if (last_valid.has_valid) {
        // No Kalman state but have last valid - fall back to hold logic
        final_azimuth = last_valid.azimuth;
        final_back_azimuth = last_valid.back_azimuth;
        final_phase_diff = last_valid.phase_diff_deg;
        final_phase_std = last_valid.phase_std_deg;
        final_confidence = last_valid.confidence * 0.8f;
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
