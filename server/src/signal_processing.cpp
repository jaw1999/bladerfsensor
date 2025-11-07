#include "signal_processing.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

void generate_window(uint32_t window_type, size_t length, std::vector<float>& window) {
    window.resize(length);

    switch (window_type) {
        case WINDOW_RECTANGULAR:
            std::fill(window.begin(), window.end(), 1.0f);
            break;

        case WINDOW_HAMMING:
            for (size_t i = 0; i < length; i++) {
                window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (length - 1));
            }
            break;

        case WINDOW_HANNING:
            for (size_t i = 0; i < length; i++) {
                window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (length - 1)));
            }
            break;

        case WINDOW_BLACKMAN:
            for (size_t i = 0; i < length; i++) {
                window[i] = 0.42f - 0.5f * cosf(2.0f * M_PI * i / (length - 1)) +
                             0.08f * cosf(4.0f * M_PI * i / (length - 1));
            }
            break;

        case WINDOW_BLACKMAN_HARRIS:
            for (size_t i = 0; i < length; i++) {
                float x = 2.0f * i / (length - 1) - 1.0f;
                window[i] = 0.402f + 0.498f * cosf(M_PI * x) + 0.098f * cosf(2.0f * M_PI * x);
            }
            break;

        case WINDOW_KAISER: {
            constexpr float beta = 8.6f;

            auto bessel_i0 = [](float x) -> float {
                float sum = 1.0f;
                float term = 1.0f;
                float x_sq_4 = (x * x) / 4.0f;
                for (int k = 1; k < 25; k++) {
                    term *= x_sq_4 / (k * k);
                    sum += term;
                    if (term < 1e-8f) break;
                }
                return sum;
            };

            const float bessel_beta = bessel_i0(beta);
            for (size_t i = 0; i < length; i++) {
                float n = static_cast<float>(i) - (length - 1) / 2.0f;
                float alpha = (length - 1) / 2.0f;
                float arg = sqrtf(1.0f - (n * n) / (alpha * alpha));
                window[i] = bessel_i0(beta * arg) / bessel_beta;
            }
            break;
        }

        case WINDOW_TUKEY: {
            constexpr float alpha = 0.5f;
            const float alpha_len = alpha * (length - 1) / 2.0f;

            for (size_t i = 0; i < length; i++) {
                if (i < alpha_len) {
                    window[i] = 0.5f * (1.0f + cosf(M_PI * (i / alpha_len - 1.0f)));
                } else if (i > (length - 1) - alpha_len) {
                    float idx = static_cast<float>(i) - (length - 1 - alpha_len);
                    window[i] = 0.5f * (1.0f + cosf(M_PI * idx / alpha_len));
                } else {
                    window[i] = 1.0f;
                }
            }
            break;
        }

        case WINDOW_GAUSSIAN: {
            constexpr float sigma = 0.4f;
            const float center = (length - 1) / 2.0f;

            for (size_t i = 0; i < length; i++) {
                float n = (static_cast<float>(i) - center) / center;
                window[i] = expf(-0.5f * (n / sigma) * (n / sigma));
            }
            break;
        }

        default:
            std::fill(window.begin(), window.end(), 1.0f);
    }
}

void apply_window(fftwf_complex* data, size_t length, const std::vector<float>& window) {
    for (size_t i = 0; i < length; i++) {
        data[i][0] *= window[i];
        data[i][1] *= window[i];
    }
}

void compute_fft(fftwf_complex *in, fftwf_complex *out, fftwf_plan plan) {
    fftwf_execute_dft(plan, in, out);
}

void compute_magnitude_db(fftwf_complex *fft_out, uint8_t *mag_out, size_t size) {
    static int debug_counter = 0;
    float min_db = 1000.0f;
    float max_db = -1000.0f;

    constexpr float DB_SCALE = 10.0f;
    constexpr float DB_OFFSET = 100.0f;
    constexpr float DB_RANGE = 120.0f;
    constexpr float NORM_SCALE = 255.0f / DB_RANGE;
    constexpr float MIN_POWER = 1e-20f;

    for (size_t i = 0; i < size; i++) {
        float real = fft_out[i][0];
        float imag = fft_out[i][1];

        float power = real * real + imag * imag;
        float db = DB_SCALE * std::log10(std::max(power, MIN_POWER));

        if (db < min_db) min_db = db;
        if (db > max_db) max_db = db;

        float normalized = (db + DB_OFFSET) * NORM_SCALE;
        mag_out[i] = static_cast<uint8_t>(std::clamp(normalized, 0.0f, 255.0f));
    }

    debug_counter++;
    if (debug_counter % 60 == 0) {
        std::cout << "FFT dB range: " << std::fixed << std::setprecision(1)
                  << min_db << " to " << max_db << " dB" << std::endl;
    }
}

void remove_dc_offset(uint8_t *magnitude, size_t size) {
    const size_t dc_bin = size / 2;

    if (dc_bin < 3 || dc_bin >= size - 3) return;

    const uint32_t weighted_avg = (
        static_cast<uint32_t>(magnitude[dc_bin - 2]) +
        2 * static_cast<uint32_t>(magnitude[dc_bin - 1]) +
        2 * static_cast<uint32_t>(magnitude[dc_bin + 1]) +
        static_cast<uint32_t>(magnitude[dc_bin + 2])
    ) / 6;

    magnitude[dc_bin] = static_cast<uint8_t>(weighted_avg);

    for (int offset = -1; offset <= 1; offset += 2) {
        size_t idx = dc_bin + offset;
        uint32_t local_avg = (
            static_cast<uint32_t>(magnitude[idx - 1]) +
            2 * static_cast<uint32_t>(magnitude[idx]) +
            static_cast<uint32_t>(magnitude[idx + 1])
        ) / 4;
        magnitude[idx] = static_cast<uint8_t>(local_avg);
    }
}

void compute_cross_correlation(fftwf_complex *fft_ch1, fftwf_complex *fft_ch2,
                              float *correlation, float *phase_diff, size_t size) {
    for (size_t i = 0; i < size; i++) {
        const float real1 = fft_ch1[i][0];
        const float imag1 = -fft_ch1[i][1];  // Complex conjugate
        const float real2 = fft_ch2[i][0];
        const float imag2 = fft_ch2[i][1];

        const float corr_real = real1 * real2 - imag1 * imag2;
        const float corr_imag = real1 * imag2 + imag1 * real2;

        const float power = corr_real * corr_real + corr_imag * corr_imag;
        correlation[i] = sqrtf(power);

        phase_diff[i] = atan2f(corr_imag, corr_real);
    }
}

void init_averaging(uint32_t num_frames, size_t fft_size,
                    std::vector<std::vector<uint8_t>>& avg_buffer_ch1,
                    std::vector<std::vector<uint8_t>>& avg_buffer_ch2) {
    avg_buffer_ch1.resize(num_frames);
    avg_buffer_ch2.resize(num_frames);
    for (auto& buf : avg_buffer_ch1) {
        buf.resize(fft_size, 0);
    }
    for (auto& buf : avg_buffer_ch2) {
        buf.resize(fft_size, 0);
    }
}

void apply_averaging(uint8_t* ch1_mag, uint8_t* ch2_mag, size_t fft_size,
                     uint32_t averaging_frames,
                     std::vector<std::vector<uint8_t>>& avg_buffer_ch1,
                     std::vector<std::vector<uint8_t>>& avg_buffer_ch2,
                     uint32_t& avg_index) {
    if (averaging_frames <= 1) {
        return;
    }

    std::copy(ch1_mag, ch1_mag + fft_size, avg_buffer_ch1[avg_index].begin());
    std::copy(ch2_mag, ch2_mag + fft_size, avg_buffer_ch2[avg_index].begin());
    avg_index = (avg_index + 1) % averaging_frames;

    for (size_t i = 0; i < fft_size; i++) {
        uint32_t sum1 = 0, sum2 = 0;
        for (uint32_t j = 0; j < averaging_frames; j++) {
            sum1 += avg_buffer_ch1[j][i];
            sum2 += avg_buffer_ch2[j][i];
        }
        ch1_mag[i] = sum1 / averaging_frames;
        ch2_mag[i] = sum2 / averaging_frames;
    }
}

void init_agc(AGCState& agc, uint32_t initial_gain_rx1, uint32_t initial_gain_rx2) {
    agc.enabled = false;
    agc.current_level = 0.0f;
    agc.current_gain_rx1 = initial_gain_rx1;
    agc.current_gain_rx2 = initial_gain_rx2;
    agc.hysteresis_counter = 0;
}

void update_agc(AGCState& agc, const uint8_t* ch1_mag, const uint8_t* ch2_mag, size_t len,
                uint32_t& gain_rx1, uint32_t& gain_rx2, bool& params_changed) {
    if (!agc.enabled) {
        return;
    }

    // Find peak magnitude across both channels
    uint8_t peak = 0;
    for (size_t i = 0; i < len; i++) {
        peak = std::max(peak, std::max(ch1_mag[i], ch2_mag[i]));
    }

    agc.current_level = peak;

    // AGC constants (from bladerf_sensor.h)
    constexpr int AGC_TARGET_LEVEL = 200;
    constexpr int AGC_HYSTERESIS = 20;

    // Signal too strong - decrease gain
    if (peak > AGC_TARGET_LEVEL + AGC_HYSTERESIS) {
        agc.hysteresis_counter++;
        if (agc.hysteresis_counter > 5) {
            if (agc.current_gain_rx1 > 0) {
                agc.current_gain_rx1 = std::max(0u, agc.current_gain_rx1 - 3);
                agc.current_gain_rx2 = std::max(0u, agc.current_gain_rx2 - 3);
                gain_rx1 = agc.current_gain_rx1;
                gain_rx2 = agc.current_gain_rx2;
                params_changed = true;
                std::cout << "AGC: Decreasing gain to " << agc.current_gain_rx1 << " dB" << std::endl;
            }
            agc.hysteresis_counter = 0;
        }
    }
    // Signal too weak - increase gain
    else if (peak < AGC_TARGET_LEVEL - AGC_HYSTERESIS) {
        agc.hysteresis_counter++;
        if (agc.hysteresis_counter > 20) {
            if (agc.current_gain_rx1 < 60) {
                agc.current_gain_rx1 = std::min(60u, agc.current_gain_rx1 + 1);
                agc.current_gain_rx2 = std::min(60u, agc.current_gain_rx2 + 1);
                gain_rx1 = agc.current_gain_rx1;
                gain_rx2 = agc.current_gain_rx2;
                params_changed = true;
                std::cout << "AGC: Increasing gain to " << agc.current_gain_rx1 << " dB" << std::endl;
            }
            agc.hysteresis_counter = 0;
        }
    }
    // Within target range - reset hysteresis counter
    else {
        agc.hysteresis_counter = 0;
    }
}
