#include "bladerf_sensor.h"
#include "web_server.h"
#include "cfar_detector.h"
#include "array_calibration.h"
#include "signal_processing.h"
#include "df_processing.h"
#include "recording.h"
#include "config_validation.h"
#include "scanner.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <ctime>

#include <fftw3.h>
#include <libbladeRF.h>

#include <signal.h>
#include <unistd.h>
#include <linux/limits.h>

// Global state variables for server operation and RF configuration
// These are atomic to allow safe access from multiple threads
std::atomic<bool> g_running{true};                    // Server running flag
std::atomic<uint64_t> g_center_freq{CENTER_FREQ};     // Current center frequency (Hz)
std::atomic<uint32_t> g_sample_rate{SAMPLE_RATE};     // Current sample rate (Hz)
std::atomic<uint32_t> g_bandwidth{BANDWIDTH};         // Current analog bandwidth (Hz)
std::atomic<uint32_t> g_gain_rx1{GAIN_RX1};           // Current RX1 gain (dB)
std::atomic<uint32_t> g_gain_rx2{GAIN_RX2};           // Current RX2 gain (dB)
std::atomic<bool> g_params_changed{false};            // Configuration change pending flag
std::mutex g_config_mutex;                            // Mutex for configuration updates

// Direction finding bin range selection (0 = entire spectrum)
std::atomic<uint32_t> g_df_start_bin{0};              // DF start bin (0 = use entire spectrum)
std::atomic<uint32_t> g_df_end_bin{0};                // DF end bin (0 = use entire spectrum)

// FFTW plans and buffers for FFT computation
// These are shared between threads and must be accessed carefully
fftwf_plan fft_plan_ch1 = nullptr;                    // FFT plan for channel 1
fftwf_plan fft_plan_ch2 = nullptr;                    // FFT plan for channel 2
std::vector<fftwf_complex> fft_in_ch1(FFT_SIZE);      // Channel 1 FFT input buffer
std::vector<fftwf_complex> fft_in_ch2(FFT_SIZE);      // Channel 2 FFT input buffer
std::vector<fftwf_complex> fft_out_ch1(FFT_SIZE);     // Channel 1 FFT output buffer
std::vector<fftwf_complex> fft_out_ch2(FFT_SIZE);     // Channel 2 FFT output buffer

// IQ processing state
DCOffsetState g_dc_offset;                            // DC offset correction state
OverlapState g_overlap;                               // Overlap-add state for smoother spectrum

// Automatic gain control (AGC) state
AGCState g_agc;

// Noise floor estimation state
NoiseFloorState g_noise_floor;

// FFT window function state
uint32_t g_window_type = WINDOW_HAMMING;              // Current window function type
std::vector<float> g_window;                          // Precomputed window coefficients

// FFT averaging state for noise reduction
uint32_t g_averaging_frames = 1;                      // Number of frames to average (1 = disabled)
std::vector<std::vector<uint8_t>> g_avg_buffer_ch1;  // Channel 1 averaging buffer
std::vector<std::vector<uint8_t>> g_avg_buffer_ch2;  // Channel 2 averaging buffer
uint32_t g_avg_index = 0;                             // Current averaging buffer index

// Last valid DoA result (hold when confidence is too low)
LastValidDoA g_last_valid_doa = {
    .has_valid = false,
    .azimuth = 0.0f,
    .back_azimuth = 0.0f,
    .phase_diff_deg = 0.0f,
    .phase_std_deg = 0.0f,
    .confidence = 0.0f,
    .snr_db = 0.0f,
    .coherence = 0.0f,
    .last_start_bin = 0,
    .last_end_bin = 0,
    .kalman = {0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}, false, 0}
};

// Signal handler for graceful shutdown on SIGINT/SIGTERM
// Stops data acquisition closes recordings and sets global shutdown flag
void signal_handler(int signum) {
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "Interrupt signal (" << signum << ") received" << std::endl;
    std::cout << "Shutting down gracefully" << std::endl;
    std::cout << "========================================\n" << std::endl;
    g_running = false;

    // Stop any active recording session
    stop_recording();
}


int initialize_bladerf(struct bladerf **dev) {
    int status;

    std::cout << "Opening bladeRF device..." << std::endl;
    status = bladerf_open(dev, NULL);
    if (status != 0) {
        std::cerr << "Failed to open bladeRF: " << bladerf_strerror(status) << std::endl;
        return status;
    }

    // Verify FPGA is configured
    bladerf_is_fpga_configured(*dev);

    // Get device info
    struct bladerf_devinfo dev_info;
    bladerf_get_devinfo(*dev, &dev_info);
    std::cout << "Device: " << dev_info.product << std::endl;

    return 0;
}

int configure_channel(struct bladerf *dev, bladerf_channel ch, uint64_t freq, uint32_t gain, uint32_t sample_rate, uint32_t bandwidth) {
    int status;

    // Set sample rate
    uint32_t actual_sample_rate;
    status = bladerf_set_sample_rate(dev, ch, sample_rate, &actual_sample_rate);
    if (status != 0) {
        std::cerr << "Failed to set sample rate: " << bladerf_strerror(status) << std::endl;
        return status;
    }
    std::cout << "Channel " << ch << " sample rate: " << actual_sample_rate << " Hz ("
              << (actual_sample_rate / 1e6) << " MHz)" << std::endl;

    // Set bandwidth
    uint32_t actual_bandwidth;
    status = bladerf_set_bandwidth(dev, ch, bandwidth, &actual_bandwidth);
    if (status != 0) {
        std::cerr << "Failed to set bandwidth: " << bladerf_strerror(status) << std::endl;
        return status;
    }
    std::cout << "Channel " << ch << " bandwidth: " << actual_bandwidth << " Hz ("
              << (actual_bandwidth / 1e6) << " MHz)" << std::endl;

    // Set frequency
    status = bladerf_set_frequency(dev, ch, freq);
    if (status != 0) {
        std::cerr << "Failed to set frequency: " << bladerf_strerror(status) << std::endl;
        return status;
    }
    std::cout << "Channel " << ch << " frequency: " << freq << " Hz" << std::endl;

    // Set gain mode to manual (disable hardware AGC)
    status = bladerf_set_gain_mode(dev, ch, BLADERF_GAIN_MGC);
    if (status != 0) {
        std::cerr << "Failed to set gain mode: " << bladerf_strerror(status) << std::endl;
        return status;
    }

    // Set gain
    status = bladerf_set_gain(dev, ch, gain);
    if (status != 0) {
        std::cerr << "Failed to set gain: " << bladerf_strerror(status) << std::endl;
        return status;
    }
    std::cout << "Channel " << ch << " gain: " << gain << " dB (manual mode)" << std::endl;

    return 0;
}

int main(int argc, char *argv[]) {
    struct bladerf *dev = nullptr;
    int status;

    // Install signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Parse command line args for frequency
    if (argc > 1) {
        try {
            uint64_t freq = std::stoull(argv[1]);
            if (validate_frequency(freq)) {
                g_center_freq = freq;
            } else {
                std::cerr << "Invalid frequency argument: " << argv[1] << std::endl;
                std::cerr << "Using default: " << (CENTER_FREQ / 1e6) << " MHz" << std::endl;
            }
        } catch (const std::exception &e) {
            std::cerr << "Error parsing frequency: " << e.what() << std::endl;
            std::cerr << "Using default: " << (CENTER_FREQ / 1e6) << " MHz" << std::endl;
        }
    }

    std::cout << "bladeRF Sensor Server" << std::endl;
    std::cout << "=====================" << std::endl;

    // Initialize FFTW with wisdom for fast startup
    std::cout << "Initializing FFTW..." << std::endl;

    // Try to load wisdom file for optimized plans
    const char* wisdom_file = "fftw_wisdom.dat";
    if (fftwf_import_wisdom_from_filename(wisdom_file)) {
        std::cout << "Loaded FFTW wisdom from " << wisdom_file << std::endl;
    } else {
        std::cout << "No FFTW wisdom file found, will create plans from scratch" << std::endl;
    }

    // Create FFT plans (FFTW_MEASURE will take time on first run, but wisdom speeds it up)
    fft_plan_ch1 = fftwf_plan_dft_1d(FFT_SIZE, fft_in_ch1.data(), fft_out_ch1.data(),
                                     FFTW_FORWARD, FFTW_MEASURE);
    fft_plan_ch2 = fftwf_plan_dft_1d(FFT_SIZE, fft_in_ch2.data(), fft_out_ch2.data(),
                                     FFTW_FORWARD, FFTW_MEASURE);

    // Export wisdom for future runs
    if (fftwf_export_wisdom_to_filename(wisdom_file)) {
        std::cout << "Saved FFTW wisdom to " << wisdom_file << std::endl;
    }

    // Initialize window function (default: Hamming)
    std::cout << "Initializing window function..." << std::endl;
    generate_window(g_window_type, FFT_SIZE, g_window);

    // Initialize averaging (default: no averaging)
    std::cout << "Initializing FFT averaging..." << std::endl;
    init_averaging(1, FFT_SIZE, g_avg_buffer_ch1, g_avg_buffer_ch2);

    // Initialize AGC
    std::cout << "Initializing AGC..." << std::endl;
    init_agc(g_agc, GAIN_RX1, GAIN_RX2);

    // Initialize IQ processing states
    std::cout << "Initializing DC offset correction..." << std::endl;
    init_dc_offset(g_dc_offset);
    std::cout << "Initializing overlap-add buffers..." << std::endl;
    init_overlap(g_overlap, FFT_SIZE);
    std::cout << "Initializing noise floor estimation..." << std::endl;
    init_noise_floor(g_noise_floor, FFT_SIZE);

    // Initialize bladeRF
    status = initialize_bladerf(&dev);
    if (status != 0) {
        return 1;
    }

    // Configure RX channels
    std::cout << "\nConfiguring RX channels..." << std::endl;
    status = configure_channel(dev, BLADERF_CHANNEL_RX(0), g_center_freq.load(), g_gain_rx1.load(), g_sample_rate.load(), g_bandwidth.load());
    if (status != 0) {
        bladerf_close(dev);
        return 1;
    }

    status = configure_channel(dev, BLADERF_CHANNEL_RX(1), g_center_freq.load(), g_gain_rx2.load(), g_sample_rate.load(), g_bandwidth.load());
    if (status != 0) {
        bladerf_close(dev);
        return 1;
    }

    // Configure sync RX
    std::cout << "\nConfiguring synchronous RX..." << std::endl;
    status = bladerf_sync_config(dev,
                                BLADERF_RX_X2,
                                BLADERF_FORMAT_SC16_Q11,
                                NUM_BUFFERS,
                                BUFFER_SIZE,
                                NUM_TRANSFERS,
                                3500);
    if (status != 0) {
        std::cerr << "Failed to configure RX sync: " << bladerf_strerror(status) << std::endl;
        bladerf_close(dev);
        return 1;
    }

    // Enable RX channels
    std::cout << "Enabling RX channels..." << std::endl;
    status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true);
    if (status != 0) {
        std::cerr << "Failed to enable RX1: " << bladerf_strerror(status) << std::endl;
        bladerf_close(dev);
        return 1;
    }

    status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(1), true);
    if (status != 0) {
        std::cerr << "Failed to enable RX2: " << bladerf_strerror(status) << std::endl;
        bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(dev);
        return 1;
    }

    // Start web server for waterfall visualization
    start_web_server();

    // Calculate sleep time between updates
    auto sleep_duration = std::chrono::microseconds(1000000 / UPDATE_RATE_HZ);

    std::cout << "\n========================================" << std::endl;
    std::cout << "Starting continuous RX capture..." << std::endl;
    std::cout << "Web interface: http://localhost:" << WEB_SERVER_PORT << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Main RX loop runs continuously independent of clients
    std::thread rx_thread([&]() {
        std::cout << "*** RX THREAD STARTED ***" << std::endl;

        // Dedicated RX buffer for this thread (avoid conflicts with client loop)
        std::vector<int16_t> thread_rx_buffer(BUFFER_SIZE * 4);

        // FPS tracking for link quality monitoring
        uint32_t frame_count = 0;
        auto fps_update_time = std::chrono::steady_clock::now();

        while (g_running) {
            // Check if parameters changed
            if (g_params_changed.load()) {
                std::lock_guard<std::mutex> lock(g_config_mutex);
                g_params_changed = false;

                std::cout << "Applying parameter changes..." << std::endl;

                // Validate parameters before applying
                const uint64_t freq = g_center_freq.load();
                const uint32_t sample_rate = g_sample_rate.load();
                const uint32_t bandwidth = g_bandwidth.load();
                const uint32_t gain_rx1 = g_gain_rx1.load();
                const uint32_t gain_rx2 = g_gain_rx2.load();

                std::cout << "  New frequency: " << (freq / 1e6) << " MHz" << std::endl;
                std::cout << "  New sample rate: " << (sample_rate / 1e6) << " MHz" << std::endl;
                std::cout << "  New bandwidth: " << (bandwidth / 1e6) << " MHz" << std::endl;
                std::cout << "  New gain RX1: " << gain_rx1 << " dB" << std::endl;
                std::cout << "  New gain RX2: " << gain_rx2 << " dB" << std::endl;

                if (!validate_frequency(freq) || !validate_gain(gain_rx1) || !validate_gain(gain_rx2)) {
                    std::cerr << "Invalid parameters - ignoring change request" << std::endl;
                    continue;
                }

                // Disable modules before reconfiguration
                bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);
                bladerf_enable_module(dev, BLADERF_CHANNEL_RX(1), false);

                // Reconfigure channels (now includes sample rate and bandwidth)
                int status = configure_channel(dev, BLADERF_CHANNEL_RX(0), freq, gain_rx1, sample_rate, bandwidth);
                if (status == 0) {
                    status = configure_channel(dev, BLADERF_CHANNEL_RX(1), freq, gain_rx2, sample_rate, bandwidth);
                }

                // Reconfigure sync RX (required after disabling modules)
                if (status == 0) {
                    status = bladerf_sync_config(dev,
                                                BLADERF_RX_X2,
                                                BLADERF_FORMAT_SC16_Q11,
                                                NUM_BUFFERS,
                                                BUFFER_SIZE,
                                                NUM_TRANSFERS,
                                                3500);
                }

                // ALWAYS re-enable modules (even if config failed)
                bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true);
                bladerf_enable_module(dev, BLADERF_CHANNEL_RX(1), true);

                if (status == 0) {
                    std::cout << "Parameters updated successfully" << std::endl;
                } else {
                    std::cerr << "Failed to update parameters - keeping previous settings" << std::endl;
                }
            }

            const auto start_time = std::chrono::steady_clock::now();

            // Receive samples
            int status = bladerf_sync_rx(dev, thread_rx_buffer.data(), BUFFER_SIZE, nullptr, 5000);
            if (status != 0) {
                std::cerr << "RX failed: " << bladerf_strerror(status) << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // Write samples to recording file if active
            write_samples_to_file(thread_rx_buffer.data(), BUFFER_SIZE * 2);

            // ===== IQ PROCESSING PIPELINE =====
            // Process IQ samples through complete pipeline: overlap-add, DC removal, window, FFT, magnitude
            uint8_t ch1_mag[FFT_SIZE], ch2_mag[FFT_SIZE];
            IQProcessingResult iq_result = process_iq_to_fft(
                thread_rx_buffer.data(),
                BUFFER_SIZE * 2,
                FFT_SIZE,
                g_center_freq.load(),
                fft_in_ch1.data(),
                fft_in_ch2.data(),
                fft_out_ch1.data(),
                fft_out_ch2.data(),
                ch1_mag,
                ch2_mag,
                g_dc_offset,
                g_overlap,
                g_window,
                fft_plan_ch1,
                fft_plan_ch2
            );

            // Reset held DoA on frequency change
            if (iq_result.freq_changed) {
                g_last_valid_doa.has_valid = false;
            }

            // Debug: Print peak sample every 60 frames
            static int web_sample_debug_counter = 0;
            web_sample_debug_counter++;
            if (web_sample_debug_counter % 60 == 0) {
                float saturation_pct = (std::abs(iq_result.peak_sample) / 32768.0f) * 100.0f;
                std::cout << "Peak ADC sample: " << iq_result.peak_sample << " ("
                          << std::fixed << std::setprecision(1) << saturation_pct
                          << "% of full scale) [Gain=" << g_gain_rx1.load() << "dB]" << std::endl;
            }

            // Apply time-domain averaging
            apply_averaging(ch1_mag, ch2_mag, FFT_SIZE, g_averaging_frames,
                           g_avg_buffer_ch1, g_avg_buffer_ch2, g_avg_index);

            // Update AGC
            uint32_t gain_rx1 = g_gain_rx1.load();
            uint32_t gain_rx2 = g_gain_rx2.load();
            bool params_changed = false;
            update_agc(g_agc, ch1_mag, ch2_mag, FFT_SIZE, gain_rx1, gain_rx2, params_changed);
            if (params_changed) {
                g_gain_rx1.store(gain_rx1);
                g_gain_rx2.store(gain_rx2);
                g_params_changed.store(true);
            }

            // Update noise floor estimation (15th percentile, 0.1 smoothing factor)
            update_noise_floor(g_noise_floor, ch1_mag, ch2_mag, FFT_SIZE, 15.0f, 0.1f);

            // Remove DC offset spike/dip at center frequency
            remove_dc_offset(ch1_mag, FFT_SIZE);
            remove_dc_offset(ch2_mag, FFT_SIZE);

            // Update waterfall buffer for web interface
            update_waterfall(ch1_mag, ch2_mag, FFT_SIZE);

            // Decimate IQ samples for constellation display
            static_assert(FFT_SIZE >= 256 && FFT_SIZE % 256 == 0, "FFT_SIZE must be >= 256 and divisible by 256");
            constexpr int decimation_step = FFT_SIZE / 256;
            int16_t ch1_iq[256][2], ch2_iq[256][2];
            for (int i = 0; i < 256; i++) {
                const size_t idx = i * decimation_step;
                ch1_iq[i][0] = static_cast<int16_t>(fft_in_ch1[idx][0] * 32767.0f);
                ch1_iq[i][1] = static_cast<int16_t>(fft_in_ch1[idx][1] * 32767.0f);
                ch2_iq[i][0] = static_cast<int16_t>(fft_in_ch2[idx][0] * 32767.0f);
                ch2_iq[i][1] = static_cast<int16_t>(fft_in_ch2[idx][1] * 32767.0f);
            }
            // Update IQ data with FFT output for frequency-domain filtering
            update_iq_data(reinterpret_cast<int16_t*>(ch1_iq), reinterpret_cast<int16_t*>(ch2_iq), 256,
                          fft_out_ch1.data(), fft_out_ch2.data(), FFT_SIZE);

            // Compute and update cross-correlation data
            float xcorr_mag[FFT_SIZE], xcorr_phase[FFT_SIZE];
            compute_cross_correlation(fft_out_ch1.data(), fft_out_ch2.data(), xcorr_mag, xcorr_phase, FFT_SIZE);
            update_xcorr_data(xcorr_mag, xcorr_phase, FFT_SIZE);

            // ===== DIRECTION OF ARRIVAL (DoA) CALCULATION =====
            // Phase-based 2-channel interferometry using FFT output (frequency domain)
            // Theory: Δφ = (2π * d * sin(θ)) / λ
            //
            // FREQUENCY-DOMAIN APPROACH:
            // - Compute phase difference per frequency bin
            // - Focus on bins with strong signals (better SNR)
            // - Can isolate individual signals and reject interference
            // - Standard method for correlative interferometry

            // ===== DIRECTION FINDING =====
            // Determine bin range for DF processing
            // If g_df_start_bin and g_df_end_bin are both 0, use entire spectrum
            const uint32_t df_start = g_df_start_bin.load();
            const uint32_t df_end = g_df_end_bin.load();
            const size_t bin_start = (df_start == 0 && df_end == 0) ? 0 : std::min(static_cast<size_t>(df_start), static_cast<size_t>(FFT_SIZE - 1));
            const size_t bin_end = (df_start == 0 && df_end == 0) ? FFT_SIZE - 1 : std::min(static_cast<size_t>(df_end), static_cast<size_t>(FFT_SIZE - 1));
            const size_t bin_count = bin_end - bin_start + 1;

            // Get current noise floor estimates for improved CFAR and SNR
            float nf_ch1, nf_ch2;
            get_noise_floor(g_noise_floor, nf_ch1, nf_ch2);

            // Perform complete direction finding analysis with dynamic noise floor
            DFResult df_result = compute_direction_finding(
                fft_out_ch1.data(), fft_out_ch2.data(),
                ch1_mag, ch2_mag,
                FFT_SIZE, bin_start, bin_end,
                g_center_freq.load(),
                g_last_valid_doa,
                nf_ch1, nf_ch2
            );

            // Debug output every 60 frames
            static int df_debug_counter = 0;
            df_debug_counter++;
            if (df_debug_counter % 60 == 0) {
                std::cout << "CFAR: " << df_result.num_signals << " signals, "
                          << df_result.num_bins << " bins";
                if (bin_start > 0 || bin_end < FFT_SIZE - 1) {
                    std::cout << " [" << bin_start << "-" << bin_end << "/" << bin_count << "]";
                }

                // Calculate mean and peak for debug output
                uint32_t magnitude_sum = 0;
                uint8_t peak_mag = 0;
                for (size_t i = bin_start; i <= bin_end; i++) {
                    const uint8_t avg_mag = (ch1_mag[i] + ch2_mag[i]) / 2;
                    magnitude_sum += avg_mag;
                    if (avg_mag > peak_mag) peak_mag = avg_mag;
                }
                const uint8_t mean_mag = magnitude_sum / bin_count;

                std::cout << " (mean=" << static_cast<int>(mean_mag)
                          << ", peak=" << static_cast<int>(peak_mag) << ") | "
                          << "Phase: " << std::fixed << std::setprecision(1)
                          << df_result.phase_diff_deg << "° ± " << df_result.phase_std_deg << "° | "
                          << "Az: " << df_result.azimuth << "°";
                if (df_result.is_holding) {
                    std::cout << " [HOLD]";
                }
                std::cout << " | SNR: " << df_result.snr_db << " dB | "
                          << "Conf: " << df_result.confidence << "%" << std::endl;
            }

            // Update DoA result buffer for web interface
            update_doa_result(df_result.azimuth, df_result.back_azimuth,
                            df_result.phase_diff_deg, df_result.phase_std_deg,
                            df_result.confidence, df_result.snr_db, df_result.coherence);


            // ===== FREQUENCY SCANNER =====
            // Scan across frequency range and detect signals above threshold
            {
                std::lock_guard<std::mutex> lock(g_scanner.mutex);
                if (g_scanner.active) {
                    // Analyze spectrum to extract signal characteristics
                    SignalCharacteristics signal = analyze_spectrum(
                        ch1_mag,
                        FFT_SIZE,
                        g_center_freq.load(),
                        g_sample_rate.load()
                    );

                    // If above threshold, add or update signal
                    if (signal.power_dbm > g_scanner.threshold_dbm) {
                        // Check if signal already exists (within 1 MHz tolerance)
                        bool found = false;
                        const int64_t tolerance = 1000000; // 1 MHz
                        for (auto& sig : g_scanner.signals) {
                            if (std::abs(static_cast<int64_t>(sig.frequency) - static_cast<int64_t>(signal.frequency)) < tolerance) {
                                // Update existing signal
                                sig.power_dbm = std::max(sig.power_dbm, signal.power_dbm);
                                sig.bandwidth_hz = std::max(sig.bandwidth_hz, signal.bandwidth_hz);
                                sig.last_seen = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
                                sig.hit_count++;
                                found = true;
                                break;
                            }
                        }

                        // Add new signal if not found
                        if (!found && g_scanner.signals.size() < 1000) { // Limit to 1000 signals
                            DetectedSignal new_sig;
                            new_sig.frequency = signal.frequency;
                            new_sig.power_dbm = signal.power_dbm;
                            new_sig.bandwidth_hz = signal.bandwidth_hz;
                            const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
                            new_sig.first_seen = now_ms;
                            new_sig.last_seen = now_ms;
                            new_sig.hit_count = 1;
                            g_scanner.signals.push_back(new_sig);
                        }
                    }

                    // Move to next frequency after dwell time
                    static auto last_scan_step = std::chrono::steady_clock::now();
                    const auto scan_now = std::chrono::steady_clock::now();
                    const auto scan_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(scan_now - last_scan_step);
                    if (scan_elapsed.count() >= g_scanner.dwell_ms) {
                        // Step to next frequency
                        g_scanner.current_freq += g_scanner.step_size;
                        if (g_scanner.current_freq > g_scanner.stop_freq) {
                            g_scanner.current_freq = g_scanner.start_freq;
                            g_scanner.scan_count++;
                        }

                        // Update hardware frequency
                        g_center_freq.store(g_scanner.current_freq);
                        g_params_changed.store(true);

                        last_scan_step = scan_now;
                    }
                }
            }

            // Update FPS tracking
            frame_count++;

            // Update link quality every second
            const auto now = std::chrono::steady_clock::now();
            const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_update_time);
            if (fps_elapsed.count() >= 1000) {
                const float actual_fps = static_cast<float>(frame_count) * 1000.0f / fps_elapsed.count();
                // Get actual HTTP bytes sent (automatically resets counter)
                uint64_t actual_bytes_sent = get_and_reset_http_bytes();
                update_link_quality(actual_fps, actual_bytes_sent);
                frame_count = 0;
                fps_update_time = now;
            }

            // Rate limiting
            const auto end_time = std::chrono::steady_clock::now();
            const auto elapsed = end_time - start_time;
            if (elapsed < sleep_duration) {
                std::this_thread::sleep_for(sleep_duration - elapsed);
            }
        }
    });

    // Wait for threads to finish (run until g_running becomes false)
    if (rx_thread.joinable()) {
        std::cout << "Waiting for RX thread to finish..." << std::endl;
        rx_thread.join();
    }

    // Server shutting down - cleanup
    std::cout << "\n========================================" << std::endl;
    std::cout << "Server shutdown initiated" << std::endl;
    std::cout << "========================================\n" << std::endl;

    std::cout << "[1/6] Stopping web server..." << std::endl;
    stop_web_server();

    std::cout << "[2/6] Disabling RX channel 1..." << std::endl;
    bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);

    std::cout << "[3/6] Disabling RX channel 2..." << std::endl;
    bladerf_enable_module(dev, BLADERF_CHANNEL_RX(1), false);

    std::cout << "[4/6] Closing bladeRF device..." << std::endl;
    bladerf_close(dev);

    std::cout << "[5/6] Destroying FFTW plans..." << std::endl;
    fftwf_destroy_plan(fft_plan_ch1);
    fftwf_destroy_plan(fft_plan_ch2);

    std::cout << "[6/6] Cleaning up FFTW..." << std::endl;
    fftwf_cleanup();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Cleanup complete. Goodbye!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
