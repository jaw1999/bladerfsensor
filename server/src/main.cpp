#include "bladerf_sensor.h"
#include "web_server.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <iomanip>
#include <iostream>
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

// FFTW plans and buffers for FFT computation
// These are shared between threads and must be accessed carefully
fftwf_plan fft_plan_ch1 = nullptr;                    // FFT plan for channel 1
fftwf_plan fft_plan_ch2 = nullptr;                    // FFT plan for channel 2
std::vector<fftwf_complex> fft_in_ch1(FFT_SIZE);      // Channel 1 FFT input buffer
std::vector<fftwf_complex> fft_in_ch2(FFT_SIZE);      // Channel 2 FFT input buffer
std::vector<fftwf_complex> fft_out_ch1(FFT_SIZE);     // Channel 1 FFT output buffer
std::vector<fftwf_complex> fft_out_ch2(FFT_SIZE);     // Channel 2 FFT output buffer

// DC offset correction using exponentially weighted moving average (EWMA)
// Tracks and removes DC bias from IQ samples for cleaner spectrum display
float g_dc_i_ch1 = 0.0f;                              // Channel 1 I component DC offset
float g_dc_q_ch1 = 0.0f;                              // Channel 1 Q component DC offset
float g_dc_i_ch2 = 0.0f;                              // Channel 2 I component DC offset
float g_dc_q_ch2 = 0.0f;                              // Channel 2 Q component DC offset
uint64_t g_last_freq_for_dc = 0;                      // Last frequency (for reset detection)
int g_dc_convergence_counter = 0;                     // Convergence tracking counter

// IQ sample recording state
RecordingState g_recording = {false, nullptr, 0, 0, 0, {}};
std::mutex g_recording_mutex;                         // Mutex for recording state access

// Automatic gain control (AGC) state
struct AGCState {
    bool enabled;                  // AGC enable flag
    float current_level;           // Current signal level (0-255)
    uint32_t current_gain_rx1;     // Current RX1 gain (dB)
    uint32_t current_gain_rx2;     // Current RX2 gain (dB)
    int hysteresis_counter;        // Counter to prevent rapid gain changes
};
AGCState g_agc = {false, 0.0f, GAIN_RX1, GAIN_RX2, 0};

// FFT window function state
uint32_t g_window_type = WINDOW_HAMMING;              // Current window function type
std::vector<float> g_window;                          // Precomputed window coefficients

// FFT averaging state for noise reduction
uint32_t g_averaging_frames = 1;                      // Number of frames to average (1 = disabled)
std::vector<std::vector<uint8_t>> g_avg_buffer_ch1;  // Channel 1 averaging buffer
std::vector<std::vector<uint8_t>> g_avg_buffer_ch2;  // Channel 2 averaging buffer
uint32_t g_avg_index = 0;                             // Current averaging buffer index


// Forward declarations
void stop_recording();

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

// ============================================================================
// IQ RECORDING SYSTEM
//
// Functions for recording raw IQ samples to disk with metadata headers
// Supports continuous recording with automatic file header updates
// ============================================================================

bool start_recording(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_recording_mutex);

    if (g_recording.active) {
        std::cerr << "Recording already in progress" << std::endl;
        return false;
    }

    g_recording.file = fopen(filename.c_str(), "wb");
    if (!g_recording.file) {
        std::cerr << "Failed to open recording file: " << filename << std::endl;
        return false;
    }

    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    g_recording.metadata.timestamp_start_sec = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    g_recording.metadata.timestamp_start_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
    g_recording.metadata.center_freq = g_center_freq;
    g_recording.metadata.sample_rate = g_sample_rate;
    g_recording.metadata.bandwidth = BANDWIDTH;
    g_recording.metadata.gain_rx1 = g_gain_rx1;
    g_recording.metadata.gain_rx2 = g_gain_rx2;
    g_recording.metadata.num_samples = 0;
    strncpy(g_recording.metadata.notes, "bladeRF recording", sizeof(g_recording.metadata.notes) - 1);

    fwrite(&g_recording.metadata, sizeof(RecordingMetadata), 1, g_recording.file);

    g_recording.active = true;
    g_recording.samples_written = 0;
    g_recording.start_time_sec = g_recording.metadata.timestamp_start_sec;
    g_recording.start_time_nsec = g_recording.metadata.timestamp_start_nsec;

    std::cout << "Recording started: " << filename << std::endl;
    return true;
}

void stop_recording() {
    std::lock_guard<std::mutex> lock(g_recording_mutex);

    if (!g_recording.active) {
        return;
    }

    g_recording.metadata.num_samples = g_recording.samples_written;
    fseek(g_recording.file, 0, SEEK_SET);
    fwrite(&g_recording.metadata, sizeof(RecordingMetadata), 1, g_recording.file);

    fclose(g_recording.file);
    g_recording.file = nullptr;
    g_recording.active = false;

    std::cout << "Recording stopped. Samples written: " << g_recording.samples_written << std::endl;
}

void write_samples_to_file(const int16_t* samples, size_t num_samples) {
    std::lock_guard<std::mutex> lock(g_recording_mutex);

    if (!g_recording.active || !g_recording.file) {
        return;
    }

    size_t written = fwrite(samples, sizeof(int16_t), num_samples * 2, g_recording.file);
    if (written != num_samples * 2) {
        std::cerr << "Warning: Incomplete write to recording file" << std::endl;
    }

    g_recording.samples_written += num_samples;

    if (g_recording.samples_written % (100 * 1024 * 1024 / 4) == 0) {
        fflush(g_recording.file);
    }
}

// ============================================================================
// AUTOMATIC GAIN CONTROL (AGC)
//
// Software-based automatic gain control to maintain optimal signal levels
// Adjusts RX gain dynamically based on measured spectrum peaks with hysteresis
// to prevent oscillation
// ============================================================================

void update_agc(const uint8_t* ch1_mag, const uint8_t* ch2_mag, size_t len) {
    if (!g_agc.enabled) {
        return;
    }

    uint8_t peak = 0;
    for (size_t i = 0; i < len; i++) {
        peak = std::max(peak, std::max(ch1_mag[i], ch2_mag[i]));
    }

    g_agc.current_level = peak;

    if (peak > AGC_TARGET_LEVEL + AGC_HYSTERESIS) {
        g_agc.hysteresis_counter++;
        if (g_agc.hysteresis_counter > 5) {
            if (g_agc.current_gain_rx1 > 0) {
                g_agc.current_gain_rx1 = std::max(0u, g_agc.current_gain_rx1 - 3);
                g_agc.current_gain_rx2 = std::max(0u, g_agc.current_gain_rx2 - 3);
                g_gain_rx1 = g_agc.current_gain_rx1;
                g_gain_rx2 = g_agc.current_gain_rx2;
                g_params_changed = true;
                std::cout << "AGC: Decreasing gain to " << g_agc.current_gain_rx1 << " dB" << std::endl;
            }
            g_agc.hysteresis_counter = 0;
        }
    } else if (peak < AGC_TARGET_LEVEL - AGC_HYSTERESIS) {
        g_agc.hysteresis_counter++;
        if (g_agc.hysteresis_counter > 20) {
            if (g_agc.current_gain_rx1 < 60) {
                g_agc.current_gain_rx1 = std::min(60u, g_agc.current_gain_rx1 + 1);
                g_agc.current_gain_rx2 = std::min(60u, g_agc.current_gain_rx2 + 1);
                g_gain_rx1 = g_agc.current_gain_rx1;
                g_gain_rx2 = g_agc.current_gain_rx2;
                g_params_changed = true;
                std::cout << "AGC: Increasing gain to " << g_agc.current_gain_rx1 << " dB" << std::endl;
            }
            g_agc.hysteresis_counter = 0;
        }
    } else {
        g_agc.hysteresis_counter = 0;
    }
}

// ============================================================================
// WINDOW FUNCTIONS
//
// FFT window function generation and application
// Reduces spectral leakage by tapering the edges of the time-domain signal
// Supports multiple window types: Rectangular Hamming Hanning Blackman Kaiser
// ============================================================================

void generate_window(uint32_t window_type, size_t length) {
    g_window.resize(length);

    switch (window_type) {
        case WINDOW_RECTANGULAR:
            std::fill(g_window.begin(), g_window.end(), 1.0f);
            break;

        case WINDOW_HAMMING:
            for (size_t i = 0; i < length; i++) {
                g_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (length - 1));
            }
            break;

        case WINDOW_HANNING:
            for (size_t i = 0; i < length; i++) {
                g_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (length - 1)));
            }
            break;

        case WINDOW_BLACKMAN:
            for (size_t i = 0; i < length; i++) {
                g_window[i] = 0.42f - 0.5f * cosf(2.0f * M_PI * i / (length - 1)) +
                             0.08f * cosf(4.0f * M_PI * i / (length - 1));
            }
            break;

        case WINDOW_KAISER:
            for (size_t i = 0; i < length; i++) {
                float x = 2.0f * i / (length - 1) - 1.0f;
                g_window[i] = 0.402f + 0.498f * cosf(M_PI * x) + 0.098f * cosf(2.0f * M_PI * x);
            }
            break;

        default:
            std::fill(g_window.begin(), g_window.end(), 1.0f);
    }
}

void apply_window(fftwf_complex* data, size_t length) {
    if (g_window.size() != length) {
        generate_window(g_window_type, length);
    }

    for (size_t i = 0; i < length; i++) {
        data[i][0] *= g_window[i];
        data[i][1] *= g_window[i];
    }
}

// ============================================================================
// FFT AVERAGING
//
// Temporal averaging of FFT magnitude spectra to reduce noise and variance
// Maintains a circular buffer of recent FFT frames and computes running average
// ============================================================================

void init_averaging(uint32_t num_frames, size_t fft_size) {
    g_averaging_frames = num_frames;
    g_avg_buffer_ch1.resize(num_frames);
    g_avg_buffer_ch2.resize(num_frames);
    for (auto& buf : g_avg_buffer_ch1) {
        buf.resize(fft_size, 0);
    }
    for (auto& buf : g_avg_buffer_ch2) {
        buf.resize(fft_size, 0);
    }
    g_avg_index = 0;
}

void apply_averaging(uint8_t* ch1_mag, uint8_t* ch2_mag, size_t fft_size) {
    if (g_averaging_frames <= 1) {
        return;
    }

    std::copy(ch1_mag, ch1_mag + fft_size, g_avg_buffer_ch1[g_avg_index].begin());
    std::copy(ch2_mag, ch2_mag + fft_size, g_avg_buffer_ch2[g_avg_index].begin());
    g_avg_index = (g_avg_index + 1) % g_averaging_frames;

    for (size_t i = 0; i < fft_size; i++) {
        uint32_t sum1 = 0, sum2 = 0;
        for (uint32_t j = 0; j < g_averaging_frames; j++) {
            sum1 += g_avg_buffer_ch1[j][i];
            sum2 += g_avg_buffer_ch2[j][i];
        }
        ch1_mag[i] = sum1 / g_averaging_frames;
        ch2_mag[i] = sum2 / g_averaging_frames;
    }
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

void compute_fft(fftwf_complex *in, fftwf_complex *out, fftwf_plan plan) {
    fftwf_execute_dft(plan, in, out);
}

void compute_magnitude_db(fftwf_complex *fft_out, uint8_t *mag_out, size_t size) {
    // Compute absolute magnitude in dB
    // This makes gain changes directly visible on the display

    static int debug_counter = 0;
    float min_db = 1000.0f;
    float max_db = -1000.0f;

    for (size_t i = 0; i < size; i++) {
        float real = fft_out[i][0];
        float imag = fft_out[i][1];
        float mag = std::sqrt(real * real + imag * imag);

        // Convert to absolute dB (magnitude squared gives power)
        float db = 10.0f * std::log10(std::max(mag * mag, 1e-20f));

        // Track min/max for debugging
        if (db < min_db) min_db = db;
        if (db > max_db) max_db = db;

        // Map wider range (-100 dB to +20 dB) to 0 to 255
        // This accommodates signals from noise floor to strong peaks
        // Total range 120 dB mapped to 0-255
        float normalized = (db + 100.0f) / 120.0f * 255.0f;
        mag_out[i] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, normalized)));
    }

    // Print debug info every 60 frames
    debug_counter++;
    if (debug_counter % 60 == 0) {
        std::cout << "FFT dB range: " << min_db << " to " << max_db
                  << " dB (gain RX1=" << g_gain_rx1.load() << " dB)" << std::endl;
    }
}

void remove_dc_offset(uint8_t *magnitude, size_t size) {
    const size_t dc_bin = size / 2;

    // Only smooth the immediate center bin to avoid artifacts
    // Don't aggressively interpolate as it can create dips
    if (dc_bin < 2 || dc_bin >= size - 2) return;

    // Simple 3-bin averaging to smooth center without creating dip
    const uint32_t avg = (static_cast<uint32_t>(magnitude[dc_bin - 1]) +
                          static_cast<uint32_t>(magnitude[dc_bin]) +
                          static_cast<uint32_t>(magnitude[dc_bin + 1])) / 3;

    magnitude[dc_bin] = static_cast<uint8_t>(avg);
}

void compute_cross_correlation(fftwf_complex *fft_ch1, fftwf_complex *fft_ch2,
                              float *correlation, float *phase_diff) {
    // Cross-correlation in frequency domain conj(FFT1) * FFT2
    for (size_t i = 0; i < FFT_SIZE; i++) {
        float real1 = fft_ch1[i][0];
        float imag1 = -fft_ch1[i][1];  // Complex conjugate
        float real2 = fft_ch2[i][0];
        float imag2 = fft_ch2[i][1];

        // Complex multiplication
        float corr_real = real1 * real2 - imag1 * imag2;
        float corr_imag = real1 * imag2 + imag1 * real2;

        // Magnitude and phase
        correlation[i] = std::sqrt(corr_real * corr_real + corr_imag * corr_imag);
        phase_diff[i] = std::atan2(corr_imag, corr_real);
    }
}

bool validate_frequency(uint64_t freq) {
    // bladeRF xA9 frequency range 47 MHz to 6 GHz
    return freq >= 47000000 && freq <= 6000000000ULL;
}

bool validate_sample_rate(uint32_t rate) {
    // Valid range 520 kHz to 61.44 MHz
    return rate >= 520000 && rate <= 61440000;
}

bool validate_gain(uint32_t gain) {
    // Valid gain range 0 to 60 dB
    return gain <= 60;
}

bool validate_bandwidth(uint32_t bw) {
    // Valid range 520 kHz to 61.44 MHz (same as sample rate)
    return bw >= 520000 && bw <= 61440000;
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

    // Initialize FFTW
    std::cout << "Initializing FFTW..." << std::endl;
    fft_plan_ch1 = fftwf_plan_dft_1d(FFT_SIZE, fft_in_ch1.data(), fft_out_ch1.data(),
                                     FFTW_FORWARD, FFTW_MEASURE);
    fft_plan_ch2 = fftwf_plan_dft_1d(FFT_SIZE, fft_in_ch2.data(), fft_out_ch2.data(),
                                     FFTW_FORWARD, FFTW_MEASURE);

    // Initialize window function (default: Hamming)
    std::cout << "Initializing window function..." << std::endl;
    generate_window(g_window_type, FFT_SIZE);

    // Initialize averaging (default: no averaging)
    std::cout << "Initializing FFT averaging..." << std::endl;
    init_averaging(1, FFT_SIZE);

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

            // Deinterleave and process samples
            const size_t samples_to_process = std::min(static_cast<size_t>(BUFFER_SIZE / 2), static_cast<size_t>(FFT_SIZE));
            constexpr float scale = 1.0f / 32768.0f;

            // Track peak sample for debugging
            static int web_sample_debug_counter = 0;
            int16_t peak_sample = 0;

            for (size_t i = 0; i < samples_to_process; i++) {
                const size_t idx = i * 4;

                // Track peak ADC values
                if (std::abs(thread_rx_buffer[idx + 0]) > std::abs(peak_sample))
                    peak_sample = thread_rx_buffer[idx + 0];
                if (std::abs(thread_rx_buffer[idx + 1]) > std::abs(peak_sample))
                    peak_sample = thread_rx_buffer[idx + 1];

                fft_in_ch1[i][0] = static_cast<float>(thread_rx_buffer[idx + 0]) * scale;
                fft_in_ch1[i][1] = static_cast<float>(thread_rx_buffer[idx + 1]) * scale;
                fft_in_ch2[i][0] = static_cast<float>(thread_rx_buffer[idx + 2]) * scale;
                fft_in_ch2[i][1] = static_cast<float>(thread_rx_buffer[idx + 3]) * scale;
            }

            // Debug: Print peak sample every 60 frames
            web_sample_debug_counter++;
            if (web_sample_debug_counter % 60 == 0) {
                float saturation_pct = (std::abs(peak_sample) / 32768.0f) * 100.0f;
                std::cout << "Peak ADC sample: " << peak_sample << " ("
                          << std::fixed << std::setprecision(1) << saturation_pct
                          << "% of full scale) [Gain=" << g_gain_rx1.load() << "dB]" << std::endl;
            }

            if (samples_to_process < FFT_SIZE) {
                std::memset(fft_in_ch1.data() + samples_to_process, 0, (FFT_SIZE - samples_to_process) * sizeof(fftwf_complex));
                std::memset(fft_in_ch2.data() + samples_to_process, 0, (FFT_SIZE - samples_to_process) * sizeof(fftwf_complex));
            }

            // Remove DC offset from IQ samples using EWMA
            const uint64_t current_freq = g_center_freq.load();
            if (current_freq != g_last_freq_for_dc) {
                g_last_freq_for_dc = current_freq;
                g_dc_convergence_counter = 0;
                g_dc_i_ch1 = 0.0f;
                g_dc_q_ch1 = 0.0f;
                g_dc_i_ch2 = 0.0f;
                g_dc_q_ch2 = 0.0f;
            }

            float dc_i_ch1 = 0.0f, dc_q_ch1 = 0.0f;
            float dc_i_ch2 = 0.0f, dc_q_ch2 = 0.0f;

            for (size_t i = 0; i < samples_to_process; i++) {
                dc_i_ch1 += fft_in_ch1[i][0];
                dc_q_ch1 += fft_in_ch1[i][1];
                dc_i_ch2 += fft_in_ch2[i][0];
                dc_q_ch2 += fft_in_ch2[i][1];
            }

            dc_i_ch1 /= samples_to_process;
            dc_q_ch1 /= samples_to_process;
            dc_i_ch2 /= samples_to_process;
            dc_q_ch2 /= samples_to_process;

            float alpha = (g_dc_convergence_counter < 20) ? 0.5f : 0.1f;
            g_dc_convergence_counter++;

            g_dc_i_ch1 = alpha * dc_i_ch1 + (1.0f - alpha) * g_dc_i_ch1;
            g_dc_q_ch1 = alpha * dc_q_ch1 + (1.0f - alpha) * g_dc_q_ch1;
            g_dc_i_ch2 = alpha * dc_i_ch2 + (1.0f - alpha) * g_dc_i_ch2;
            g_dc_q_ch2 = alpha * dc_q_ch2 + (1.0f - alpha) * g_dc_q_ch2;

            for (size_t i = 0; i < samples_to_process; i++) {
                fft_in_ch1[i][0] -= g_dc_i_ch1;
                fft_in_ch1[i][1] -= g_dc_q_ch1;
                fft_in_ch2[i][0] -= g_dc_i_ch2;
                fft_in_ch2[i][1] -= g_dc_q_ch2;
            }

            // Apply window function
            apply_window(fft_in_ch1.data(), FFT_SIZE);
            apply_window(fft_in_ch2.data(), FFT_SIZE);

            // Compute FFTs
            compute_fft(fft_in_ch1.data(), fft_out_ch1.data(), fft_plan_ch1);
            compute_fft(fft_in_ch2.data(), fft_out_ch2.data(), fft_plan_ch2);

            // Compute magnitudes
            uint8_t ch1_mag[FFT_SIZE], ch2_mag[FFT_SIZE];
            compute_magnitude_db(fft_out_ch1.data(), ch1_mag, FFT_SIZE);
            compute_magnitude_db(fft_out_ch2.data(), ch2_mag, FFT_SIZE);

            // Apply averaging
            apply_averaging(ch1_mag, ch2_mag, FFT_SIZE);

            // Update AGC
            update_agc(ch1_mag, ch2_mag, FFT_SIZE);

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
            update_iq_data(reinterpret_cast<int16_t*>(ch1_iq), reinterpret_cast<int16_t*>(ch2_iq), 256);

            // Compute and update cross-correlation data
            float xcorr_mag[FFT_SIZE], xcorr_phase[FFT_SIZE];
            for (size_t i = 0; i < FFT_SIZE; i++) {
                // Compute complex conjugate multiplication: conj(ch1) * ch2
                // This gives phase difference: phase(ch2) - phase(ch1)
                const float real1 = fft_out_ch1[i][0];
                const float imag1 = -fft_out_ch1[i][1];  // Complex conjugate
                const float real2 = fft_out_ch2[i][0];
                const float imag2 = fft_out_ch2[i][1];

                const float xcorr_real = real1 * real2 - imag1 * imag2;
                const float xcorr_imag = real1 * imag2 + imag1 * real2;

                xcorr_mag[i] = std::sqrt(xcorr_real * xcorr_real + xcorr_imag * xcorr_imag);
                xcorr_phase[i] = std::atan2(xcorr_imag, xcorr_real);
            }
            update_xcorr_data(xcorr_mag, xcorr_phase, FFT_SIZE);

            // ===== DIRECTION OF ARRIVAL (DoA) CALCULATION =====
            // Phase-based 2-channel interferometry using full IQ buffer
            // Theory: Δφ = (2π * d * sin(θ)) / λ
            //
            // Calculate instantaneous phase difference for each sample
            float phase_diff_sum = 0.0f;
            std::vector<float> phase_values;
            phase_values.reserve(samples_to_process);

            for (size_t i = 0; i < samples_to_process; i++) {
                // Convert I/Q to phase for each antenna (after DC removal and windowing)
                const float phase1 = std::atan2(fft_in_ch1[i][1], fft_in_ch1[i][0]);  // CH1: atan2(Q, I)
                const float phase2 = std::atan2(fft_in_ch2[i][1], fft_in_ch2[i][0]);  // CH2: atan2(Q, I)

                // Compute phase difference (CH2 - CH1)
                // Note: No calibration offset applied here (could be added later via config)
                float diff = phase2 - phase1;

                // Wrap phase difference to [-π, π] to handle 2π ambiguity
                while (diff > M_PI) diff -= 2.0f * M_PI;
                while (diff < -M_PI) diff += 2.0f * M_PI;

                phase_diff_sum += diff;
                phase_values.push_back(diff);
            }

            // Average phase difference across all samples
            const float avg_phase_diff_rad = phase_diff_sum / samples_to_process;  // radians
            const float avg_phase_diff_deg = avg_phase_diff_rad * 180.0f / M_PI;   // degrees

            // Calculate phase standard deviation (measure of signal quality)
            float variance = 0.0f;
            for (const float phase_val : phase_values) {
                const float diff = phase_val - avg_phase_diff_rad;
                variance += diff * diff;
            }
            const float std_dev_rad = std::sqrt(variance / samples_to_process);
            const float std_dev_deg = std_dev_rad * 180.0f / M_PI;

            // Convert phase difference to angle of arrival
            // Antenna spacing assumption: 0.5 wavelengths (typical for DF)
            // For bladeRF at 915 MHz: λ = c/f = 0.328m, so 0.5λ = 0.164m = 164mm
            constexpr float antenna_spacing_wavelengths = 0.5f;
            constexpr float lambda = 1.0f;  // Normalized (spacing already in wavelengths)

            // Apply interferometer equation: sin(θ) = (Δφ * λ) / (2π * d)
            float sin_theta = (avg_phase_diff_rad * lambda) / (2.0f * M_PI * antenna_spacing_wavelengths);
            sin_theta = std::max(-1.0f, std::min(1.0f, sin_theta));  // Clamp to [-1, 1] for asin

            // Get primary angle from arcsin (returns angle in [-90°, 90°])
            const float azimuth_deg = std::asin(sin_theta) * 180.0f / M_PI;

            // Calculate back azimuth (180° ambiguity inherent to 2-channel DF)
            // For 2-element array: sin(θ) = sin(180° - θ)
            const float back_azimuth_deg = 180.0f - azimuth_deg;

            // Normalize to [0, 360) for display
            float azimuth_norm = azimuth_deg < 0 ? azimuth_deg + 360.0f : azimuth_deg;
            float back_azimuth_norm = back_azimuth_deg < 0 ? back_azimuth_deg + 360.0f : back_azimuth_deg;

            // Calculate SNR estimate from IQ power (RMS power)
            float sum_power = 0.0f;
            for (size_t i = 0; i < samples_to_process; i++) {
                const float i_sample = fft_in_ch1[i][0];
                const float q_sample = fft_in_ch1[i][1];
                sum_power += i_sample * i_sample + q_sample * q_sample;
            }
            const float signal_rms = std::sqrt(sum_power / samples_to_process);
            const float snr_db = 20.0f * std::log10(signal_rms / 0.01f);  // vs noise baseline

            // Calculate confidence based on phase stability
            // Lower std_dev = higher confidence, reduced for 180° ambiguity
            const float confidence = std::max(0.0f, std::min(100.0f, (100.0f - std_dev_deg * 2.0f) * 0.9f));

            // Calculate coherence metric (exponential decay with std_dev)
            const float coherence = std::exp(-std_dev_deg / 10.0f);

            // Update DoA result buffer for web interface
            update_doa_result(azimuth_norm, back_azimuth_norm, avg_phase_diff_deg,
                            std_dev_deg, confidence, snr_db, coherence);

            // ===== FREQUENCY SCANNER =====
            // Scan across frequency range and detect signals above threshold
            {
                std::lock_guard<std::mutex> lock(g_scanner.mutex);
                if (g_scanner.active) {
                    // Detect signals in current FFT data
                    // Find peak power in spectrum
                    uint8_t peak_mag = 0;
                    size_t peak_bin = 0;
                    for (size_t i = 0; i < FFT_SIZE; i++) {
                        if (ch1_mag[i] > peak_mag) {
                            peak_mag = ch1_mag[i];
                            peak_bin = i;
                        }
                    }

                    // Convert 8-bit magnitude to approximate dBm
                    // Assuming 0-255 maps to roughly -120 to 0 dBm range
                    const float power_dbm = (peak_mag / 255.0f) * 120.0f - 120.0f;

                    // If above threshold, add or update signal
                    if (power_dbm > g_scanner.threshold_dbm) {
                        // Calculate signal frequency (peak bin relative to center)
                        const uint64_t current_freq = g_center_freq.load();
                        const uint32_t sample_rate = g_sample_rate.load();
                        const int64_t bin_offset = static_cast<int64_t>(peak_bin) - static_cast<int64_t>(FFT_SIZE / 2);
                        const uint64_t signal_freq = current_freq + (bin_offset * sample_rate / FFT_SIZE);

                        // Estimate bandwidth (count bins above threshold - 6dB)
                        const uint8_t bw_threshold = peak_mag > 12 ? peak_mag - 12 : 0;
                        size_t bw_bins = 0;
                        for (size_t i = 0; i < FFT_SIZE; i++) {
                            if (ch1_mag[i] >= bw_threshold) bw_bins++;
                        }
                        const float bandwidth_hz = (bw_bins * sample_rate) / static_cast<float>(FFT_SIZE);

                        // Check if signal already exists (within 1 MHz tolerance)
                        bool found = false;
                        const int64_t tolerance = 1000000; // 1 MHz
                        for (auto& sig : g_scanner.signals) {
                            if (std::abs(static_cast<int64_t>(sig.frequency) - static_cast<int64_t>(signal_freq)) < tolerance) {
                                // Update existing signal
                                sig.power_dbm = std::max(sig.power_dbm, power_dbm);
                                sig.bandwidth_hz = std::max(sig.bandwidth_hz, bandwidth_hz);
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
                            new_sig.frequency = signal_freq;
                            new_sig.power_dbm = power_dbm;
                            new_sig.bandwidth_hz = bandwidth_hz;
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
