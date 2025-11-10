#include "bladerf_sensor.h"
#include "web_server.h"
#include "cfar_detector.h"
#include "array_calibration.h"
#include "signal_processing.h"
#include "df_processing.h"
#include "recording.h"
#include "config_validation.h"
#include "telemetry.h"
#include "pipeline.h"
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

// Watchdog state for RX thread health monitoring
std::atomic<uint64_t> g_rx_heartbeat{0};              // Incremented by RX thread each cycle
std::atomic<bool> g_watchdog_enabled{true};           // Watchdog monitoring enabled flag

// Direction finding bin range selection (0 = entire spectrum)
std::atomic<uint32_t> g_df_start_bin{0};              // DF start bin (0 = use entire spectrum)
std::atomic<uint32_t> g_df_end_bin{0};                // DF end bin (0 = use entire spectrum)

// FFT window function state (used for initial pipeline setup only)
uint32_t g_window_type = WINDOW_HAMMING;              // Current window function type

// Global noise floor state (shared for web server reporting)
NoiseFloorState g_noise_floor;

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

    // Initialize telemetry
    std::cout << "Initializing telemetry..." << std::endl;
    init_telemetry();

    // Initialize global noise floor state (for web server reporting)
    std::cout << "Initializing noise floor estimation..." << std::endl;
    init_noise_floor(g_noise_floor, FFT_SIZE);

    // Initialize FFTW with wisdom for fast startup
    std::cout << "Initializing FFTW..." << std::endl;
    const char* wisdom_file = "fftw_wisdom.dat";
    if (fftwf_import_wisdom_from_filename(wisdom_file)) {
        std::cout << "Loaded FFTW wisdom from " << wisdom_file << std::endl;
    } else {
        std::cout << "No FFTW wisdom file found, will create plans from scratch" << std::endl;
    }

    // Initialize pipeline infrastructure
    std::cout << "Initializing pipeline queues..." << std::endl;
    auto sample_queue = new LockFreeQueue<SampleBuffer>(PipelineConfig::SAMPLE_QUEUE_SIZE);
    auto fft_queue = new LockFreeQueue<FFTBuffer>(PipelineConfig::FFT_QUEUE_SIZE);

    // Create pipeline context
    PipelineContext pipeline_ctx;
    pipeline_ctx.sample_queue = sample_queue;
    pipeline_ctx.fft_queue = fft_queue;
    pipeline_ctx.running = &g_running;
    pipeline_ctx.center_freq = &g_center_freq;
    pipeline_ctx.sample_rate = &g_sample_rate;
    pipeline_ctx.bandwidth = &g_bandwidth;
    pipeline_ctx.gain_rx1 = &g_gain_rx1;
    pipeline_ctx.gain_rx2 = &g_gain_rx2;
    pipeline_ctx.params_changed = &g_params_changed;
    pipeline_ctx.config_mutex = &g_config_mutex;
    pipeline_ctx.df_start_bin = &g_df_start_bin;
    pipeline_ctx.df_end_bin = &g_df_end_bin;

    // Initialize processing state (owned by processing thread)
    init_dc_offset(pipeline_ctx.dc_offset);
    init_overlap(pipeline_ctx.overlap, FFT_SIZE);
    init_noise_floor(pipeline_ctx.noise_floor, FFT_SIZE);
    pipeline_ctx.global_noise_floor = &g_noise_floor;
    generate_window(g_window_type, FFT_SIZE, pipeline_ctx.window);

    // Allocate FFT buffers for processing thread (using raw allocation since fftwf_complex is float[2])
    pipeline_ctx.fft_in_ch1 = (fftwf_complex*)malloc(sizeof(fftwf_complex) * FFT_SIZE);
    pipeline_ctx.fft_in_ch2 = (fftwf_complex*)malloc(sizeof(fftwf_complex) * FFT_SIZE);
    pipeline_ctx.fft_out_ch1 = (fftwf_complex*)malloc(sizeof(fftwf_complex) * FFT_SIZE);
    pipeline_ctx.fft_out_ch2 = (fftwf_complex*)malloc(sizeof(fftwf_complex) * FFT_SIZE);
    pipeline_ctx.fft_size = FFT_SIZE;

    // Create FFT plans for processing thread
    pipeline_ctx.fft_plan_ch1 = fftwf_plan_dft_1d(FFT_SIZE, pipeline_ctx.fft_in_ch1,
                                                   pipeline_ctx.fft_out_ch1,
                                                   FFTW_FORWARD, FFTW_MEASURE);
    pipeline_ctx.fft_plan_ch2 = fftwf_plan_dft_1d(FFT_SIZE, pipeline_ctx.fft_in_ch2,
                                                   pipeline_ctx.fft_out_ch2,
                                                   FFTW_FORWARD, FFTW_MEASURE);

    // Save wisdom for future runs
    if (fftwf_export_wisdom_to_filename(wisdom_file)) {
        std::cout << "Saved FFTW wisdom to " << wisdom_file << std::endl;
    }

    std::cout << "Pipeline infrastructure initialized" << std::endl;

    // Initialize bladeRF
    status = initialize_bladerf(&dev);
    if (status != 0) {
        return 1;
    }

    // Attach device to pipeline context
    pipeline_ctx.device = dev;

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

    // Start watchdog thread to monitor RX thread health
    std::thread watchdog_thread([&]() {
        std::cout << "*** WATCHDOG THREAD STARTED ***" << std::endl;
        uint64_t last_heartbeat = 0;
        uint32_t stall_count = 0;
        constexpr uint32_t STALL_THRESHOLD = 3;  // Alert after 3 seconds of no progress

        while (g_running && g_watchdog_enabled) {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            uint64_t current_heartbeat = g_rx_heartbeat.load();
            if (current_heartbeat == last_heartbeat) {
                stall_count++;
                if (stall_count >= STALL_THRESHOLD) {
                    std::cerr << "*** WATCHDOG ALERT: RX thread appears stalled (no heartbeat for "
                              << stall_count << " seconds) ***" << std::endl;
                    if (stall_count >= 10) {
                        std::cerr << "*** WATCHDOG CRITICAL: RX thread hung for 10+ seconds - triggering shutdown ***" << std::endl;
                        g_running = false;
                    }
                }
            } else {
                if (stall_count > 0) {
                    std::cout << "WATCHDOG: RX thread recovered after " << stall_count << " second stall" << std::endl;
                }
                stall_count = 0;
                last_heartbeat = current_heartbeat;
            }
        }
        std::cout << "*** WATCHDOG THREAD STOPPED ***" << std::endl;
    });

    std::cout << "\n========================================" << std::endl;
    std::cout << "Starting 3-stage pipeline..." << std::endl;
    std::cout << "Web interface: http://localhost:" << WEB_SERVER_PORT << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Launch 3-stage pipeline threads
    std::thread acquisition_thread([&]() {
        acquisition_thread_func(&pipeline_ctx);
    });

    std::thread processing_thread([&]() {
        processing_thread_func(&pipeline_ctx);
    });

    std::thread analysis_thread([&]() {
        analysis_thread_func(&pipeline_ctx);
    });

    // Wait for pipeline threads to finish (run until g_running becomes false)
    std::cout << "Waiting for pipeline threads to finish..." << std::endl;

    if (acquisition_thread.joinable()) {
        std::cout << "  Waiting for acquisition thread..." << std::endl;
        acquisition_thread.join();
    }

    if (processing_thread.joinable()) {
        std::cout << "  Waiting for processing thread..." << std::endl;
        processing_thread.join();
    }

    if (analysis_thread.joinable()) {
        std::cout << "  Waiting for analysis thread..." << std::endl;
        analysis_thread.join();
    }

    if (watchdog_thread.joinable()) {
        std::cout << "  Waiting for watchdog thread..." << std::endl;
        watchdog_thread.join();
    }

    // Server shutting down - cleanup
    std::cout << "\n========================================" << std::endl;
    std::cout << "Server shutdown initiated" << std::endl;
    std::cout << "========================================\n" << std::endl;

    std::cout << "[1/8] Stopping web server..." << std::endl;
    stop_web_server();

    std::cout << "[2/8] Disabling RX channel 1..." << std::endl;
    bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);

    std::cout << "[3/8] Disabling RX channel 2..." << std::endl;
    bladerf_enable_module(dev, BLADERF_CHANNEL_RX(1), false);

    std::cout << "[4/8] Closing bladeRF device..." << std::endl;
    bladerf_close(dev);

    std::cout << "[5/8] Destroying pipeline FFTW plans..." << std::endl;
    fftwf_destroy_plan(pipeline_ctx.fft_plan_ch1);
    fftwf_destroy_plan(pipeline_ctx.fft_plan_ch2);

    std::cout << "[6/8] Freeing pipeline FFT buffers..." << std::endl;
    free(pipeline_ctx.fft_in_ch1);
    free(pipeline_ctx.fft_in_ch2);
    free(pipeline_ctx.fft_out_ch1);
    free(pipeline_ctx.fft_out_ch2);

    std::cout << "[7/8] Deleting pipeline queues..." << std::endl;
    delete sample_queue;
    delete fft_queue;

    std::cout << "[8/8] Cleaning up FFTW..." << std::endl;
    fftwf_cleanup();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Cleanup complete. Goodbye!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
