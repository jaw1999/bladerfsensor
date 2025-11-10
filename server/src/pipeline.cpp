#include "pipeline.h"
#include "telemetry.h"
#include "config.h"
#include "signal_processing.h"
#include "df_processing.h"
#include "cfar_detector.h"
#include "web_server.h"
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <thread>

// External watchdog heartbeat
extern std::atomic<uint64_t> g_rx_heartbeat;

// External DoA state for bearing hold and Kalman filtering
extern LastValidDoA g_last_valid_doa;

// ============================================================================
// Stage 1: Sample Acquisition Thread
// Continuously reads samples from bladeRF and pushes to sample queue
// ============================================================================

void acquisition_thread_func(PipelineContext* ctx) {
    std::cout << "[Pipeline] Acquisition thread started" << std::endl;

    // Allocate sample buffer (reused across iterations)
    constexpr size_t NUM_SAMPLES = 16384;  // Matches main.cpp buffer size
    constexpr size_t BUFFER_SIZE = NUM_SAMPLES * 2 * 2;  // 2 channels, I+Q

    SampleBuffer sample_buf;
    sample_buf.samples.resize(BUFFER_SIZE);
    sample_buf.count = NUM_SAMPLES;

    // USB error recovery state
    uint32_t consecutive_errors = 0;
    uint32_t error_backoff_ms = USBConfig::INITIAL_BACKOFF_MS;

    while (ctx->running->load(std::memory_order_acquire)) {
        // Check if parameters changed
        if (ctx->params_changed->load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(*ctx->config_mutex);
            ctx->params_changed->store(false, std::memory_order_release);

            std::cout << "[Acquisition] Applying parameter changes..." << std::endl;

            // Get new parameters
            const uint64_t freq = ctx->center_freq->load(std::memory_order_relaxed);
            const uint32_t sample_rate = ctx->sample_rate->load(std::memory_order_relaxed);
            const uint32_t bandwidth = ctx->bandwidth->load(std::memory_order_relaxed);
            const uint32_t gain_rx1 = ctx->gain_rx1->load(std::memory_order_relaxed);
            const uint32_t gain_rx2 = ctx->gain_rx2->load(std::memory_order_relaxed);

            std::cout << "  New frequency: " << (freq / 1e6) << " MHz" << std::endl;
            std::cout << "  New sample rate: " << (sample_rate / 1e6) << " MHz" << std::endl;
            std::cout << "  New bandwidth: " << (bandwidth / 1e6) << " MHz" << std::endl;
            std::cout << "  New gain RX1: " << gain_rx1 << " dB" << std::endl;
            std::cout << "  New gain RX2: " << gain_rx2 << " dB" << std::endl;

            // Validate parameters
            extern bool validate_frequency(uint64_t freq);
            extern bool validate_gain(uint32_t gain);
            if (!validate_frequency(freq) || !validate_gain(gain_rx1) || !validate_gain(gain_rx2)) {
                std::cerr << "[Acquisition] Invalid parameters - ignoring change request" << std::endl;
                continue;
            }

            // Disable modules before reconfiguration
            bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(0), false);
            bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(1), false);

            // Reconfigure channels
            extern int configure_channel(struct bladerf *dev, bladerf_channel ch, uint64_t freq,
                                        uint32_t gain, uint32_t sample_rate, uint32_t bandwidth);
            int status = configure_channel(ctx->device, BLADERF_CHANNEL_RX(0), freq, gain_rx1, sample_rate, bandwidth);
            if (status == 0) {
                status = configure_channel(ctx->device, BLADERF_CHANNEL_RX(1), freq, gain_rx2, sample_rate, bandwidth);
            }

            // Reconfigure sync RX (required after disabling modules)
            if (status == 0) {
                status = bladerf_sync_config(ctx->device,
                                            BLADERF_RX_X2,
                                            BLADERF_FORMAT_SC16_Q11,
                                            NUM_BUFFERS,
                                            BUFFER_SIZE,
                                            NUM_TRANSFERS,
                                            3500);
            }

            // ALWAYS re-enable modules (even if config failed)
            bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(0), true);
            bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(1), true);

            if (status == 0) {
                std::cout << "[Acquisition] Parameters updated successfully" << std::endl;
            } else {
                std::cerr << "[Acquisition] Failed to update parameters - keeping previous settings" << std::endl;
            }

            // Reset consecutive errors after reconfiguration
            consecutive_errors = 0;
            error_backoff_ms = USBConfig::INITIAL_BACKOFF_MS;
        }

        // Record timestamp before acquisition
        auto start = std::chrono::high_resolution_clock::now();
        sample_buf.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            start.time_since_epoch()).count();

        // Acquire samples from bladeRF
        int status = bladerf_sync_rx(ctx->device, sample_buf.samples.data(),
                                     NUM_SAMPLES, nullptr, 5000);

        g_telemetry.usb_transfer_count.fetch_add(1);

        if (status != 0) {
            // USB error - apply exponential backoff
            consecutive_errors++;
            g_telemetry.usb_errors.fetch_add(1);

            std::cerr << "[Acquisition] USB error (code " << status
                      << "), consecutive errors: " << consecutive_errors << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(error_backoff_ms));
            error_backoff_ms = std::min(error_backoff_ms * 2, USBConfig::MAX_BACKOFF_MS);

            if (consecutive_errors >= USBConfig::MAX_CONSECUTIVE_ERRORS) {
                std::cerr << "[Acquisition] Maximum consecutive errors reached, attempting device reset" << std::endl;

                // Attempt device reset
                bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(0), false);
                bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(1), false);
                std::this_thread::sleep_for(std::chrono::milliseconds(USBConfig::RESET_SETTLE_TIME_MS));

                int reset_status = bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(0), true);
                reset_status |= bladerf_enable_module(ctx->device, BLADERF_CHANNEL_RX(1), true);

                if (reset_status == 0) {
                    std::cout << "[Acquisition] Device reset successful, resuming acquisition" << std::endl;
                    g_telemetry.usb_recoveries.fetch_add(1);
                    consecutive_errors = 0;
                    error_backoff_ms = USBConfig::INITIAL_BACKOFF_MS;
                } else {
                    std::cerr << "[Acquisition] Device reset failed, stopping acquisition" << std::endl;
                    ctx->running->store(false, std::memory_order_release);
                    break;
                }
            }
            continue;
        }

        // Success - reset error tracking
        consecutive_errors = 0;
        error_backoff_ms = USBConfig::INITIAL_BACKOFF_MS;

        // Update watchdog heartbeat
        g_rx_heartbeat.fetch_add(1);

        // Push to processing queue
        if (!ctx->sample_queue->push(sample_buf)) {
            // Queue full - processing is falling behind
            ctx->stats.sample_queue_full.fetch_add(1);
            std::cerr << "[Acquisition] Sample queue full, dropping frame" << std::endl;
            g_telemetry.frames_dropped.fetch_add(1);
        } else {
            ctx->stats.samples_acquired.fetch_add(1);
        }
    }

    std::cout << "[Pipeline] Acquisition thread stopped" << std::endl;
}

// ============================================================================
// Stage 2: Signal Processing Thread
// Pops samples, performs FFT and magnitude computation, pushes to FFT queue
// ============================================================================

void processing_thread_func(PipelineContext* ctx) {
    std::cout << "[Pipeline] Processing thread started" << std::endl;

    // Allocate FFT output buffer (reused across iterations)
    FFTBuffer fft_buf;
    SampleBuffer sample_buf;

    // FPS tracking for link quality monitoring
    uint32_t frame_count = 0;
    auto fps_update_time = std::chrono::steady_clock::now();

    while (ctx->running->load(std::memory_order_acquire)) {
        // Pop samples from acquisition queue
        if (!ctx->sample_queue->pop(sample_buf)) {
            // Queue empty - acquisition is falling behind or we're faster
            ctx->stats.sample_queue_empty.fetch_add(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Time the FFT processing
        auto fft_start = std::chrono::high_resolution_clock::now();

        // Allocate magnitude output buffers
        std::vector<uint8_t> ch1_mag(ctx->fft_size);
        std::vector<uint8_t> ch2_mag(ctx->fft_size);

        // Get current center frequency from context
        uint64_t current_freq = ctx->center_freq->load(std::memory_order_relaxed);

        // Perform FFT and magnitude computation
        (void)process_iq_to_fft(
            sample_buf.samples.data(),
            sample_buf.count,
            ctx->fft_size,
            current_freq,
            ctx->fft_in_ch1,
            ctx->fft_in_ch2,
            ctx->fft_out_ch1,
            ctx->fft_out_ch2,
            ch1_mag.data(),
            ch2_mag.data(),
            ctx->dc_offset,
            ctx->overlap,
            ctx->window,
            ctx->fft_plan_ch1,
            ctx->fft_plan_ch2
        );

        auto fft_end = std::chrono::high_resolution_clock::now();
        auto fft_time_us = std::chrono::duration_cast<std::chrono::microseconds>(fft_end - fft_start);
        g_telemetry.total_fft_time_us.fetch_add(fft_time_us.count());

        // Update noise floor estimation (15th percentile, 0.1 smoothing factor)
        update_noise_floor(ctx->noise_floor, ch1_mag.data(), ch2_mag.data(), ctx->fft_size, 15.0f, 0.1f);

        // Also update global noise floor for web server reporting
        update_noise_floor(*ctx->global_noise_floor, ch1_mag.data(), ch2_mag.data(), ctx->fft_size, 15.0f, 0.1f);

        // Remove DC offset spike/dip at center frequency
        remove_dc_offset(ch1_mag.data(), ctx->fft_size);
        remove_dc_offset(ch2_mag.data(), ctx->fft_size);

        // Update waterfall display
        update_waterfall(ch1_mag.data(), ch2_mag.data(), ctx->fft_size);

        // Decimate IQ samples for constellation display
        static_assert(4096 >= 256 && 4096 % 256 == 0, "FFT_SIZE must be >= 256 and divisible by 256");
        constexpr int decimation_step = 4096 / 256;
        int16_t ch1_iq[256][2], ch2_iq[256][2];
        for (int i = 0; i < 256; i++) {
            const size_t idx = i * decimation_step;
            ch1_iq[i][0] = static_cast<int16_t>(ctx->fft_in_ch1[idx][0] * 32767.0f);
            ch1_iq[i][1] = static_cast<int16_t>(ctx->fft_in_ch1[idx][1] * 32767.0f);
            ch2_iq[i][0] = static_cast<int16_t>(ctx->fft_in_ch2[idx][0] * 32767.0f);
            ch2_iq[i][1] = static_cast<int16_t>(ctx->fft_in_ch2[idx][1] * 32767.0f);
        }
        // Update IQ data with FFT output for frequency-domain filtering
        update_iq_data(reinterpret_cast<int16_t*>(ch1_iq), reinterpret_cast<int16_t*>(ch2_iq), 256,
                      ctx->fft_out_ch1, ctx->fft_out_ch2, ctx->fft_size);

        // Compute and update cross-correlation data
        std::vector<float> xcorr_mag(ctx->fft_size);
        std::vector<float> xcorr_phase(ctx->fft_size);
        compute_cross_correlation(ctx->fft_out_ch1, ctx->fft_out_ch2,
                                 xcorr_mag.data(), xcorr_phase.data(), ctx->fft_size);
        update_xcorr_data(xcorr_mag.data(), xcorr_phase.data(), ctx->fft_size);

        // Copy results to FFT buffer
        fft_buf.ch1_mag = ch1_mag;
        fft_buf.ch2_mag = ch2_mag;

        // Convert fftwf_complex to ComplexSample
        fft_buf.ch1_fft.resize(ctx->fft_size);
        fft_buf.ch2_fft.resize(ctx->fft_size);
        for (size_t i = 0; i < ctx->fft_size; i++) {
            fft_buf.ch1_fft[i].from_fftw(ctx->fft_out_ch1[i]);
            fft_buf.ch2_fft[i].from_fftw(ctx->fft_out_ch2[i]);
        }

        fft_buf.size = ctx->fft_size;
        fft_buf.timestamp_us = sample_buf.timestamp_us;

        // Get current noise floor estimates for analysis stage
        get_noise_floor(ctx->noise_floor, fft_buf.noise_floor_ch1, fft_buf.noise_floor_ch2);

        // Push to analysis queue
        if (!ctx->fft_queue->push(fft_buf)) {
            // Queue full - analysis is falling behind
            ctx->stats.fft_queue_full.fetch_add(1);
            std::cerr << "[Processing] FFT queue full, dropping frame" << std::endl;
            g_telemetry.frames_dropped.fetch_add(1);
        } else {
            ctx->stats.samples_processed.fetch_add(1);
        }

        // Update FPS tracking and link quality every second
        frame_count++;
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
    }

    std::cout << "[Pipeline] Processing thread stopped" << std::endl;
}

// ============================================================================
// Stage 3: Analysis Thread
// Pops FFT results, performs CFAR/DF/cross-correlation, updates displays
// ============================================================================

void analysis_thread_func(PipelineContext* ctx) {
    std::cout << "[Pipeline] Analysis thread started" << std::endl;

    FFTBuffer fft_buf;

    // Allocate temporary buffers for converting ComplexSample back to fftwf_complex
    // Use raw pointers since fftwf_complex is float[2] and can't be in vectors
    fftwf_complex* fft_ch1_tmp = (fftwf_complex*)malloc(sizeof(fftwf_complex) * ctx->fft_size);
    fftwf_complex* fft_ch2_tmp = (fftwf_complex*)malloc(sizeof(fftwf_complex) * ctx->fft_size);

    // Use global DoA state for proper bearing hold and Kalman filtering across frames

    while (ctx->running->load(std::memory_order_acquire)) {
        // Pop FFT results from processing queue
        if (!ctx->fft_queue->pop(fft_buf)) {
            // Queue empty - processing is falling behind or we're faster
            ctx->stats.fft_queue_empty.fetch_add(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Convert ComplexSample back to fftwf_complex for DF processing
        for (size_t i = 0; i < fft_buf.size; i++) {
            fft_buf.ch1_fft[i].to_fftw(fft_ch1_tmp[i]);
            fft_buf.ch2_fft[i].to_fftw(fft_ch2_tmp[i]);
        }

        // Determine bin range for DF processing
        // If both start and end are 0, use entire spectrum
        const uint32_t df_start_cfg = ctx->df_start_bin->load(std::memory_order_relaxed);
        const uint32_t df_end_cfg = ctx->df_end_bin->load(std::memory_order_relaxed);
        const size_t bin_start = (df_start_cfg == 0 && df_end_cfg == 0) ? 0 :
                                 std::min(static_cast<size_t>(df_start_cfg), fft_buf.size - 1);
        const size_t bin_end = (df_start_cfg == 0 && df_end_cfg == 0) ? fft_buf.size - 1 :
                               std::min(static_cast<size_t>(df_end_cfg), fft_buf.size - 1);

        // Get current center frequency
        const uint64_t center_freq = ctx->center_freq->load(std::memory_order_relaxed);

        // Time the direction finding (includes CFAR internally)
        auto df_start = std::chrono::high_resolution_clock::now();

        // Perform direction finding (CFAR is done inside this function)
        DFResult df_result = compute_direction_finding(
            fft_ch1_tmp,
            fft_ch2_tmp,
            fft_buf.ch1_mag.data(),
            fft_buf.ch2_mag.data(),
            fft_buf.size,
            bin_start,
            bin_end,
            center_freq,
            g_last_valid_doa,
            fft_buf.noise_floor_ch1,
            fft_buf.noise_floor_ch2
        );

        auto df_end = std::chrono::high_resolution_clock::now();
        auto df_time_us = std::chrono::duration_cast<std::chrono::microseconds>(df_end - df_start);
        g_telemetry.total_df_time_us.fetch_add(df_time_us.count());
        g_telemetry.df_computations.fetch_add(1);
        g_telemetry.signals_detected.fetch_add(df_result.num_signals);

        // Update DoA result for web interface
        update_doa_result(df_result.azimuth, df_result.back_azimuth,
                         df_result.phase_diff_deg, df_result.phase_std_deg,
                         df_result.confidence, df_result.snr_db, df_result.coherence);

        ctx->stats.samples_analyzed.fetch_add(1);
        g_telemetry.frames_processed.fetch_add(1);
    }

    // Cleanup
    free(fft_ch1_tmp);
    free(fft_ch2_tmp);

    std::cout << "[Pipeline] Analysis thread stopped" << std::endl;
}
