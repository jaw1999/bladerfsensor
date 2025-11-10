#ifndef PIPELINE_H
#define PIPELINE_H

#include "lockfree_queue.h"
#include "bladerf_sensor.h"
#include "signal_processing.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <libbladeRF.h>

// Pipeline configuration
namespace PipelineConfig {
    constexpr size_t SAMPLE_QUEUE_SIZE = 8;     // Samples between acquisition and processing
    constexpr size_t FFT_QUEUE_SIZE = 8;        // FFT results between processing and analysis
}

// Pipeline state and statistics
struct PipelineStats {
    std::atomic<uint64_t> samples_acquired{0};
    std::atomic<uint64_t> samples_processed{0};
    std::atomic<uint64_t> samples_analyzed{0};
    std::atomic<uint64_t> sample_queue_full{0};
    std::atomic<uint64_t> fft_queue_full{0};
    std::atomic<uint64_t> sample_queue_empty{0};
    std::atomic<uint64_t> fft_queue_empty{0};
};

// Shared pipeline context
struct PipelineContext {
    // Hardware
    struct bladerf* device;

    // Queues
    LockFreeQueue<SampleBuffer>* sample_queue;
    LockFreeQueue<FFTBuffer>* fft_queue;

    // Control flags
    std::atomic<bool>* running;

    // Configuration (read-only references to global atomics)
    std::atomic<uint64_t>* center_freq;
    std::atomic<uint32_t>* sample_rate;
    std::atomic<uint32_t>* bandwidth;
    std::atomic<uint32_t>* gain_rx1;
    std::atomic<uint32_t>* gain_rx2;
    std::atomic<bool>* params_changed;
    std::mutex* config_mutex;
    std::atomic<uint32_t>* df_start_bin;
    std::atomic<uint32_t>* df_end_bin;

    // Statistics
    PipelineStats stats;

    // Processing state (owned by processing thread)
    DCOffsetState dc_offset;
    OverlapState overlap;
    NoiseFloorState noise_floor;            // Noise floor estimation (local)
    NoiseFloorState* global_noise_floor;    // Global noise floor (for web server reporting)
    std::vector<float> window;
    fftwf_plan fft_plan_ch1;
    fftwf_plan fft_plan_ch2;
    fftwf_complex* fft_in_ch1;      // Raw pointers since fftwf_complex is float[2]
    fftwf_complex* fft_in_ch2;
    fftwf_complex* fft_out_ch1;
    fftwf_complex* fft_out_ch2;
    size_t fft_size;                // Size of FFT buffers
};

// Stage 1: Sample Acquisition Thread
// Continuously acquires samples from bladeRF and pushes to sample queue
void acquisition_thread_func(PipelineContext* ctx);

// Stage 2: Signal Processing Thread
// Pops samples, performs FFT/magnitude computation, pushes to FFT queue
void processing_thread_func(PipelineContext* ctx);

// Stage 3: Analysis Thread
// Pops FFT results, performs CFAR/DF/cross-correlation, updates displays
void analysis_thread_func(PipelineContext* ctx);

#endif // PIPELINE_H
