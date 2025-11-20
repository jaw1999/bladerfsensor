#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <atomic>
#include <chrono>
#include <mutex>

// Telemetry counters for performance monitoring and diagnostics
// All counters are atomic for thread-safe access from multiple threads
struct TelemetryCounters {
    // Frame processing metrics
    std::atomic<uint64_t> frames_processed{0};          // Total frames processed since startup
    std::atomic<uint64_t> frames_dropped{0};            // Frames dropped due to overload

    // Timing metrics (microseconds)
    std::atomic<uint64_t> total_fft_time_us{0};         // Cumulative FFT computation time
    std::atomic<uint64_t> total_cfar_time_us{0};        // Cumulative CFAR detection time
    std::atomic<uint64_t> total_df_time_us{0};          // Cumulative direction finding time
    std::atomic<uint64_t> total_processing_time_us{0};  // Cumulative total processing time

    // USB transfer metrics
    std::atomic<uint64_t> usb_transfer_count{0};        // Total USB transfers completed
    std::atomic<uint64_t> usb_errors{0};                // Total USB errors encountered
    std::atomic<uint64_t> usb_recoveries{0};            // Successful USB error recoveries

    // Signal detection metrics
    std::atomic<uint64_t> signals_detected{0};          // Total signals detected by CFAR
    std::atomic<uint64_t> df_computations{0};           // Total DF computations performed

    // Memory metrics
    std::atomic<uint64_t> buffer_allocations{0};        // Buffer allocation count
    std::atomic<uint64_t> buffer_reallocations{0};      // Buffer reallocation count (should be minimal)

    // HTTP metrics
    std::atomic<uint64_t> http_requests{0};             // Total HTTP requests served
    std::atomic<uint64_t> http_bytes_sent{0};           // Total bytes sent via HTTP

    // Compression metrics
    std::atomic<uint64_t> compression_raw_bytes{0};     // Total uncompressed bytes
    std::atomic<uint64_t> compression_compressed_bytes{0}; // Total compressed bytes sent
    std::atomic<uint64_t> compression_frames{0};        // Total frames compressed

    // Last update timestamp
    std::atomic<uint64_t> last_update_ms{0};            // Last telemetry update time
};

// Global telemetry instance
extern TelemetryCounters g_telemetry;

// Helper class for measuring execution time with RAII
// Automatically records elapsed time to specified counter on destruction
class ScopedTimer {
public:
    ScopedTimer(std::atomic<uint64_t>& counter)
        : counter_(counter), start_(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        counter_.fetch_add(elapsed.count());
    }

private:
    std::atomic<uint64_t>& counter_;
    std::chrono::high_resolution_clock::time_point start_;
};

// Initialize telemetry system
void init_telemetry();

// Get telemetry snapshot as JSON string
std::string get_telemetry_json();

// Reset all telemetry counters
void reset_telemetry();

#endif // TELEMETRY_H
