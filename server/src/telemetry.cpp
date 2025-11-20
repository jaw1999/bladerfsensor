#include "telemetry.h"
#include <sstream>
#include <iomanip>
#include <chrono>

// Global telemetry instance
TelemetryCounters g_telemetry;

void init_telemetry() {
    // Reset all counters to zero
    g_telemetry.frames_processed.store(0);
    g_telemetry.frames_dropped.store(0);
    g_telemetry.total_fft_time_us.store(0);
    g_telemetry.total_cfar_time_us.store(0);
    g_telemetry.total_df_time_us.store(0);
    g_telemetry.total_processing_time_us.store(0);
    g_telemetry.usb_transfer_count.store(0);
    g_telemetry.usb_errors.store(0);
    g_telemetry.usb_recoveries.store(0);
    g_telemetry.signals_detected.store(0);
    g_telemetry.df_computations.store(0);
    g_telemetry.buffer_allocations.store(0);
    g_telemetry.buffer_reallocations.store(0);
    g_telemetry.http_requests.store(0);
    g_telemetry.http_bytes_sent.store(0);
    g_telemetry.compression_raw_bytes.store(0);
    g_telemetry.compression_compressed_bytes.store(0);
    g_telemetry.compression_frames.store(0);

    // Set initial timestamp
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    g_telemetry.last_update_ms.store(ms.count());
}

std::string get_telemetry_json() {
    // Capture snapshot of all counters
    uint64_t frames = g_telemetry.frames_processed.load();
    uint64_t dropped = g_telemetry.frames_dropped.load();
    uint64_t fft_time = g_telemetry.total_fft_time_us.load();
    uint64_t cfar_time = g_telemetry.total_cfar_time_us.load();
    uint64_t df_time = g_telemetry.total_df_time_us.load();
    uint64_t proc_time = g_telemetry.total_processing_time_us.load();
    uint64_t usb_xfers = g_telemetry.usb_transfer_count.load();
    uint64_t usb_errs = g_telemetry.usb_errors.load();
    uint64_t usb_recov = g_telemetry.usb_recoveries.load();
    uint64_t signals = g_telemetry.signals_detected.load();
    uint64_t df_count = g_telemetry.df_computations.load();
    uint64_t buf_alloc = g_telemetry.buffer_allocations.load();
    uint64_t buf_realloc = g_telemetry.buffer_reallocations.load();
    uint64_t http_reqs = g_telemetry.http_requests.load();
    uint64_t http_bytes = g_telemetry.http_bytes_sent.load();
    uint64_t comp_raw = g_telemetry.compression_raw_bytes.load();
    uint64_t comp_compressed = g_telemetry.compression_compressed_bytes.load();
    uint64_t comp_frames = g_telemetry.compression_frames.load();

    // Calculate averages (avoid division by zero)
    double avg_fft_us = (frames > 0) ? static_cast<double>(fft_time) / frames : 0.0;
    double avg_cfar_us = (frames > 0) ? static_cast<double>(cfar_time) / frames : 0.0;
    double avg_df_us = (df_count > 0) ? static_cast<double>(df_time) / df_count : 0.0;
    double avg_proc_us = (frames > 0) ? static_cast<double>(proc_time) / frames : 0.0;
    double drop_rate = (frames > 0) ? 100.0 * dropped / frames : 0.0;
    double usb_error_rate = (usb_xfers > 0) ? 100.0 * usb_errs / usb_xfers : 0.0;
    double compression_ratio = (comp_compressed > 0) ? static_cast<double>(comp_raw) / comp_compressed : 1.0;
    double bandwidth_savings_pct = (comp_raw > 0) ? 100.0 * (1.0 - static_cast<double>(comp_compressed) / comp_raw) : 0.0;

    // Update timestamp
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    g_telemetry.last_update_ms.store(ms.count());

    // Build JSON
    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    json << "{\n";
    json << "  \"frames\": {\n";
    json << "    \"processed\": " << frames << ",\n";
    json << "    \"dropped\": " << dropped << ",\n";
    json << "    \"drop_rate_pct\": " << drop_rate << "\n";
    json << "  },\n";
    json << "  \"timing_us\": {\n";
    json << "    \"avg_fft\": " << avg_fft_us << ",\n";
    json << "    \"avg_cfar\": " << avg_cfar_us << ",\n";
    json << "    \"avg_df\": " << avg_df_us << ",\n";
    json << "    \"avg_total\": " << avg_proc_us << ",\n";
    json << "    \"total_fft\": " << fft_time << ",\n";
    json << "    \"total_cfar\": " << cfar_time << ",\n";
    json << "    \"total_df\": " << df_time << ",\n";
    json << "    \"total_processing\": " << proc_time << "\n";
    json << "  },\n";
    json << "  \"usb\": {\n";
    json << "    \"transfers\": " << usb_xfers << ",\n";
    json << "    \"errors\": " << usb_errs << ",\n";
    json << "    \"recoveries\": " << usb_recov << ",\n";
    json << "    \"error_rate_pct\": " << usb_error_rate << "\n";
    json << "  },\n";
    json << "  \"signal_processing\": {\n";
    json << "    \"signals_detected\": " << signals << ",\n";
    json << "    \"df_computations\": " << df_count << "\n";
    json << "  },\n";
    json << "  \"memory\": {\n";
    json << "    \"buffer_allocations\": " << buf_alloc << ",\n";
    json << "    \"buffer_reallocations\": " << buf_realloc << "\n";
    json << "  },\n";
    json << "  \"http\": {\n";
    json << "    \"requests\": " << http_reqs << ",\n";
    json << "    \"bytes_sent\": " << http_bytes << "\n";
    json << "  },\n";
    json << "  \"compression\": {\n";
    json << "    \"raw_bytes\": " << comp_raw << ",\n";
    json << "    \"compressed_bytes\": " << comp_compressed << ",\n";
    json << "    \"frames\": " << comp_frames << ",\n";
    json << "    \"compression_ratio\": " << compression_ratio << ",\n";
    json << "    \"bandwidth_savings_pct\": " << bandwidth_savings_pct << "\n";
    json << "  },\n";
    json << "  \"timestamp_ms\": " << g_telemetry.last_update_ms.load() << "\n";
    json << "}";

    return json.str();
}

void reset_telemetry() {
    init_telemetry();
}
