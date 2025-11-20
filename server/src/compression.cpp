#include "compression.h"
#include <algorithm>
#include <cstring>

bool delta_encode(const uint8_t* current, size_t size, DeltaState& state,
                  std::vector<int8_t>& delta_out) {
    delta_out.resize(size);

    if (!state.initialized) {
        // First frame - send full data (delta from zero)
        for (size_t i = 0; i < size; i++) {
            delta_out[i] = static_cast<int8_t>(current[i]);
        }
        state.last_frame.assign(current, current + size);
        state.initialized = true;
        return false;  // Not a delta frame (full frame)
    }

    // Subsequent frames - compute delta
    for (size_t i = 0; i < size; i++) {
        // Delta = current - last (as signed 8-bit)
        int16_t diff = static_cast<int16_t>(current[i]) - static_cast<int16_t>(state.last_frame[i]);
        delta_out[i] = static_cast<int8_t>(std::clamp(diff, static_cast<int16_t>(-128), static_cast<int16_t>(127)));
    }

    // Update state
    state.last_frame.assign(current, current + size);
    return true;  // This is a delta frame
}

size_t gzip_compress(const void* input, size_t input_size,
                     std::vector<uint8_t>& output) {
    // Calculate maximum compressed size
    uLongf max_compressed_size = compressBound(input_size);
    output.resize(max_compressed_size);

    uLongf compressed_size = max_compressed_size;

    // Compress with Z_BEST_SPEED for low latency
    int result = compress2(output.data(), &compressed_size,
                          static_cast<const Bytef*>(input), input_size,
                          Z_BEST_SPEED);

    if (result != Z_OK) {
        return 0;  // Compression failed
    }

    output.resize(compressed_size);
    return compressed_size;
}

size_t compress_with_delta(const uint8_t* data, size_t size, DeltaState& state,
                           std::vector<uint8_t>& compressed_out, bool& is_delta_frame) {
    // Step 1: Apply delta encoding
    std::vector<int8_t> delta;
    is_delta_frame = delta_encode(data, size, state, delta);

    // Step 2: Compress the delta data with gzip
    size_t compressed_size = gzip_compress(delta.data(), delta.size(), compressed_out);

    return compressed_size;
}

CompressionStats calculate_compression_stats(size_t raw_bytes, size_t compressed_bytes) {
    CompressionStats stats;
    stats.raw_bytes = raw_bytes;
    stats.compressed_bytes = compressed_bytes;
    stats.delta_bytes = 0;  // Not tracked separately

    if (raw_bytes > 0) {
        stats.compression_ratio = static_cast<float>(raw_bytes) / static_cast<float>(compressed_bytes);
        stats.bandwidth_savings_percent = 100.0f * (1.0f - static_cast<float>(compressed_bytes) / static_cast<float>(raw_bytes));
    } else {
        stats.compression_ratio = 1.0f;
        stats.bandwidth_savings_percent = 0.0f;
    }

    return stats;
}
