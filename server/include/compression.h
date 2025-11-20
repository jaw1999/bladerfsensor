#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <zlib.h>

// Delta encoding state for a single data stream
struct DeltaState {
    std::vector<uint8_t> last_frame;
    bool initialized;

    DeltaState() : initialized(false) {}

    void init(size_t size) {
        last_frame.resize(size, 0);
        initialized = false;
    }
};

// Compression statistics
struct CompressionStats {
    size_t raw_bytes;
    size_t compressed_bytes;
    size_t delta_bytes;
    float compression_ratio;
    float bandwidth_savings_percent;
};

// Apply delta encoding: output[i] = current[i] - last[i]
// Returns true if this is a delta frame, false if full frame
bool delta_encode(const uint8_t* current, size_t size, DeltaState& state,
                  std::vector<int8_t>& delta_out);

// Compress data using gzip (best speed)
// Returns compressed size, or 0 on error
size_t gzip_compress(const void* input, size_t input_size,
                     std::vector<uint8_t>& output);

// Compress with delta encoding + gzip
// Returns compressed size, or 0 on error
// Sets is_delta_frame to indicate if this is delta (true) or full (false)
size_t compress_with_delta(const uint8_t* data, size_t size, DeltaState& state,
                           std::vector<uint8_t>& compressed_out, bool& is_delta_frame);

// Calculate compression statistics
CompressionStats calculate_compression_stats(size_t raw_bytes, size_t compressed_bytes);

#endif // COMPRESSION_H
