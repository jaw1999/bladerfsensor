#include "recording.h"
#include "bladerf_sensor.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <mutex>

// Recording state
static RecordingState g_recording = {false, nullptr, 0, 0, 0, {}};
static std::mutex g_recording_mutex;

bool start_recording(const std::string& filename, uint64_t center_freq,
                    uint32_t sample_rate, uint32_t bandwidth,
                    uint32_t gain_rx1, uint32_t gain_rx2) {
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
    g_recording.metadata.center_freq = center_freq;
    g_recording.metadata.sample_rate = sample_rate;
    g_recording.metadata.bandwidth = bandwidth;
    g_recording.metadata.gain_rx1 = gain_rx1;
    g_recording.metadata.gain_rx2 = gain_rx2;
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
}

bool is_recording() {
    std::lock_guard<std::mutex> lock(g_recording_mutex);
    return g_recording.active;
}

bool get_recording_status(uint64_t& samples_written) {
    std::lock_guard<std::mutex> lock(g_recording_mutex);
    samples_written = g_recording.samples_written;
    return g_recording.active;
}
