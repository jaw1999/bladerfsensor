#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <complex>
#include <fftw3.h>

constexpr int WEB_SERVER_PORT = 8080;          // HTTP server port for web interface
constexpr int WATERFALL_HEIGHT = 512;          // Number of FFT frames stored in history
constexpr int WATERFALL_WIDTH = 4096;          // Maximum FFT size supported
constexpr int IQ_SAMPLES = 256;                // Number of IQ samples for constellation display

// Waterfall display buffer for storing spectrum history
// Maintains a circular buffer of FFT magnitude data for both channels
// used to generate waterfall visualizations in the web interface
struct WaterfallBuffer {
    std::vector<std::vector<uint8_t>> ch1_history;  // Channel 1 FFT history (circular buffer)
    std::vector<std::vector<uint8_t>> ch2_history;  // Channel 2 FFT history (circular buffer)
    int write_index;                                // Current write position in circular buffer
    std::mutex mutex;                               // Mutex for thread-safe access

    WaterfallBuffer() : write_index(0) {
        ch1_history.resize(WATERFALL_HEIGHT);
        ch2_history.resize(WATERFALL_HEIGHT);
        for (auto& row : ch1_history) {
            row.resize(WATERFALL_WIDTH, 0);
        }
        for (auto& row : ch2_history) {
            row.resize(WATERFALL_WIDTH, 0);
        }
    }
};

// IQ constellation data buffer for both channels
// Stores decimated IQ samples for constellation display and full FFT data for filtering
struct IQBuffer {
    int16_t ch1_i[IQ_SAMPLES];     // Channel 1 I samples (decimated)
    int16_t ch1_q[IQ_SAMPLES];     // Channel 1 Q samples (decimated)
    int16_t ch2_i[IQ_SAMPLES];     // Channel 2 I samples (decimated)
    int16_t ch2_q[IQ_SAMPLES];     // Channel 2 Q samples (decimated)

    // FFT data for frequency-domain filtering
    std::vector<std::complex<float>> ch1_fft;  // Channel 1 FFT output (4096 bins)
    std::vector<std::complex<float>> ch2_fft;  // Channel 2 FFT output (4096 bins)

    std::mutex mutex;               // Mutex for thread-safe access

    IQBuffer() : ch1_fft(WATERFALL_WIDTH), ch2_fft(WATERFALL_WIDTH) {
        memset(ch1_i, 0, sizeof(ch1_i));
        memset(ch1_q, 0, sizeof(ch1_q));
        memset(ch2_i, 0, sizeof(ch2_i));
        memset(ch2_q, 0, sizeof(ch2_q));
    }
};

// Cross-correlation data buffer
// Stores cross-correlation magnitude and phase for direction finding
struct XCorrBuffer {
    std::vector<float> magnitude;   // Cross-correlation magnitude
    std::vector<float> phase;       // Cross-correlation phase (radians)
    std::mutex mutex;               // Mutex for thread-safe access
    std::atomic<uint32_t> update_counter{0};  // Update counter for rate limiting

    XCorrBuffer() : magnitude(WATERFALL_WIDTH, 0.0f), phase(WATERFALL_WIDTH, 0.0f) {}
};

// Link quality metrics for adaptive streaming
struct LinkQuality {
    std::atomic<float> rtt_ms{0.0f};           // Round-trip time in milliseconds
    std::atomic<float> packet_loss{0.0f};      // Packet loss percentage (0.0-1.0)
    std::atomic<float> fps{0.0f};              // Actual frames per second
    std::atomic<uint64_t> bytes_sent{0};       // Total bytes sent for bandwidth calculation
    std::chrono::steady_clock::time_point last_update;  // Last update time
    std::mutex mutex;

    LinkQuality() : last_update(std::chrono::steady_clock::now()) {}
};

// Direction of Arrival (DoA) result buffer
// Stores calculated bearing from 2-channel phase interferometry
struct DoAResult {
    float azimuth;              // Primary azimuth angle (0-360 degrees)
    float back_azimuth;         // Back azimuth (180° ambiguity)
    float phase_diff_deg;       // Phase difference in degrees
    float phase_std_deg;        // Phase standard deviation (quality metric)
    float confidence;           // Confidence percentage (0-100)
    float snr_db;               // Signal-to-noise ratio estimate (dB)
    float coherence;            // Coherence metric (0-1)
    bool has_ambiguity;         // True for 2-channel systems (always true)
    std::mutex mutex;           // Mutex for thread-safe access

    DoAResult() : azimuth(0), back_azimuth(0), phase_diff_deg(0), phase_std_deg(0),
                  confidence(0), snr_db(0), coherence(0), has_ambiguity(true) {}
};

// Classified signal entry
struct ClassifiedSignal {
    uint64_t frequency_hz;      // Signal frequency in Hz
    float bandwidth_hz;         // Occupied bandwidth in Hz
    char modulation[32];        // Detected modulation type (e.g., "FM", "AM", "BPSK")
    uint8_t confidence;         // Confidence percentage (0-100)
    float power_db;             // Signal power in dB
    uint64_t timestamp_ms;      // Timestamp of classification (milliseconds since epoch)
};

// GPS position data
// Stores current position from GPS or manual entry
// Note: For DF work, we only need position, not heading (use compass for that)
struct GPSPosition {
    enum class Mode { MANUAL, GPS_AUTO };

    Mode mode;                  // Position source mode
    bool valid;                 // Position data is valid
    double latitude;            // Latitude in decimal degrees
    double longitude;           // Longitude in decimal degrees
    double altitude_m;          // Altitude in meters (MSL)
    uint64_t timestamp_ms;      // Last update timestamp
    uint8_t satellites;         // Number of satellites (GPS mode only)
    float hdop;                 // Horizontal dilution of precision (GPS mode only)
    std::mutex mutex;           // Mutex for thread-safe access

    GPSPosition() : mode(Mode::MANUAL), valid(false), latitude(0), longitude(0),
                    altitude_m(0), timestamp_ms(0), satellites(0), hdop(99.9f) {}
};

// Signal classification buffer
// Stores recent signal classifications for display
constexpr int MAX_CLASSIFICATIONS = 50;  // Maximum number of classifications to store

struct ClassificationBuffer {
    ClassifiedSignal classifications[MAX_CLASSIFICATIONS];  // Circular buffer of classifications
    int write_index;                                        // Current write position
    int count;                                              // Number of valid classifications
    std::mutex mutex;                                       // Mutex for thread-safe access

    ClassificationBuffer() : write_index(0), count(0) {
        memset(classifications, 0, sizeof(classifications));
    }
};

// Global buffer instances
extern WaterfallBuffer g_waterfall;
extern IQBuffer g_iq_data;
extern XCorrBuffer g_xcorr_data;
extern LinkQuality g_link_quality;
extern DoAResult g_doa_result;
extern ClassificationBuffer g_classifications;
extern GPSPosition g_gps_position;

// Web server function declarations

// Start the HTTP web server thread for spectrum visualization
void start_web_server();

// Stop the HTTP web server and clean up resources
void stop_web_server();

// Update waterfall buffer with new FFT magnitude data
// Thread-safe function to append new spectrum data to the circular buffer
void update_waterfall(const uint8_t* ch1_mag, const uint8_t* ch2_mag, size_t fft_size);

// Update IQ constellation data for both channels
// Args:
//   ch1_iq: Channel 1 IQ samples as interleaved I Q pairs
//   ch2_iq: Channel 2 IQ samples as interleaved I Q pairs
//   count: Number of IQ pairs (should be IQ_SAMPLES)
void update_iq_data(const int16_t* ch1_iq, const int16_t* ch2_iq, size_t count,
                    const fftwf_complex* ch1_fft = nullptr, const fftwf_complex* ch2_fft = nullptr, size_t fft_size = 0);

// Update cross-correlation data
// Args:
//   magnitude: Cross-correlation magnitude array
//   phase: Cross-correlation phase array (radians)
//   size: Array size (should match FFT size)
void update_xcorr_data(const float* magnitude, const float* phase, size_t size);

// Update link quality metrics
// Args:
//   fps: Current frames per second
//   bytes: Bytes sent in last measurement period
void update_link_quality(float fps, uint64_t bytes);

// Update Direction of Arrival result
// Args:
//   azimuth: Primary azimuth angle (0-360 degrees)
//   back_azimuth: Back azimuth (180° ambiguity)
//   phase_diff: Phase difference in degrees
//   phase_std: Phase standard deviation in degrees
//   confidence: Confidence percentage (0-100)
//   snr: Signal-to-noise ratio in dB
//   coherence: Coherence metric (0-1)
void update_doa_result(float azimuth, float back_azimuth, float phase_diff,
                       float phase_std, float confidence, float snr, float coherence);

// Add a signal classification result
// Args:
//   frequency_hz: Signal frequency in Hz
//   bandwidth_hz: Occupied bandwidth in Hz
//   modulation: Modulation type string
//   confidence: Confidence percentage (0-100)
//   power_db: Signal power in dB
//   timestamp_ms: Timestamp in milliseconds
void add_classification(uint64_t frequency_hz, float bandwidth_hz, const char* modulation,
                       uint8_t confidence, float power_db, uint64_t timestamp_ms);

// Get and reset HTTP bytes sent counter
// Returns the number of bytes sent since last call and resets counter to zero
uint64_t get_and_reset_http_bytes();

// Update GPS position from manual entry
// Args:
//   latitude: Latitude in decimal degrees
//   longitude: Longitude in decimal degrees
//   altitude_m: Altitude in meters MSL
void set_manual_position(double latitude, double longitude, double altitude_m);

// Enable/disable GPS auto mode (gpsd)
// Args:
//   enable: true to start GPS auto mode, false to stop
void set_gps_mode(bool enable);

#endif // WEB_SERVER_H
