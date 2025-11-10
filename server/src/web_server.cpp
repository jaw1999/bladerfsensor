// Lightweight HTTP web server for waterfall spectrum visualization
// Implements a real-time waterfall display using the Mongoose embedded web server
// Provides interactive controls for frequency gain and display parameters

#include "web_server.h"
#include "bladerf_sensor.h"
#include "signal_processing.h"
#include "recording.h"
#include "telemetry.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <string>
#include <algorithm>
#include <atomic>
#include <thread>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <complex>
#include <fftw3.h>

// PNG image writer (single-header library)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Mongoose embedded web server library
// Download from https://github.com/cesanta/mongoose
// Place mongoose.h and mongoose.c in server/src/
#ifdef USE_MONGOOSE
extern "C" {
#include "mongoose.h"
}
#else
// Stubs when mongoose is not available
struct mg_mgr { int dummy; };
struct mg_connection { int dummy; };
struct mg_http_message {
    struct mg_str { const char* ptr; size_t len; } uri, query, body;
};
void mg_mgr_init(struct mg_mgr *mgr) { (void)mgr; }
void mg_mgr_free(struct mg_mgr *mgr) { (void)mgr; }
struct mg_connection *mg_http_listen(struct mg_mgr *mgr, const char *url,
                                     void (*fn)(struct mg_connection *, int, void *), void *fn_data) {
    (void)mgr; (void)url; (void)fn; (void)fn_data; return nullptr;
}
void mg_mgr_poll(struct mg_mgr *mgr, int ms) { (void)mgr; (void)ms; }
void mg_http_reply(struct mg_connection *c, int code, const char *headers, const char *fmt, ...) {
    (void)c; (void)code; (void)headers; (void)fmt;
}
#define MG_EV_HTTP_MSG 6
#endif

// Global state for web server operation
WaterfallBuffer g_waterfall;                         // Waterfall spectrum history buffer
IQBuffer g_iq_data;                                  // IQ constellation data buffer
XCorrBuffer g_xcorr_data;                            // Cross-correlation data buffer
LinkQuality g_link_quality;                          // Link quality metrics buffer
DoAResult g_doa_result;                              // Direction of Arrival result buffer
ClassificationBuffer g_classifications;              // Signal classification buffer
GPSPosition g_gps_position;                          // GPS position data buffer
static std::atomic<bool> g_web_running{false};       // Web server thread running flag
static std::thread g_web_thread;                     // Web server worker thread
static std::atomic<uint64_t> g_http_bytes_sent{0};   // Actual HTTP bytes sent counter

// GPS thread state
static std::atomic<bool> g_gps_running{false};       // GPS thread running flag
static std::thread g_gps_thread;                     // GPS worker thread

#ifdef USE_MONGOOSE
static struct mg_mgr g_mgr;                          // Mongoose event manager
#endif

// External globals from main.cpp (shared state with RF processing)
extern std::atomic<uint64_t> g_center_freq;
extern std::atomic<uint32_t> g_sample_rate;
extern std::atomic<uint32_t> g_bandwidth;
extern std::atomic<uint32_t> g_gain_rx1;
extern std::atomic<uint32_t> g_gain_rx2;
extern std::atomic<bool> g_params_changed;
extern std::mutex g_config_mutex;
extern NoiseFloorState g_noise_floor;

// Direction finding bin range selection
extern std::atomic<uint32_t> g_df_start_bin;
extern std::atomic<uint32_t> g_df_end_bin;

// Recording functions now in recording.h

// Perceptually uniform color mapping for waterfall display
struct RGB {
    uint8_t r, g, b;
};

// Convert a normalized magnitude value to RGB color using Viridis colormap
// This is a perceptually uniform colormap that maps signal strength to color
// dark purple (low) to blue to cyan to green to yellow (high)
// Args
//   value Normalized magnitude (0.0 to 1.0)
// Returns
//   RGB color structure with 8-bit components
RGB viridis_colormap(float value) {
    // Clamp input to valid range
    value = std::max(0.0f, std::min(1.0f, value));

    RGB color;
    if (value < 0.25f) {
        // Dark purple to blue
        float t = value / 0.25f;
        color.r = static_cast<uint8_t>(68 + t * (59 - 68));
        color.g = static_cast<uint8_t>(1 + t * (82 - 1));
        color.b = static_cast<uint8_t>(84 + t * (139 - 84));
    } else if (value < 0.5f) {
        // Blue to teal/cyan
        float t = (value - 0.25f) / 0.25f;
        color.r = static_cast<uint8_t>(59 + t * (33 - 59));
        color.g = static_cast<uint8_t>(82 + t * (145 - 82));
        color.b = static_cast<uint8_t>(139 + t * (140 - 139));
    } else if (value < 0.75f) {
        // Cyan to green
        float t = (value - 0.5f) / 0.25f;
        color.r = static_cast<uint8_t>(33 + t * (94 - 33));
        color.g = static_cast<uint8_t>(145 + t * (201 - 145));
        color.b = static_cast<uint8_t>(140 + t * (98 - 140));
    } else {
        // Green to bright yellow
        float t = (value - 0.75f) / 0.25f;
        color.r = static_cast<uint8_t>(94 + t * (253 - 94));
        color.g = static_cast<uint8_t>(201 + t * (231 - 201));
        color.b = static_cast<uint8_t>(98 + t * (37 - 98));
    }

    return color;
}

// Update waterfall buffer with new FFT magnitude data
// Thread-safe function that adds new spectrum data to the circular buffer
// Args
//   ch1_mag Channel 1 FFT magnitude data (8-bit quantized)
//   ch2_mag Channel 2 FFT magnitude data (8-bit quantized)
//   fft_size Number of FFT bins in input arrays
void update_waterfall(const uint8_t* ch1_mag, const uint8_t* ch2_mag, size_t fft_size) {
    std::lock_guard<std::mutex> lock(g_waterfall.mutex);

    // Copy FFT magnitude to waterfall buffer (up to maximum width)
    size_t copy_size = std::min(fft_size, static_cast<size_t>(WATERFALL_WIDTH));
    std::copy(ch1_mag, ch1_mag + copy_size, g_waterfall.ch1_history[g_waterfall.write_index].begin());
    std::copy(ch2_mag, ch2_mag + copy_size, g_waterfall.ch2_history[g_waterfall.write_index].begin());

    // Advance write index in circular buffer
    g_waterfall.write_index = (g_waterfall.write_index + 1) % WATERFALL_HEIGHT;
}

// Update IQ constellation data for both channels
// Thread-safe function that stores decimated IQ samples and full FFT data
// Args
//   ch1_iq Channel 1 IQ samples as interleaved I Q pairs
//   ch2_iq Channel 2 IQ samples as interleaved I Q pairs
//   count Number of IQ pairs to copy (should be IQ_SAMPLES)
//   ch1_fft Channel 1 FFT output (optional, for frequency-domain filtering)
//   ch2_fft Channel 2 FFT output (optional, for frequency-domain filtering)
//   fft_size Size of FFT arrays (should be 4096)
void update_iq_data(const int16_t* ch1_iq, const int16_t* ch2_iq, size_t count,
                    const fftwf_complex* ch1_fft, const fftwf_complex* ch2_fft, size_t fft_size) {
    std::lock_guard<std::mutex> lock(g_iq_data.mutex);

    // Copy IQ samples up to buffer size
    size_t copy_count = std::min(count, static_cast<size_t>(IQ_SAMPLES));

    for (size_t i = 0; i < copy_count; i++) {
        g_iq_data.ch1_i[i] = ch1_iq[i * 2];      // Extract I samples
        g_iq_data.ch1_q[i] = ch1_iq[i * 2 + 1];  // Extract Q samples
        g_iq_data.ch2_i[i] = ch2_iq[i * 2];
        g_iq_data.ch2_q[i] = ch2_iq[i * 2 + 1];
    }

    // Store FFT data if provided (for frequency-domain filtering)
    if (ch1_fft && ch2_fft && fft_size > 0) {
        // Ensure vectors are sized correctly
        if (g_iq_data.ch1_fft.size() != fft_size) {
            g_iq_data.ch1_fft.resize(fft_size);
            g_iq_data.ch2_fft.resize(fft_size);
        }

        // Copy FFT data as complex numbers
        for (size_t i = 0; i < fft_size; i++) {
            g_iq_data.ch1_fft[i] = std::complex<float>(ch1_fft[i][0], ch1_fft[i][1]);
            g_iq_data.ch2_fft[i] = std::complex<float>(ch2_fft[i][0], ch2_fft[i][1]);
        }
    }
}

// Update cross-correlation data with rate limiting
// Thread-safe function that stores magnitude and phase arrays for direction finding
// Args
//   magnitude Cross-correlation magnitude array
//   phase Cross-correlation phase array in radians
//   size Array size (should match FFT size)
void update_xcorr_data(const float* magnitude, const float* phase, size_t size) {
    // Rate limit to 2 Hz using atomic counter
    // This prevents excessive bandwidth usage on tactical links
    uint32_t counter = g_xcorr_data.update_counter.fetch_add(1);
    if (counter % 5 != 0) {  // Only update every 5th call (10 Hz / 5 = 2 Hz)
        return;
    }

    std::lock_guard<std::mutex> lock(g_xcorr_data.mutex);

    // Resize vectors if needed and copy data
    size_t copy_size = std::min(size, static_cast<size_t>(WATERFALL_WIDTH));
    if (g_xcorr_data.magnitude.size() < copy_size) {
        g_xcorr_data.magnitude.resize(copy_size);
        g_xcorr_data.phase.resize(copy_size);
    }

    std::copy(magnitude, magnitude + copy_size, g_xcorr_data.magnitude.begin());
    std::copy(phase, phase + copy_size, g_xcorr_data.phase.begin());
}

// Update link quality metrics with current performance data
// Thread-safe function that tracks FPS and bandwidth for adaptive streaming
// Args
//   fps Current frames per second achieved
//   bytes Bytes sent in last measurement period
void update_link_quality(float fps, uint64_t bytes) {
    std::lock_guard<std::mutex> lock(g_link_quality.mutex);

    auto now = std::chrono::steady_clock::now();

    // Update FPS
    g_link_quality.fps.store(fps);

    // Store bytes for this period (not cumulative)
    g_link_quality.bytes_sent.store(bytes);

    // Update timestamp
    g_link_quality.last_update = now;
}

// Update Direction of Arrival result from phase-based interferometry
void update_doa_result(float azimuth, float back_azimuth, float phase_diff,
                       float phase_std, float confidence, float snr, float coherence) {
    std::lock_guard<std::mutex> lock(g_doa_result.mutex);

    g_doa_result.azimuth = azimuth;
    g_doa_result.back_azimuth = back_azimuth;
    g_doa_result.phase_diff_deg = phase_diff;
    g_doa_result.phase_std_deg = phase_std;
    g_doa_result.confidence = confidence;
    g_doa_result.snr_db = snr;
    g_doa_result.coherence = coherence;
    g_doa_result.has_ambiguity = true;  // Always true for 2-channel systems
}

// Add a signal classification result to the circular buffer
void add_classification(uint64_t frequency_hz, float bandwidth_hz, const char* modulation,
                       uint8_t confidence, float power_db, uint64_t timestamp_ms) {
    std::lock_guard<std::mutex> lock(g_classifications.mutex);

    // Add to circular buffer
    ClassifiedSignal& entry = g_classifications.classifications[g_classifications.write_index];
    entry.frequency_hz = frequency_hz;
    entry.bandwidth_hz = bandwidth_hz;
    strncpy(entry.modulation, modulation, sizeof(entry.modulation) - 1);
    entry.modulation[sizeof(entry.modulation) - 1] = '\0';
    entry.confidence = confidence;
    entry.power_db = power_db;
    entry.timestamp_ms = timestamp_ms;

    // Advance write index
    g_classifications.write_index = (g_classifications.write_index + 1) % MAX_CLASSIFICATIONS;

    // Update count
    if (g_classifications.count < MAX_CLASSIFICATIONS) {
        g_classifications.count++;
    }
}

// Get and reset HTTP bytes sent counter
// Returns the number of bytes sent since last call and resets counter to zero
uint64_t get_and_reset_http_bytes() {
    return g_http_bytes_sent.exchange(0);
}

// ============================================================================
// GPS POSITION MANAGEMENT
//
// Functions for managing GPS position data from gpsd or manual entry
// ============================================================================

// Update GPS position from manual entry
void set_manual_position(double latitude, double longitude, double altitude_m) {
    std::lock_guard<std::mutex> lock(g_gps_position.mutex);

    // Stop GPS thread if running
    if (g_gps_running.load()) {
        g_gps_running.store(false);
        if (g_gps_thread.joinable()) {
            g_gps_thread.join();
        }
    }

    g_gps_position.mode = GPSPosition::Mode::MANUAL;
    g_gps_position.valid = true;
    g_gps_position.latitude = latitude;
    g_gps_position.longitude = longitude;
    g_gps_position.altitude_m = altitude_m;
    g_gps_position.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    g_gps_position.satellites = 0;
    g_gps_position.hdop = 0;

    std::cout << "GPS: Manual position set to " << std::fixed << std::setprecision(6)
              << latitude << ", " << longitude << " @ " << altitude_m << "m" << std::endl;
}

// GPS client thread - connects to gpsd and updates position
static void gps_thread_func() {
    std::cout << "GPS: Client thread started, connecting to gpsd..." << std::endl;

    while (g_gps_running.load()) {
        // Try to connect to gpsd on localhost:2947
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "GPS: Failed to create socket" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(2947);
        inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

        if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "GPS: Could not connect to gpsd (is it running?)" << std::endl;
            close(sock);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }

        std::cout << "GPS: Connected to gpsd" << std::endl;

        // Send ?WATCH command to start streaming
        const char* watch_cmd = "?WATCH={\"enable\":true,\"json\":true}\n";
        send(sock, watch_cmd, strlen(watch_cmd), 0);

        char buffer[4096];
        while (g_gps_running.load()) {
            int n = recv(sock, buffer, sizeof(buffer) - 1, 0);
            if (n <= 0) {
                std::cerr << "GPS: Connection lost" << std::endl;
                break;
            }

            buffer[n] = '\0';

            // Simple JSON parsing for TPV (Time-Position-Velocity) messages
            // Look for "class":"TPV" messages
            const char* tpv_start = strstr(buffer, "\"class\":\"TPV\"");
            if (tpv_start) {
                double lat = 0, lon = 0, alt = 0;
                int mode = 0;

                // Extract latitude
                const char* lat_str = strstr(tpv_start, "\"lat\":");
                if (lat_str) sscanf(lat_str + 6, "%lf", &lat);

                // Extract longitude
                const char* lon_str = strstr(tpv_start, "\"lon\":");
                if (lon_str) sscanf(lon_str + 6, "%lf", &lon);

                // Extract altitude
                const char* alt_str = strstr(tpv_start, "\"alt\":");
                if (alt_str) sscanf(alt_str + 6, "%lf", &alt);

                // Extract mode (0=no fix, 2=2D, 3=3D)
                const char* mode_str = strstr(tpv_start, "\"mode\":");
                if (mode_str) sscanf(mode_str + 7, "%d", &mode);

                // Update position if we have at least 2D fix
                if (mode >= 2 && lat != 0 && lon != 0) {
                    std::lock_guard<std::mutex> lock(g_gps_position.mutex);
                    g_gps_position.mode = GPSPosition::Mode::GPS_AUTO;
                    g_gps_position.valid = true;
                    g_gps_position.latitude = lat;
                    g_gps_position.longitude = lon;
                    g_gps_position.altitude_m = alt;
                    g_gps_position.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();

                    static int gps_update_counter = 0;
                    if (++gps_update_counter % 10 == 0) {  // Log every 10 updates
                        std::cout << "GPS: Position " << std::fixed << std::setprecision(6)
                                  << lat << ", " << lon << " @ " << alt << "m" << std::endl;
                    }
                }
            }

            // Parse SKY message for satellite info
            const char* sky_start = strstr(buffer, "\"class\":\"SKY\"");
            if (sky_start) {
                int sats = 0;
                float hdop = 99.9f;

                const char* sats_str = strstr(sky_start, "\"uSat\":");
                if (sats_str) sscanf(sats_str + 7, "%d", &sats);

                const char* hdop_str = strstr(sky_start, "\"hdop\":");
                if (hdop_str) sscanf(hdop_str + 7, "%f", &hdop);

                std::lock_guard<std::mutex> lock(g_gps_position.mutex);
                g_gps_position.satellites = sats;
                g_gps_position.hdop = hdop;
            }
        }

        close(sock);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "GPS: Client thread stopped" << std::endl;
}

// Enable/disable GPS auto mode
void set_gps_mode(bool enable) {
    if (enable) {
        // Start GPS thread if not already running
        if (!g_gps_running.load()) {
            g_gps_running.store(true);
            g_gps_thread = std::thread(gps_thread_func);
            std::cout << "GPS: Auto mode enabled" << std::endl;
        }
    } else {
        // Stop GPS thread
        if (g_gps_running.load()) {
            g_gps_running.store(false);
            if (g_gps_thread.joinable()) {
                g_gps_thread.join();
            }
            std::cout << "GPS: Auto mode disabled" << std::endl;
        }
    }
}

// Generate PNG image from waterfall buffer history
// Converts the circular buffer of FFT magnitudes into a color-mapped PNG image
// Args
//   channel Channel to render (1 or 2)
// Returns
//   Vector containing PNG-encoded image data (empty on error)
std::vector<uint8_t> generate_waterfall_png(int channel) {
    std::lock_guard<std::mutex> lock(g_waterfall.mutex);

    const auto& history = (channel == 1) ? g_waterfall.ch1_history : g_waterfall.ch2_history;

    // Create RGB image data
    std::vector<uint8_t> pixels(WATERFALL_WIDTH * WATERFALL_HEIGHT * 3);

    // Fill pixels (top to bottom newest at bottom)
    for (int y = 0; y < WATERFALL_HEIGHT; y++) {
        // Calculate actual row index (accounting for circular buffer)
        int row_idx = (g_waterfall.write_index + y) % WATERFALL_HEIGHT;

        for (int x = 0; x < WATERFALL_WIDTH; x++) {
            float value = history[row_idx][x] / 255.0f;
            RGB color = viridis_colormap(value);

            int idx = (y * WATERFALL_WIDTH + x) * 3;
            pixels[idx + 0] = color.r;
            pixels[idx + 1] = color.g;
            pixels[idx + 2] = color.b;
        }
    }

    // Write PNG to memory
    int png_size = 0;
    unsigned char* png_data = stbi_write_png_to_mem(
        pixels.data(),
        WATERFALL_WIDTH * 3,  // stride
        WATERFALL_WIDTH,
        WATERFALL_HEIGHT,
        3,  // channels
        &png_size
    );

    if (!png_data || png_size == 0) {
        std::cerr << "PNG generation failed" << std::endl;
        return std::vector<uint8_t>();  // Return empty vector on error
    }

    // Copy to vector and free stbi memory
    std::vector<uint8_t> result(png_data, png_data + png_size);
    STBIW_FREE(png_data);  // Use STBIW_FREE instead of free

    return result;
}

// HTML page - Flowing waterfall display with Canvas
const char* html_page = R"HTMLDELIM(
<!DOCTYPE html>
<html>
<head>
    <title>bladeRF Waterfall</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: #000;
            color: #fff;
            overflow: hidden;
        }
        .header {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            background: rgba(0, 0, 0, 0.9);
            padding: 10px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            z-index: 1002;
            border-bottom: 1px solid #333;
        }
        .title {
            font-size: 16px;
            font-weight: 600;
            color: #0ff;
            letter-spacing: 1px;
        }
        .status-info {
            display: flex;
            gap: 12px;
            font-size: 12px;
            color: #888;
        }
        .status-value {
            color: #0ff;
            font-weight: 500;
        }
        .controls {
            display: flex;
            gap: 20px;
            align-items: center;
            justify-content: space-between;
            width: 100%;
        }
        .header-left-controls {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .workspace-tabs {
            display: flex;
            gap: 0;
        }
        .workspace-tab {
            padding: 6px 16px;
            background: rgba(30, 30, 30, 0.5);
            border: 1px solid #333;
            border-bottom: none;
            color: #888;
            font-size: 12px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            user-select: none;
            border-radius: 4px 4px 0 0;
            margin-right: -1px;
        }
        .workspace-tab:hover {
            background: rgba(40, 40, 40, 0.8);
            color: #0ff;
        }
        .workspace-tab.active {
            background: rgba(0, 255, 255, 0.15);
            border-color: #0ff;
            color: #0ff;
            box-shadow: 0 -2px 0 #0ff inset;
        }
        .header-right-controls {
            display: flex;
            gap: 10px;
            align-items: center;
        }
        .channel-switch, .spectrum-toggle {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid #333;
            border-radius: 4px;
            padding: 5px 10px;
            color: #fff;
            font-size: 12px;
            cursor: pointer;
            transition: all 0.2s;
        }
        .channel-switch:hover, .spectrum-toggle:hover {
            border-color: #0ff;
        }
        .spectrum-toggle.active {
            background: rgba(0, 255, 255, 0.2);
            border-color: #0ff;
            color: #0ff;
        }
        #workspace-container {
            position: absolute;
            top: 50px;
            left: 0;
            right: 0;
            bottom: 0;
            overflow: hidden;
        }
        .workspace-content {
            display: none;
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
        }
        .workspace-content.active {
            display: block;
        }
        .workspace-panel-slot {
            background: rgba(10, 10, 10, 0.5);
            border: 1px solid #222;
            border-radius: 5px;
            overflow: hidden;
            min-height: 0;
        }
        .workspace-panel-slot::-webkit-scrollbar {
            width: 8px;
            height: 8px;
        }
        .workspace-panel-slot::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.05);
        }
        .workspace-panel-slot::-webkit-scrollbar-thumb {
            background: rgba(0, 255, 255, 0.3);
            border-radius: 4px;
        }
        .workspace-panel-slot::-webkit-scrollbar-thumb:hover {
            background: rgba(0, 255, 255, 0.5);
        }
        .draggable-panel {
            position: fixed;
            background: rgba(10, 10, 10, 0.95);
            border: 1px solid #333;
            border-radius: 4px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.5);
            z-index: 100;
            resize: both;
            overflow: hidden;
        }
        .draggable-panel.active {
            border-color: #0ff;
            z-index: 200;
        }
        .meas-tab {
            flex: 1;
            padding: 8px 12px;
            text-align: center;
            cursor: pointer;
            font-size: 11px;
            color: #888;
            border-right: 1px solid #222;
            transition: all 0.2s;
            user-select: none;
        }
        .meas-tab:last-child {
            border-right: none;
        }
        .meas-tab:hover {
            background: #1a1a1a;
            color: #0ff;
        }
        .meas-tab.active {
            background: #0a3a3a;
            color: #0ff;
            border-bottom: 2px solid #0ff;
        }
        .meas-content {
            animation: fadeIn 0.2s;
        }
        @keyframes fadeIn {
            from { opacity: 0; }
            to { opacity: 1; }
        }
        .panel-header {
            background: rgba(20, 20, 20, 0.9);
            border-bottom: 1px solid #333;
            padding: 6px 10px;
            cursor: move;
            user-select: none;
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-shrink: 0;
        }
        .workspace-panel-slot .panel-header {
            cursor: default;
        }
        .workspace-panel-slot .draggable-panel {
            position: relative !important;
            width: 100% !important;
            height: 100% !important;
            top: auto !important;
            left: auto !important;
            right: auto !important;
            bottom: auto !important;
            border-radius: 0;
            border: none;
            box-shadow: none;
            display: flex;
            flex-direction: column;
        }
        .workspace-panel-slot .draggable-panel > div:not(.panel-header) {
            flex: 1;
            overflow-y: auto;
            overflow-x: hidden;
            min-height: 0;
        }
        .draggable-panel > div:not(.panel-header) {
            overflow-y: auto;
            overflow-x: hidden;
        }
        .draggable-panel {
            overflow: hidden;
        }
        .draggable-panel:not(.workspace-panel-slot *) {
            resize: both;
        }
        .panel-title {
            color: #0ff;
            font-size: 12px;
            font-weight: bold;
        }
        .panel-close {
            color: #888;
            cursor: pointer;
            font-size: 16px;
            line-height: 1;
            padding: 0 4px;
        }
        .panel-close:hover {
            color: #f00;
        }
        .panel-detach {
            color: #888;
            cursor: pointer;
            font-size: 12px;
            line-height: 1;
            padding: 0 6px;
            margin-right: 5px;
            display: none;
        }
        .panel-detach:hover {
            color: #0ff;
        }
        .workspace-panel-slot .panel-detach {
            display: inline-block;
        }
        #spectrum {
            position: fixed;
            top: 50px;
            left: 50px;
            width: calc(100% - 60px);
            height: 200px;
            display: none;
            box-sizing: border-box;
            border: none;
            padding: 0;
            margin: 0;
        }
        #spectrum2 {
            position: fixed;
            top: 50px;
            left: 50px;
            width: calc(100% - 60px);
            height: 200px;
            display: none;
            box-sizing: border-box;
            border: none;
            padding: 0;
            margin: 0;
        }
        .display-label {
            position: fixed;
            background: rgba(0, 0, 0, 0.7);
            color: #0ff;
            font-size: 10px;
            font-weight: bold;
            padding: 2px 8px;
            border-radius: 3px;
            z-index: 1001;
            border: 1px solid rgba(0, 255, 255, 0.3);
        }
        #spectrum-label {
            top: 52px;
            right: 12px;
        }
        #waterfall-label, #waterfall-label-ch1, #waterfall-label-ch2 {
            display: none;
        }
        #waterfall-label {
            top: 252px;
            right: 12px;
        }
        #waterfall-label-ch1 {
            top: 252px;
            left: calc(50% - 100px);
        }
        #waterfall-label-ch2 {
            top: 252px;
            right: 12px;
        }
        #iq_constellation {
            top: 60px;
            right: 20px;
            width: 320px;
            height: 340px;
            display: none;
        }
        #xcorr_display {
            bottom: 40px;
            left: 80px;
            width: 600px;
            height: 240px;
            display: none;
        }
        #waterfall, #waterfall2 {
            position: fixed;
            top: 50px;
            left: 50px;
            width: calc(100% - 60px);
            height: calc(100% - 80px);
            image-rendering: pixelated;
            box-sizing: border-box;
            border: none;
            padding: 0;
            margin: 0;
        }
        #waterfall2 {
            display: none;
        }
        /* Visual separator between dual channels */
        #channel-divider {
            position: fixed;
            top: 50px;
            width: 2px;
            height: calc(100% - 80px);
            background: linear-gradient(to bottom,
                rgba(0, 255, 255, 0.6) 0%,
                rgba(0, 255, 255, 0.8) 50%,
                rgba(0, 255, 255, 0.6) 100%);
            box-shadow: 0 0 10px rgba(0, 255, 255, 0.8);
            z-index: 1002;
            display: none;
            pointer-events: none;
        }
        .fps {
            position: fixed;
            top: 55px;
            left: 10px;
            font-size: 11px;
            color: #0ff;
            opacity: 0.7;
            background: rgba(0, 0, 0, 0.6);
            padding: 4px 8px;
            border-radius: 3px;
            z-index: 1001;
        }
        .resolution-info {
            position: fixed;
            top: 80px;
            left: 10px;
            font-size: 10px;
            color: #888;
            background: rgba(0, 0, 0, 0.6);
            padding: 3px 6px;
            border-radius: 3px;
            z-index: 1001;
        }
        .control-panel {
            position: fixed;
            top: 110px;
            left: 10px;
            bottom: 40px;
            background: rgba(0, 0, 0, 0.85);
            border: 1px solid #333;
            border-radius: 5px;
            padding: 10px;
            padding-right: 5px;
            z-index: 1001;
            font-size: 12px;
            max-width: 280px;
            overflow-y: auto;
            overflow-x: hidden;
        }
        .control-panel::-webkit-scrollbar {
            width: 8px;
        }
        .control-panel::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.05);
            border-radius: 4px;
        }
        .control-panel::-webkit-scrollbar-thumb {
            background: rgba(0, 255, 255, 0.3);
            border-radius: 4px;
        }
        .control-panel::-webkit-scrollbar-thumb:hover {
            background: rgba(0, 255, 255, 0.5);
        }
        .control-panel h3 {
            margin: 0 0 10px 0;
            font-size: 13px;
            color: #0ff;
            border-bottom: 1px solid #333;
            padding-bottom: 5px;
        }
        .control-group {
            margin-bottom: 8px;
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .control-group label {
            color: #888;
            font-size: 10px;
            line-height: 1.2;
        }
        .control-group label.checkbox-label {
            display: flex;
            align-items: center;
            cursor: pointer;
            margin: 0;
        }
        .control-group input[type="checkbox"] {
            width: auto;
            margin-right: 8px;
            cursor: pointer;
        }
        .control-group .input-row {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .control-group input[type="number"],
        .control-group input[type="text"] {
            flex: 1;
            min-width: 0;
            background: rgba(255, 255, 255, 0.1);
            border: 1px solid #444;
            border-radius: 3px;
            padding: 4px 6px;
            color: #fff;
            font-size: 11px;
        }
        .control-group input[type="range"] {
            flex: 1;
            min-width: 0;
            height: 20px;
        }
        .control-group select {
            width: 100%;
            background: rgba(255, 255, 255, 0.1);
            border: 1px solid #444;
            border-radius: 3px;
            padding: 4px 6px;
            color: #fff;
            font-size: 11px;
            cursor: pointer;
        }
        .control-group input:focus,
        .control-group select:focus {
            outline: none;
            border-color: #0ff;
        }
        .control-group button {
            flex-shrink: 0;
            min-width: 45px;
            background: rgba(0, 255, 255, 0.2);
            border: 1px solid #0ff;
            border-radius: 3px;
            padding: 4px 8px;
            color: #0ff;
            font-size: 10px;
            cursor: pointer;
            white-space: nowrap;
        }
        .control-group button:hover {
            background: rgba(0, 255, 255, 0.3);
        }
        .control-group button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        .control-group .range-value {
            flex-shrink: 0;
            min-width: 40px;
            text-align: right;
            color: #0ff;
            font-size: 10px;
        }
        .toggle-controls {
            position: fixed;
            top: 110px;
            left: 10px;
            background: rgba(0, 0, 0, 0.6);
            border: 1px solid #333;
            border-radius: 3px;
            padding: 4px 8px;
            color: #0ff;
            font-size: 11px;
            cursor: pointer;
            z-index: 1000;
        }
        .toggle-controls:hover {
            background: rgba(0, 0, 0, 0.8);
        }
        .axis-label {
            position: fixed;
            font-size: 10px;
            color: #888;
            font-family: monospace;
        }
        .freq-axis {
            bottom: 5px;
            left: 60px;
            right: 10px;
            height: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .time-axis {
            top: 50px;
            left: 0;
            width: 55px;
            bottom: 30px;
            display: flex;
            flex-direction: column;
            justify-content: space-between;
            align-items: flex-end;
            padding-right: 5px;
            transition: top 0.3s ease;
        }
        /* Toast Notification System */
        .notification-container {
            position: fixed;
            top: 60px;
            right: 20px;
            z-index: 10000;
            display: flex;
            flex-direction: column;
            gap: 10px;
            pointer-events: none;
        }
        .notification {
            min-width: 300px;
            max-width: 400px;
            padding: 12px 16px;
            border-radius: 4px;
            font-size: 12px;
            color: #fff;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
            display: flex;
            align-items: center;
            gap: 10px;
            animation: slideIn 0.3s ease-out, fadeOut 0.3s ease-in 2.7s;
            pointer-events: auto;
            border: 1px solid;
        }
        .notification-info {
            background: rgba(0, 170, 255, 0.2);
            border-color: #0af;
        }
        .notification-success {
            background: rgba(0, 255, 100, 0.2);
            border-color: #0f0;
        }
        .notification-warning {
            background: rgba(255, 200, 0, 0.2);
            border-color: #fc0;
        }
        .notification-error {
            background: rgba(255, 50, 50, 0.2);
            border-color: #f33;
        }
        @keyframes slideIn {
            from {
                transform: translateX(400px);
                opacity: 0;
            }
            to {
                transform: translateX(0);
                opacity: 1;
            }
        }
        @keyframes fadeOut {
            to {
                opacity: 0;
                transform: translateX(400px);
            }
        }
        /* Connection Status Indicator */
        .connection-status {
            display: inline-flex;
            align-items: center;
            gap: 5px;
            padding: 3px 8px;
            border-radius: 3px;
            font-size: 10px;
            font-weight: bold;
        }
        .connection-status.connected {
            background: rgba(0, 255, 0, 0.2);
            color: #0f0;
            border: 1px solid #0f0;
        }
        .connection-status.disconnected {
            background: rgba(255, 0, 0, 0.2);
            color: #f33;
            border: 1px solid #f33;
        }
        .connection-status.connecting {
            background: rgba(255, 200, 0, 0.2);
            color: #fc0;
            border: 1px solid #fc0;
        }
        .connection-status::before {
            content: '●';
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        /* Loading Indicator */
        .loading-overlay {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: rgba(0, 0, 0, 0.5);
            display: none;
            justify-content: center;
            align-items: center;
            z-index: 9999;
        }
        .loading-overlay.active {
            display: flex;
        }
        .loading-spinner {
            width: 60px;
            height: 60px;
            border: 4px solid rgba(0, 255, 255, 0.2);
            border-top-color: #0ff;
            border-radius: 50%;
            animation: spin 0.8s linear infinite;
        }
        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        /* Tooltip Styles */
        [title]:not([title=""]) {
            position: relative;
            cursor: help;
        }
        [title]:not([title=""]):hover::after {
            content: attr(title);
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            margin-bottom: 8px;
            padding: 8px 12px;
            background: rgba(0, 0, 0, 0.95);
            color: #0ff;
            font-size: 11px;
            white-space: nowrap;
            border: 1px solid #0ff;
            border-radius: 4px;
            z-index: 10000;
            pointer-events: none;
            box-shadow: 0 2px 10px rgba(0, 255, 255, 0.3);
        }
        [title]:not([title=""]):hover::before {
            content: '';
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            margin-bottom: 2px;
            border: 6px solid transparent;
            border-top-color: #0ff;
            z-index: 10000;
            pointer-events: none;
        }
        /* Button tooltips appear above */
        button[title]:not([title=""]):hover::after {
            bottom: calc(100% + 5px);
        }
        /* Input tooltips appear to the right */
        input[title]:not([title=""]):hover::after {
            bottom: auto;
            left: calc(100% + 10px);
            top: 50%;
            transform: translateY(-50%);
            margin-bottom: 0;
        }
        input[title]:not([title=""]):hover::before {
            bottom: auto;
            left: calc(100% + 4px);
            top: 50%;
            transform: translateY(-50%) rotate(-90deg);
            margin-bottom: 0;
        }
    </style>
</head>
<body>
    <div class="header">
        <div class="title">bladeRF SPECTRUM WATERFALL</div>
        <div class="status-info">
            <span>FREQ: <span class="status-value" id="freq">---</span></span>
            <span>SR: <span class="status-value" id="sr">---</span></span>
            <span>RES: <span class="status-value" id="resolution">---</span></span>
            <span id="zoom_indicator" style="display: none;">ZOOM: <span class="status-value" id="zoom_level">1x</span></span>
            <span>GAIN: <span class="status-value" id="gain">---</span></span>
            <span>LINK: <span class="status-value" id="link_quality_bar">●●●●●</span></span>
            <span>RTT: <span class="status-value" id="rtt">--</span></span>
            <span>BW: <span class="status-value" id="bandwidth">--</span></span>
            <span id="gps_status_bar">GPS: <span class="status-value" id="gps_mode_bar">OFF</span></span>
        </div>
        <div class="controls">
            <div class="header-left-controls">
                <span class="connection-status connecting" id="connectionStatus">CONNECTING</span>
                <div class="workspace-tabs">
                    <div class="workspace-tab active" data-tab="live">LIVE</div>
                    <div class="workspace-tab" data-tab="direction">DIRECTION</div>
                    <div class="workspace-tab" data-tab="iq">IQ</div>
                    <div class="workspace-tab" data-tab="xcorr">XCORR</div>
                </div>
            </div>
            <div class="header-right-controls">
                <button id="spectrum_toggle" class="spectrum-toggle active">Spectrum</button>
                <button id="cursor_toggle" class="spectrum-toggle">Cursor</button>
                <button id="recorder_toggle" class="spectrum-toggle" onclick="toggleRecorder()">Record</button>
                <button id="gps_toggle" class="spectrum-toggle" onclick="toggleGPS()">📍 GPS</button>
                <select id="channel_select" class="channel-switch">
                    <option value="1">RX1</option>
                    <option value="2">RX2</option>
                    <option value="both">Both</option>
                </select>
            </div>
        </div>
    </div>

    <!-- Workspace Content Areas -->
    <div id="workspace-container">

        <!-- TAB 1: LIVE ANALYSIS -->
        <div class="workspace-content active" id="workspace-live">
            <!-- Main Waterfall Display -->
            <canvas id="spectrum"></canvas>
            <canvas id="spectrum2"></canvas>
            <canvas id="waterfall"></canvas>
            <canvas id="waterfall2"></canvas>

            <!-- Display labels -->
            <div id="spectrum-label" class="display-label" style="display: none;">SPECTRUM</div>
            <div id="spectrum-label-ch1" class="display-label" style="display: none;">RX1 SPECTRUM</div>
            <div id="spectrum-label-ch2" class="display-label" style="display: none;">RX2 SPECTRUM</div>
            <div id="waterfall-label" class="display-label">WATERFALL</div>
            <div id="waterfall-label-ch1" class="display-label">RX1 WATERFALL</div>
            <div id="waterfall-label-ch2" class="display-label">RX2 WATERFALL</div>

            <!-- Channel divider for dual-channel mode -->
            <div id="channel-divider"></div>

            <!-- Live Stats Overlay with Signal Strength Meter -->
            <div style="position: fixed; bottom: 20px; right: 20px; background: rgba(0,0,0,0.85); padding: 12px; border: 1px solid #0ff; border-radius: 4px; font-size: 11px; z-index: 50; min-width: 220px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 8px; border-bottom: 1px solid #333; padding-bottom: 5px;">LIVE STATS</div>

                <!-- Signal Strength Meter -->
                <div style="margin-bottom: 10px;">
                    <div style="font-size: 10px; color: #888; margin-bottom: 3px;">Signal Strength</div>
                    <div style="width: 100%; height: 20px; background: #0a0a0a; border: 1px solid #333; border-radius: 3px; overflow: hidden; position: relative;">
                        <div id="signal_strength_bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #0f0, #ff0, #f00); transition: width 0.2s;"></div>
                        <div style="position: absolute; top: 0; left: 0; right: 0; bottom: 0; display: flex; align-items: center; justify-content: center; font-size: 10px; font-weight: bold; color: #fff; text-shadow: 1px 1px 2px #000;">
                            <span id="signal_strength_text">--</span>
                        </div>
                    </div>
                </div>

                <div style="color: #888; font-family: monospace;">
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Peak:</span>
                        <span style="color: #0f0;" id="live_peak_power">-- dBm</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Avg:</span>
                        <span style="color: #ff0;" id="live_avg_power">-- dBm</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Floor:</span>
                        <span style="color: #888;" id="live_noise_floor">-- dBm</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0; padding-top: 3px; border-top: 1px solid #222;">
                        <span>FPS:</span>
                        <span style="color: #0ff;" id="live_fps_display">--</span>
                    </div>
                </div>
            </div>

        </div> <!-- End workspace-live -->

        <!-- TAB 2: DIRECTION FINDING -->
        <div class="workspace-content" id="workspace-direction">
            <div style="display: flex; flex-direction: column; height: 100%; gap: 8px;">

                <!-- Top 55%: Spectrum Display with Controls -->
                <div style="flex: 0 0 55%; display: flex; flex-direction: column; background: #000; border: 2px solid #0ff; border-radius: 5px; overflow: hidden;">
                    <!-- Spectrum Controls Bar -->
                    <div style="background: #111; padding: 8px; border-bottom: 1px solid #0ff; display: flex; justify-content: space-between; align-items: center; gap: 15px;">
                        <div style="display: flex; align-items: center; gap: 15px; flex: 1;">
                            <div style="color: #0ff; font-weight: bold; font-size: 11px;">SPECTRUM SELECTION</div>
                            <div style="font-size: 10px;">
                                <span>Center: <span style="color: #0f0; font-weight: bold;" id="doa_sel_center">-- MHz</span></span>
                                <span style="margin-left: 15px;">BW: <span style="color: #ff0; font-weight: bold;" id="doa_sel_bw">-- kHz</span></span>
                            </div>
                        </div>
                        <div style="display: flex; gap: 5px; align-items: center;">
                            <span style="font-size: 10px; color: #888;">Vertical Offset:</span>
                            <button onclick="adjustSpectrumOffset(10)" style="padding: 2px 8px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">▲</button>
                            <button onclick="resetSpectrumOffset()" style="padding: 2px 8px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">Reset</button>
                            <button onclick="adjustSpectrumOffset(-10)" style="padding: 2px 8px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">▼</button>
                        </div>
                    </div>

                    <!-- Spectrum Canvas Container -->
                    <div style="flex: 1; position: relative; background: #000;">
                        <canvas id="direction_spectrum" style="width: 100%; height: 100%; display: block;"></canvas>

                        <!-- Selection Cursors Overlay -->
                        <div id="doa_selection_overlay" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none;">
                            <div id="doa_cursor_left" style="position: absolute; width: 3px; height: 100%; background: #0ff; box-shadow: 0 0 10px #0ff; display: none; pointer-events: auto; cursor: ew-resize;"></div>
                            <div id="doa_cursor_right" style="position: absolute; width: 3px; height: 100%; background: #0ff; box-shadow: 0 0 10px #0ff; display: none; pointer-events: auto; cursor: ew-resize;"></div>
                        </div>

                        <!-- Instructions Overlay (shows when no selection) -->
                        <div id="doa_instructions" style="position: absolute; top: 10px; right: 10px; background: rgba(0, 0, 0, 0.85); padding: 10px 15px; border: 1px solid #0ff; border-radius: 4px; pointer-events: none; max-width: 300px;">
                            <div style="color: #0ff; font-size: 11px; font-weight: bold; margin-bottom: 6px;">📍 SELECT SIGNAL</div>
                            <div style="font-size: 9px; color: #aaa; line-height: 1.5;">
                                Click & drag to select → Adjust cursors → Start DF
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Bottom 45%: Analysis Displays -->
                <div style="flex: 1; display: grid; grid-template-columns: 1.2fr 0.8fr 1fr; grid-template-rows: 1fr 1fr; gap: 8px; min-height: 0;">

                    <!-- Polar DoA Display (spans 2 rows) -->
                    <div class="workspace-panel-slot" style="grid-row: 1 / 3; grid-column: 1; overflow-y: auto;">
                        <div style="padding: 12px; display: flex; flex-direction: column; min-height: 100%; gap: 8px;">
                            <div style="display: flex; justify-content: space-between; align-items: center;">
                                <strong style="color: #0ff; font-size: 13px;">AZIMUTH</strong>
                                <div style="font-size: 10px; font-family: monospace;">
                                    <span style="color: #0f0; font-weight: bold; font-size: 16px;" id="doa_azimuth_main">--</span><span style="color: #888;">°</span>
                                </div>
                            </div>
                            <canvas id="doa_polar_main" style="flex: 1; min-height: 200px; background: #0a0a0a; border: 1px solid #333; border-radius: 3px;"></canvas>
                            <div style="font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px; display: grid; grid-template-columns: 1fr 1fr; gap: 8px;">
                                <div>Confidence: <span style="color: #ff0;" id="doa_confidence_main">--%</span></div>
                                <div>SNR: <span style="color: #0f0;" id="doa_snr">-- dB</span></div>
                                <div>Coherence: <span style="color: #0f0;" id="doa_coherence_mag">--</span></div>
                                <div>Quality: <span style="color: #ff0;" id="doa_quality">--</span></div>
                            </div>
                        </div>
                    </div>

                    <!-- Configuration Panel -->
                    <div class="workspace-panel-slot" style="grid-row: 1; grid-column: 2;">
                        <div style="padding: 12px; font-size: 11px; height: 100%; display: flex; flex-direction: column; gap: 10px; overflow-y: auto;">
                            <strong style="color: #0ff; font-size: 12px;">CONFIG</strong>

                            <div>
                                <label style="font-size: 10px; color: #888;">Array Spacing (λ):</label>
                                <input type="number" id="doa_spacing_main" value="0.5" step="0.1" min="0.1" max="2.0" style="width: 100%; margin-top: 3px;">
                                <div style="font-size: 9px; color: #666; margin-top: 2px;">0.5λ = 164mm @ 915 MHz</div>
                            </div>

                            <div>
                                <label style="font-size: 10px; color: #888;">Algorithm:</label>
                                <select id="doa_algorithm" style="width: 100%; margin-top: 3px; font-size: 10px;">
                                    <option value="phase">Phase Difference (2-Ch Interferometry)</option>
                                </select>
                            </div>

                            <div>
                                <label style="font-size: 10px; color: #888;">Update Rate:</label>
                                <select id="doa_update_rate" style="width: 100%; margin-top: 3px; font-size: 10px;">
                                    <option value="100">10 Hz</option>
                                    <option value="200" selected>5 Hz</option>
                                    <option value="500">2 Hz</option>
                                    <option value="1000">1 Hz</option>
                                </select>
                            </div>

                            <div style="margin-top: auto;">
                                <label style="display: flex; align-items: center; cursor: pointer; font-size: 10px;">
                                    <input type="checkbox" id="doa_multi_source" style="margin-right: 5px;">
                                    <span>Multi-Source</span>
                                </label>
                            </div>
                        </div>
                    </div>

                    <!-- Control Buttons -->
                    <div class="workspace-panel-slot" style="grid-row: 2; grid-column: 2;">
                        <div style="padding: 12px; display: flex; flex-direction: column; gap: 8px; height: 100%; overflow-y: auto;">
                            <strong style="color: #0ff; font-size: 12px;">CONTROL</strong>

                            <div style="display: flex; flex-direction: column; gap: 6px; flex: 1; justify-content: center;">
                                <button onclick="startDoA()" style="padding: 12px; background: #0a3a3a; border: 2px solid #0ff; color: #0ff; cursor: pointer; border-radius: 4px; font-weight: bold; font-size: 12px;" title="Start direction finding on selected frequency range">
                                    ▶ START
                                </button>
                                <button onclick="stopDoA()" style="padding: 10px; background: #3a0a0a; border: 2px solid #f00; color: #f00; cursor: pointer; border-radius: 4px; font-weight: bold; font-size: 11px;" title="Stop direction finding">
                                    ■ STOP
                                </button>
                                <button onclick="calibrateDoAMain()" style="padding: 8px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 10px;" title="Calibrate phase offset with known signal at 0°">
                                    Calibrate Array
                                </button>
                                <button id="doa_freeze_btn" onclick="toggleDoAFreeze()" style="padding: 8px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 10px;" title="Freeze display to analyze current bearing">
                                    ❄ Freeze
                                </button>
                                <button onclick="showBearingExportDialog()" style="padding: 8px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 10px;" title="Export bearing history to CSV/JSON/KML">
                                    📁 Export
                                </button>
                                <button onclick="openStreamOutConfig()" style="padding: 8px; background: #1a1a3a; border: 1px solid #88f; color: #aaf; cursor: pointer; border-radius: 3px; font-size: 10px;" title="Configure TAK/CoT streaming output">
                                    Stream Out
                                </button>
                            </div>

                            <div style="text-align: center; padding: 6px; background: #0a0a0a; border-radius: 3px; font-size: 10px;">
                                Status: <span style="color: #0f0; font-weight: bold;" id="doa_status_live">Idle</span>
                            </div>
                            <div style="text-align: center; padding: 4px; background: #0a0a0a; border-radius: 3px; font-size: 9px; border-top: 1px solid #222;">
                                Stream: <span style="color: #888; font-weight: bold;" id="doa_stream_status">Off</span>
                            </div>
                        </div>
                    </div>

                    <!-- Phase Analysis -->
                    <div class="workspace-panel-slot" style="grid-row: 1; grid-column: 3;">
                        <div style="padding: 12px; font-size: 11px; height: 100%; overflow-y: auto;">
                            <strong style="color: #0ff; font-size: 12px;">PHASE ANALYSIS</strong>

                            <div style="margin-top: 10px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                                <div style="margin-bottom: 6px; padding-bottom: 6px; border-bottom: 1px solid #222;">
                                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                                        <span>CH1-CH2:</span><span style="color: #0f0;" id="doa_phase_diff">-- deg</span>
                                    </div>
                                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                                        <span>Unwrapped:</span><span style="color: #ff0;" id="doa_phase_unwrap">-- deg</span>
                                    </div>
                                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                                        <span>Std Dev:</span><span style="color: #888;" id="doa_phase_std">-- deg</span>
                                    </div>
                                </div>

                                <div style="margin-top: 6px;">
                                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                                        <span>Samples:</span><span style="color: #888;" id="doa_samples">0</span>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- Direction History Timeline -->
                    <div class="workspace-panel-slot" style="grid-row: 2; grid-column: 3;">
                        <div style="padding: 12px; display: flex; flex-direction: column; height: 100%; gap: 8px;">
                            <strong style="color: #0ff; font-size: 12px;">HISTORY</strong>
                            <canvas id="doa_timeline" style="flex: 1; background: #0a0a0a; border: 1px solid #333; border-radius: 3px; min-height: 0;"></canvas>
                            <div style="display: flex; gap: 4px;">
                                <button onclick="clearDoAHistory()" style="flex: 1; padding: 4px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                                    Clear
                                </button>
                                <button onclick="exportDoAData()" style="flex: 1; padding: 4px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                                    Export
                                </button>
                            </div>
                        </div>
                    </div>

                </div>
            </div>
        </div> <!-- End workspace-direction -->

        <!-- TAB 3: IQ CONSTELLATION -->
        <div class="workspace-content" id="workspace-iq">
            <div style="display: flex; height: 100%; gap: 10px; padding: 10px; background: #0a0a0a;">

                <!-- Left: Main Display Area -->
                <div style="flex: 1; display: flex; flex-direction: column; gap: 10px;">
                    <!-- Top Row: Split into two panels (50/50) -->
                    <div style="flex: 0 0 48%; display: flex; gap: 10px;">
                        <!-- IQ Constellation (Left Half) -->
                        <div style="flex: 1; display: flex; flex-direction: column; background: #000; border: 2px solid #0ff; border-radius: 5px; overflow: hidden;">
                            <div style="background: linear-gradient(to bottom, #003333, #001a1a); padding: 8px; border-bottom: 1px solid #0ff; display: flex; justify-content: space-between; align-items: center;">
                                <span style="color: #0ff; font-weight: bold; font-size: 12px;">IQ CONSTELLATION</span>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <button id="iq_auto_scale_btn" onclick="iqAutoScale()" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;" title="Auto-scale to fit data">Auto</button>
                                    <button id="iq_clear_btn" onclick="iqClearPersistence()" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;" title="Clear persistence trail">Clear</button>
                                </div>
                            </div>
                            <div id="iq_fullscreen_container" style="flex: 1; position: relative; overflow: hidden;">
                                <canvas id="iq_fullscreen" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></canvas>
                            </div>
                        </div>

                        <!-- Eye Diagram (Right Half) -->
                        <div style="flex: 1; display: flex; flex-direction: column; background: #000; border: 2px solid #0ff; border-radius: 5px; overflow: hidden;">
                            <div style="background: linear-gradient(to bottom, #003333, #001a1a); padding: 8px; border-bottom: 1px solid #0ff; display: flex; justify-content: space-between; align-items: center;">
                                <span style="color: #0ff; font-weight: bold; font-size: 12px;">EYE DIAGRAM</span>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <label style="font-size: 9px; color: #888;">Symbol Rate:</label>
                                    <select id="eye_symbol_rate" onchange="eyeUpdateSymbolRate()" style="padding: 2px 4px; background: #0a0a0a; border: 1px solid #0ff; color: #0ff; font-size: 9px; border-radius: 3px;">
                                        <option value="1000000">1 Msym/s</option>
                                        <option value="500000">500 ksym/s</option>
                                        <option value="250000">250 ksym/s</option>
                                        <option value="100000">100 ksym/s</option>
                                        <option value="50000">50 ksym/s</option>
                                    </select>
                                    <button onclick="eyeClear()" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">Clear</button>
                                </div>
                            </div>
                            <div style="flex: 1; position: relative; overflow: hidden;">
                                <canvas id="eye_diagram_canvas" style="width: 100%; height: 100%; display: block;"></canvas>
                            </div>
                        </div>
                    </div>

                    <!-- Bottom Row: Split into two panels (50/50) -->
                    <div style="flex: 1; display: flex; gap: 10px;">
                        <!-- IQ Waveform Time-Domain (Left Half) -->
                        <div style="flex: 1; display: flex; flex-direction: column; background: #000; border: 2px solid #0ff; border-radius: 5px; overflow: hidden;">
                            <div style="background: linear-gradient(to bottom, #003333, #001a1a); padding: 8px; border-bottom: 1px solid #0ff; display: flex; justify-content: space-between; align-items: center;">
                                <span style="color: #0ff; font-weight: bold; font-size: 12px;">IQ WAVEFORM (TIME DOMAIN)</span>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <label style="font-size: 9px; color: #888;">View:</label>
                                    <select id="waveform_view_mode" onchange="waveformUpdateMode()" style="padding: 2px 4px; background: #0a0a0a; border: 1px solid #0ff; color: #0ff; font-size: 9px; border-radius: 3px;">
                                        <option value="i_and_q">I & Q</option>
                                        <option value="magnitude">Magnitude</option>
                                        <option value="phase">Phase</option>
                                        <option value="i_only">I Only</option>
                                        <option value="q_only">Q Only</option>
                                    </select>
                                </div>
                            </div>
                            <div style="flex: 1; position: relative; overflow: hidden;">
                                <canvas id="waveform_canvas" style="width: 100%; height: 100%; display: block;"></canvas>
                            </div>
                        </div>

                        <!-- Spectrum Display (Right Half) -->
                        <div style="flex: 1; display: flex; flex-direction: column; background: #000; border: 2px solid #0ff; border-radius: 5px; overflow: hidden;">
                            <div style="background: linear-gradient(to bottom, #003333, #001a1a); padding: 8px; border-bottom: 1px solid #0ff; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 4px;">
                                <span style="color: #0ff; font-weight: bold; font-size: 12px;">SPECTRUM</span>
                                <div style="display: flex; gap: 4px; align-items: center; flex-wrap: wrap;">
                                    <button onclick="tuneDown(1000000)" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">-1M</button>
                                    <button onclick="tuneDown(100000)" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">-100k</button>
                                    <div style="display: flex; flex-direction: column; align-items: center; gap: 1px;">
                                        <span id="iq_center_freq" style="color: #0ff; font-size: 10px; font-weight: bold;">915.000 MHz</span>
                                        <span id="iq_span" style="color: #888; font-size: 8px;">40.00 MHz</span>
                                    </div>
                                    <button onclick="tuneUp(100000)" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">+100k</button>
                                    <button onclick="tuneUp(1000000)" style="padding: 3px 6px; font-size: 9px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">+1M</button>
                                </div>
                            </div>
                            <div style="flex: 1; position: relative; overflow: hidden;">
                                <canvas id="iq_spectrum" style="width: 100%; height: 100%; display: block;"></canvas>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Right: Control Panel -->
                <div style="width: 280px; display: flex; flex-direction: column; gap: 10px; overflow-y: auto;">

                    <!-- Signal Detection Panel -->
                    <div style="background: #1a1a1a; border: 1px solid #0ff; border-radius: 5px; padding: 10px;">
                        <div style="color: #0ff; font-weight: bold; font-size: 11px; margin-bottom: 8px; border-bottom: 1px solid #0ff; padding-bottom: 5px;">SIGNAL DETECTION</div>
                        <div style="display: flex; flex-direction: column; gap: 6px; font-family: monospace; font-size: 10px;">
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Status:</span>
                                <span id="iq_signal_status" style="color: #ff0;">NO SIGNAL</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">SNR:</span>
                                <span id="iq_snr" style="color: #0ff;">-- dB</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Peak Power:</span>
                                <span id="iq_peak_power" style="color: #0ff;">-- dBFS</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Noise Floor:</span>
                                <span id="iq_noise_floor" style="color: #888;">-- dBFS</span>
                            </div>
                        </div>

                        <!-- Signal Strength Indicator -->
                        <div style="margin-top: 10px;">
                            <div style="font-size: 9px; color: #888; margin-bottom: 3px;">Signal Strength</div>
                            <div style="height: 20px; background: #0a0a0a; border: 1px solid #0ff; border-radius: 3px; overflow: hidden; position: relative;">
                                <div id="iq_signal_bar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #0a0, #0f0); transition: width 0.3s;"></div>
                                <div style="position: absolute; top: 0; left: 0; right: 0; bottom: 0; display: flex; align-items: center; justify-content: center; font-size: 9px; color: #fff; font-weight: bold; text-shadow: 1px 1px 2px #000;">
                                    <span id="iq_signal_bar_text">0%</span>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- IQ Statistics Panel -->
                    <div style="background: #1a1a1a; border: 1px solid #0ff; border-radius: 5px; padding: 10px;">
                        <div style="color: #0ff; font-weight: bold; font-size: 11px; margin-bottom: 8px; border-bottom: 1px solid #0ff; padding-bottom: 5px;">IQ STATISTICS</div>
                        <div style="display: flex; flex-direction: column; gap: 6px; font-family: monospace; font-size: 10px;">
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">I Mean:</span>
                                <span id="iq_i_mean" style="color: #0ff;">--</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Q Mean:</span>
                                <span id="iq_q_mean" style="color: #0ff;">--</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">I RMS:</span>
                                <span id="iq_i_rms" style="color: #0ff;">--</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Q RMS:</span>
                                <span id="iq_q_rms" style="color: #0ff;">--</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">EVM:</span>
                                <span id="iq_evm" style="color: #0ff;">-- %</span>
                            </div>
                            <div style="display: flex; justify-content: space-between;">
                                <span style="color: #888;">Phase Noise:</span>
                                <span id="iq_phase_noise" style="color: #0ff;">-- °</span>
                            </div>
                        </div>
                    </div>

                    <!-- Fine Tuning Panel -->
                    <div style="background: #1a1a1a; border: 1px solid #0ff; border-radius: 5px; padding: 10px;">
                        <div style="color: #0ff; font-weight: bold; font-size: 11px; margin-bottom: 8px; border-bottom: 1px solid #0ff; padding-bottom: 5px;">FINE TUNING</div>

                        <div style="display: flex; flex-direction: column; gap: 8px;">
                            <!-- Frequency Offset -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Freq Offset (kHz)</label>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <button onclick="iqFreqOffset(-10)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">-10</button>
                                    <button onclick="iqFreqOffset(-1)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">-1</button>
                                    <input type="number" id="iq_freq_offset" value="0" step="1" style="flex: 1; padding: 4px; background: #0a0a0a; border: 1px solid #0ff; color: #0ff; font-size: 10px; text-align: center; border-radius: 3px;">
                                    <button onclick="iqFreqOffset(1)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">+1</button>
                                    <button onclick="iqFreqOffset(10)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">+10</button>
                                </div>
                                <button onclick="iqApplyOffset()" style="width: 100%; margin-top: 4px; padding: 6px; background: #006666; color: #0ff; border: 1px solid #0ff; font-size: 10px; font-weight: bold; cursor: pointer; border-radius: 3px;">Apply Offset</button>
                            </div>

                            <!-- Gain Control -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">RX Gain</label>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <button onclick="iqGainAdjust(-3)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">-3dB</button>
                                    <span id="iq_current_gain" style="flex: 1; text-align: center; color: #0ff; font-size: 11px; font-weight: bold;">60 dB</span>
                                    <button onclick="iqGainAdjust(3)" style="padding: 4px 8px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">+3dB</button>
                                </div>
                            </div>

                            <!-- Bandwidth Control -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">IF Bandwidth (MHz)</label>
                                <select id="iq_bandwidth_select" onchange="iqBandwidthChange()" style="width: 100%; padding: 5px; background: #0a0a0a; border: 1px solid #0ff; color: #0ff; font-size: 10px; border-radius: 3px;">
                                    <option value="1.5">1.5 MHz</option>
                                    <option value="1.75">1.75 MHz</option>
                                    <option value="2.5">2.5 MHz</option>
                                    <option value="2.75">2.75 MHz</option>
                                    <option value="3">3 MHz</option>
                                    <option value="5">5 MHz</option>
                                    <option value="10">10 MHz</option>
                                    <option value="20">20 MHz</option>
                                    <option value="28" selected>28 MHz</option>
                                    <option value="40">40 MHz (Full)</option>
                                </select>
                            </div>
                        </div>
                    </div>

                    <!-- Display Settings Panel -->
                    <div style="background: #1a1a1a; border: 1px solid #0ff; border-radius: 5px; padding: 10px;">
                        <div style="color: #0ff; font-weight: bold; font-size: 11px; margin-bottom: 8px; border-bottom: 1px solid #0ff; padding-bottom: 5px;">DISPLAY SETTINGS</div>

                        <div style="display: flex; flex-direction: column; gap: 8px;">
                            <!-- Modulation Type -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Modulation Type</label>
                                <select id="iq_modulation_type" onchange="iqModulationTypeChange(this.value)" style="width: 100%; padding: 4px; font-size: 10px; background: #000; color: #0ff; border: 1px solid #0ff;">
                                    <option value="none">None (Raw IQ)</option>
                                    <option value="bpsk">BPSK</option>
                                    <option value="qpsk">QPSK</option>
                                    <option value="8psk">8-PSK</option>
                                    <option value="16qam">16-QAM</option>
                                    <option value="64qam">64-QAM</option>
                                    <option value="256qam">256-QAM</option>
                                    <option value="ask2">2-ASK (OOK)</option>
                                    <option value="ask4">4-ASK</option>
                                    <option value="fm">FM (Analog)</option>
                                    <option value="am">AM (Analog)</option>
                                    <option value="fsk2">2-FSK</option>
                                    <option value="fsk4">4-FSK</option>
                                </select>
                            </div>

                            <!-- Persistence -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Persistence (0-100%)</label>
                                <input type="range" id="iq_persistence" min="0" max="100" value="95" oninput="iqPersistenceChange(this.value)" style="width: 100%;">
                                <div style="text-align: center; font-size: 9px; color: #0ff; margin-top: 2px;">
                                    <span id="iq_persistence_value">95%</span>
                                </div>
                            </div>

                            <!-- Point Size -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Point Size</label>
                                <input type="range" id="iq_point_size" min="1" max="5" value="2" oninput="iqPointSizeChange(this.value)" style="width: 100%;">
                                <div style="text-align: center; font-size: 9px; color: #0ff; margin-top: 2px;">
                                    <span id="iq_point_size_value">2px</span>
                                </div>
                            </div>

                            <!-- Zoom Level -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Zoom Level</label>
                                <div style="display: flex; gap: 4px;">
                                    <button onclick="iqZoom(0.5)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">0.5x</button>
                                    <button onclick="iqZoom(1.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">1x</button>
                                    <button onclick="iqZoom(2.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">2x</button>
                                    <button onclick="iqZoom(4.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">4x</button>
                                </div>
                            </div>

                            <!-- Toggle Options -->
                            <div style="display: flex; flex-direction: column; gap: 4px; margin-top: 4px;">
                                <label style="display: flex; align-items: center; gap: 6px; font-size: 10px; color: #888; cursor: pointer;">
                                    <input type="checkbox" id="iq_show_grid" checked onchange="iqToggleGrid()" style="cursor: pointer;">
                                    Show Grid
                                </label>
                                <label style="display: flex; align-items: center; gap: 6px; font-size: 10px; color: #888; cursor: pointer;">
                                    <input type="checkbox" id="iq_show_stats" checked onchange="iqToggleStats()" style="cursor: pointer;">
                                    Show Statistics Overlay
                                </label>
                                <label style="display: flex; align-items: center; gap: 6px; font-size: 10px; color: #888; cursor: pointer;">
                                    <input type="checkbox" id="iq_density_mode" checked onchange="iqToggleDensity()" style="cursor: pointer;">
                                    Density Heatmap Mode
                                </label>
                            </div>
                        </div>
                    </div>

                    <!-- Waveform/Eye Diagram Controls -->
                    <div style="background: #1a1a1a; border: 1px solid #0ff; border-radius: 5px; padding: 10px; margin-top: 10px;">
                        <div style="color: #0ff; font-weight: bold; font-size: 11px; margin-bottom: 8px; border-bottom: 1px solid #0ff; padding-bottom: 5px;">WAVEFORM & EYE CONTROLS</div>

                        <div style="display: flex; flex-direction: column; gap: 8px;">
                            <!-- Waveform View Mode -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Waveform View</label>
                                <select id="waveform_view_mode" onchange="waveformViewModeChange(this.value)" style="width: 100%; padding: 4px; font-size: 10px; background: #000; color: #0ff; border: 1px solid #0ff;">
                                    <option value="i_and_q">I & Q</option>
                                    <option value="magnitude">Magnitude</option>
                                    <option value="phase">Phase</option>
                                    <option value="i_only">I Only</option>
                                    <option value="q_only">Q Only</option>
                                </select>
                            </div>

                            <!-- Eye Diagram Symbol Rate -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Eye Symbol Rate</label>
                                <select id="eye_symbol_rate" onchange="eyeSymbolRateChange(this.value)" style="width: 100%; padding: 4px; font-size: 10px; background: #000; color: #0ff; border: 1px solid #0ff;">
                                    <option value="50000">50 kSym/s</option>
                                    <option value="100000">100 kSym/s</option>
                                    <option value="250000">250 kSym/s</option>
                                    <option value="500000">500 kSym/s</option>
                                    <option value="1000000" selected>1 MSym/s</option>
                                    <option value="2000000">2 MSym/s</option>
                                    <option value="5000000">5 MSym/s</option>
                                </select>
                            </div>

                            <!-- Zoom Controls -->
                            <div>
                                <label style="font-size: 10px; color: #888; display: block; margin-bottom: 3px;">Zoom (Mouse Wheel)</label>
                                <div style="display: flex; gap: 4px;">
                                    <button onclick="waveformEyeZoom(1.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">1x</button>
                                    <button onclick="waveformEyeZoom(2.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">2x</button>
                                    <button onclick="waveformEyeZoom(5.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">5x</button>
                                    <button onclick="waveformEyeZoom(10.0)" style="flex: 1; padding: 4px; font-size: 10px; background: #004444; color: #0ff; border: 1px solid #0ff; cursor: pointer; border-radius: 3px;">10x</button>
                                </div>
                            </div>

                            <!-- Reset View -->
                            <div>
                                <button onclick="waveformEyeResetView()" style="width: 100%; padding: 6px; font-size: 10px; background: #440000; color: #ff0; border: 1px solid #ff0; cursor: pointer; border-radius: 3px; font-weight: bold;">Reset View (Double Click)</button>
                            </div>

                            <!-- Instructions -->
                            <div style="font-size: 9px; color: #666; margin-top: 4px; padding: 6px; background: #0a0a0a; border-radius: 3px;">
                                <strong style="color: #888;">Controls:</strong><br>
                                • Mouse Wheel: Zoom in/out<br>
                                • Click & Drag: Pan view<br>
                                • Double Click: Reset zoom/pan
                            </div>
                        </div>
                    </div>

                </div> <!-- End Right Control Panel -->

            </div>
        </div> <!-- End workspace-iq -->

        <!-- TAB 5: CROSS-CORRELATION -->
        <div class="workspace-content" id="workspace-xcorr">
            <div style="display: flex; flex-direction: column; height: 100%; gap: 10px; padding: 10px; background: #0a0a0a;">

                <!-- Top: Spectrum Display (30%) -->
                <div style="flex: 0 0 30%; display: flex; flex-direction: column; background: #000; border: 2px solid #ff00ff; border-radius: 5px; overflow: hidden;">
                    <div style="background: linear-gradient(to bottom, #330033, #1a001a); padding: 8px; border-bottom: 1px solid #ff00ff; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;">
                        <span style="color: #ff00ff; font-weight: bold; font-size: 12px;">SPECTRUM ANALYZER</span>
                        <div style="display: flex; gap: 8px; align-items: center; flex-wrap: wrap;">
                            <!-- Tuning Controls -->
                            <button onclick="tuneDown(10000000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune down 10 MHz">-10M</button>
                            <button onclick="tuneDown(1000000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune down 1 MHz">-1M</button>
                            <button onclick="tuneDown(100000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune down 100 kHz">-100k</button>
                            <div style="display: flex; flex-direction: column; align-items: center; gap: 2px;">
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <span style="color: #888; font-size: 10px;">Center:</span>
                                    <span id="xcorr_center_freq" style="color: #ff00ff; font-size: 11px; font-weight: bold;">915.000 MHz</span>
                                </div>
                                <div style="display: flex; gap: 4px; align-items: center;">
                                    <span style="color: #888; font-size: 10px;">Span:</span>
                                    <span id="xcorr_span" style="color: #ff00ff; font-size: 11px; font-weight: bold;">40.00 MHz</span>
                                </div>
                            </div>
                            <button onclick="tuneUp(100000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune up 100 kHz">+100k</button>
                            <button onclick="tuneUp(1000000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune up 1 MHz">+1M</button>
                            <button onclick="tuneUp(10000000)" style="padding: 4px 8px; font-size: 10px; background: #440044; color: #ff00ff; border: 1px solid #ff00ff; cursor: pointer; border-radius: 3px;" title="Tune up 10 MHz">+10M</button>
                        </div>
                    </div>
                    <div style="flex: 1; position: relative; overflow: hidden;">
                        <canvas id="xcorr_spectrum" style="width: 100%; height: 100%; display: block;"></canvas>
                    </div>
                </div>

                <!-- Bottom: Cross-Correlation Display (70%) -->
                <div style="flex: 1; display: flex; flex-direction: column; background: #000; border: 2px solid #ff00ff; border-radius: 5px; overflow: hidden;">
                    <div style="background: linear-gradient(to bottom, #330033, #1a001a); padding: 8px; border-bottom: 1px solid #ff00ff;">
                        <span style="color: #ff00ff; font-weight: bold; font-size: 12px;">CROSS-CORRELATION ANALYSIS</span>
                    </div>
                    <div id="xcorr_fullscreen_container" style="flex: 1; position: relative; overflow: hidden;">
                        <canvas id="xcorr_fullscreen" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></canvas>
                    </div>
                </div>

            </div>
        </div> <!-- End workspace-xcorr -->

    </div> <!-- End workspace-container -->

    <!-- Stream Out Configuration Modal -->
    <div id="streamout_modal" style="display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0, 0, 0, 0.8); z-index: 2000; justify-content: center; align-items: center;">
        <div style="background: #1a1a1a; border: 2px solid #88f; border-radius: 8px; padding: 20px; max-width: 500px; width: 90%; max-height: 80vh; overflow-y: auto;">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px;">
                <h3 style="margin: 0; color: #88f; font-size: 16px;">Stream Out Configuration</h3>
                <span onclick="closeStreamOutConfig()" style="cursor: pointer; color: #f00; font-size: 20px; font-weight: bold;">&times;</span>
            </div>

            <!-- Network Configuration -->
            <div style="margin-bottom: 15px;">
                <h4 style="color: #0ff; font-size: 13px; margin: 0 0 10px 0; border-bottom: 1px solid #333; padding-bottom: 5px;">Network</h4>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Endpoint (IP/Hostname)</label>
                    <input type="text" id="stream_endpoint" placeholder="192.168.1.100 or hostname" value="127.0.0.1" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                </div>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Port</label>
                    <input type="number" id="stream_port" placeholder="8089" value="8089" min="1" max="65535" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                </div>
            </div>

            <!-- Data Format -->
            <div style="margin-bottom: 15px;">
                <h4 style="color: #0ff; font-size: 13px; margin: 0 0 10px 0; border-bottom: 1px solid #333; padding-bottom: 5px;">Data Format</h4>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Protocol</label>
                    <select id="stream_protocol" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        <option value="http">HTTP/HTTPS (POST)</option>
                        <option value="udp">UDP (via server relay)</option>
                    </select>
                </div>
                <div>
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Data Format</label>
                    <select id="stream_format" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        <option value="cot">Cursor on Target (CoT XML)</option>
                        <option value="json">JSON</option>
                        <option value="proto">Protobuf (Binary)</option>
                        <option value="csv">CSV</option>
                        <option value="nmea">NMEA-like</option>
                    </select>
                    <div style="font-size: 9px; color: #666; margin-top: 5px;">CoT format compatible with ATAK/WinTAK</div>
                </div>
            </div>

            <!-- Sensor Position -->
            <div style="margin-bottom: 15px;">
                <h4 style="color: #0ff; font-size: 13px; margin: 0 0 10px 0; border-bottom: 1px solid #333; padding-bottom: 5px;">Sensor Position</h4>
                <div style="margin-bottom: 10px;">
                    <label style="display: flex; align-items: center; cursor: pointer; margin-bottom: 8px;">
                        <input type="radio" name="position_mode" value="static" checked onchange="togglePositionMode()" style="margin-right: 8px;">
                        <span style="font-size: 11px;">Static (Manual Entry)</span>
                    </label>
                    <label style="display: flex; align-items: center; cursor: pointer;">
                        <input type="radio" name="position_mode" value="gps" onchange="togglePositionMode()" style="margin-right: 8px;">
                        <span style="font-size: 11px;">GPS (from gpsd)</span>
                    </label>
                </div>

                <div id="static_position_inputs">
                    <div style="margin-bottom: 10px;">
                        <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Coordinate Format</label>
                        <select id="coord_format" onchange="toggleCoordFormat()" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                            <option value="latlon">Lat/Lon (Decimal Degrees)</option>
                            <option value="mgrs">MGRS</option>
                        </select>
                    </div>

                    <div id="latlon_inputs">
                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px;">
                            <div>
                                <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Latitude</label>
                                <input type="number" id="sensor_lat" placeholder="37.7749" step="0.000001" value="37.7749" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                            </div>
                            <div>
                                <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Longitude</label>
                                <input type="number" id="sensor_lon" placeholder="-122.4194" step="0.000001" value="-122.4194" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                            </div>
                        </div>
                        <div>
                            <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Altitude (m HAE)</label>
                            <input type="number" id="sensor_alt" placeholder="10" value="10" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        </div>
                    </div>

                    <div id="mgrs_inputs" style="display: none;">
                        <div style="margin-bottom: 10px;">
                            <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">MGRS Coordinate</label>
                            <input type="text" id="sensor_mgrs" placeholder="10SEG1234567890" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                            <div style="font-size: 9px; color: #666; margin-top: 3px;">Example: 10SEG1234567890</div>
                        </div>
                        <div>
                            <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Altitude (m HAE)</label>
                            <input type="number" id="sensor_alt_mgrs" placeholder="10" value="10" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        </div>
                    </div>
                </div>

                <div id="gps_position_info" style="display: none;">
                    <div style="background: #0a0a0a; padding: 10px; border-radius: 3px; border: 1px solid #333; font-size: 11px;">
                        <div style="margin-bottom: 8px;">
                            <strong style="color: #0ff;">GPS Status:</strong>
                            <span id="streamout_gps_status" style="color: #f80;">Checking...</span>
                        </div>
                        <div style="margin-bottom: 8px;">
                            <strong style="color: #0ff;">Position:</strong><br>
                            <span id="streamout_gps_position" style="font-family: monospace; font-size: 10px;">--</span>
                        </div>
                        <div style="margin-bottom: 8px;">
                            <strong style="color: #0ff;">Satellites:</strong>
                            <span id="streamout_gps_sats">0</span>
                        </div>
                        <div style="font-size: 9px; color: #666; margin-top: 8px;">
                            Position will be pulled from GPS Monitor
                        </div>
                    </div>
                </div>
            </div>

            <!-- Stream Settings -->
            <div style="margin-bottom: 15px;">
                <h4 style="color: #0ff; font-size: 13px; margin: 0 0 10px 0; border-bottom: 1px solid #333; padding-bottom: 5px;">Stream Settings</h4>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Platform Type</label>
                    <select id="platform_type" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        <option value="ugv">UGV (Unmanned Ground Vehicle)</option>
                        <option value="uav-fixed">UAV Fixed-Wing</option>
                        <option value="uav-rotary">UAV Rotary-Wing</option>
                        <option value="usv">USV (Unmanned Surface Vehicle)</option>
                        <option value="ground-station">Ground Station</option>
                    </select>
                    <div style="font-size: 9px; color: #666; margin-top: 3px;">Determines the icon displayed in TAKX</div>
                </div>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">LoB Range (meters)</label>
                    <input type="number" id="lob_range" placeholder="10000" value="10000" min="1000" max="100000" step="1000" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                    <div style="font-size: 9px; color: #666; margin-top: 3px;">Line of bearing display length (default: 10km)</div>
                </div>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Update Rate</label>
                    <select id="stream_rate" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                        <option value="1000">1 Hz (Every 1 second)</option>
                        <option value="500" selected>2 Hz (Every 500ms)</option>
                        <option value="200">5 Hz (Every 200ms)</option>
                        <option value="100">10 Hz (Every 100ms)</option>
                    </select>
                </div>
                <div style="margin-bottom: 10px;">
                    <label style="display: block; font-size: 11px; color: #888; margin-bottom: 3px;">Sensor UID</label>
                    <input type="text" id="sensor_uid" placeholder="DF-SENSOR-001" value="DF-SENSOR-001" style="width: 100%; padding: 6px; background: #0a0a0a; border: 1px solid #444; color: #fff; border-radius: 3px; font-size: 11px;">
                    <div style="font-size: 9px; color: #666; margin-top: 3px;">Unique identifier for this DF sensor</div>
                </div>
            </div>

            <!-- Control Buttons -->
            <div style="display: flex; gap: 10px; justify-content: flex-end;">
                <button onclick="closeStreamOutConfig()" style="padding: 8px 16px; background: #3a0a0a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 11px;">
                    Cancel
                </button>
                <button onclick="toggleStreamOut()" id="stream_toggle_btn" style="padding: 8px 16px; background: #0a3a0a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px; font-size: 11px; font-weight: bold;">
                    Start Streaming
                </button>
            </div>
        </div>
    </div>

    <!-- GPS Monitor Panel -->
    <div id="gps_panel" class="draggable-panel" style="display: none; top: 100px; left: 20px; width: 350px;">
        <div class="panel-header">
            <span class="panel-title">📍 GPS Monitor</span>
            <div>
                <span class="panel-close" onclick="toggleGPS()">&times;</span>
            </div>
        </div>
        <div style="padding: 12px; font-size: 11px;">
            <!-- GPS Status Display -->
            <div style="background: #0a0a0a; padding: 10px; border-radius: 3px; border: 1px solid #333; margin-bottom: 10px;">
                <div style="margin-bottom: 6px;">
                    <strong style="color: #0ff;">Status:</strong>
                    <span id="gps_panel_status" style="color: #888;">Connecting...</span>
                </div>
                <div style="margin-bottom: 6px;">
                    <strong style="color: #0ff;">Position:</strong><br>
                    <span id="gps_panel_position" style="color: #888; font-family: monospace; font-size: 10px;">--</span>
                </div>
                <div style="margin-bottom: 6px;">
                    <strong style="color: #0ff;">Altitude:</strong>
                    <span id="gps_panel_altitude" style="color: #888;">-- m</span>
                </div>
                <div style="margin-bottom: 6px;">
                    <strong style="color: #0ff;">Satellites:</strong>
                    <span id="gps_panel_sats" style="color: #888;">0</span>
                </div>
                <div>
                    <strong style="color: #0ff;">HDOP:</strong>
                    <span id="gps_panel_hdop" style="color: #888;">--</span>
                </div>
            </div>

            <div style="font-size: 9px; color: #666;">
                Monitoring gpsd on localhost:2947
            </div>
            <button onclick="connectGPSD()" style="width: 100%; padding: 6px; margin-top: 8px; background: #0a3a0a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px; font-size: 10px;">
                🔄 Reconnect
            </button>
        </div>
    </div>

    <!-- Global draggable panels (outside workspaces) -->
    <div id="iq_constellation" class="draggable-panel" style="display: none; top: 100px; right: 20px; width: 450px;">
        <div class="panel-header">
            <span class="panel-title">IQ Constellation (Enhanced)</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('iq_constellation')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleIQ()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px;">
            <canvas id="iq_canvas" width="1720" height="1600" style="width: 430px; height: 400px;"></canvas>
        </div>
    </div>

    <div id="xcorr_display" class="draggable-panel" style="display: none; bottom: 20px; left: 100px; width: 750px; max-height: 500px;">
        <div class="panel-header">
            <span class="panel-title">Cross-Correlation Analysis (Enhanced)</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('xcorr_display')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleXCorr()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; overflow-y: auto; max-height: 450px;">
            <div style="display: flex; gap: 15px; margin-bottom: 8px; font-size: 10px; color: #888; flex-wrap: wrap;">
                <div>Coherence: <span id="xcorr_coherence" style="color: #0f0; font-weight: bold;">--</span></div>
                <div>Time Delay: <span id="xcorr_delay" style="color: #0ff; font-weight: bold;">--</span></div>
                <div>Phase Diff: <span id="xcorr_phase" style="color: #fa0; font-weight: bold;">--</span></div>
            </div>
            <div id="xcorr_canvas_container" style="width: 100%; height: 350px; position: relative;">
                <canvas id="xcorr_canvas" style="width: 100%; height: 100%;"></canvas>
            </div>
        </div>
    </div>

    <!-- Professional RF Measurements Panel -->
    <div id="signal_analysis" class="draggable-panel" style="display: none; top: 60px; left: 20px; width: 450px;">
        <div class="panel-header">
            <span class="panel-title">RF Measurements</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('signal_analysis')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleSignalAnalysis()">&times;</span>
            </div>
        </div>

        <!-- Tabbed Interface -->
        <div style="display: flex; background: #111; border-bottom: 1px solid #333;">
            <div class="meas-tab active" onclick="switchMeasTab('basic')" id="tab-basic">Basic</div>
            <div class="meas-tab" onclick="switchMeasTab('power')" id="tab-power">Power</div>
            <div class="meas-tab" onclick="switchMeasTab('spectral')" id="tab-spectral">Spectral</div>
            <div class="meas-tab" onclick="switchMeasTab('advanced')" id="tab-advanced">Advanced</div>
            <div class="meas-tab" onclick="switchMeasTab('mask')" id="tab-mask">Mask</div>
        </div>

        <div style="padding: 10px; font-size: 11px;">
            <!-- Basic Measurements Tab -->
            <div id="meas-content-basic" class="meas-content">
                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Power Measurements</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Peak Power:</span><span style="color: #0f0;" id="rf_peak_power">-- dBm</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Average Power:</span><span style="color: #0f0;" id="rf_avg_power">-- dBm</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>RMS Power:</span><span style="color: #0f0;" id="rf_rms_power">-- dBm</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Peak/Avg Ratio:</span><span style="color: #ff0;" id="rf_crest_factor">-- dB</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Bandwidth</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>OBW (99%):</span><span style="color: #0f0;" id="rf_obw_99">-- MHz</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>OBW (-3dB):</span><span style="color: #0f0;" id="rf_obw_3db">-- MHz</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>OBW (-20dB):</span><span style="color: #0f0;" id="rf_obw_20db">-- MHz</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Center Freq:</span><span style="color: #ff0;" id="rf_center_measured">-- MHz</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Noise & Quality</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Noise Floor:</span><span style="color: #0f0;" id="rf_noise_floor">-- dBm</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>SNR:</span><span style="color: #0f0;" id="rf_snr">-- dB</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>SINAD:</span><span style="color: #0f0;" id="rf_sinad">-- dB</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>SFDR:</span><span style="color: #0f0;" id="rf_sfdr">-- dBc</span>
                        </div>
                    </div>
                </div>

                <button onclick="runBasicMeasurements()" style="padding: 6px 12px; width: 100%; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ▶ Run Measurements
                </button>
            </div>

            <!-- Channel Power Tab -->
            <div id="meas-content-power" class="meas-content" style="display: none;">
                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Channel Power</strong>
                    <div style="margin-top: 8px;">
                        <label>Integration BW (MHz):</label>
                        <input type="number" id="chan_bw" value="1.0" step="0.1" style="width: 80px; margin-left: 5px;">
                    </div>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Channel Power:</span><span style="color: #0f0;" id="rf_chan_power">-- dBm</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Power Density:</span><span style="color: #0f0;" id="rf_power_density">-- dBm/Hz</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">ACPR (Adjacent Channel Power)</strong>
                    <div style="margin-top: 8px;">
                        <label>Offset (MHz):</label>
                        <input type="number" id="acpr_offset" value="1.5" step="0.1" style="width: 80px; margin-left: 5px;">
                    </div>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Lower ACPR:</span><span style="color: #f90;" id="rf_acpr_lower">-- dBc</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Upper ACPR:</span><span style="color: #f90;" id="rf_acpr_upper">-- dBc</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Alt Lower:</span><span style="color: #888;" id="rf_aclr_lower">-- dBc</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Alt Upper:</span><span style="color: #888;" id="rf_aclr_upper">-- dBc</span>
                        </div>
                    </div>
                </div>

                <button onclick="runChannelPowerMeasurements()" style="padding: 6px 12px; width: 100%; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ▶ Measure Channel Power
                </button>
            </div>

            <!-- Spectral Analysis Tab -->
            <div id="meas-content-spectral" class="meas-content" style="display: none;">
                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Spectral Characteristics</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Spectral Flatness:</span><span style="color: #0f0;" id="rf_spectral_flatness">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Spectral Entropy:</span><span style="color: #0f0;" id="rf_spectral_entropy">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Spectral Kurtosis:</span><span style="color: #0f0;" id="rf_spectral_kurtosis">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>RBW (effective):</span><span style="color: #ff0;" id="rf_rbw">-- kHz</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Peak Analysis</strong>
                    <div style="margin-top: 5px;">
                        <label>Threshold:</label>
                        <input type="number" id="peak_threshold_pro" value="-60" step="5" style="width: 60px; margin-left: 5px;">
                        <label style="margin-left: 10px;">Excursion:</label>
                        <input type="number" id="peak_excursion" value="10" step="1" style="width: 50px; margin-left: 5px;">
                        <span style="color: #888; font-size: 9px;">dB</span>
                    </div>
                    <div id="peak_table_pro" style="margin-top: 8px; max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 9px; background: #0a0a0a; padding: 5px; border-radius: 3px;">
                        <table style="width: 100%; border-collapse: collapse;">
                            <thead style="color: #0ff; border-bottom: 1px solid #333;">
                                <tr><th>#</th><th>Freq (MHz)</th><th>Power (dBm)</th><th>Δf (kHz)</th></tr>
                            </thead>
                            <tbody id="peak_table_body"></tbody>
                        </table>
                    </div>
                </div>

                <button onclick="runSpectralAnalysis()" style="padding: 6px 12px; width: 100%; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ▶ Analyze Spectrum
                </button>
            </div>

            <!-- Advanced Analysis Tab -->
            <div id="meas-content-advanced" class="meas-content" style="display: none;">
                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Burst Detection</strong>
                    <div style="margin-top: 8px;">
                        <label>Threshold (dB):</label>
                        <input type="number" id="burst_threshold" value="-70" step="5" style="width: 70px; margin-left: 5px;">
                        <label style="margin-left: 10px;">Min Duration (ms):</label>
                        <input type="number" id="burst_min_duration" value="10" step="5" style="width: 60px; margin-left: 5px;">
                    </div>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Bursts Detected:</span><span style="color: #0f0;" id="burst_count">0</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Avg Duration:</span><span style="color: #0f0;" id="burst_avg_duration">-- ms</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Burst Rate:</span><span style="color: #0f0;" id="burst_rate">-- Hz</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Frequency Hopping</strong>
                    <div style="margin-top: 5px;">
                        <label style="font-size: 10px;">
                            <input type="checkbox" id="fh_detect_enable"> Enable Detection
                        </label>
                        <label style="margin-left: 10px; font-size: 10px;">
                            History: <input type="number" id="fh_history" value="100" step="10" style="width: 50px;"> frames
                        </label>
                    </div>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Hop Rate:</span><span style="color: #0f0;" id="fh_hop_rate">-- hops/sec</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Channels Used:</span><span style="color: #0f0;" id="fh_channels">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Pattern Type:</span><span style="color: #ff0;" id="fh_pattern">--</span>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Interference Analysis</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Narrowband Interf:</span><span style="color: #f90;" id="interf_narrowband">0</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Wideband Interf:</span><span style="color: #f90;" id="interf_wideband">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Duty Cycle:</span><span style="color: #0f0;" id="interf_duty_cycle">--%</span>
                        </div>
                    </div>
                </div>

                <button onclick="runAdvancedAnalysis()" style="padding: 6px 12px; width: 100%; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    Run Advanced Analysis
                </button>
            </div>

            <!-- Spectrum Mask Testing Tab -->
            <div id="meas-content-mask" class="meas-content" style="display: none;">
                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Mask Template</strong>
                    <div style="margin-top: 8px;">
                        <select id="mask_template" onchange="loadMaskTemplate()" style="width: 100%; margin-bottom: 8px;">
                            <option value="custom">Custom</option>
                            <option value="fcc_part15">FCC Part 15 (ISM 915)</option>
                            <option value="etsi_300220">ETSI 300 220 (868 MHz)</option>
                            <option value="wifi_2ghz">WiFi 2.4 GHz</option>
                            <option value="lte_20mhz">LTE 20 MHz</option>
                            <option value="bluetooth">Bluetooth LE</option>
                        </select>
                        <div style="display: flex; gap: 5px;">
                            <button onclick="createMaskPoint()" style="flex: 1; padding: 4px; font-size: 10px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 2px;">
                                + Point
                            </button>
                            <button onclick="clearMask()" style="flex: 1; padding: 4px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                                Clear
                            </button>
                        </div>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Mask Points</strong>
                    <div id="mask_points_table" style="max-height: 200px; overflow-y: auto; margin-top: 5px; font-family: monospace; font-size: 9px; background: #0a0a0a; padding: 5px; border-radius: 3px;">
                        <table style="width: 100%; border-collapse: collapse;">
                            <thead style="color: #0ff; border-bottom: 1px solid #333;">
                                <tr>
                                    <th style="text-align: left;">Freq (MHz)</th>
                                    <th style="text-align: right;">Level (dBm)</th>
                                    <th style="text-align: center;">Del</th>
                                </tr>
                            </thead>
                            <tbody id="mask_points_body">
                                <tr><td colspan="3" style="text-align: center; color: #888; padding: 10px;">No mask defined</td></tr>
                            </tbody>
                        </table>
                    </div>
                </div>

                <div style="margin-bottom: 12px;">
                    <strong style="color: #0ff; font-size: 12px;">Test Results</strong>
                    <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Status:</span><span id="mask_status" style="color: #888;">Not tested</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Violations:</span><span id="mask_violations" style="color: #f90;">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Max Margin:</span><span id="mask_max_margin" style="color: #0f0;">-- dB</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                            <span>Min Margin:</span><span id="mask_min_margin" style="color: #f90;">-- dB</span>
                        </div>
                    </div>
                </div>

                <div style="display: flex; gap: 5px;">
                    <button onclick="testMask()" style="flex: 1; padding: 6px 12px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                        ▶ Test Mask
                    </button>
                    <button onclick="toggleMaskOverlay()" style="flex: 1; padding: 6px 12px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px;">
                        Show/Hide
                    </button>
                </div>
            </div>

            <!-- Persistence & Trace Options (always visible at bottom) -->
            <div style="margin-top: 15px; padding-top: 12px; border-top: 2px solid #333;">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;">
                    <strong style="color: #0ff; font-size: 11px;">Trace Mode:</strong>
                    <select id="persistence_mode" onchange="changePersistenceMode()" style="width: 150px; font-size: 10px;">
                        <option value="none">Live</option>
                        <option value="max">Max Hold</option>
                        <option value="min">Min Hold</option>
                        <option value="avg">Average</option>
                        <option value="decay">Decay</option>
                    </select>
                </div>
                <div style="display: flex; gap: 5px;">
                    <button onclick="saveReferenceTrace()" style="flex: 1; padding: 4px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        Save Ref
                    </button>
                    <button onclick="clearReferenceTrace()" style="flex: 1; padding: 4px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        Clear Ref
                    </button>
                    <button onclick="resetPersistence()" style="flex: 1; padding: 4px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        Reset
                    </button>
                </div>
            </div>
        </div>
    </div>

    <!-- Demodulation Panel -->
    <div id="demod_panel" class="draggable-panel" style="display: none; top: 100px; right: 20px; width: 300px;">
        <div class="panel-header">
            <span class="panel-title">Demodulator</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('demod_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleDemod()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff;">Mode:</strong>
                <select id="demod_mode" onchange="changeDemodMode()" style="width: 100%; margin-top: 5px;">
                    <option value="off">Off</option>
                    <option value="am">AM</option>
                    <option value="fm">FM (NFM)</option>
                    <option value="wfm">FM (WFM)</option>
                    <option value="usb">USB</option>
                    <option value="lsb">LSB</option>
                    <option value="fsk">FSK</option>
                    <option value="psk">PSK (BPSK)</option>
                </select>
            </div>
            <div style="margin-bottom: 10px;">
                <label>Center Freq:</label>
                <input type="number" id="demod_freq" value="0" step="0.001" style="width: 100%; margin-top: 3px;" placeholder="Offset from CF (MHz)">
            </div>
            <div style="margin-bottom: 10px;">
                <label>Bandwidth:</label>
                <input type="number" id="demod_bw" value="15" step="1" style="width: 100%; margin-top: 3px;" placeholder="kHz">
            </div>
            <div style="margin-bottom: 10px;">
                <label>Volume:</label>
                <input type="range" id="demod_volume" min="0" max="100" value="50" style="width: 100%;">
            </div>
            <div style="margin-bottom: 10px;">
                <button id="demod_start_btn" onclick="startDemod()" style="width: 100%; padding: 5px;">Start Demodulation</button>
            </div>
            <div id="demod_status" style="margin-top: 5px; padding: 5px; background: #111; border-radius: 3px; font-size: 10px; font-family: monospace;">
                Status: Stopped
            </div>
            <div id="demod_decode" style="margin-top: 10px; padding: 5px; background: #111; border-radius: 3px; font-size: 10px; font-family: monospace; max-height: 150px; overflow-y: auto; display: none;">
                <strong>Decoded Data:</strong>
                <div id="demod_output" style="margin-top: 5px; color: #0f0;"></div>
            </div>
        </div>
    </div>

    <!-- Signal Tracker Panel (Phase 3) -->
    <div id="signal_tracker_panel" class="draggable-panel" style="display: none; top: 100px; right: 340px; width: 320px;">
        <div class="panel-header">
            <span class="panel-title">Signal Tracker</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('signal_tracker_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleSignalTracker()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 10px;">
            <div style="margin-bottom: 10px; padding: 8px; background: #0a0a0a; border-radius: 3px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">ACTIVE SIGNALS</div>
                <div style="color: #888;">
                    <div>Tracked: <span style="color: #0f0;" id="tracker_count">0</span> / 20</div>
                    <div>Threshold: <span style="color: #ff0;" id="tracker_threshold">-80</span> dBm</div>
                </div>
            </div>

            <div style="margin-bottom: 10px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">HOPPING DETECTION</div>
                <div id="hopping_status" style="padding: 5px; background: #0a0a0a; border-radius: 3px; font-family: monospace; font-size: 9px;">
                    <div style="color: #888;">No hopping detected</div>
                </div>
            </div>

            <div style="max-height: 250px; overflow-y: auto; background: #0a0a0a; border-radius: 3px; padding: 5px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px; font-size: 9px;">SIGNAL LIST</div>
                <div id="signal_list" style="font-family: monospace; font-size: 8px; color: #888;">
                    No signals detected
                </div>
            </div>

            <div style="margin-top: 10px;">
                <button onclick="clearSignalTracker()" style="width: 100%; padding: 5px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px;">Clear Tracker</button>
            </div>
        </div>
    </div>

    <!-- Interference Analysis Panel (Phase 3) -->
    <div id="interference_panel" class="draggable-panel" style="display: none; top: 100px; right: 680px; width: 340px;">
        <div class="panel-header">
            <span class="panel-title">Interference Analysis</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('interference_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleInterference()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 10px;">
            <div style="margin-bottom: 10px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">HARMONICS</div>
                <div id="harmonics_list" style="max-height: 120px; overflow-y: auto; padding: 5px; background: #0a0a0a; border-radius: 3px; font-family: monospace; font-size: 9px; color: #888;">
                    No harmonics detected
                </div>
            </div>

            <div style="margin-bottom: 10px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">INTERMODULATION (IMD)</div>
                <div id="imd_list" style="max-height: 120px; overflow-y: auto; padding: 5px; background: #0a0a0a; border-radius: 3px; font-family: monospace; font-size: 9px; color: #888;">
                    No IMD products detected
                </div>
            </div>

            <div style="margin-bottom: 10px;">
                <div style="color: #ff0; font-weight: bold; margin-bottom: 5px;">RECOMMENDATIONS</div>
                <div id="interference_recommendations" style="max-height: 100px; overflow-y: auto; padding: 5px; background: #0a0a0a; border-radius: 3px; font-size: 9px; color: #888;">
                    No interference detected
                </div>
            </div>

            <div style="margin-top: 10px; padding: 8px; background: #0a0a0a; border-radius: 3px;">
                <div style="color: #888; font-size: 9px;">
                    <div>Analysis updates every 500ms</div>
                    <div>Checks up to 10th harmonic</div>
                    <div>Detects 3rd order IMD products</div>
                </div>
            </div>
        </div>
    </div>

    <!-- Protocol Decoder Output Panel (Phase 3) -->
    <div id="decoder_panel" class="draggable-panel" style="display: none; top: 100px; right: 1040px; width: 360px;">
        <div class="panel-header">
            <span class="panel-title">Protocol Decoder</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('decoder_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleDecoder()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 10px; display: flex; flex-direction: column; height: calc(100% - 40px);">
            <div style="margin-bottom: 10px; padding: 8px; background: #0a0a0a; border-radius: 3px;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">ACTIVE DECODERS</div>
                <div style="color: #888; font-size: 9px;">
                    <div>ADS-B: <span style="color: #ff0;" id="adsb_status">Inactive</span> (Enable PSK mode)</div>
                    <div>AIS: <span style="color: #ff0;" id="ais_status">Inactive</span> (Enable FSK mode)</div>
                </div>
            </div>

            <div style="flex: 1; display: flex; flex-direction: column; overflow: hidden;">
                <div style="color: #0ff; font-weight: bold; margin-bottom: 5px;">DECODED MESSAGES</div>
                <div id="decoded_messages" style="flex: 1; padding: 5px; background: #0a0a0a; border-radius: 3px; font-family: monospace; font-size: 8px; overflow-y: auto; color: #888;">
                    <div style="text-align: center; margin-top: 20px;">No messages decoded yet</div>
                </div>
            </div>

            <div style="margin-top: 10px;">
                <button onclick="clearDecodedMessages()" style="width: 100%; padding: 5px; font-size: 10px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px;">Clear Messages</button>
            </div>
        </div>
    </div>

    <!-- Signal Recorder Panel -->
    <div id="recorder_panel" class="draggable-panel" style="display: none; top: 100px; left: 20px; width: 380px;">
        <div class="panel-header">
            <span class="panel-title">Signal Recorder</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('recorder_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleRecorder()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="margin-bottom: 12px;">
                <strong style="color: #0ff; font-size: 12px;">Recording Settings</strong>
                <div style="margin-top: 8px;">
                    <div style="margin-bottom: 8px;">
                        <label>Format:</label>
                        <select id="record_format" style="width: 100%; margin-top: 3px;">
                            <option value="iq_int16">IQ Int16 (Raw)</option>
                            <option value="iq_float32">IQ Float32</option>
                            <option value="wav">WAV (Audio)</option>
                            <option value="sigmf">SigMF (Metadata)</option>
                        </select>
                    </div>
                    <div style="margin-bottom: 8px;">
                        <label>Duration (sec):</label>
                        <input type="number" id="record_duration" value="10" min="1" max="3600" style="width: 100%; margin-top: 3px;">
                    </div>
                    <div style="margin-bottom: 8px;">
                        <label>Trigger:</label>
                        <select id="record_trigger" style="width: 100%; margin-top: 3px;">
                            <option value="manual">Manual</option>
                            <option value="level">Power Level</option>
                            <option value="edge">Rising Edge</option>
                            <option value="schedule">Scheduled</option>
                        </select>
                    </div>
                    <div id="record_trigger_level" style="margin-bottom: 8px; display: none;">
                        <label>Threshold (dBm):</label>
                        <input type="number" id="record_threshold" value="-60" step="1" style="width: 100%; margin-top: 3px;">
                    </div>
                </div>
            </div>

            <div style="margin-bottom: 12px;">
                <strong style="color: #0ff; font-size: 12px;">Status</strong>
                <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                    <div>State: <span id="record_state" style="color: #888;">Idle</span></div>
                    <div>Samples: <span id="record_samples" style="color: #0f0;">0</span></div>
                    <div>Size: <span id="record_size" style="color: #0f0;">0 MB</span></div>
                    <div>File: <span id="record_filename" style="color: #888;">--</span></div>
                </div>
            </div>

            <div style="display: flex; gap: 5px; margin-bottom: 10px;">
                <button onclick="startRecording()" style="flex: 1; padding: 6px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ● Record
                </button>
                <button onclick="stopRecording()" style="flex: 1; padding: 6px; background: #3a0a0a; border: 1px solid #f00; color: #f00; cursor: pointer; border-radius: 3px;">
                    ■ Stop
                </button>
            </div>

            <div style="margin-bottom: 12px; padding-top: 10px; border-top: 1px solid #333;">
                <strong style="color: #0ff; font-size: 12px;">Recordings</strong>
                <div id="recordings_list" style="max-height: 150px; overflow-y: auto; margin-top: 5px; font-family: monospace; font-size: 9px; background: #0a0a0a; padding: 5px; border-radius: 3px;">
                    <div style="text-align: center; color: #888; padding: 20px;">No recordings</div>
                </div>
            </div>
        </div>
    </div>

    <!-- Direction Finding (DoA) Panel -->
    <div id="doa_panel" class="draggable-panel" style="display: none; top: 100px; right: 20px; width: 400px;">
        <div class="panel-header">
            <span class="panel-title">Direction Finding</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('doa_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleDoA()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff; font-size: 12px;">Coherent Dual-Channel DoA</strong>
                <div style="margin-top: 5px; font-size: 9px; color: #888; line-height: 1.4;">
                    Uses phase difference between CH1 and CH2 to estimate signal direction of arrival
                </div>
            </div>

            <div style="margin-bottom: 12px;">
                <strong style="color: #0ff; font-size: 11px;">Array Configuration</strong>
                <div style="margin-top: 5px;">
                    <label>Element Spacing (λ):</label>
                    <input type="number" id="doa_spacing" value="0.5" step="0.1" min="0.1" max="2.0" style="width: 100%; margin-top: 3px;">
                    <div style="font-size: 9px; color: #888; margin-top: 2px;">0.5λ = ~164mm @ 915 MHz</div>
                </div>
                <div style="margin-top: 8px;">
                    <label>Method:</label>
                    <select id="doa_method" style="width: 100%; margin-top: 3px;">
                        <option value="phase">Phase Difference (2-Ch Interferometry)</option>
                    </select>
                </div>
            </div>

            <!-- Polar DoA Display -->
            <div style="margin-bottom: 12px;">
                <strong style="color: #0ff; font-size: 11px;">Angle of Arrival</strong>
                <canvas id="doa_polar" width="360" height="200" style="background: #0a0a0a; border: 1px solid #333; border-radius: 3px; margin-top: 5px;"></canvas>
            </div>

            <div style="margin-bottom: 12px;">
                <strong style="color: #0ff; font-size: 11px;">Results</strong>
                <div style="margin-top: 5px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Azimuth:</span><span id="doa_azimuth" style="color: #0f0;">-- deg</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Phase Diff:</span><span id="doa_phase" style="color: #0f0;">-- deg</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Confidence:</span><span id="doa_confidence" style="color: #ff0;">--%</span>
                    </div>
                    <div style="display: flex; justify-content: space-between; margin: 3px 0;">
                        <span>Coherence:</span><span id="doa_coherence" style="color: #0f0;">--</span>
                    </div>
                </div>
            </div>

            <div style="display: flex; gap: 5px;">
                <button onclick="updateDoA()" style="flex: 1; padding: 6px 12px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ▶ Update
                </button>
                <button onclick="calibrateDoA()" style="flex: 1; padding: 6px 12px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px;">
                    Calibrate
                </button>
            </div>
        </div>
    </div>

    <!-- Signal Activity Timeline -->
    <div id="activity_timeline" class="draggable-panel" style="display: none; bottom: 20px; right: 20px; width: 500px;">
        <div class="panel-header">
            <span class="panel-title">Signal Activity Timeline</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('activity_timeline')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" onclick="toggleActivityTimeline()">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <canvas id="timeline_canvas" width="480" height="150" style="margin-bottom: 10px;"></canvas>
            <div>
                <button onclick="clearActivityHistory()" style="padding: 3px 8px;">Clear History</button>
                <span style="margin-left: 10px; color: #888;">Tracking: <span id="activity_duration">0</span>s</span>
            </div>
        </div>
    </div>

    <!-- Professional Marker System -->
    <div id="marker_panel" class="draggable-panel" style="display: none; top: 60px; right: 20px; width: 400px;">
        <div class="panel-header">
            <span class="panel-title">Marker System</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('marker_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" id="marker_panel_close">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="display: flex; gap: 5px; margin-bottom: 10px;">
                <button onclick="addMarker()" style="flex: 1; padding: 5px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px; font-size: 10px;">
                    + Add Marker
                </button>
                <button onclick="clearAllMarkers()" style="flex: 1; padding: 5px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 10px;">
                    Clear All
                </button>
            </div>

            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff; font-size: 11px;">Quick Actions:</strong>
                <div style="display: flex; gap: 5px; margin-top: 5px; flex-wrap: wrap;">
                    <button onclick="markerToPeak()" style="padding: 3px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        → Peak
                    </button>
                    <button onclick="markerToCenter()" style="padding: 3px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        → Center
                    </button>
                    <button onclick="markNextPeak()" style="padding: 3px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        Next Peak
                    </button>
                    <button onclick="exportMarkers()" style="padding: 3px 8px; font-size: 9px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 2px;">
                        Export CSV
                    </button>
                </div>
            </div>

            <div id="marker_table" style="max-height: 400px; overflow-y: auto; font-family: monospace; font-size: 9px;">
                <table style="width: 100%; border-collapse: collapse; background: #0a0a0a;">
                    <thead style="color: #0ff; background: #111; position: sticky; top: 0;">
                        <tr>
                            <th style="padding: 5px; text-align: left;">M</th>
                            <th style="padding: 5px; text-align: right;">Frequency</th>
                            <th style="padding: 5px; text-align: right;">Power</th>
                            <th style="padding: 5px; text-align: right;">Delta</th>
                            <th style="padding: 5px; text-align: center;">Act</th>
                        </tr>
                    </thead>
                    <tbody id="marker_table_body">
                        <tr><td colspan="5" style="text-align: center; color: #888; padding: 20px;">No markers</td></tr>
                    </tbody>
                </table>
            </div>

            <div style="margin-top: 10px; padding-top: 10px; border-top: 1px solid #333;">
                <strong style="color: #0ff; font-size: 11px;">Delta Measurements:</strong>
                <div style="margin-top: 5px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                    <div>Reference: <span style="color: #ff0;" id="delta_ref_marker">None</span></div>
                    <div>Δf: <span style="color: #0f0;" id="delta_freq">-- kHz</span></div>
                    <div>ΔPower: <span style="color: #0f0;" id="delta_power">-- dB</span></div>
                </div>
            </div>
        </div>
    </div>

    <!-- Vector Signal Analyzer Panel -->
    <div id="vsa_panel" class="draggable-panel" style="display: none; bottom: 60px; right: 20px; width: 500px;">
        <div class="panel-header">
            <span class="panel-title">Vector Signal Analysis</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('vsa_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" id="vsa_panel_close">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                <!-- Constellation Diagram -->
                <div>
                    <strong style="color: #0ff; font-size: 11px;">IQ Constellation</strong>
                    <canvas id="constellation_canvas" width="220" height="220" style="background: #0a0a0a; border: 1px solid #333; border-radius: 3px; margin-top: 5px;"></canvas>
                </div>

                <!-- EVM & Quality Metrics -->
                <div>
                    <strong style="color: #0ff; font-size: 11px;">Quality Metrics</strong>
                    <div style="margin-top: 5px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px; height: 220px;">
                        <div style="margin: 5px 0;">EVM RMS: <span style="color: #0f0;" id="vsa_evm_rms">--%</span></div>
                        <div style="margin: 5px 0;">EVM Peak: <span style="color: #f90;" id="vsa_evm_peak">--%</span></div>
                        <div style="margin: 5px 0;">Mag Error: <span style="color: #0f0;" id="vsa_mag_error">-- dB</span></div>
                        <div style="margin: 5px 0;">Phase Error: <span style="color: #0f0;" id="vsa_phase_error">-- deg</span></div>
                        <div style="margin: 5px 0; padding-top: 5px; border-top: 1px solid #222;">IQ Offset: <span style="color: #ff0;" id="vsa_iq_offset">-- dB</span></div>
                        <div style="margin: 5px 0;">Quad Error: <span style="color: #ff0;" id="vsa_quad_error">-- deg</span></div>
                        <div style="margin: 5px 0;">Gain Imbal: <span style="color: #ff0;" id="vsa_gain_imbal">-- dB</span></div>
                    </div>
                </div>
            </div>

            <!-- Eye Diagram -->
            <div style="margin-top: 10px;">
                <strong style="color: #0ff; font-size: 11px;">Eye Diagram</strong>
                <canvas id="eye_diagram_canvas" width="480" height="150" style="background: #0a0a0a; border: 1px solid #333; border-radius: 3px; margin-top: 5px;"></canvas>
            </div>

            <div style="margin-top: 10px;">
                <button onclick="updateVSA()" style="padding: 6px 12px; width: 100%; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px;">
                    ▶ Update VSA
                </button>
            </div>
        </div>
    </div>

    <!-- Statistics & CCDF Panel -->
    <div id="stats_panel" class="draggable-panel" style="display: none; bottom: 60px; left: 20px; width: 450px;">
        <div class="panel-header">
            <span class="panel-title">Statistics & CCDF</span>
            <div>
                <span class="panel-detach" onclick="detachPanel('stats_panel')" title="Detach to floating">&#8599;</span>
                <span class="panel-close" id="stats_panel_close">&times;</span>
            </div>
        </div>
        <div style="padding: 10px; font-size: 11px;">
            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff; font-size: 12px;">Power Statistics</strong>
                <div style="margin-top: 8px; font-family: monospace; font-size: 10px; background: #0a0a0a; padding: 8px; border-radius: 3px;">
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 5px;">
                        <div>Mean: <span style="color: #0f0;" id="stats_mean">-- dBm</span></div>
                        <div>Median: <span style="color: #0f0;" id="stats_median">-- dBm</span></div>
                        <div>Std Dev: <span style="color: #ff0;" id="stats_stddev">-- dB</span></div>
                        <div>Variance: <span style="color: #ff0;" id="stats_variance">--</span></div>
                        <div>Skewness: <span style="color: #0f0;" id="stats_skew">--</span></div>
                        <div>Kurtosis: <span style="color: #0f0;" id="stats_kurt">--</span></div>
                    </div>
                </div>
            </div>

            <!-- CCDF Plot -->
            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff; font-size: 12px;">CCDF (Complementary CDF)</strong>
                <canvas id="ccdf_canvas" width="430" height="200" style="background: #0a0a0a; border: 1px solid #333; border-radius: 3px; margin-top: 5px;"></canvas>
            </div>

            <!-- Histogram -->
            <div style="margin-bottom: 10px;">
                <strong style="color: #0ff; font-size: 12px;">Power Histogram</strong>
                <canvas id="histogram_canvas" width="430" height="150" style="background: #0a0a0a; border: 1px solid #333; border-radius: 3px; margin-top: 5px;"></canvas>
            </div>

            <div style="display: flex; gap: 5px;">
                <button onclick="updateStatistics()" style="flex: 1; padding: 6px 12px; background: #0a3a3a; border: 1px solid #0ff; color: #0ff; cursor: pointer; border-radius: 3px; font-size: 10px;">
                    ▶ Update Stats
                </button>
                <button onclick="resetStatistics()" style="flex: 1; padding: 6px 12px; background: #1a1a1a; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 3px; font-size: 10px;">
                    Reset
                </button>
            </div>
        </div>
    </div>

    <!-- Measurement Cursors Canvas Overlay -->
    <canvas id="cursor_overlay" style="position: fixed; left: 0; top: 0; width: 100%; height: 100%; pointer-events: none; z-index: 500;"></canvas>

    <!-- Notification Container -->
    <div id="notificationContainer" class="notification-container"></div>

    <!-- Loading Overlay -->
    <div id="loadingOverlay" class="loading-overlay">
        <div class="loading-spinner"></div>
    </div>

    <!-- Global UI elements (outside workspaces) -->
    <div class="fps" id="fps">-- FPS</div>
    <div class="resolution-info" id="resolution">--</div>

    <div class="toggle-controls" id="toggleControls" onclick="toggleControlPanel()">Controls</div>
    <div class="control-panel" id="controlPanel" style="display: none;">
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px;">
            <h3 style="margin: 0;">RF Parameters</h3>
            <span onclick="toggleControlPanel()" style="cursor: pointer; color: #f00; font-weight: bold; font-size: 16px; padding: 0 5px;" title="Close">X</span>
        </div>
        <div class="control-group">
            <label>Frequency (MHz)</label>
            <div class="input-row">
                <input type="number" id="freqInput" step="0.01" min="47" max="6000" value="915.00" title="Center frequency in MHz (47-6000)">
                <button onclick="applyFrequency()" title="Apply frequency">Set</button>
            </div>
        </div>
        <div class="control-group">
            <label>Frequency Presets</label>
            <select id="freqPresets" onchange="applyFrequencyPreset()" title="Quick access to common frequency bands">
                <option value="">-- Select Preset --</option>
                <optgroup label="ISM Bands">
                    <option value="315.0">315 MHz ISM</option>
                    <option value="433.92">433.92 MHz ISM</option>
                    <option value="868.0">868 MHz ISM (EU)</option>
                    <option value="915.0">915 MHz ISM (US)</option>
                    <option value="2450.0">2.4 GHz ISM</option>
                    <option value="5800.0">5.8 GHz ISM</option>
                </optgroup>
                <optgroup label="Radio Services">
                    <option value="155.0">155 MHz Marine VHF</option>
                    <option value="162.5">162.5 MHz NOAA Weather</option>
                    <option value="462.5">462.5 MHz FRS/GMRS</option>
                    <option value="850.0">850 MHz Cellular</option>
                    <option value="1900.0">1900 MHz PCS</option>
                    <option value="1090.0">1090 MHz ADS-B</option>
                </optgroup>
                <optgroup label="Ham Bands">
                    <option value="144.0">144 MHz (2m)</option>
                    <option value="220.0">220 MHz (1.25m)</option>
                    <option value="440.0">440 MHz (70cm)</option>
                    <option value="1296.0">1296 MHz (23cm)</option>
                    <option value="2320.0">2320 MHz (13cm)</option>
                </optgroup>
                <optgroup label="WiFi">
                    <option value="2412.0">WiFi Ch 1 (2412 MHz)</option>
                    <option value="2437.0">WiFi Ch 6 (2437 MHz)</option>
                    <option value="2462.0">WiFi Ch 11 (2462 MHz)</option>
                    <option value="5180.0">WiFi Ch 36 (5180 MHz)</option>
                    <option value="5500.0">WiFi Ch 100 (5500 MHz)</option>
                    <option value="5745.0">WiFi Ch 149 (5745 MHz)</option>
                </optgroup>
            </select>
        </div>
        <div class="control-group">
            <label>Sample Rate (MHz)</label>
            <div class="input-row">
                <input type="number" id="srInput" step="0.1" min="0.52" max="61.44" value="40.0">
                <button onclick="applySampleRate()">Set</button>
            </div>
        </div>
        <div class="control-group">
            <label>Bandwidth (MHz)</label>
            <div class="input-row">
                <input type="number" id="bwInput" step="0.1" min="0.52" max="61.44" value="40.0">
                <button onclick="applyBandwidth()">Set</button>
            </div>
        </div>
        <div class="control-group">
            <label>Gain RX1 (dB)</label>
            <div class="input-row">
                <input type="number" id="gain1Input" step="1" min="0" max="60" value="40" title="RF gain for receiver channel 1 (0-60 dB)">
                <button onclick="applyGain1()" title="Apply gain to RX1">Set</button>
            </div>
        </div>
        <div class="control-group">
            <label>Gain RX2 (dB)</label>
            <div class="input-row">
                <input type="number" id="gain2Input" step="1" min="0" max="60" value="40" title="RF gain for receiver channel 2 (0-60 dB)">
                <button onclick="applyGain2()" title="Apply gain to RX2">Set</button>
            </div>
        </div>
        <h3 style="margin-top: 15px; border-top: 1px solid #333; padding-top: 15px;">Display Settings</h3>
        <div class="control-group">
            <label>Quality Profile</label>
            <select id="qualityProfile" onchange="applyQualityProfile()">
                <option value="high">High (Full Quality)</option>
                <option value="medium">Medium (Balanced)</option>
                <option value="low">Low (Tactical Link)</option>
                <option value="custom">Custom</option>
            </select>
        </div>
        <div class="control-group">
            <label>Color Palette</label>
            <select id="colorPalette" onchange="changeColorPalette()">
                <option value="viridis">Viridis</option>
                <option value="plasma">Plasma</option>
                <option value="inferno">Inferno</option>
                <option value="turbo">Turbo</option>
                <option value="hot">Hot</option>
                <option value="cool">Cool</option>
                <option value="grayscale">Grayscale</option>
                <option value="rainbow">Rainbow</option>
            </select>
        </div>
        <div class="control-group">
            <label>Waterfall Intensity</label>
            <div class="input-row">
                <input type="range" id="waterfallIntensity" min="0.5" max="2" step="0.1" value="1.0" oninput="updateWaterfallIntensity(this.value)">
                <span id="intensityValue" class="range-value">1.0x</span>
            </div>
        </div>
        <div class="control-group">
            <label>Waterfall Contrast</label>
            <div class="input-row">
                <input type="range" id="waterfallContrast" min="0.5" max="2" step="0.1" value="1.0" oninput="updateWaterfallContrast(this.value)">
                <span id="contrastValue" class="range-value">1.0x</span>
            </div>
        </div>
        <div class="control-group">
            <label>Spectrum Min (dB)</label>
            <div class="input-row">
                <input type="number" id="spectrumMin" step="10" min="-120" max="-20" value="-100">
                <button onclick="updateSpectrumRange()">Set</button>
            </div>
        </div>
        <div class="control-group">
            <label>Spectrum Max (dB)</label>
            <div class="input-row">
                <input type="number" id="spectrumMax" step="10" min="-60" max="20" value="-10">
                <button onclick="updateSpectrumRange()">Set</button>
            </div>
        </div>
        <div class="control-group">
            <div class="input-row">
                <label class="checkbox-label">
                    <input type="checkbox" id="peakHoldCheckbox" onchange="togglePeakHold(this.checked)" title="Hold maximum spectrum values">
                    Peak Hold
                </label>
                <button onclick="clearPeakHold()" title="Clear peak hold data">Clear</button>
            </div>
        </div>
        <div class="control-group">
            <div class="input-row">
                <label class="checkbox-label">
                    <input type="checkbox" id="minHoldCheckbox" onchange="toggleMinHold(this.checked)" title="Hold minimum spectrum values">
                    Min Hold
                </label>
                <button onclick="clearMinHold()" title="Clear min hold data">Clear</button>
            </div>
        </div>
        <div class="control-group">
            <label class="checkbox-label">
                <input type="checkbox" id="refMarkersCheckbox" onchange="toggleRefMarkers(this.checked)" title="Show reference level markers">
                Reference Markers
            </label>
        </div>
        <div class="control-group">
            <div class="input-row">
                <label class="checkbox-label">
                    <input type="checkbox" id="bwMeasureCheckbox" onchange="toggleBandwidthMeasure(this.checked)" title="Measure bandwidth between two points">
                    Bandwidth Measure
                </label>
                <span id="bw_measurement" class="range-value">--</span>
            </div>
        </div>
        <div class="control-group">
            <button onclick="exportSpectrumCSV()" title="Export current spectrum to CSV" style="width: 100%;">
                Export Spectrum CSV
            </button>
        </div>
        <div class="control-group">
            <label class="checkbox-label">
                <input type="checkbox" id="persistenceCheckbox" onchange="togglePersistence(this.checked)" title="Show signal trails on waterfall">
                Persistence Mode
            </label>
        </div>
        <div class="control-group" id="persistenceControls" style="display: none; padding-left: 20px;">
            <label>Decay Rate:</label>
            <div class="input-row">
                <input type="range" id="persistenceDecay" min="0.1" max="0.95" step="0.05" value="0.7" oninput="updatePersistenceDecay(this.value)" title="How fast trails fade">
                <span id="persistenceDecayValue" class="range-value">0.7</span>
            </div>
        </div>
        <div class="control-group">
            <label>Waterfall Speed:</label>
            <div class="input-row">
                <input type="range" id="waterfallSpeed" min="1" max="20" step="1" value="1" oninput="updateWaterfallSpeed(this.value)" title="Waterfall scroll speed">
                <span id="waterfallSpeedValue" class="range-value">1x</span>
            </div>
        </div>
        <div class="control-group">
            <label class="checkbox-label">
                <input type="checkbox" id="signalLogCheckbox" onchange="toggleSignalLog(this.checked)" title="Automatically log strong signals">
                Signal Logging
            </label>
        </div>
        <div class="control-group">
            <button onclick="downloadSignalLog()" id="downloadLogBtn" title="Download logged signals as CSV" disabled style="width: 100%;">
                Download Signal Log
            </button>
        </div>

        <h3 style="margin-top: 15px; border-top: 1px solid #333; padding-top: 15px;">Configuration Presets</h3>
        <div class="control-group">
            <label>Preset Name</label>
            <input type="text" id="presetName" placeholder="Enter name">
        </div>
        <div class="control-group">
            <label>Load Preset</label>
            <select id="presetSelect" onchange="loadPreset()">
                <option value="">-- Select --</option>
            </select>
        </div>
        <div class="control-group">
            <div class="input-row">
                <button onclick="savePreset()">Save</button>
                <button onclick="deletePreset()">Delete</button>
            </div>
        </div>
        <div class="control-group">
            <div class="input-row">
                <button onclick="exportPresets()">Export</button>
                <button onclick="importPresets()">Import</button>
            </div>
        </div>
        <input type="file" id="presetImportFile" accept=".json" style="display: none;" onchange="handlePresetImport(event)" />

        <h3 style="margin-top: 15px; border-top: 1px solid #333; padding-top: 15px;">Recording</h3>
        <div class="control-group">
            <button id="recordButton" onclick="toggleRecording()" style="width: 100%;">Start Recording</button>
        </div>
        <div class="control-group">
            <label>Duration (sec):</label>
            <input type="number" id="recordDuration" min="1" max="300" value="10">
        </div>
        <div class="control-group">
            <label>Mode:</label>
            <select id="recordMode" onchange="updateRecordMode()">
                <option value="full">Full Spectrum</option>
                <option value="band">Specific Band</option>
            </select>
        </div>
        <div id="recordBandControls" style="display: none; margin-top: 5px;">
            <div class="control-group">
                <label style="font-size: 10px;">Center (MHz):</label>
                <input type="number" id="recordCenterFreq" step="0.1" min="47" max="6000" value="915" style="width: 80px;">
            </div>
            <div class="control-group">
                <label style="font-size: 10px;">BW (MHz):</label>
                <input type="number" id="recordBandwidth" step="0.1" min="0.1" max="61.44" value="10" style="width: 80px;">
            </div>
        </div>
        <div class="control-group" style="margin-top: 5px;">
            <span id="recordingStatus" style="color: #888; font-size: 10px;">Ready</span>
        </div>

        <h3 style="margin-top: 15px; border-top: 1px solid #333; padding-top: 15px;">Data Export</h3>
        <div class="control-group">
            <button onclick="exportSpectrumCSV()" style="padding: 6px 12px; width: 100%;">Export Spectrum to CSV</button>
        </div>

        <h3 style="margin-top: 15px; border-top: 1px solid #333; padding-top: 15px;">Application Settings</h3>
        <div class="control-group" style="display: grid; grid-template-columns: 1fr 1fr; gap: 6px;">
            <button onclick="exportAllSettings()" style="padding: 6px;" title="Download all app settings as JSON">
                📥 Export Config
            </button>
            <button onclick="importAllSettings()" style="padding: 6px;" title="Load settings from JSON file">
                📤 Import Config
            </button>
        </div>
        <div class="control-group">
            <button onclick="resetAllSettings()" style="padding: 6px 12px; width: 100%; background: rgba(255, 0, 0, 0.1);" title="Reset all settings to defaults">
                🔄 Reset to Defaults
            </button>
        </div>
        <input type="file" id="settingsImportFile" accept=".json" style="display: none;" onchange="handleSettingsImport(event)" />
    </div>

    <div class="axis-label freq-axis" id="freq-axis"></div>
    <div class="axis-label time-axis" id="time-axis"></div>

    <script>
        const canvas = document.getElementById('waterfall');
        const ctx = canvas.getContext('2d');
        const canvas2 = document.getElementById('waterfall2');
        const ctx2 = canvas2.getContext('2d');
        const spectrumCanvas = document.getElementById('spectrum');
        const spectrumCtx = spectrumCanvas.getContext('2d');
        const spectrumCanvas2 = document.getElementById('spectrum2');
        const spectrumCtx2 = spectrumCanvas2.getContext('2d');
        const iqCanvas = document.getElementById('iq_canvas');
        const iqCtx = iqCanvas.getContext('2d');
        const xcorrCanvas = document.getElementById('xcorr_canvas');
        const xcorrCtx = xcorrCanvas.getContext('2d');

        // Offscreen canvases for double buffering (eliminates flicker)
        const spectrumOffscreen = document.createElement('canvas');
        const spectrumOffscreenCtx = spectrumOffscreen.getContext('2d');
        spectrumOffscreen.width = spectrumCanvas.width;
        spectrumOffscreen.height = spectrumCanvas.height;

        const spectrumOffscreen2 = document.createElement('canvas');
        const spectrumOffscreenCtx2 = spectrumOffscreen2.getContext('2d');
        spectrumOffscreen2.width = spectrumCanvas2.width;
        spectrumOffscreen2.height = spectrumCanvas2.height;

        const iqOffscreen = document.createElement('canvas');
        const iqOffscreenCtx = iqOffscreen.getContext('2d');
        iqOffscreen.width = iqCanvas.width;
        iqOffscreen.height = iqCanvas.height;

        const FFT_SIZE = 4096;
        const IQ_SAMPLES = 256;
        let currentChannel = 1;

        // Initialize display modules (loaded from modular JS files)
        // Note: These will be properly initialized after zoomState is defined below
        let modulesInitialized = false;

        // ===== SETTINGS PERSISTENCE INTEGRATION =====
        // Note: Settings module is loaded from settings.js
        // Add convenience functions to load/save display-specific settings

        function loadDisplaySettings() {
            if (typeof Settings === 'undefined') return;

            console.log('Loading display settings...');

            // Display settings
            spectrumMinDb = Settings.get('live_spectrum_min_db', -100);
            spectrumMaxDb = Settings.get('live_spectrum_max_db', -10);
            waterfallIntensity = Settings.get('live_waterfall_intensity', 1.0);
            waterfallContrast = Settings.get('live_waterfall_contrast', 1.0);

            // Apply to UI controls
            const minInput = document.getElementById('spectrumMin');
            const maxInput = document.getElementById('spectrumMax');
            const intensityInput = document.getElementById('waterfallIntensity');
            const contrastInput = document.getElementById('waterfallContrast');

            if (minInput) minInput.value = spectrumMinDb;
            if (maxInput) maxInput.value = spectrumMaxDb;
            if (intensityInput) intensityInput.value = waterfallIntensity;
            if (contrastInput) contrastInput.value = waterfallContrast;

            // Feature toggles
            const savedPeakHold = Settings.get('live_peak_hold', false);
            const savedMinHold = Settings.get('live_min_hold', false);
            const savedRefMarkers = Settings.get('live_ref_markers', false);

            if (savedPeakHold) document.getElementById('peakHoldCheckbox')?.click();
            if (savedMinHold) document.getElementById('minHoldCheckbox')?.click();
            if (savedRefMarkers) document.getElementById('refMarkersCheckbox')?.click();

            console.log('✓ Display settings loaded');
        }

        function saveDisplaySettings() {
            if (typeof Settings === 'undefined') return;

            Settings.set('live_spectrum_min_db', spectrumMinDb);
            Settings.set('live_spectrum_max_db', spectrumMaxDb);
            Settings.set('live_waterfall_intensity', waterfallIntensity);
            Settings.set('live_waterfall_contrast', waterfallContrast);
            Settings.set('live_peak_hold', peakHoldEnabled);
            Settings.set('live_min_hold', minHoldEnabled);
            Settings.set('live_ref_markers', refMarkersEnabled);
        }
        let frameCount = 0;
        let lastFpsUpdate = Date.now();
        let measuredFPS = 20;  // Track actual FPS for time axis calculations
        let showSpectrum = false;
        let showIQ = false;
        let showXCorr = false;

        // Spectrum selection state for IQ and XCORR workspaces
        const iqSelection = {
            active: false,
            leftPercent: 40,
            rightPercent: 60,
            dragging: null  // null, 'left', 'right', or 'selecting'
        };

        const xcorrSelection = {
            active: false,
            leftPercent: 40,
            rightPercent: 60,
            dragging: null
        };
        let latestFFTData = null;  // Store latest FFT data for spectrum display
        let latestFFTData2 = null;  // Store CH2 FFT data for dual display

        // Spectrum Y-axis control
        let spectrumYOffset = 0;
        let spectrumYScale = 1.0;
        let spectrumMinDb = -100;
        let spectrumMaxDb = -10;

        // Waterfall display controls
        let waterfallIntensity = 1.0;
        let waterfallContrast = 1.0;
        let waterfallScrollMultiplier = 1;  // Speed multiplier for waterfall scrolling

        // Spectrum hold traces (peakHoldEnabled and peakHoldData declared later with other spectrum state)
        let minHoldEnabled = false;
        let minHoldTrace = null;   // Array of min values

        // Reference markers and bandwidth measurement
        let refMarkersEnabled = false;
        let bwMeasureEnabled = false;
        let bwMeasurePoints = [];  // Array of {x, freq, db} for measurement points

        // Zoom state (display-only zoom, no hardware reconfiguration)
        let zoomState = {
            isSelecting: false,
            startX: 0,
            currentX: 0,
            fullBandwidth: 40000000,  // Hardware sample rate
            centerFreq: 915000000,    // Hardware center frequency
            // Display zoom in FFT bin indices
            zoomStartBin: 0,          // Start FFT bin of zoom region
            zoomEndBin: 4095,         // End FFT bin of zoom region (FFT_SIZE-1)
            isZoomed: false           // Whether we're zoomed in
        };

        // Filter selection state for IQ/XCorr filtering
        let filterState = {
            enabled: false,           // Filter mode enabled
            isSelecting: false,       // Currently selecting region
            startX: 0,                // Start X coordinate of selection
            currentX: 0,              // Current X coordinate during selection
            filterStartBin: 0,        // Start FFT bin of filter region
            filterEndBin: 4095,       // End FFT bin of filter region (FFT_SIZE-1)
            isFiltered: false         // Whether a filter is active
        };

        // Global state for signal analysis
        let signalAnalysis = {
            persistenceMode: 'none',
            persistenceData: null,
            persistenceDecayRate: 0.95,
            avgCount: 0,
            colorPalette: 'viridis',
            bookmarks: JSON.parse(localStorage.getItem('signal_bookmarks') || '[]'),
            cursorsEnabled: false,
            cursorPos: { x: null, y: null },
            peakMarkers: []
        };

        // Configuration Constants
        const CONFIG = {
            DEFAULT_SAMPLE_RATE: 40e6,           // 40 MHz
            FETCH_TIMEOUT_MS: 5000,              // 5 second timeout
            WATERFALL_UPDATE_INTERVAL_MS: 100,   // 10 Hz
            STATUS_UPDATE_INTERVAL_MS: 2000,     // 2 seconds
            IQ_UPDATE_INTERVAL_MS: 100,          // 10 Hz
            XCORR_UPDATE_INTERVAL_MS: 500,       // 2 Hz
            LINK_QUALITY_UPDATE_MS: 1000,        // 1 Hz
            DEFAULT_FPS: 20,
            FREQ_MIN_MHZ: 47,
            FREQ_MAX_MHZ: 6000,
            SR_MIN_MHZ: 0.52,
            SR_MAX_MHZ: 61.44,
            GAIN_MIN_DB: 0,
            GAIN_MAX_DB: 60,
            NOTIFICATION_DURATION_MS: 3000,
            MAX_STORAGE_SIZE_BYTES: 5000000      // 5MB localStorage limit
        };

        // Connection state
        let connectionState = {
            isConnected: false,
            lastSuccessfulFetch: Date.now(),
            consecutiveFailures: 0
        };

        /**
         * Toast Notification System
         * Displays non-blocking notifications to the user
         * @param {string} message - The message to display
         * @param {string} type - Notification type: 'info', 'success', 'warning', 'error'
         * @param {number} duration - Duration in ms (default: 3000)
         */
        function showNotification(message, type = 'info', duration = CONFIG.NOTIFICATION_DURATION_MS) {
            const container = document.getElementById('notificationContainer');
            if (!container) {
                console.warn('Notification container not found');
                return;
            }

            const notification = document.createElement('div');
            notification.className = `notification notification-${type}`;

            // Add icon based on type
            const icons = {
                info: 'ℹ️',
                success: '✓',
                warning: '⚠️',
                error: '✕'
            };

            notification.innerHTML = `
                <span style="font-size: 16px;">${icons[type] || ''}</span>
                <span>${message}</span>
            `;

            container.appendChild(notification);

            // Auto-remove after duration
            setTimeout(() => {
                notification.style.animation = 'fadeOut 0.3s ease-in forwards';
                setTimeout(() => notification.remove(), 300);
            }, duration);
        }

        /**
         * Update connection status indicator
         * @param {string} status - 'connected', 'disconnected', or 'connecting'
         */
        function updateConnectionStatus(status) {
            const indicator = document.getElementById('connectionStatus');
            if (!indicator) return;

            indicator.className = `connection-status ${status}`;
            indicator.textContent = status.toUpperCase();

            connectionState.isConnected = (status === 'connected');

            if (status === 'connected') {
                connectionState.lastSuccessfulFetch = Date.now();
                connectionState.consecutiveFailures = 0;
            }
        }

        /**
         * Show/hide loading overlay
         * @param {boolean} show - Whether to show the loading indicator
         */
        function setLoading(show) {
            const overlay = document.getElementById('loadingOverlay');
            if (!overlay) return;

            if (show) {
                overlay.classList.add('active');
            } else {
                overlay.classList.remove('active');
            }
        }

        /**
         * Fetch wrapper with timeout and error handling
         * @param {string} url - The URL to fetch
         * @param {object} options - Fetch options
         * @param {number} timeout - Timeout in milliseconds
         * @returns {Promise<Response>}
         */
        async function fetchWithTimeout(url, options = {}, timeout = CONFIG.FETCH_TIMEOUT_MS) {
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), timeout);

            try {
                const response = await fetch(url, {
                    ...options,
                    signal: controller.signal
                });
                clearTimeout(timeoutId);

                if (!response.ok) {
                    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
                }

                connectionState.lastSuccessfulFetch = Date.now();
                connectionState.consecutiveFailures = 0;

                if (!connectionState.isConnected) {
                    updateConnectionStatus('connected');
                }

                return response;
            } catch (error) {
                clearTimeout(timeoutId);

                connectionState.consecutiveFailures++;

                if (connectionState.consecutiveFailures >= 3) {
                    updateConnectionStatus('disconnected');
                }

                if (error.name === 'AbortError') {
                    throw new Error(`Request timeout after ${timeout}ms`);
                }

                throw error;
            }
        }

        /**
         * Safe DOM element getter with null check
         * @param {string} id - Element ID
         * @returns {HTMLElement|null}
         */
        function getElement(id) {
            const element = document.getElementById(id);
            if (!element) {
                console.warn(`Element not found: ${id}`);
            }
            return element;
        }

        /**
         * Safely update element text content
         * @param {string} id - Element ID
         * @param {string} text - Text to set
         */
        function setElementText(id, text) {
            const element = getElement(id);
            if (element) {
                element.textContent = text;
            }
        }

        /**
         * Validate and sanitize preset name
         * @param {string} name - Preset name to validate
         * @returns {object} {valid: boolean, error: string}
         */
        function validatePresetName(name) {
            if (!name || name.trim().length === 0) {
                return {valid: false, error: 'Preset name is required'};
            }

            if (name.length > 100) {
                return {valid: false, error: 'Preset name too long (max 100 characters)'};
            }

            if (!/^[a-zA-Z0-9\-_ ]+$/.test(name)) {
                return {valid: false, error: 'Invalid characters (use only letters, numbers, spaces, dashes, underscores)'};
            }

            if (['__proto__', 'constructor', 'prototype'].includes(name)) {
                return {valid: false, error: 'Invalid preset name'};
            }

            return {valid: true};
        }

        /**
         * Safe localStorage setItem with quota handling
         * @param {string} key - Storage key
         * @param {string} value - Value to store
         * @returns {boolean} Success status
         */
        function safeLocalStorageSet(key, value) {
            try {
                if (value.length > CONFIG.MAX_STORAGE_SIZE_BYTES) {
                    showNotification('Data too large to save (max 5MB)', 'error');
                    return false;
                }
                localStorage.setItem(key, value);
                return true;
            } catch (e) {
                if (e.name === 'QuotaExceededError') {
                    showNotification('Storage quota exceeded. Delete old data to continue', 'error');
                } else {
                    showNotification(`Failed to save: ${e.message}`, 'error');
                }
                return false;
            }
        }

        // Helper functions for dB conversion
        function rawToDb(raw) {
            return (raw / 255.0) * 120.0 - 100.0;
        }

        function dbToRaw(db) {
            return ((db + 100.0) / 120.0) * 255.0;
        }

        // Viridis colormap (JavaScript version)
        // Dark purple → blue → cyan → green → yellow
        function viridisColor(value) {
            value = Math.max(0, Math.min(1, value));
            let r, g, b;

            if (value < 0.25) {
                // Dark purple to blue
                const t = value / 0.25;
                r = 68 + t * (59 - 68);
                g = 1 + t * (82 - 1);
                b = 84 + t * (139 - 84);
            } else if (value < 0.5) {
                // Blue to teal/cyan
                const t = (value - 0.25) / 0.25;
                r = 59 + t * (33 - 59);
                g = 82 + t * (145 - 82);
                b = 139 + t * (140 - 139);
            } else if (value < 0.75) {
                // Cyan to green
                const t = (value - 0.5) / 0.25;
                r = 33 + t * (94 - 33);
                g = 145 + t * (201 - 145);
                b = 140 + t * (98 - 140);
            } else {
                // Green to bright yellow
                const t = (value - 0.75) / 0.25;
                r = 94 + t * (253 - 94);
                g = 201 + t * (231 - 201);
                b = 98 + t * (37 - 98);
            }

            return [Math.floor(r), Math.floor(g), Math.floor(b)];
        }

        // Make panels draggable
        function makeDraggable(element) {
            let pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
            const header = element.querySelector('.panel-header');

            if (header) {
                header.onmousedown = dragMouseDown;
            }

            function dragMouseDown(e) {
                e.preventDefault();
                pos3 = e.clientX;
                pos4 = e.clientY;
                document.onmouseup = closeDragElement;
                document.onmousemove = elementDrag;
                element.classList.add('active');
            }

            function elementDrag(e) {
                e.preventDefault();
                pos1 = pos3 - e.clientX;
                pos2 = pos4 - e.clientY;
                pos3 = e.clientX;
                pos4 = e.clientY;
                element.style.top = (element.offsetTop - pos2) + "px";
                element.style.left = (element.offsetLeft - pos1) + "px";
            }

            function closeDragElement() {
                document.onmouseup = null;
                document.onmousemove = null;
                element.classList.remove('active');
            }
        }

        // Resize canvas to fill window
        function resizeCanvas() {
            const waterfallTop = showSpectrum ? 250 : 50;
            const waterfallBottom = showXCorr ? 210 : 30;
            const chSelect = document.getElementById('channel_select').value;
            const isDualChannel = (chSelect === 'both');

            // Update label visibility
            document.getElementById('waterfall-label').style.display = (!isDualChannel && !showSpectrum) ? 'block' : 'none';
            // Show dual-channel labels whenever dual-channel mode is active, not just when spectrum is on
            document.getElementById('waterfall-label-ch1').style.display = isDualChannel ? 'block' : 'none';
            document.getElementById('waterfall-label-ch2').style.display = isDualChannel ? 'block' : 'none';

            // Spectrum labels
            document.getElementById('spectrum-label').style.display = (showSpectrum && !isDualChannel) ? 'block' : 'none';
            document.getElementById('spectrum-label-ch1').style.display = (showSpectrum && isDualChannel) ? 'block' : 'none';
            document.getElementById('spectrum-label-ch2').style.display = (showSpectrum && isDualChannel) ? 'block' : 'none';

            if (isDualChannel) {
                // Dual-channel mode: split screen 50/50
                const halfWidth = Math.floor((window.innerWidth - 60) / 2);

                console.log('Setting up dual-channel: halfWidth=' + halfWidth);

                // Only change canvas dimensions if they actually changed (setting .width clears canvas!)
                const newWidth1 = Math.max(halfWidth, FFT_SIZE);
                const newHeight = window.innerHeight - waterfallTop - waterfallBottom;

                if (canvas.width !== newWidth1 || canvas.height !== newHeight) {
                    console.log('Resizing canvas1 from ' + canvas.width + 'x' + canvas.height + ' to ' + newWidth1 + 'x' + newHeight);
                    canvas.width = newWidth1;
                    canvas.height = newHeight;
                }

                canvas.style.top = waterfallTop + 'px';
                canvas.style.left = '50px';
                canvas.style.width = halfWidth + 'px';
                canvas.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;
                canvas.style.display = 'block';

                const newWidth2 = Math.max(halfWidth, FFT_SIZE);
                if (canvas2.width !== newWidth2 || canvas2.height !== newHeight) {
                    console.log('Resizing canvas2 from ' + canvas2.width + 'x' + canvas2.height + ' to ' + newWidth2 + 'x' + newHeight);
                    canvas2.width = newWidth2;
                    canvas2.height = newHeight;
                }
                canvas2.style.top = waterfallTop + 'px';
                canvas2.style.left = (50 + halfWidth) + 'px';
                canvas2.style.width = halfWidth + 'px';
                canvas2.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;
                canvas2.style.display = 'block';

                // Position dual-channel waterfall labels
                document.getElementById('waterfall-label-ch1').style.left = (50 + halfWidth / 2 - 50) + 'px';
                document.getElementById('waterfall-label-ch2').style.left = (50 + halfWidth + halfWidth / 2 - 50) + 'px';

                // Position dual-channel spectrum labels
                if (showSpectrum) {
                    document.getElementById('spectrum-label-ch1').style.left = (50 + halfWidth / 2 - 60) + 'px';
                    document.getElementById('spectrum-label-ch1').style.top = '10px';
                    document.getElementById('spectrum-label-ch2').style.left = (50 + halfWidth + halfWidth / 2 - 60) + 'px';
                    document.getElementById('spectrum-label-ch2').style.top = '10px';
                }

                // Show and position channel divider
                const divider = document.getElementById('channel-divider');
                divider.style.display = 'block';
                divider.style.left = (50 + halfWidth) + 'px';
                divider.style.top = (showSpectrum ? 50 : waterfallTop) + 'px';
                divider.style.height = `calc(100% - ${showSpectrum ? 50 : waterfallTop}px - ${waterfallBottom}px)`;

                console.log('Canvas1: left=50px, width=' + halfWidth + 'px, buffer=' + canvas.width + 'px');
                console.log('Canvas2: left=' + (50 + halfWidth) + 'px, width=' + halfWidth + 'px, buffer=' + canvas2.width + 'px, display=' + canvas2.style.display);
                console.log('Divider: left=' + (50 + halfWidth) + 'px');
            } else {
                // Single-channel mode: full width
                // Only change canvas dimensions if they actually changed (setting .width clears canvas!)
                const viewWidth = window.innerWidth - 60;
                const newWidth = Math.max(viewWidth, FFT_SIZE);
                const newHeight = window.innerHeight - waterfallTop - waterfallBottom;

                if (canvas.width !== newWidth || canvas.height !== newHeight) {
                    console.log('Resizing canvas from ' + canvas.width + 'x' + canvas.height + ' to ' + newWidth + 'x' + newHeight);
                    canvas.width = newWidth;
                    canvas.height = newHeight;
                }

                canvas.style.top = waterfallTop + 'px';
                canvas.style.left = '50px';
                canvas.style.width = `calc(100% - 60px)`;
                canvas.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;
                canvas.style.display = 'block';

                canvas2.style.display = 'none';

                // Hide channel divider in single-channel mode
                document.getElementById('channel-divider').style.display = 'none';

                console.log('Single-channel: view=' + viewWidth + 'px, buffer=' + canvas.width + 'px');
            }

            // Spectrum canvas sizing (handle dual-channel mode)
            const newSpecHeight = 200;
            const specWidth = window.innerWidth - 60;  // Calculate specWidth for all modes

            if (isDualChannel && showSpectrum) {
                // Dual spectrum mode: split 50/50 like waterfalls
                const halfWidth = Math.floor((window.innerWidth - 60) / 2);
                const newSpecWidth = Math.max(halfWidth, FFT_SIZE);

                // Resize spectrum canvas 1
                if (spectrumCanvas.width !== newSpecWidth || spectrumCanvas.height !== newSpecHeight) {
                    spectrumCanvas.width = newSpecWidth;
                    spectrumCanvas.height = newSpecHeight;
                    spectrumOffscreen.width = newSpecWidth;
                    spectrumOffscreen.height = newSpecHeight;
                }
                spectrumCanvas.style.left = '50px';
                spectrumCanvas.style.width = halfWidth + 'px';
                spectrumCanvas.style.display = 'block';

                // Resize spectrum canvas 2
                if (spectrumCanvas2.width !== newSpecWidth || spectrumCanvas2.height !== newSpecHeight) {
                    spectrumCanvas2.width = newSpecWidth;
                    spectrumCanvas2.height = newSpecHeight;
                    spectrumOffscreen2.width = newSpecWidth;
                    spectrumOffscreen2.height = newSpecHeight;
                }
                spectrumCanvas2.style.left = (50 + halfWidth) + 'px';
                spectrumCanvas2.style.width = halfWidth + 'px';
                spectrumCanvas2.style.display = 'block';

                // Update SpectrumDisplay module's offscreen buffers
                if (typeof SpectrumDisplay !== 'undefined') {
                    SpectrumDisplay.resize(newSpecWidth, newSpecHeight, newSpecWidth, newSpecHeight);
                }
            } else {
                // Single spectrum mode: full width
                const newSpecWidth = Math.max(specWidth, FFT_SIZE);

                if (spectrumCanvas.width !== newSpecWidth || spectrumCanvas.height !== newSpecHeight) {
                    spectrumCanvas.width = newSpecWidth;
                    spectrumCanvas.height = newSpecHeight;
                    spectrumOffscreen.width = newSpecWidth;
                    spectrumOffscreen.height = newSpecHeight;
                }
                spectrumCanvas.style.left = '50px';
                spectrumCanvas.style.width = `calc(100% - 60px)`;

                // Hide second spectrum canvas
                spectrumCanvas2.style.display = 'none';

                // Update SpectrumDisplay module's offscreen buffer (single-channel)
                if (typeof SpectrumDisplay !== 'undefined') {
                    SpectrumDisplay.resize(newSpecWidth, newSpecHeight);
                }
            }

            if (showXCorr) {
                const newXCorrWidth = Math.max(specWidth, FFT_SIZE);
                const newXCorrHeight = 180;

                if (xcorrCanvas.width !== newXCorrWidth || xcorrCanvas.height !== newXCorrHeight) {
                    xcorrCanvas.width = newXCorrWidth;
                    xcorrCanvas.height = newXCorrHeight;
                }
            }

            updateTimeAxis();

            // Resize Eye Diagram and Waveform Display if in IQ workspace
            if (document.getElementById('workspace-iq') && document.getElementById('workspace-iq').classList.contains('active')) {
                if (typeof EyeDiagram !== 'undefined' && EyeDiagram.resize) {
                    EyeDiagram.resize();
                }
                if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.resize) {
                    WaveformDisplay.resize();
                }
            }
        }

        // Fetch and display IQ constellation data
        async function updateIQData() {
            if (!showIQ) return;
            if (isUpdatingIQ) return; // Skip if previous request still running

            isUpdatingIQ = true;
            try {
                // Add filter parameters if filter is active
                let url = '/iq_data?t=' + Date.now();
                if (filterState.isFiltered) {
                    url += `&start_bin=${filterState.filterStartBin}&end_bin=${filterState.filterEndBin}`;
                }
                const response = await fetchWithTimeout(url);
                const buffer = await response.arrayBuffer();
                const data = new Int16Array(buffer);

                // Data format: CH1_I (256), CH1_Q (256), CH2_I (256), CH2_Q (256)
                const ch1_i = data.slice(0, IQ_SAMPLES);
                const ch1_q = data.slice(IQ_SAMPLES, IQ_SAMPLES * 2);
                const ch2_i = data.slice(IQ_SAMPLES * 2, IQ_SAMPLES * 3);
                const ch2_q = data.slice(IQ_SAMPLES * 3, IQ_SAMPLES * 4);

                // Debug: Check if we're getting non-zero data
                const hasData = Math.max(...ch1_i.map(Math.abs), ...ch1_q.map(Math.abs)) > 0;
                if (!hasData) {
                    console.warn('IQ data appears to be all zeros');
                }

                // Use IQ Constellation Enhanced module
                if (typeof IQConstellationEnhanced !== 'undefined') {
                    console.log('[Main] Using IQConstellationEnhanced module');
                    // Normalize int16 to float [-1, 1]
                    const normalize = (arr) => {
                        const result = new Float32Array(arr.length);
                        for (let i = 0; i < arr.length; i++) {
                            result[i] = arr[i] / 2048.0;
                        }
                        return result;
                    };
                    IQConstellationEnhanced.draw(normalize(ch1_i), normalize(ch1_q), normalize(ch2_i), normalize(ch2_q));
                } else {
                    console.error('[Main] IQConstellationEnhanced module not loaded! Cannot display IQ constellation.');
                }

                // Update IQ statistics panel if in IQ workspace
                const iqWorkspace = document.getElementById('workspace-iq');
                if (iqWorkspace && iqWorkspace.classList.contains('active')) {
                    // Combine both channels for statistics
                    const combinedIQ = new Float32Array(IQ_SAMPLES * 2);
                    for (let i = 0; i < IQ_SAMPLES; i++) {
                        combinedIQ[i * 2] = (ch1_i[i] + ch2_i[i]) / 2.0 / 2048.0;  // Normalize and average
                        combinedIQ[i * 2 + 1] = (ch1_q[i] + ch2_q[i]) / 2.0 / 2048.0;
                    }
                    updateIQStatistics(combinedIQ);

                    // Update signal detection metrics using raw IQ data
                    updateIQSignalMetrics(ch1_i, ch1_q, ch2_i, ch2_q);

                    // Update eye diagram and waveform displays
                    if (typeof EyeDiagram !== 'undefined') {
                        EyeDiagram.update(ch1_i, ch1_q, ch2_i, ch2_q);
                    }
                    if (typeof WaveformDisplay !== 'undefined') {
                        WaveformDisplay.update(ch1_i, ch1_q, ch2_i, ch2_q);
                    }
                }
            } catch (err) {
                console.error('IQ data fetch error:', err);
            } finally {
                isUpdatingIQ = false;
            }
        }

        // Fetch and display cross-correlation data
        async function updateXCorrData() {
            if (!showXCorr) return;
            if (isUpdatingXCorr) return; // Skip if previous request still running

            isUpdatingXCorr = true;
            try {
                // Add filter parameters if filter is active
                let url = '/xcorr_data?t=' + Date.now();
                if (filterState.isFiltered) {
                    url += `&start_bin=${filterState.filterStartBin}&end_bin=${filterState.filterEndBin}`;
                }
                const response = await fetchWithTimeout(url);
                const buffer = await response.arrayBuffer();
                const data = new Float32Array(buffer);

                // Data format: magnitude (4096), phase (4096)
                const halfLen = data.length / 2;
                const magnitude = data.slice(0, halfLen);
                const phase = data.slice(halfLen);

                // Debug: Check if we're getting non-zero data
                const maxMag = Math.max(...magnitude);
                const maxPhase = Math.max(...phase.map(Math.abs));
                if (maxMag === 0 && maxPhase === 0) {
                    console.warn('XCorr data appears to be all zeros');
                } else {
                    console.log('XCorr data range - mag:', maxMag.toFixed(2), 'phase:', maxPhase.toFixed(2));
                }

                // Draw cross-correlation using the enhanced module or fallback
                if (typeof CrossCorrelationEnhanced !== 'undefined' && CrossCorrelationEnhanced.draw) {
                    CrossCorrelationEnhanced.draw(magnitude, phase);
                } else {
                    drawXCorr(magnitude, phase);
                }
            } catch (err) {
                console.error('XCorr data fetch error:', err);
            } finally {
                isUpdatingXCorr = false;
            }
        }

        // Update link quality indicator
        async function updateLinkQuality() {
            if (isUpdatingLinkQuality) return; // Skip if previous request still running

            isUpdatingLinkQuality = true;
            try {
                const response = await fetchWithTimeout('/link_quality');
                const data = await response.json();

                // Update RTT
                setElementText('rtt', data.rtt_ms.toFixed(0) + 'ms');

                // Update bandwidth
                const bw = data.bandwidth_kbps;
                let bwStr = '';
                if (bw >= 1000) {
                    bwStr = (bw / 1000).toFixed(2) + 'Mbps';
                } else {
                    bwStr = bw.toFixed(0) + 'kbps';
                }
                setElementText('bandwidth', bwStr);

                // Update link quality bar (5 bars based on quality)
                const fps = data.fps;
                const loss = data.packet_loss;
                let bars = 5;

                if (fps < 2 || loss > 0.3) bars = 1;
                else if (fps < 5 || loss > 0.1) bars = 2;
                else if (fps < 8 || loss > 0.05) bars = 3;
                else if (fps < 9.5 || loss > 0.01) bars = 4;

                const fullBars = '●'.repeat(bars);
                const emptyBars = '○'.repeat(5 - bars);
                const barEl = document.getElementById('link_quality_bar');
                barEl.textContent = fullBars + emptyBars;

                // Color code by quality
                if (bars >= 4) {
                    barEl.style.color = '#0f0';  // Green - excellent
                } else if (bars >= 3) {
                    barEl.style.color = '#ff0';  // Yellow - good
                } else {
                    barEl.style.color = '#f80';  // Orange/red - poor
                }
            } catch (err) {
                console.error('Link quality fetch error:', err);
            } finally {
                isUpdatingLinkQuality = false;
            }
        }

        resizeCanvas();
        window.addEventListener('resize', resizeCanvas);

        // Keyboard navigation for zoom and controls
        window.addEventListener('keydown', (e) => {
            // Don't trigger if user is typing in an input field
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') {
                return;
            }

            switch(e.key) {
                case '-':
                case '_':
                    // Zoom out
                    e.preventDefault();
                    zoomOut();
                    showNotification('Zoomed out to full spectrum', 'info', 1500);
                    break;

                case '0':
                case ')':
                    // Reset zoom (same as zoom out)
                    e.preventDefault();
                    zoomOut();
                    showNotification('Reset to full spectrum view', 'info', 1500);
                    break;

                case 'Escape':
                    // Cancel selection if in progress
                    if (zoomState.isSelecting) {
                        e.preventDefault();
                        zoomState.isSelecting = false;
                        showNotification('Selection cancelled', 'info', 1500);
                    } else if (zoomState.isZoomed) {
                        // If not selecting but zoomed, escape zooms out
                        e.preventDefault();
                        zoomOut();
                        showNotification('Reset to full spectrum view', 'info', 1500);
                    }
                    break;

                case '?':
                    // Show keyboard shortcuts help
                    e.preventDefault();
                    showNotification('Keyboard: [-] Zoom Out | [0] Reset | [Esc] Cancel/Reset', 'info', 5000);
                    break;
            }
        });

        // Initialize draggable panels
        makeDraggable(document.getElementById('iq_constellation'));
        makeDraggable(document.getElementById('xcorr_display'));

        // Fetch and render FFT data
        let fetchTimeout = null;

        function updateWaterfall() {
            const chSelect = document.getElementById('channel_select').value;

            // Handle dual-channel mode
            if (chSelect === 'both') {
                updateWaterfallDualChannel();
                return;
            }

            const ch = chSelect;

            fetchWithTimeout('/fft?ch=' + ch + '&t=' + Date.now(), {
                method: 'GET',
                cache: 'no-cache'
            })
                .then(response => response.arrayBuffer())
                .then(buffer => {
                    let data = new Uint8Array(buffer);

                    if (data.length !== FFT_SIZE) {
                        console.warn('Size mismatch: got ' + data.length + ' bytes, expected ' + FFT_SIZE);
                        isUpdating = false;
                        return;
                    }

                    // Apply persistence mode if enabled
                    data = applyPersistence(data);

                    // Store latest FFT data for spectrum display
                    latestFFTData = data;

                    // Use WaterfallDisplay module if available
                    if (typeof WaterfallDisplay !== 'undefined') {
                        WaterfallDisplay.draw(data, null);
                    } else {
                        // Fallback to inline waterfall rendering
                        // Scroll canvas down by 1 pixel (GPU-accelerated)
                        if (canvas.width > 0 && canvas.height > 1) {
                            try {
                                ctx.drawImage(canvas, 0, 0, canvas.width, canvas.height - 1, 0, 1, canvas.width, canvas.height - 1);
                            } catch (e) {
                                console.warn('Canvas scroll skipped:', e.message);
                            }
                        }

                        // Draw new FFT line at top
                        const lineData = ctx.createImageData(canvas.width, 1);
                        for (let x = 0; x < canvas.width; x++) {
                            const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                            const fftIdx = zoomState.zoomStartBin + Math.floor((x / canvas.width) * zoomedBins);
                            let value = data[fftIdx];

                            value = value * waterfallIntensity;
                            value = 128 + (value - 128) * waterfallContrast;
                            value = Math.max(0, Math.min(255, value));

                            const mag = value / 255.0;
                            const [r, g, b] = getColorForValue(mag, signalAnalysis.colorPalette);
                            const idx = x * 4;
                            lineData.data[idx + 0] = r;
                            lineData.data[idx + 1] = g;
                            lineData.data[idx + 2] = b;
                            lineData.data[idx + 3] = 255;
                        }
                        ctx.putImageData(lineData, 0, 0);
                    }

                    // Capture recording frame if recording
                    captureRecordingFrame(data);

                    // Draw spectrum if enabled
                    if (showSpectrum) {
                        drawSpectrum(data, null);
                    }

                    // Update FPS counter
                    frameCount++;
                    const now = Date.now();
                    if (now - lastFpsUpdate >= 1000) {
                        measuredFPS = frameCount;  // Save measured FPS
                        document.getElementById('fps').textContent = frameCount + ' FPS';
                        updateTimeAxis();  // Update time labels when FPS updates
                        frameCount = 0;
                        lastFpsUpdate = now;
                    }

                    // Unlock after successful render
                    isUpdating = false;
                })
                .catch(err => {
                    // Don't log aborted fetches (intentional cleanup)
                    if (err.name !== 'AbortError') {
                        console.error('FFT fetch error:', err);
                    }
                    isUpdating = false;
                });
        }

        // Update waterfall with dual-channel side-by-side
        async function updateWaterfallDualChannel() {
            try {
                // Fetch both channels in parallel
                const [ch1Response, ch2Response] = await Promise.all([
                    fetchWithTimeout('/fft?ch=1&t=' + Date.now()),
                    fetchWithTimeout('/fft?ch=2&t=' + Date.now())
                ]);

                const [ch1Buffer, ch2Buffer] = await Promise.all([
                    ch1Response.arrayBuffer(),
                    ch2Response.arrayBuffer()
                ]);

                let ch1Data = new Uint8Array(ch1Buffer);
                let ch2Data = new Uint8Array(ch2Buffer);

                if (ch1Data.length !== FFT_SIZE || ch2Data.length !== FFT_SIZE) {
                    console.warn('Size mismatch in dual-channel mode');
                    isUpdating = false;
                    return;
                }

                // Apply persistence mode if enabled
                ch1Data = applyPersistence(ch1Data);
                ch2Data = applyPersistence(ch2Data);

                // Store latest data for spectrum display
                latestFFTData = ch1Data;
                latestFFTData2 = ch2Data;

                // Use WaterfallDisplay module for dual-channel rendering
                if (typeof WaterfallDisplay !== 'undefined') {
                    WaterfallDisplay.draw(ch1Data, ch2Data);
                } else {
                    // Fallback to inline waterfall rendering
                    // Update CH1 waterfall (left)
                    if (canvas.width > 0 && canvas.height > 1) {
                        try {
                            ctx.drawImage(canvas, 0, 0, canvas.width, canvas.height - 1, 0, 1, canvas.width, canvas.height - 1);
                        } catch (e) {
                            console.warn('Canvas CH1 scroll skipped:', e.message);
                        }
                    }

                    const lineData1 = ctx.createImageData(canvas.width, 1);
                    for (let x = 0; x < canvas.width; x++) {
                        const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                        const fftIdx = zoomState.zoomStartBin + Math.floor((x / canvas.width) * zoomedBins);
                        let val = ch1Data[fftIdx];

                        val = val * waterfallIntensity;
                        val = 128 + (val - 128) * waterfallContrast;
                        val = Math.max(0, Math.min(255, val));

                        const rgb = getColorForValue(val / 255.0, signalAnalysis.colorPalette);
                        const idx = x * 4;
                        lineData1.data[idx + 0] = rgb[0];
                        lineData1.data[idx + 1] = rgb[1];
                        lineData1.data[idx + 2] = rgb[2];
                        lineData1.data[idx + 3] = 255;
                    }
                    ctx.putImageData(lineData1, 0, 0);

                    // Update CH2 waterfall (right)
                    if (canvas2.width > 0 && canvas2.height > 1) {
                        try {
                            ctx2.drawImage(canvas2, 0, 0, canvas2.width, canvas2.height - 1, 0, 1, canvas2.width, canvas2.height - 1);
                        } catch (e) {
                            console.warn('Canvas CH2 scroll skipped:', e.message);
                        }
                    }

                    const lineData2 = ctx2.createImageData(canvas2.width, 1);
                    for (let x = 0; x < canvas2.width; x++) {
                        const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                        const fftIdx = zoomState.zoomStartBin + Math.floor((x / canvas2.width) * zoomedBins);
                        let val = ch2Data[fftIdx];

                        val = val * waterfallIntensity;
                        val = 128 + (val - 128) * waterfallContrast;
                        val = Math.max(0, Math.min(255, val));

                        const rgb = getColorForValue(val / 255.0, signalAnalysis.colorPalette);
                        const idx = x * 4;
                        lineData2.data[idx + 0] = rgb[0];
                        lineData2.data[idx + 1] = rgb[1];
                        lineData2.data[idx + 2] = rgb[2];
                        lineData2.data[idx + 3] = 255;
                    }
                    ctx2.putImageData(lineData2, 0, 0);
                }

                // Update FPS counter
                frameCount++;
                const now = Date.now();
                if (now - lastFpsUpdate >= 1000) {
                    measuredFPS = frameCount;
                    document.getElementById('fps').textContent = frameCount + ' FPS (Dual)';
                    updateTimeAxis();
                    frameCount = 0;
                    lastFpsUpdate = now;
                }

                // Draw spectrum if enabled
                if (showSpectrum && latestFFTData && latestFFTData2) {
                    drawSpectrum(latestFFTData, latestFFTData2);
                }

                // Update IQ signal metrics if in IQ workspace - need to convert uint8 magnitude to FFT format
                const iqWorkspace = document.getElementById('workspace-iq');
                if (iqWorkspace && iqWorkspace.classList.contains('active') && latestFFTData && latestFFTData2) {
                    // Convert magnitude spectrum to fake FFT format for signal detection
                    // (Real part = magnitude, Imaginary part = 0)
                    const ch1_fft = new Float32Array(FFT_SIZE * 2);
                    const ch2_fft = new Float32Array(FFT_SIZE * 2);
                    for (let i = 0; i < FFT_SIZE; i++) {
                        ch1_fft[i * 2] = latestFFTData[i] / 255.0;  // Real
                        ch1_fft[i * 2 + 1] = 0;  // Imaginary
                        ch2_fft[i * 2] = latestFFTData2[i] / 255.0;  // Real
                        ch2_fft[i * 2 + 1] = 0;  // Imaginary
                    }
                    updateIQSignalMetrics(ch1_fft, ch2_fft);
                    updateIQWorkspaceFreqDisplay();
                }

                isUpdating = false;
            } catch (err) {
                console.error('Dual-channel waterfall error:', err);
                isUpdating = false;
            }
        }

        // Peak hold state
        let peakHoldEnabled = false;
        let peakHoldData = null;

        // Persistence mode state
        let persistenceEnabled = false;
        let persistenceBuffer = null;
        let persistenceDecay = 0.7; // How fast trails fade (0-1)

        function togglePersistence(enabled) {
            persistenceEnabled = enabled;
            const controls = getElement('persistenceControls');

            if (enabled) {
                showNotification('Persistence mode enabled - signals will leave trails', 'success', 2000);
                if (controls) controls.style.display = 'block';
                // Initialize persistence buffer
                persistenceBuffer = null;
            } else {
                showNotification('Persistence mode disabled', 'info', 2000);
                if (controls) controls.style.display = 'none';
                // Clear persistence buffer
                persistenceBuffer = null;
            }

            if (typeof Settings !== 'undefined') {
                Settings.set('persistence_enabled', enabled);
            }
        }

        function updatePersistenceDecay(value) {
            persistenceDecay = parseFloat(value);
            const display = getElement('persistenceDecayValue');
            if (display) display.textContent = value;

            if (typeof Settings !== 'undefined') {
                Settings.set('persistence_decay', persistenceDecay);
            }
        }

        function applyPersistence(newData) {
            if (!persistenceEnabled) {
                return newData;
            }

            // Initialize persistence buffer if needed
            if (!persistenceBuffer || persistenceBuffer.length !== newData.length) {
                persistenceBuffer = new Uint8Array(newData);
                return newData;
            }

            // Create output with persistence applied
            const output = new Uint8Array(newData.length);

            for (let i = 0; i < newData.length; i++) {
                // Decay old values
                persistenceBuffer[i] = Math.floor(persistenceBuffer[i] * persistenceDecay);

                // Take maximum of new data and decayed persistence
                output[i] = Math.max(newData[i], persistenceBuffer[i]);

                // Update persistence buffer with current max
                persistenceBuffer[i] = output[i];
            }

            return output;
        }

        function togglePeakHold(enabled) {
            peakHoldEnabled = enabled;
            if (!enabled) {
                peakHoldData = null; // Clear peak hold data
            }
            if (typeof Settings !== 'undefined') {
                Settings.set('live_peak_hold', enabled);
            }
        }

        function clearPeakHold() {
            peakHoldData = null;
            showNotification('Peak hold cleared', 'info', 1000);
        }

        function toggleMinHold(enabled) {
            minHoldEnabled = enabled;
            if (!enabled) {
                minHoldTrace = null;
            }
            saveDisplaySettings();
        }

        function clearMinHold() {
            minHoldTrace = null;
            showNotification('Min hold cleared', 'info', 1000);
        }

        function toggleRefMarkers(enabled) {
            refMarkersEnabled = enabled;
            saveDisplaySettings();
        }

        function toggleBandwidthMeasure(enabled) {
            bwMeasureEnabled = enabled;
            if (!enabled) {
                bwMeasurePoints = [];
                document.getElementById('bw_measurement').textContent = '--';
            } else {
                showNotification('Click two points on spectrum to measure bandwidth', 'info', 3000);
            }
        }

        function exportSpectrumCSV() {
            if (!latestFFTData || latestFFTData.length === 0) {
                showNotification('No spectrum data available', 'warning');
                return;
            }

            const currentSR = zoomState.fullBandwidth || 40000000;
            const currentCF = zoomState.centerFreq || 915000000;
            const binWidth = currentSR / FFT_SIZE;
            const fullStartFreq = currentCF - currentSR / 2;

            let csv = 'Frequency (Hz),Power (dBFS),Power (dBm)\n';

            for (let i = 0; i < latestFFTData.length; i++) {
                const freq = fullStartFreq + (i * binWidth);
                const dbfs = rawToDb(latestFFTData[i]);
                const dbm = dbfs; // Simplified - adjust based on your calibration
                csv += `${freq.toFixed(0)},${dbfs.toFixed(2)},${dbm.toFixed(2)}\n`;
            }

            const blob = new Blob([csv], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = `spectrum_${currentCF}_${Date.now()}.csv`;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);

            showNotification('Spectrum exported to CSV', 'success', 2000);
        }

        function updateWaterfallSpeed(value) {
            waterfallScrollMultiplier = parseInt(value);
            document.getElementById('waterfallSpeedValue').textContent = value + 'x';
        }

        // Signal logging state
        let signalLogEnabled = false;
        let signalLog = [];
        const SIGNAL_LOG_THRESHOLD = -60; // dBm
        const SIGNAL_LOG_INTERVAL = 5000; // Log every 5 seconds
        let lastSignalLogTime = 0;

        function toggleSignalLog(enabled) {
            signalLogEnabled = enabled;
            const downloadBtn = getElement('downloadLogBtn');

            if (enabled) {
                showNotification('Signal logging started', 'success', 2000);
                if (downloadBtn) downloadBtn.disabled = false;
            } else {
                showNotification('Signal logging stopped', 'info', 2000);
            }

            if (typeof Settings !== 'undefined') {
                Settings.set('signal_log_enabled', enabled);
            }
        }

        function logSignals(fftData) {
            if (!signalLogEnabled || !fftData) return;

            const now = Date.now();
            if (now - lastSignalLogTime < SIGNAL_LOG_INTERVAL) return;
            lastSignalLogTime = now;

            // Find peaks in spectrum
            const centerFreq = parseFloat(document.getElementById('freq').textContent) || 915;
            const sampleRate = parseFloat(document.getElementById('sr').textContent) || 40;
            const peaks = [];

            // Simple peak detection
            for (let i = 10; i < fftData.length - 10; i++) {
                const magDb = rawToDb(fftData[i]);
                if (magDb > SIGNAL_LOG_THRESHOLD) {
                    // Check if it's a local maximum
                    let isPeak = true;
                    for (let j = i - 5; j <= i + 5; j++) {
                        if (j !== i && fftData[j] > fftData[i]) {
                            isPeak = false;
                            break;
                        }
                    }

                    if (isPeak) {
                        const binFreq = centerFreq - sampleRate / 2 + (i / fftData.length) * sampleRate;
                        peaks.push({
                            timestamp: new Date().toISOString(),
                            frequency: binFreq.toFixed(6),
                            power: magDb.toFixed(2),
                            centerFreq: centerFreq,
                            sampleRate: sampleRate
                        });
                    }
                }
            }

            // Add peaks to log
            if (peaks.length > 0) {
                signalLog.push(...peaks);
                console.log(`Logged ${peaks.length} signals`);

                // Limit log size to 10000 entries
                if (signalLog.length > 10000) {
                    signalLog = signalLog.slice(-5000);
                }
            }
        }

        function downloadSignalLog() {
            if (signalLog.length === 0) {
                showNotification('No signals logged yet', 'warning', 3000);
                return;
            }

            // Build CSV
            let csv = 'Timestamp,Frequency (MHz),Power (dBm),Center Freq (MHz),Sample Rate (MHz)\n';
            signalLog.forEach(signal => {
                csv += `${signal.timestamp},${signal.frequency},${signal.power},${signal.centerFreq},${signal.sampleRate}\n`;
            });

            // Download
            if (typeof Utils !== 'undefined' && Utils.downloadFile) {
                const filename = `signal_log_${new Date().toISOString().split('T')[0]}.csv`;
                Utils.downloadFile(csv, filename, 'text/csv');
                showNotification(`Downloaded ${signalLog.length} logged signals to ${filename}`, 'success', 3000);
            }
        }

        // Simple spectrum drawer for IQ and XCORR workspace canvases
        function drawSimpleSpectrum(data, ctx, width, height, selection) {
            if (!data || !ctx) return;

            // Clear
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, width, height);

            // Draw grid
            ctx.strokeStyle = 'rgba(80, 80, 80, 0.3)';
            ctx.lineWidth = 1;
            for (let i = 0; i <= 10; i++) {
                const y = (height / 10) * i;
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(width, y);
                ctx.stroke();
            }

            // Draw spectrum line
            const dbRange = spectrumMaxDb - spectrumMinDb;
            ctx.strokeStyle = '#00ffff';
            ctx.lineWidth = 2;
            ctx.beginPath();

            for (let x = 0; x < width; x++) {
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
                const raw = data[fftIdx];
                const magDb = rawToDb(raw);
                const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
                const y = height - (normalizedMag * height);

                if (x === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }

            ctx.stroke();

            // Add gradient fill
            ctx.lineTo(width, height);
            ctx.lineTo(0, height);
            ctx.closePath();
            const gradient = ctx.createLinearGradient(0, 0, 0, height);
            gradient.addColorStop(0, 'rgba(0, 255, 255, 0.3)');
            gradient.addColorStop(1, 'rgba(0, 255, 255, 0.05)');
            ctx.fillStyle = gradient;
            ctx.fill();

            // Draw selection highlight AFTER spectrum (so it's on top)
            if (selection && selection.active) {
                const leftX = (selection.leftPercent / 100) * width;
                const rightX = (selection.rightPercent / 100) * width;

                // Draw highlighted background with yellow tint for contrast
                ctx.fillStyle = 'rgba(255, 255, 0, 0.2)';
                ctx.fillRect(leftX, 0, rightX - leftX, height);

                // Draw bright yellow edge lines for clear boundaries
                ctx.strokeStyle = '#ffff00';
                ctx.lineWidth = 3;
                ctx.beginPath();
                ctx.moveTo(leftX, 0);
                ctx.lineTo(leftX, height);
                ctx.moveTo(rightX, 0);
                ctx.lineTo(rightX, height);
                ctx.stroke();
            }
        }

        // Draw FFT spectrum (SDR++/sigdigger style) with double buffering
        function drawSpectrum(data, data2) {
            if (!data) return;

            // Log signals if enabled (only once)
            logSignals(data);

            // Use SpectrumDisplay module if available
            if (typeof SpectrumDisplay !== 'undefined') {
                SpectrumDisplay.draw(data, data2);

                // Also draw to IQ and XCORR workspace spectrum canvases if active
                const iqWorkspace = document.getElementById('workspace-iq');
                const xcorrWorkspace = document.getElementById('workspace-xcorr');

                if (iqWorkspace && iqWorkspace.classList.contains('active')) {
                    const iqSpecCanvas = document.getElementById('iq_spectrum');
                    if (iqSpecCanvas && iqSpecCanvas.width > 0) {
                        const ctx = iqSpecCanvas.getContext('2d');
                        const rect = iqSpecCanvas.getBoundingClientRect();
                        drawSimpleSpectrum(data, ctx, rect.width, rect.height, iqSelection);
                    }
                }

                if (xcorrWorkspace && xcorrWorkspace.classList.contains('active')) {
                    const xcorrSpecCanvas = document.getElementById('xcorr_spectrum');
                    if (xcorrSpecCanvas && xcorrSpecCanvas.width > 0) {
                        const ctx = xcorrSpecCanvas.getContext('2d');
                        const rect = xcorrSpecCanvas.getBoundingClientRect();
                        drawSimpleSpectrum(data, ctx, rect.width, rect.height, xcorrSelection);
                    }
                }
            } else {
                // Fallback to inline rendering
                // Update peak hold data (only once for channel 1)
                if (peakHoldEnabled) {
                    if (!peakHoldData || peakHoldData.length !== data.length) {
                        peakHoldData = new Uint8Array(data);
                    } else {
                        for (let i = 0; i < data.length; i++) {
                            if (data[i] > peakHoldData[i]) {
                                peakHoldData[i] = data[i];
                            }
                        }
                    }
                }

                // Check if we're in dual-channel mode
                const chSelect = document.getElementById('channel_select').value;
                const isDualChannel = (chSelect === 'both' && data2);

                if (isDualChannel) {
                    // Draw both channels side-by-side
                    drawSpectrumToCanvas(data, spectrumOffscreenCtx, spectrumOffscreen, spectrumCtx);
                    drawSpectrumToCanvas(data2, spectrumOffscreenCtx2, spectrumOffscreen2, spectrumCtx2);
                } else {
                    // Single channel mode - draw to first canvas only
                    drawSpectrumToCanvas(data, spectrumOffscreenCtx, spectrumOffscreen, spectrumCtx);
                }
            }
        }

        // Helper function to draw a single spectrum to specific canvas/context
        function drawSpectrumToCanvas(data, offscreenCtx, offscreenCanvas, finalCtx) {
            if (!data) return;

            // Render to offscreen canvas (double buffering)
            const width = offscreenCanvas.width;
            const height = offscreenCanvas.height;

            // Clear offscreen canvas with dark background
            offscreenCtx.fillStyle = '#0a0a0a';
            offscreenCtx.fillRect(0, 0, width, height);

            // Draw horizontal grid lines (more visible)
            offscreenCtx.strokeStyle = 'rgba(80, 80, 80, 0.3)';
            offscreenCtx.lineWidth = 1;
            for (let i = 0; i <= 10; i++) {
                const y = (height / 10) * i;
                offscreenCtx.beginPath();
                offscreenCtx.moveTo(0, y);
                offscreenCtx.lineTo(width, y);
                offscreenCtx.stroke();
            }

            // Draw vertical grid lines
            offscreenCtx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
            for (let i = 0; i <= 10; i++) {
                const x = (width / 10) * i;
                offscreenCtx.beginPath();
                offscreenCtx.moveTo(x, 0);
                offscreenCtx.lineTo(x, height);
                offscreenCtx.stroke();
            }

            // Enable smoothing for better visual quality
            offscreenCtx.imageSmoothingEnabled = true;
            offscreenCtx.imageSmoothingQuality = 'high';

            // Calculate Y-axis mapping based on current zoom/scroll range
            const dbRange = spectrumMaxDb - spectrumMinDb;

            // Store path for gradient fill
            offscreenCtx.beginPath();
            offscreenCtx.moveTo(0, height);

            // Draw spectrum line with gradient color based on amplitude
            const points = [];
            for (let x = 0; x < width; x++) {
                // Map canvas X to FFT bin, respecting zoom
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
                const raw = data[fftIdx];
                const magDb = rawToDb(raw);

                // Map magnitude to visible range
                const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
                const y = height - (normalizedMag * height);
                points.push({ x: x, y: y, mag: normalizedMag });

                offscreenCtx.lineTo(x, y);
            }

            // Complete the path for fill
            offscreenCtx.lineTo(width, height);
            offscreenCtx.closePath();

            // Create gradient fill (green-yellow gradient like SDR++)
            const gradient = offscreenCtx.createLinearGradient(0, 0, 0, height);
            gradient.addColorStop(0, 'rgba(255, 255, 0, 0.4)');    // Yellow at top (strong signals)
            gradient.addColorStop(0.3, 'rgba(0, 255, 100, 0.3)');  // Green
            gradient.addColorStop(0.7, 'rgba(0, 255, 200, 0.2)');  // Cyan
            gradient.addColorStop(1, 'rgba(0, 100, 255, 0.1)');    // Blue at bottom (noise floor)

            offscreenCtx.fillStyle = gradient;
            offscreenCtx.fill();

            // Draw spectrum line with variable color
            offscreenCtx.lineJoin = 'round';
            offscreenCtx.lineCap = 'round';
            offscreenCtx.lineWidth = 1.5;

            // Draw line with gradient based on signal strength
            for (let i = 0; i < points.length - 1; i++) {
                const p1 = points[i];
                const p2 = points[i + 1];

                // Color based on magnitude (green-yellow gradient)
                const mag = (p1.mag + p2.mag) / 2;
                if (mag > 0.8) {
                    offscreenCtx.strokeStyle = '#ffff00';  // Yellow for strong signals
                } else if (mag > 0.5) {
                    offscreenCtx.strokeStyle = '#88ff00';  // Yellow-green
                } else if (mag > 0.3) {
                    offscreenCtx.strokeStyle = '#00ff88';  // Green-cyan
                } else {
                    offscreenCtx.strokeStyle = '#00ffff';  // Cyan for weak signals
                }

                offscreenCtx.beginPath();
                offscreenCtx.moveTo(p1.x, p1.y);
                offscreenCtx.lineTo(p2.x, p2.y);
                offscreenCtx.stroke();
            }

            // Draw dB scale labels (now shows scrolled/zoomed range)
            offscreenCtx.fillStyle = '#888';
            offscreenCtx.font = '10px monospace';
            offscreenCtx.textAlign = 'right';
            for (let i = 0; i <= 10; i++) {
                const y = (height / 10) * i;
                const dbValue = Math.floor(spectrumMaxDb - (i / 10) * dbRange);
                offscreenCtx.fillText(dbValue + ' dB', width - 5, y + 3);
            }

            // Draw Y-axis range indicator (top-left of spectrum)
            // Always show if not at default range
            if (spectrumMinDb !== -100 || spectrumMaxDb !== -10) {
                offscreenCtx.fillStyle = 'rgba(255, 255, 0, 0.8)';
                offscreenCtx.font = 'bold 11px monospace';
                offscreenCtx.textAlign = 'left';
                offscreenCtx.fillText(`Range: ${spectrumMinDb.toFixed(0)}-${spectrumMaxDb.toFixed(0)} dBFS`, 5, 15);
                offscreenCtx.fillStyle = 'rgba(255, 255, 255, 0.6)';
                offscreenCtx.font = '10px monospace';
                offscreenCtx.fillText('(Scroll: pan, Ctrl+Scroll: zoom, DblClick: reset)', 5, 28);
            }

            // Draw peak hold overlay
            if (peakHoldEnabled && peakHoldData) {
                offscreenCtx.strokeStyle = 'rgba(255, 0, 0, 0.6)';
                offscreenCtx.lineWidth = 1;
                offscreenCtx.beginPath();

                for (let x = 0; x < width; x++) {
                    const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                    const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
                    const raw = peakHoldData[fftIdx];
                    const magDb = rawToDb(raw);
                    const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
                    const y = height - (normalizedMag * height);

                    if (x === 0) {
                        offscreenCtx.moveTo(x, y);
                    } else {
                        offscreenCtx.lineTo(x, y);
                    }
                }

                offscreenCtx.stroke();

                // Draw "Peak Hold" label
                offscreenCtx.fillStyle = 'rgba(255, 0, 0, 0.8)';
                offscreenCtx.font = 'bold 10px monospace';
                offscreenCtx.textAlign = 'left';
                offscreenCtx.fillText('PEAK HOLD', 5, height - 5);
            }

            // Update and draw min hold trace
            if (minHoldEnabled) {
                if (!minHoldTrace || minHoldTrace.length !== data.length) {
                    minHoldTrace = new Uint8Array(data);
                } else {
                    for (let i = 0; i < data.length; i++) {
                        if (data[i] < minHoldTrace[i]) {
                            minHoldTrace[i] = data[i];
                        }
                    }
                }

                offscreenCtx.strokeStyle = 'rgba(0, 128, 255, 0.6)';
                offscreenCtx.lineWidth = 1;
                offscreenCtx.beginPath();

                for (let x = 0; x < width; x++) {
                    const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                    const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
                    const raw = minHoldTrace[fftIdx];
                    const magDb = rawToDb(raw);
                    const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
                    const y = height - (normalizedMag * height);

                    if (x === 0) {
                        offscreenCtx.moveTo(x, y);
                    } else {
                        offscreenCtx.lineTo(x, y);
                    }
                }

                offscreenCtx.stroke();

                offscreenCtx.fillStyle = 'rgba(0, 128, 255, 0.8)';
                offscreenCtx.font = 'bold 10px monospace';
                offscreenCtx.textAlign = 'left';
                offscreenCtx.fillText('MIN HOLD', 80, height - 5);
            }

            // Draw reference level markers
            if (refMarkersEnabled) {
                const refLevels = [-100, -80, -60, -40, -20, 0];
                offscreenCtx.strokeStyle = 'rgba(255, 255, 0, 0.3)';
                offscreenCtx.setLineDash([5, 5]);
                offscreenCtx.lineWidth = 1;

                refLevels.forEach(dbLevel => {
                    if (dbLevel >= spectrumMinDb && dbLevel <= spectrumMaxDb) {
                        const normalizedPos = (dbLevel - spectrumMinDb) / dbRange;
                        const y = height - (normalizedPos * height);

                        offscreenCtx.beginPath();
                        offscreenCtx.moveTo(0, y);
                        offscreenCtx.lineTo(width, y);
                        offscreenCtx.stroke();

                        offscreenCtx.fillStyle = 'rgba(255, 255, 0, 0.7)';
                        offscreenCtx.font = '9px monospace';
                        offscreenCtx.textAlign = 'left';
                        offscreenCtx.fillText(`${dbLevel} dB`, 2, y - 2);
                    }
                });

                offscreenCtx.setLineDash([]);
            }

            // Draw bandwidth measurement points and result
            if (bwMeasureEnabled && bwMeasurePoints.length > 0) {
                bwMeasurePoints.forEach((point, idx) => {
                    // Convert normalized position to canvas X coordinate
                    const canvasX = point.normalizedX * width;

                    // Draw marker
                    offscreenCtx.fillStyle = '#ff00ff';
                    offscreenCtx.beginPath();
                    offscreenCtx.arc(canvasX, height - 20, 5, 0, 2 * Math.PI);
                    offscreenCtx.fill();

                    // Draw vertical line
                    offscreenCtx.strokeStyle = 'rgba(255, 0, 255, 0.5)';
                    offscreenCtx.setLineDash([3, 3]);
                    offscreenCtx.beginPath();
                    offscreenCtx.moveTo(canvasX, 0);
                    offscreenCtx.lineTo(canvasX, height);
                    offscreenCtx.stroke();
                    offscreenCtx.setLineDash([]);

                    // Label
                    offscreenCtx.fillStyle = '#ff00ff';
                    offscreenCtx.font = '10px monospace';
                    offscreenCtx.fillText(`${idx + 1}`, canvasX + 8, height - 15);
                });

                // Draw measurement result if two points selected
                if (bwMeasurePoints.length === 2) {
                    const bw = Math.abs(bwMeasurePoints[1].freq - bwMeasurePoints[0].freq);
                    const canvasX0 = bwMeasurePoints[0].normalizedX * width;
                    const canvasX1 = bwMeasurePoints[1].normalizedX * width;
                    const centerX = (canvasX0 + canvasX1) / 2;

                    // Draw bandwidth span
                    offscreenCtx.strokeStyle = '#ff00ff';
                    offscreenCtx.lineWidth = 2;
                    offscreenCtx.beginPath();
                    offscreenCtx.moveTo(canvasX0, height - 30);
                    offscreenCtx.lineTo(canvasX1, height - 30);
                    offscreenCtx.stroke();

                    // Arrows
                    [canvasX0, canvasX1].forEach(x => {
                        offscreenCtx.beginPath();
                        offscreenCtx.moveTo(x, height - 30);
                        offscreenCtx.lineTo(x, height - 35);
                        offscreenCtx.stroke();
                    });

                    // Update display
                    const bwText = bw >= 1e6 ? (bw / 1e6).toFixed(3) + ' MHz' : (bw / 1e3).toFixed(1) + ' kHz';
                    document.getElementById('bw_measurement').textContent = bwText;
                }
            }

            // Draw markers on spectrum (from marker_system.js)
            if (typeof drawMarkersOnSpectrum === 'function') {
                drawMarkersOnSpectrum(offscreenCtx, width, height);
            }

            // Copy offscreen canvas to visible canvas (double buffering - eliminates flicker)
            finalCtx.drawImage(offscreenCanvas, 0, 0);
        }

        // Toggle spectrum display
        function toggleSpectrum() {
            showSpectrum = !showSpectrum;
            const button = document.getElementById('spectrum_toggle');
            const timeAxis = document.getElementById('time-axis');
            const freqAxis = document.getElementById('freq-axis');
            const spectrum2Canvas = document.getElementById('spectrum2');

            if (showSpectrum) {
                spectrumCanvas.style.display = 'block';
                if (spectrum2Canvas) {
                    spectrum2Canvas.style.display = 'block';
                }
                button.classList.add('active');
                timeAxis.style.top = '250px';
            } else {
                spectrumCanvas.style.display = 'none';
                if (spectrum2Canvas) {
                    spectrum2Canvas.style.display = 'none';
                }
                button.classList.remove('active');
                timeAxis.style.top = '50px';
            }

            resizeCanvas();
        }

        // Toggle IQ constellation display
        function toggleIQ() {
            showIQ = !showIQ;
            const button = document.getElementById('iq_toggle');
            const panel = document.getElementById('iq_constellation');

            if (showIQ) {
                panel.style.display = 'block';
                button.classList.add('active');
                // Canvas size is handled by CSS - don't manually resize (it clears the canvas!)
            } else {
                panel.style.display = 'none';
                button.classList.remove('active');
            }

            // No need to call resizeCanvas() - IQ plot doesn't affect waterfall layout
        }

        // Toggle cross-correlation display
        function toggleXCorr() {
            showXCorr = !showXCorr;
            const button = document.getElementById('xcorr_toggle');
            const panel = document.getElementById('xcorr_display');

            if (showXCorr) {
                panel.style.display = 'block';
                button.classList.add('active');

                // Initialize canvas size based on container (auto-resize in drawXCorr will handle it)
                const container = document.getElementById('xcorr_canvas_container');
                if (container) {
                    xcorrCanvas.width = container.clientWidth;
                    xcorrCanvas.height = container.clientHeight;
                }
            } else {
                panel.style.display = 'none';
                button.classList.remove('active');
            }

            // Update waterfall positioning without clearing canvas
            const waterfallTop = showSpectrum ? 250 : 50;
            const waterfallBottom = showXCorr ? 210 : 30;

            // Only update CSS dimensions, not canvas buffer (which would clear it)
            canvas.style.top = waterfallTop + 'px';
            canvas.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;
            canvas2.style.top = waterfallTop + 'px';
            canvas2.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;

            // Update divider position if dual channel
            const divider = document.getElementById('channel-divider');
            divider.style.top = waterfallTop + 'px';
            divider.style.height = `calc(100% - ${waterfallTop}px - ${waterfallBottom}px)`;
        }

        // Toggle filter selection mode
        function toggleFilterMode() {
            filterState.enabled = !filterState.enabled;
            const button = document.getElementById('filter_mode_toggle');
            const statusDiv = document.getElementById('filter_status');

            if (filterState.enabled) {
                button.checked = true;
                statusDiv.textContent = 'Click and drag on spectrum to select filter region';
                statusDiv.style.color = '#fa0';
                if (filterState.isFiltered) {
                    updateFilterStatus();
                }
            } else {
                button.checked = false;
                statusDiv.textContent = '';
                // Clear filter when disabling mode
                if (filterState.isFiltered) {
                    filterState.isFiltered = false;
                    filterState.filterStartBin = 0;
                    filterState.filterEndBin = FFT_SIZE - 1;
                    drawSelectionBox(true);
                }
            }
        }

        // Apply filter to selected spectrum region
        function applyFilter(x1, x2) {
            // Calculate FFT bin indices for the selected region (respecting current zoom)
            const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
            const binOffset = Math.floor((x1 / canvas.width) * zoomedBins);
            const binEnd = Math.floor((x2 / canvas.width) * zoomedBins);

            filterState.filterStartBin = zoomState.zoomStartBin + binOffset;
            filterState.filterEndBin = zoomState.zoomStartBin + binEnd;
            filterState.isFiltered = true;

            // Update status display
            updateFilterStatus();

            // Redraw to show filter overlay
            drawSelectionBox();
        }

        // Update filter status text
        function updateFilterStatus() {
            const statusDiv = document.getElementById('filter_status');
            const currentSR = zoomState.fullBandwidth;
            const currentCF = zoomState.centerFreq;
            const binWidth = currentSR / FFT_SIZE;
            const fullStartFreq = currentCF - currentSR / 2;

            const freq1 = fullStartFreq + (filterState.filterStartBin * binWidth);
            const freq2 = fullStartFreq + ((filterState.filterEndBin + 1) * binWidth);
            const bw = freq2 - freq1;

            const formatFreq = (f) => {
                if (f >= 1e9) return (f / 1e9).toFixed(3) + ' GHz';
                if (f >= 1e6) return (f / 1e6).toFixed(2) + ' MHz';
                return (f / 1e3).toFixed(1) + ' kHz';
            };

            statusDiv.textContent = `Filtered: ${formatFreq(bw)} (bins ${filterState.filterStartBin}-${filterState.filterEndBin})`;
            statusDiv.style.color = '#fa0';
        }


        // Phase unwrapping helper
        function unwrapPhase(phase) {
            const unwrapped = new Float32Array(phase.length);
            unwrapped[0] = phase[0];

            for (let i = 1; i < phase.length; i++) {
                let delta = phase[i] - phase[i-1];

                // Detect and correct phase jumps > π
                while (delta > Math.PI) delta -= 2 * Math.PI;
                while (delta < -Math.PI) delta += 2 * Math.PI;

                unwrapped[i] = unwrapped[i-1] + delta;
            }

            return unwrapped;
        }

        // Calculate coherence metric between channels
        function calculateCoherence(magnitude) {
            // Average normalized cross-correlation magnitude
            // Note: True coherence requires |Gxy|²/(Gxx*Gyy), but we only have |Gxy| here
            // So we normalize by the maximum value to get a 0-1 range metric

            // Find max magnitude for normalization
            let max = 0;
            for (let i = 0; i < magnitude.length; i++) {
                if (magnitude[i] > max) max = magnitude[i];
            }

            if (max === 0) return 0;

            // Calculate average normalized magnitude
            let sum = 0;
            for (let i = 0; i < magnitude.length; i++) {
                sum += magnitude[i] / max;
            }
            return sum / magnitude.length;
        }

        // Calculate time delay from phase slope
        function calculateTimeDelay(phase, sampleRate) {
            // Time delay from phase slope: τ = dφ/dω / (2π)
            // Where dφ/dω is the phase slope vs angular frequency
            const unwrapped = unwrapPhase(phase);

            // Use middle 50% of data to avoid edge effects
            const startIdx = Math.floor(phase.length * 0.25);
            const endIdx = Math.floor(phase.length * 0.75);

            const phaseDiff = unwrapped[endIdx] - unwrapped[startIdx];
            const freqDiff = (endIdx - startIdx) * (sampleRate / FFT_SIZE);

            // delay = phase_slope / (2πΔf)
            // Positive phase slope = signal arrives later on CH2
            const delay = phaseDiff / (2 * Math.PI * freqDiff);

            return delay;
        }

        // Enhanced cross-correlation with frequency-domain coherence and group delay
        function drawXCorr(magnitude, phase) {
            // Determine which canvas to render to (prioritize fullscreen if available)
            const xcorrWorkspace = document.getElementById('workspace-xcorr');
            const xcorrFullCanvas = document.getElementById('xcorr_fullscreen');

            let targetCanvas, targetCtx, width, height;

            if (xcorrWorkspace && xcorrWorkspace.classList.contains('active') &&
                xcorrFullCanvas && xcorrFullCanvas.width > 0) {
                // Render to fullscreen canvas in XCORR workspace
                targetCanvas = xcorrFullCanvas;
                targetCtx = xcorrFullCanvas.getContext('2d');
                const rect = xcorrFullCanvas.getBoundingClientRect();
                width = rect.width;
                height = rect.height;
                console.log(`[drawXCorr] Rendering to fullscreen canvas: ${width}x${height}`);
            } else {
                // Render to small panel canvas
                const container = document.getElementById('xcorr_canvas_container');
                if (!container) {
                    console.warn('[drawXCorr] No container found');
                    return;
                }
                const containerWidth = container.clientWidth;
                const containerHeight = container.clientHeight;

                if (xcorrCanvas.width !== containerWidth || xcorrCanvas.height !== containerHeight) {
                    xcorrCanvas.width = containerWidth;
                    xcorrCanvas.height = containerHeight;
                }

                targetCanvas = xcorrCanvas;
                targetCtx = xcorrCtx;
                width = xcorrCanvas.width;
                height = xcorrCanvas.height;
                console.log(`[drawXCorr] Rendering to small canvas: ${width}x${height}`);
            }
            const plotHeight = height / 3 - 5;  // Three plots vertically

            // Clear canvas
            targetCtx.fillStyle = '#0a0a0a';
            targetCtx.fillRect(0, 0, width, height);

            // Calculate metrics
            const coherence = calculateCoherence(magnitude);
            const sampleRate = zoomState.fullBandwidth || 40000000;
            const timeDelay = calculateTimeDelay(phase, sampleRate);
            const avgPhase = phase.reduce((a, b) => a + b, 0) / phase.length;

            // Calculate frequency-domain coherence (normalized magnitude)
            const maxMag = Math.max(...magnitude);
            const coherenceSpectrum = magnitude.map(m => m / Math.max(maxMag, 1e-10));

            // Calculate group delay (derivative of unwrapped phase)
            const unwrappedPhase = unwrapPhase(phase);
            const groupDelay = new Float32Array(unwrappedPhase.length - 1);
            const freqStep = sampleRate / FFT_SIZE;

            for (let i = 0; i < groupDelay.length; i++) {
                const phaseDiff = unwrappedPhase[i + 1] - unwrappedPhase[i];
                groupDelay[i] = -phaseDiff / (2 * Math.PI * freqStep) * 1e9;  // Convert to nanoseconds
            }

            // Update display values (improved)
            const peakCoherence = Math.max(...coherenceSpectrum);
            const meanCoherence = coherenceSpectrum.reduce((a, b) => a + b) / coherenceSpectrum.length;

            document.getElementById('xcorr_coherence').textContent = `${peakCoherence.toFixed(3)} (μ=${meanCoherence.toFixed(3)})`;
            document.getElementById('xcorr_delay').textContent = (timeDelay * 1e9).toFixed(2) + ' ns';
            document.getElementById('xcorr_phase').textContent = (avgPhase * 180 / Math.PI).toFixed(1) + '°';

            // ===== PLOT 1: Coherence Spectrum (top third) =====
            const plot1Y = 0;
            const plot1Height = plotHeight;

            // Grid
            targetCtx.strokeStyle = 'rgba(80, 80, 80, 0.25)';
            targetCtx.lineWidth = 0.5;
            for (let i = 0; i <= 4; i++) {
                const y = plot1Y + (plot1Height / 4) * i;
                targetCtx.beginPath();
                targetCtx.moveTo(0, y);
                targetCtx.lineTo(width, y);
                targetCtx.stroke();
            }

            // Vertical grid
            for (let i = 0; i <= 10; i++) {
                const x = (width / 10) * i;
                targetCtx.beginPath();
                targetCtx.moveTo(x, plot1Y);
                targetCtx.lineTo(x, plot1Y + plot1Height);
                targetCtx.stroke();
            }

            // Fill area under coherence curve
            targetCtx.fillStyle = 'rgba(0, 255, 100, 0.15)';
            targetCtx.beginPath();
            targetCtx.moveTo(0, plot1Y + plot1Height);

            for (let x = 0; x < width; x++) {
                const idx = Math.floor((x / width) * coherenceSpectrum.length);
                const coh = Math.min(1.0, coherenceSpectrum[idx]);
                const y = plot1Y + plot1Height * (1 - coh);
                targetCtx.lineTo(x, y);
            }

            targetCtx.lineTo(width, plot1Y + plot1Height);
            targetCtx.closePath();
            targetCtx.fill();

            // Draw coherence line with gradient color
            for (let x = 0; x < width - 1; x++) {
                const idx = Math.floor((x / width) * coherenceSpectrum.length);
                const coh = Math.min(1.0, coherenceSpectrum[idx]);
                const y = plot1Y + plot1Height * (1 - coh);

                // Color by coherence: red (low) -> yellow -> green (high)
                let r, g, b;
                if (coh < 0.5) {
                    r = 255;
                    g = coh * 2 * 255;
                    b = 0;
                } else {
                    r = (1 - coh) * 2 * 255;
                    g = 255;
                    b = 0;
                }

                targetCtx.strokeStyle = `rgb(${r},${g},${b})`;
                targetCtx.lineWidth = 1.5;
                targetCtx.beginPath();
                targetCtx.moveTo(x, y);
                targetCtx.lineTo(x + 1, y);
                targetCtx.stroke();
            }

            // Threshold lines
            targetCtx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            targetCtx.setLineDash([5, 5]);
            targetCtx.lineWidth = 1;

            // 0.7 threshold (good coherence)
            let y07 = plot1Y + plot1Height * (1 - 0.7);
            targetCtx.beginPath();
            targetCtx.moveTo(0, y07);
            targetCtx.lineTo(width, y07);
            targetCtx.stroke();

            targetCtx.fillStyle = '#888';
            targetCtx.font = '8px monospace';
            targetCtx.fillText('0.7', 3, y07 - 2);

            targetCtx.setLineDash([]);

            // Label
            targetCtx.fillStyle = '#0f0';
            targetCtx.font = '10px monospace';
            targetCtx.textAlign = 'left';
            targetCtx.fillText('Coherence (freq domain)', 5, plot1Y + 12);

            // ===== PLOT 2: Magnitude (middle third) =====
            const plot2Y = plot1Height + 10;
            const plot2Height = plotHeight;

            // Grid
            targetCtx.strokeStyle = 'rgba(80, 80, 80, 0.25)';
            targetCtx.lineWidth = 0.5;
            for (let i = 0; i <= 4; i++) {
                const y = plot2Y + (plot2Height / 4) * i;
                targetCtx.beginPath();
                targetCtx.moveTo(0, y);
                targetCtx.lineTo(width, y);
                targetCtx.stroke();
            }

            // Draw magnitude with peak highlighting
            const peakIdx = magnitude.indexOf(Math.max(...magnitude));
            const peakX = (peakIdx / magnitude.length) * width;

            targetCtx.strokeStyle = '#00ffff';
            targetCtx.lineWidth = 1.5;
            targetCtx.beginPath();

            for (let x = 0; x < width; x++) {
                const idx = Math.floor((x / width) * magnitude.length);
                const mag = Math.min(1.0, magnitude[idx]);
                const y = plot2Y + plot2Height * (1 - mag);
                if (x === 0) {
                    targetCtx.moveTo(x, y);
                } else {
                    targetCtx.lineTo(x, y);
                }
            }
            targetCtx.stroke();

            // Mark peak
            targetCtx.strokeStyle = '#ff0';
            targetCtx.fillStyle = '#ff0';
            targetCtx.lineWidth = 2;
            const peakY = plot2Y + plot2Height * (1 - magnitude[peakIdx] / Math.max(maxMag, 1e-10));
            targetCtx.beginPath();
            targetCtx.arc(peakX, peakY, 3, 0, 2 * Math.PI);
            targetCtx.fill();

            // Peak label
            targetCtx.font = '8px monospace';
            targetCtx.fillText(`Peak: ${magnitude[peakIdx].toFixed(3)}`, peakX + 5, peakY - 5);

            // Label
            targetCtx.fillStyle = '#0ff';
            targetCtx.font = '10px monospace';
            targetCtx.fillText('Cross-Correlation Magnitude', 5, plot2Y + 12);

            // ===== PLOT 3: Group Delay (bottom third) =====
            const plot3Y = (plot1Height + plot2Height) + 20;
            const plot3Height = plotHeight;

            // Grid
            targetCtx.strokeStyle = 'rgba(80, 80, 80, 0.25)';
            targetCtx.lineWidth = 0.5;
            for (let i = 0; i <= 4; i++) {
                const y = plot3Y + (plot3Height / 4) * i;
                targetCtx.beginPath();
                targetCtx.moveTo(0, y);
                targetCtx.lineTo(width, y);
                targetCtx.stroke();
            }

            // Auto-scale group delay
            const gdMin = Math.min(...groupDelay);
            const gdMax = Math.max(...groupDelay);
            const gdRange = gdMax - gdMin || 1;

            // Draw group delay
            targetCtx.strokeStyle = '#ffa500';
            targetCtx.lineWidth = 1.5;
            targetCtx.beginPath();

            for (let x = 0; x < width; x++) {
                const idx = Math.floor((x / width) * groupDelay.length);
                const gd = groupDelay[idx];
                const gdNorm = (gd - gdMin) / gdRange;
                const y = plot3Y + plot3Height * (1 - gdNorm);
                if (x === 0) {
                    targetCtx.moveTo(x, y);
                } else {
                    targetCtx.lineTo(x, y);
                }
            }
            targetCtx.stroke();

            // Zero reference line
            if (gdMin < 0 && gdMax > 0) {
                const zeroY = plot3Y + plot3Height * (1 - (-gdMin / gdRange));
                targetCtx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
                targetCtx.setLineDash([5, 5]);
                targetCtx.beginPath();
                targetCtx.moveTo(0, zeroY);
                targetCtx.lineTo(width, zeroY);
                targetCtx.stroke();
                targetCtx.setLineDash([]);

                targetCtx.fillStyle = '#888';
                targetCtx.font = '8px monospace';
                targetCtx.fillText('0 ns', 3, zeroY - 2);
            }

            // Y-axis labels
            targetCtx.fillStyle = '#888';
            targetCtx.font = '8px monospace';
            targetCtx.textAlign = 'right';
            targetCtx.fillText(gdMax.toFixed(1) + ' ns', width - 3, plot3Y + 10);
            targetCtx.fillText(gdMin.toFixed(1) + ' ns', width - 3, plot3Y + plot3Height - 3);

            // Label
            targetCtx.fillStyle = '#fa0';
            targetCtx.font = '10px monospace';
            targetCtx.textAlign = 'left';
            targetCtx.fillText('Group Delay (dφ/dω)', 5, plot3Y + 12);
        }

        // Track bandwidth for display
        let currentBandwidth = 40000000;

        // Update frequency axis labels
        function updateFreqAxis(centerFreq, sampleRate, bandwidth) {
            const freqAxisEl = document.getElementById('freq-axis');
            currentBandwidth = bandwidth || sampleRate;  // Store for overlay rendering

            // Calculate frequency range based on zoom state
            const binWidth = sampleRate / FFT_SIZE;  // Hz per FFT bin
            const fullStartFreq = centerFreq - sampleRate / 2;

            // When zoomed, show only the zoomed frequency range
            const startBinOffset = zoomState.zoomStartBin * binWidth;
            const endBinOffset = (zoomState.zoomEndBin + 1) * binWidth;

            const startFreq = fullStartFreq + startBinOffset;
            const endFreq = fullStartFreq + endBinOffset;
            const displayBandwidth = endFreq - startFreq;

            const formatFreq = (freq) => {
                if (freq >= 1e9) return (freq / 1e9).toFixed(3) + ' GHz';
                if (freq >= 1e6) return (freq / 1e6).toFixed(2) + ' MHz';
                if (freq >= 1e3) return (freq / 1e3).toFixed(1) + ' kHz';
                return freq.toFixed(0) + ' Hz';
            };

            let html = '<span>' + formatFreq(startFreq) + '</span>';

            // Add 3 intermediate labels (using display bandwidth, which accounts for zoom)
            for (let i = 1; i <= 3; i++) {
                const freq = startFreq + (displayBandwidth * i / 4);
                html += '<span>' + formatFreq(freq) + '</span>';
            }

            html += '<span>' + formatFreq(endFreq) + '</span>';

            // Add bandwidth indicator if BW < SR
            if (bandwidth < sampleRate) {
                const bwSpan = (bandwidth / sampleRate * 100).toFixed(0);
                html += `<span style="color: #ff0; margin-left: 20px;">BW: ${(bandwidth/1e6).toFixed(1)} MHz (${bwSpan}% of SR)</span>`;
            }

            freqAxisEl.innerHTML = html;
        }

        // Update time axis labels
        function updateTimeAxis() {
            const timeAxisEl = document.getElementById('time-axis');
            const canvasHeight = canvas.height;
            const totalSeconds = Math.floor(canvasHeight / measuredFPS);

            let html = '<span style="color: #0ff;">NOW</span>';

            // Add time labels at regular intervals
            const intervals = 5;
            for (let i = 1; i <= intervals; i++) {
                const seconds = Math.floor(totalSeconds * (i / intervals));
                html += '<span>-' + seconds + 's</span>';
            }

            timeAxisEl.innerHTML = html;
        }

        // Update status
        async function updateStatus() {
            if (isUpdatingStatus) return; // Skip if previous request still running

            isUpdatingStatus = true;
            try {
                const response = await fetchWithTimeout('/status');
                const data = await response.json();
                const ch = document.getElementById('channel_select').value;

                document.getElementById('freq').textContent = (data.freq / 1e6).toFixed(2) + ' MHz';
                document.getElementById('sr').textContent = (data.sr / 1e6).toFixed(1) + ' MHz';
                document.getElementById('gain').textContent = (ch === '1' ? data.g1 : data.g2) + ' dB';

                // Update IQ and XCORR workspace frequency displays
                const iqCenterFreq = document.getElementById('iq_center_freq');
                const xcorrCenterFreq = document.getElementById('xcorr_center_freq');
                const iqSpan = document.getElementById('iq_span');
                const xcorrSpan = document.getElementById('xcorr_span');

                if (iqCenterFreq) iqCenterFreq.textContent = (data.freq / 1e6).toFixed(3) + ' MHz';
                if (xcorrCenterFreq) xcorrCenterFreq.textContent = (data.freq / 1e6).toFixed(3) + ' MHz';
                if (iqSpan) iqSpan.textContent = (data.sr / 1e6).toFixed(2) + ' MHz';
                if (xcorrSpan) xcorrSpan.textContent = (data.sr / 1e6).toFixed(2) + ' MHz';

                // Update control panel inputs with current values (only if not focused)
                const freqInput = document.getElementById('freqInput');
                const srInput = document.getElementById('srInput');
                const bwInput = document.getElementById('bwInput');
                const gain1Input = document.getElementById('gain1Input');
                const gain2Input = document.getElementById('gain2Input');

                if (document.activeElement !== freqInput) {
                    freqInput.value = (data.freq / 1e6).toFixed(2);
                }
                if (document.activeElement !== srInput) {
                    srInput.value = (data.sr / 1e6).toFixed(1);
                }
                if (document.activeElement !== bwInput) {
                    bwInput.value = (data.bw / 1e6).toFixed(1);
                }
                if (document.activeElement !== gain1Input) {
                    gain1Input.value = data.g1;
                }
                if (document.activeElement !== gain2Input) {
                    gain2Input.value = data.g2;
                }

                // Calculate and display frequency resolution
                const binResolution = data.sr / FFT_SIZE;
                let resText = '';
                if (binResolution >= 1000) {
                    resText = (binResolution / 1000).toFixed(2) + ' kHz/bin';
                } else {
                    resText = binResolution.toFixed(0) + ' Hz/bin';
                }
                document.getElementById('resolution').textContent = resText;

                // Update zoom level indicator
                updateZoomIndicator();

                // Update frequency axis
                updateFreqAxis(data.freq, data.sr, data.bw);

                // Update zoom state with current parameters
                updateZoomState(data.freq, data.sr);
            } catch (err) {
                console.error('Status update failed:', err);
            } finally {
                isUpdatingStatus = false;
            }
        }

        // Toggle control panel visibility
        function toggleControlPanel() {
            const panel = document.getElementById('controlPanel');
            const toggle = document.getElementById('toggleControls');
            const isHidden = panel.style.display === 'none' || panel.style.display === '';

            if (isHidden) {
                panel.style.display = 'block';
                toggle.textContent = 'Close';
                toggle.style.background = 'rgba(255, 100, 100, 0.3)';
            } else {
                panel.style.display = 'none';
                toggle.textContent = 'Controls';
                toggle.style.background = '';
            }
        }

        // Apply frequency change
        async function applyFrequency() {
            const freqInput = getElement('freqInput');
            if (!freqInput) return;

            const freq = parseFloat(freqInput.value);
            if (isNaN(freq)) {
                showNotification('Invalid frequency format', 'error');
                return;
            }
            if (freq < CONFIG.FREQ_MIN_MHZ || freq > CONFIG.FREQ_MAX_MHZ) {
                showNotification(`Frequency must be between ${CONFIG.FREQ_MIN_MHZ} and ${CONFIG.FREQ_MAX_MHZ} MHz`, 'warning');
                return;
            }

            const currentSR = zoomState.fullBandwidth || CONFIG.DEFAULT_SAMPLE_RATE;
            await sendControlUpdate(Math.floor(freq * 1e6), currentSR, null, null, null);
        }

        // Apply frequency from preset dropdown
        function applyFrequencyPreset() {
            const presetSelect = getElement('freqPresets');
            if (!presetSelect || !presetSelect.value) return;

            const freq = parseFloat(presetSelect.value);
            const freqInput = getElement('freqInput');
            if (freqInput) {
                freqInput.value = freq.toFixed(2);
            }

            // Apply the frequency
            applyFrequency();

            // Reset dropdown to default selection
            presetSelect.selectedIndex = 0;

            // Save preset usage to Settings if available
            if (typeof Settings !== 'undefined') {
                Settings.set('last_preset_frequency', freq);
            }

            showNotification(`Tuned to ${freq} MHz`, 'success', 2000);
        }

        // Apply sample rate change
        async function applySampleRate() {
            const srInput = getElement('srInput');
            if (!srInput) return;

            const sr = parseFloat(srInput.value);
            if (isNaN(sr)) {
                showNotification('Invalid sample rate format', 'error');
                return;
            }
            if (sr < CONFIG.SR_MIN_MHZ || sr > CONFIG.SR_MAX_MHZ) {
                showNotification(`Sample rate must be between ${CONFIG.SR_MIN_MHZ} and ${CONFIG.SR_MAX_MHZ} MHz`, 'warning');
                return;
            }

            const currentFreq = zoomState.centerFreq || 915000000;
            await sendControlUpdate(currentFreq, Math.floor(sr * 1e6), null, null, null);
        }

        // Apply bandwidth change
        async function applyBandwidth() {
            const bwInput = getElement('bwInput');
            if (!bwInput) return;

            const bw = parseFloat(bwInput.value);
            if (isNaN(bw)) {
                showNotification('Invalid bandwidth format', 'error');
                return;
            }
            if (bw < CONFIG.SR_MIN_MHZ || bw > CONFIG.SR_MAX_MHZ) {
                showNotification(`Bandwidth must be between ${CONFIG.SR_MIN_MHZ} and ${CONFIG.SR_MAX_MHZ} MHz`, 'warning');
                return;
            }

            const currentFreq = zoomState.centerFreq || 915000000;
            const currentSR = zoomState.fullBandwidth || CONFIG.DEFAULT_SAMPLE_RATE;
            await sendControlUpdate(currentFreq, currentSR, Math.floor(bw * 1e6), null, null);
        }

        // Apply gain RX1 change
        async function applyGain1() {
            const gainInput = getElement('gain1Input');
            if (!gainInput) return;

            const gain = parseInt(gainInput.value);
            if (isNaN(gain)) {
                showNotification('Invalid gain format', 'error');
                return;
            }
            if (gain < CONFIG.GAIN_MIN_DB || gain > CONFIG.GAIN_MAX_DB) {
                showNotification(`Gain must be between ${CONFIG.GAIN_MIN_DB} and ${CONFIG.GAIN_MAX_DB} dB`, 'warning');
                return;
            }

            await sendControlUpdate(null, null, null, gain, null);
        }

        // Apply gain RX2 change
        async function applyGain2() {
            const gainInput = getElement('gain2Input');
            if (!gainInput) return;

            const gain = parseInt(gainInput.value);
            if (isNaN(gain)) {
                showNotification('Invalid gain format', 'error');
                return;
            }
            if (gain < CONFIG.GAIN_MIN_DB || gain > CONFIG.GAIN_MAX_DB) {
                showNotification(`Gain must be between ${CONFIG.GAIN_MIN_DB} and ${CONFIG.GAIN_MAX_DB} dB`, 'warning');
                return;
            }

            await sendControlUpdate(null, null, null, null, gain);
        }

        function updateWaterfallIntensity(value) {
            waterfallIntensity = parseFloat(value);
            document.getElementById('intensityValue').textContent = parseFloat(value).toFixed(1) + 'x';

            // Update WaterfallDisplay module
            if (typeof WaterfallDisplay !== 'undefined') {
                WaterfallDisplay.setIntensity(waterfallIntensity);
            }
        }

        function updateWaterfallContrast(value) {
            waterfallContrast = parseFloat(value);
            document.getElementById('contrastValue').textContent = parseFloat(value).toFixed(1) + 'x';

            // Update WaterfallDisplay module
            if (typeof WaterfallDisplay !== 'undefined') {
                WaterfallDisplay.setContrast(waterfallContrast);
            }
        }

        function updateSpectrumRange() {
            spectrumMinDb = parseInt(document.getElementById('spectrumMin').value);
            spectrumMaxDb = parseInt(document.getElementById('spectrumMax').value);
        }

        // Apply quality profile settings
        function applyQualityProfile() {
            const profile = document.getElementById('qualityProfile').value;

            if (profile === 'custom') {
                return;  // User controls everything manually
            }

            // High quality: all features enabled
            if (profile === 'high') {
                // Enable all displays
                if (!showSpectrum) toggleSpectrum();
                if (!showIQ) toggleIQ();
                if (!showXCorr) toggleXCorr();

                console.log('Quality profile: HIGH (all features enabled)');
            }
            // Medium quality: spectrum and IQ only
            else if (profile === 'medium') {
                if (!showSpectrum) toggleSpectrum();
                if (!showIQ) toggleIQ();
                if (showXCorr) toggleXCorr();  // Disable XCorr

                console.log('Quality profile: MEDIUM (spectrum + IQ only)');
            }
            // Low quality: waterfall only
            else if (profile === 'low') {
                if (showSpectrum) toggleSpectrum();  // Disable all extras
                if (showIQ) toggleIQ();
                if (showXCorr) toggleXCorr();

                console.log('Quality profile: LOW (waterfall only)');
            }
        }

        // ===== Configuration Preset Management =====

        // Get current configuration from all UI elements
        function getCurrentConfig() {
            return {
                frequency: parseFloat(document.getElementById('freqInput').value) * 1e6,
                sampleRate: parseFloat(document.getElementById('srInput').value) * 1e6,
                bandwidth: parseFloat(document.getElementById('bwInput').value) * 1e6,
                gain1: parseInt(document.getElementById('gain1Input').value),
                gain2: parseInt(document.getElementById('gain2Input').value),
                waterfallIntensity: parseFloat(document.getElementById('waterfallIntensity').value),
                waterfallContrast: parseFloat(document.getElementById('waterfallContrast').value),
                spectrumMin: parseInt(document.getElementById('spectrumMin').value),
                spectrumMax: parseInt(document.getElementById('spectrumMax').value),
                channel: currentChannel
            };
        }

        // Apply a configuration to all UI elements
        async function applyConfig(config) {
            if (config.frequency !== undefined) {
                document.getElementById('freqInput').value = (config.frequency / 1e6).toFixed(3);
            }
            if (config.sampleRate !== undefined) {
                document.getElementById('srInput').value = (config.sampleRate / 1e6).toFixed(2);
            }
            if (config.bandwidth !== undefined) {
                document.getElementById('bwInput').value = (config.bandwidth / 1e6).toFixed(2);
            }
            if (config.gain1 !== undefined) {
                document.getElementById('gain1Input').value = config.gain1;
            }
            if (config.gain2 !== undefined) {
                document.getElementById('gain2Input').value = config.gain2;
            }
            if (config.waterfallIntensity !== undefined) {
                document.getElementById('waterfallIntensity').value = config.waterfallIntensity;
                updateWaterfallIntensity(config.waterfallIntensity);
            }
            if (config.waterfallContrast !== undefined) {
                document.getElementById('waterfallContrast').value = config.waterfallContrast;
                updateWaterfallContrast(config.waterfallContrast);
            }
            if (config.spectrumMin !== undefined) {
                document.getElementById('spectrumMin').value = config.spectrumMin;
                updateSpectrumRange();
            }
            if (config.spectrumMax !== undefined) {
                document.getElementById('spectrumMax').value = config.spectrumMax;
                updateSpectrumRange();
            }
            if (config.channel !== undefined) {
                currentChannel = config.channel;
            }

            // Apply RF settings to device
            await sendControlUpdate(
                config.frequency || null,
                config.sampleRate || null,
                config.bandwidth || null,
                config.gain1 || null,
                config.gain2 || null
            );
        }

        // Tune up by a given amount in Hz
        async function tuneUp(deltaHz) {
            const freqInput = document.getElementById('freqInput');
            if (!freqInput) return;

            const currentFreq = parseFloat(freqInput.value) * 1e6; // Convert MHz to Hz
            const newFreq = currentFreq + deltaHz;

            freqInput.value = (newFreq / 1e6).toFixed(3);
            await sendControlUpdate(newFreq, null, null, null, null, null);

            // Reset IQ constellation when tuning
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
            }

            console.log(`Tuned UP by ${(deltaHz/1e6).toFixed(3)} MHz to ${(newFreq/1e6).toFixed(3)} MHz`);
        }

        // Tune down by a given amount in Hz
        async function tuneDown(deltaHz) {
            const freqInput = document.getElementById('freqInput');
            if (!freqInput) return;

            const currentFreq = parseFloat(freqInput.value) * 1e6; // Convert MHz to Hz
            const newFreq = currentFreq - deltaHz;

            freqInput.value = (newFreq / 1e6).toFixed(3);
            await sendControlUpdate(newFreq, null, null, null, null, null);

            // Reset IQ constellation when tuning
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
            }

            console.log(`Tuned DOWN by ${(deltaHz/1e6).toFixed(3)} MHz to ${(newFreq/1e6).toFixed(3)} MHz`);
        }

        // Tune to a selected spectrum region
        // Make this a global function so it can be called from event handlers in other script blocks
        // Filter analysis to selected spectrum portion (bandpass filter)
        window.filterToSelection = function(selection) {
            console.log('[Filter to Selection] Function entry, selection:', selection);

            // Calculate bin indices from selection percentages
            const startBin = Math.floor((selection.leftPercent / 100) * FFT_SIZE);
            const endBin = Math.floor((selection.rightPercent / 100) * FFT_SIZE);

            console.log(`[Filter to Selection] Filtering to bins ${startBin} - ${endBin} (${selection.leftPercent.toFixed(1)}% - ${selection.rightPercent.toFixed(1)}%)`);

            // Update filter state to pass to /iq_data and /xcorr_data endpoints
            filterState.isFiltered = true;
            filterState.filterStartBin = startBin;
            filterState.filterEndBin = endBin;
            filterState.leftPercent = selection.leftPercent;
            filterState.rightPercent = selection.rightPercent;

            // Calculate center frequency of selection for display
            const freqInput = document.getElementById('freqInput');
            const sampleRateInput = document.getElementById('srInput');

            if (freqInput && sampleRateInput) {
                const currentCenterFreq = parseFloat(freqInput.value) * 1e6;
                const sampleRate = parseFloat(sampleRateInput.value) * 1e6;
                const displayBandwidth = sampleRate;
                const startFreq = currentCenterFreq - (displayBandwidth / 2);
                const endFreq = currentCenterFreq + (displayBandwidth / 2);

                const selectedStartFreq = startFreq + (selection.leftPercent / 100) * displayBandwidth;
                const selectedEndFreq = startFreq + (selection.rightPercent / 100) * displayBandwidth;
                const selectedCenterFreq = (selectedStartFreq + selectedEndFreq) / 2;
                const selectedBandwidth = selectedEndFreq - selectedStartFreq;

                console.log(`[Filter] Selected freq range: ${(selectedStartFreq/1e6).toFixed(3)} - ${(selectedEndFreq/1e6).toFixed(3)} MHz`);
                console.log(`[Filter] Center: ${(selectedCenterFreq/1e6).toFixed(3)} MHz, BW: ${(selectedBandwidth/1e3).toFixed(1)} kHz`);

                filterState.selectedCenterFreq = selectedCenterFreq;
                filterState.selectedBandwidth = selectedBandwidth;
            }

            // Reset analysis displays to show filtered data
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
            }
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.clear) {
                EyeDiagram.clear();
            }

            // Show notification
            const bwKHz = filterState.selectedBandwidth ? (filterState.selectedBandwidth / 1e3).toFixed(1) : '?';
            showNotification(`Analysis filtered to ${bwKHz} kHz bandwidth`, 'success', 2000);

            console.log(`✓ Filtered analysis to bins ${startBin}-${endBin}`);
        };

        // Clear spectrum filter (analyze full bandwidth)
        window.clearFilter = function() {
            console.log('[Clear Filter] Removing bandpass filter');

            filterState.isFiltered = false;
            filterState.filterStartBin = 0;
            filterState.filterEndBin = FFT_SIZE - 1;

            // Reset analysis displays
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
            }
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.clear) {
                EyeDiagram.clear();
            }

            showNotification('Filter cleared - analyzing full bandwidth', 'info', 2000);
            console.log('✓ Filter cleared');
        };

        // === IQ WORKSPACE TAB CONTROL FUNCTIONS ===

        // IQ Auto-scale to fit current data
        function iqAutoScale() {
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.autoScale) {
                IQConstellationEnhanced.autoScale();
                console.log('[IQ] Auto-scaled to fit data');
            }
        }

        // Reset IQ view to default zoom/position
        function iqResetView() {
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.resetView) {
                IQConstellationEnhanced.resetView();
                console.log('[IQ] Reset view to default');
            }
        }

        // Clear IQ persistence trail
        function iqClearPersistence() {
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
                console.log('[IQ] Cleared persistence trail');
            }
        }

        // Apply frequency offset
        async function iqApplyOffset() {
            const offsetInput = document.getElementById('iq_freq_offset');
            const freqInput = document.getElementById('freqInput');
            if (!offsetInput || !freqInput) return;

            const offsetKHz = parseFloat(offsetInput.value);
            const currentFreqMHz = parseFloat(freqInput.value);
            const newFreqMHz = currentFreqMHz + (offsetKHz / 1000);

            freqInput.value = newFreqMHz.toFixed(3);
            await sendControlUpdate(newFreqMHz * 1e6, null, null, null, null, null);

            // Reset offset input
            offsetInput.value = '0';

            // Reset IQ constellation
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.reset) {
                IQConstellationEnhanced.reset();
            }

            console.log(`[IQ] Applied offset: ${offsetKHz} kHz, new freq: ${newFreqMHz.toFixed(3)} MHz`);
        }

        // Adjust frequency offset
        function iqFreqOffset(deltaKHz) {
            const offsetInput = document.getElementById('iq_freq_offset');
            if (!offsetInput) return;

            const currentOffset = parseFloat(offsetInput.value) || 0;
            offsetInput.value = (currentOffset + deltaKHz).toFixed(0);
        }

        // Adjust gain
        async function iqGainAdjust(deltaDb) {
            const gainInput = document.getElementById('gainInput');
            const currentGainDisplay = document.getElementById('iq_current_gain');
            if (!gainInput) return;

            const currentGain = parseFloat(gainInput.value);
            const newGain = Math.max(0, Math.min(66, currentGain + deltaDb)); // Clamp to bladeRF range

            gainInput.value = newGain.toFixed(0);
            if (currentGainDisplay) {
                currentGainDisplay.textContent = `${newGain.toFixed(0)} dB`;
            }

            await sendControlUpdate(null, null, newGain, null, null, null);

            console.log(`[IQ] Adjusted gain by ${deltaDb} dB to ${newGain} dB`);
        }

        // Change IF bandwidth
        async function iqBandwidthChange() {
            const bwSelect = document.getElementById('iq_bandwidth_select');
            const bwInput = document.getElementById('bwInput');
            if (!bwSelect) {
                console.warn('[IQ] Bandwidth select not found');
                return;
            }

            const newBwMHz = parseFloat(bwSelect.value);

            // Update main bandwidth input if it exists
            if (bwInput) {
                bwInput.value = newBwMHz.toFixed(2);
            }

            // Validate bandwidth range (bladeRF supports 0.52 - 61.44 MHz)
            if (newBwMHz < 0.52 || newBwMHz > 61.44) {
                console.error(`[IQ] Bandwidth ${newBwMHz} MHz out of valid range (0.52-61.44 MHz)`);
                return;
            }

            console.log(`[IQ] Changing bandwidth to ${newBwMHz} MHz (${newBwMHz * 1e6} Hz)`);

            try {
                await sendControlUpdate(null, null, null, newBwMHz * 1e6, null, null);
                console.log(`[IQ] Bandwidth changed successfully to ${newBwMHz} MHz`);
            } catch (err) {
                console.error(`[IQ] Failed to change bandwidth:`, err);
            }
        }

        // Change persistence
        function iqPersistenceChange(value) {
            const displayEl = document.getElementById('iq_persistence_value');
            if (displayEl) {
                displayEl.textContent = `${value}%`;
            }

            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setPersistence) {
                IQConstellationEnhanced.setPersistence(parseFloat(value) / 100);
            }
        }

        // Change point size
        function iqPointSizeChange(value) {
            const displayEl = document.getElementById('iq_point_size_value');
            if (displayEl) {
                displayEl.textContent = `${value}px`;
            }

            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setPointSize) {
                IQConstellationEnhanced.setPointSize(parseFloat(value));
            }
        }

        // Set zoom level
        function iqZoom(scale) {
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setZoom) {
                IQConstellationEnhanced.setZoom(scale);
                console.log(`[IQ] Zoom set to ${scale}x`);
            }
        }

        // Toggle grid display
        function iqToggleGrid() {
            const checkbox = document.getElementById('iq_show_grid');
            if (!checkbox) return;

            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setShowGrid) {
                IQConstellationEnhanced.setShowGrid(checkbox.checked);
            }
        }

        // Toggle statistics overlay
        function iqToggleStats() {
            const checkbox = document.getElementById('iq_show_stats');
            if (!checkbox) return;

            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setShowStats) {
                IQConstellationEnhanced.setShowStats(checkbox.checked);
            }
        }

        // Toggle density heatmap mode
        function iqToggleDensity() {
            const checkbox = document.getElementById('iq_density_mode');
            if (!checkbox) return;

            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setDensityMode) {
                IQConstellationEnhanced.setDensityMode(checkbox.checked);
            }
        }

        // Change modulation type
        function iqModulationTypeChange(modType) {
            if (typeof IQConstellationEnhanced !== 'undefined' && IQConstellationEnhanced.setModulationType) {
                IQConstellationEnhanced.setModulationType(modType);
                console.log(`[IQ] Modulation type: ${modType}`);
            }
        }

        // ===================== WAVEFORM & EYE DIAGRAM ZOOM/PAN CONTROLS =====================

        // Change waveform view mode
        function waveformViewModeChange(mode) {
            if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.setViewMode) {
                WaveformDisplay.setViewMode(mode);
                console.log(`[Waveform] View mode: ${mode}`);
            }
        }

        // Change eye diagram symbol rate
        function eyeSymbolRateChange(rate) {
            const symbolRate = parseFloat(rate);
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.setSymbolRate) {
                EyeDiagram.setSymbolRate(symbolRate);
                console.log(`[Eye Diagram] Symbol rate: ${(symbolRate / 1e6).toFixed(3)} Msym/s`);
            }
        }

        // Set zoom level for both waveform and eye diagram
        function waveformEyeZoom(zoom) {
            if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.setZoom) {
                WaveformDisplay.setZoom(zoom);
                console.log(`[Waveform] Zoom: ${zoom}x`);
            }
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.setZoom) {
                EyeDiagram.setZoom(zoom);
                console.log(`[Eye Diagram] Zoom: ${zoom}x`);
            }
        }

        // Reset view for both waveform and eye diagram
        function waveformEyeResetView() {
            if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.resetView) {
                WaveformDisplay.resetView();
                console.log('[Waveform] View reset');
            }
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.resetView) {
                EyeDiagram.resetView();
                console.log('[Eye Diagram] View reset');
            }
        }

        // ===================== EYE DIAGRAM CONTROLS =====================

        // Update eye diagram symbol rate
        function eyeUpdateSymbolRate() {
            const select = document.getElementById('eye_symbol_rate');
            if (!select) return;

            const symbolRate = parseFloat(select.value);
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.setSymbolRate) {
                EyeDiagram.setSymbolRate(symbolRate);
                console.log(`[Eye Diagram] Symbol rate: ${(symbolRate / 1e6).toFixed(3)} Msym/s`);
            }
        }

        // Clear eye diagram persistence
        function eyeClear() {
            if (typeof EyeDiagram !== 'undefined' && EyeDiagram.clear) {
                EyeDiagram.clear();
                console.log('[Eye Diagram] Cleared');
            }
        }

        // ===================== WAVEFORM DISPLAY CONTROLS =====================

        // Update waveform view mode
        function waveformUpdateMode() {
            const select = document.getElementById('waveform_view_mode');
            if (!select) return;

            const mode = select.value;
            if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.setViewMode) {
                WaveformDisplay.setViewMode(mode);
                console.log(`[Waveform] View mode: ${mode}`);
            }
        }

        // Update IQ workspace frequency/span displays
        function updateIQWorkspaceFreqDisplay() {
            const freqInput = document.getElementById('freqInput');
            const srInput = document.getElementById('srInput');
            const iqCenterFreq = document.getElementById('iq_center_freq');
            const iqSpan = document.getElementById('iq_span');

            if (freqInput && iqCenterFreq) {
                iqCenterFreq.textContent = parseFloat(freqInput.value).toFixed(3) + ' MHz';
            }

            if (srInput && iqSpan) {
                iqSpan.textContent = parseFloat(srInput.value).toFixed(2) + ' MHz';
            }
        }

        // Update IQ signal detection metrics using raw IQ samples
        function updateIQSignalMetrics(ch1_i, ch1_q, ch2_i, ch2_q) {
            if (!ch1_i || !ch1_q || !ch2_i || !ch2_q) return;

            const numSamples = Math.min(ch1_i.length, ch1_q.length, ch2_i.length, ch2_q.length);

            // Calculate instantaneous power for each sample (I^2 + Q^2)
            let maxPower = 0;
            let avgPower = 0;
            let minPower = Infinity;

            for (let i = 0; i < numSamples; i++) {
                // Average both channels, normalize int16 to float
                const iVal = (ch1_i[i] + ch2_i[i]) / 2.0 / 2048.0;
                const qVal = (ch1_q[i] + ch2_q[i]) / 2.0 / 2048.0;

                // Power = I^2 + Q^2
                const power = iVal * iVal + qVal * qVal;

                maxPower = Math.max(maxPower, power);
                minPower = Math.min(minPower, power);
                avgPower += power;
            }
            avgPower /= numSamples;

            // Noise floor estimate: use minimum power or average of lowest 25% of samples
            const powerSamples = [];
            for (let i = 0; i < numSamples; i++) {
                const iVal = (ch1_i[i] + ch2_i[i]) / 2.0 / 2048.0;
                const qVal = (ch1_q[i] + ch2_q[i]) / 2.0 / 2048.0;
                powerSamples.push(iVal * iVal + qVal * qVal);
            }
            powerSamples.sort((a, b) => a - b);
            const noiseFloorPower = powerSamples[Math.floor(numSamples * 0.25)]; // 25th percentile

            // Convert to dBFS (reference = 1.0 for full scale)
            const peakPowerDb = 10 * Math.log10(maxPower + 1e-10);
            const avgPowerDb = 10 * Math.log10(avgPower + 1e-10);
            const noiseFloorDb = 10 * Math.log10(noiseFloorPower + 1e-10);
            const snrDb = peakPowerDb - noiseFloorDb;

            console.log(`[IQ Signal Metrics] Peak: ${peakPowerDb.toFixed(1)} dBFS, Avg: ${avgPowerDb.toFixed(1)} dBFS, Noise: ${noiseFloorDb.toFixed(1)} dBFS, SNR: ${snrDb.toFixed(1)} dB`);

            // Update displays
            const statusEl = document.getElementById('iq_signal_status');
            const snrEl = document.getElementById('iq_snr');
            const peakPowerEl = document.getElementById('iq_peak_power');
            const noiseFloorEl = document.getElementById('iq_noise_floor');
            const signalBarEl = document.getElementById('iq_signal_bar');
            const signalBarTextEl = document.getElementById('iq_signal_bar_text');

            if (snrDb > 10) {
                if (statusEl) {
                    statusEl.textContent = 'SIGNAL DETECTED';
                    statusEl.style.color = '#0f0';
                }
            } else if (snrDb > 3) {
                if (statusEl) {
                    statusEl.textContent = 'WEAK SIGNAL';
                    statusEl.style.color = '#ff0';
                }
            } else {
                if (statusEl) {
                    statusEl.textContent = 'NO SIGNAL';
                    statusEl.style.color = '#888';
                }
            }

            if (snrEl) snrEl.textContent = snrDb.toFixed(1) + ' dB';
            if (peakPowerEl) peakPowerEl.textContent = peakPowerDb.toFixed(1) + ' dBFS';
            if (noiseFloorEl) noiseFloorEl.textContent = noiseFloorDb.toFixed(1) + ' dBFS';

            // Update signal strength bar (scale SNR to 0-100%, 20dB = 100%)
            const signalStrength = Math.max(0, Math.min(100, snrDb * 5));
            if (signalBarEl) {
                signalBarEl.style.width = signalStrength.toFixed(0) + '%';
            }
            if (signalBarTextEl) {
                signalBarTextEl.textContent = signalStrength.toFixed(0) + '%';
            }
        }

        // Update IQ statistics panel
        function updateIQStatistics(iq_data) {
            if (!iq_data || iq_data.length < 4) return;

            let sumI = 0, sumQ = 0;
            let sumI2 = 0, sumQ2 = 0;
            const numSamples = iq_data.length / 2;

            for (let i = 0; i < iq_data.length; i += 2) {
                const I = iq_data[i];
                const Q = iq_data[i + 1];
                sumI += I;
                sumQ += Q;
                sumI2 += I * I;
                sumQ2 += Q * Q;
            }

            const meanI = sumI / numSamples;
            const meanQ = sumQ / numSamples;
            const rmsI = Math.sqrt(sumI2 / numSamples);
            const rmsQ = Math.sqrt(sumQ2 / numSamples);

            // Calculate EVM (Error Vector Magnitude)
            let evmSum = 0;
            let refPowerSum = 0;
            for (let i = 0; i < iq_data.length; i += 2) {
                const I = iq_data[i];
                const Q = iq_data[i + 1];
                const errorI = I - meanI;
                const errorQ = Q - meanQ;
                evmSum += errorI * errorI + errorQ * errorQ;
                refPowerSum += I * I + Q * Q;
            }
            const evmPercent = Math.sqrt(evmSum / refPowerSum) * 100;

            // Calculate phase noise (std dev of phase)
            let phaseSum = 0;
            let phaseSquareSum = 0;
            for (let i = 0; i < iq_data.length; i += 2) {
                const phase = Math.atan2(iq_data[i + 1], iq_data[i]) * (180 / Math.PI);
                phaseSum += phase;
                phaseSquareSum += phase * phase;
            }
            const meanPhase = phaseSum / numSamples;
            const phaseVariance = (phaseSquareSum / numSamples) - (meanPhase * meanPhase);
            const phaseNoiseStdDev = Math.sqrt(Math.max(0, phaseVariance));

            // Update display
            const iMeanEl = document.getElementById('iq_i_mean');
            const qMeanEl = document.getElementById('iq_q_mean');
            const iRmsEl = document.getElementById('iq_i_rms');
            const qRmsEl = document.getElementById('iq_q_rms');
            const evmEl = document.getElementById('iq_evm');
            const phaseNoiseEl = document.getElementById('iq_phase_noise');

            if (iMeanEl) iMeanEl.textContent = meanI.toFixed(4);
            if (qMeanEl) qMeanEl.textContent = meanQ.toFixed(4);
            if (iRmsEl) iRmsEl.textContent = rmsI.toFixed(4);
            if (qRmsEl) qRmsEl.textContent = rmsQ.toFixed(4);
            if (evmEl) evmEl.textContent = evmPercent.toFixed(2) + ' %';
            if (phaseNoiseEl) phaseNoiseEl.textContent = phaseNoiseStdDev.toFixed(2) + ' °';
        }

        // === END IQ WORKSPACE TAB CONTROL FUNCTIONS ===

        // Save current configuration as a preset
        function savePreset() {
            const presetNameInput = getElement('presetName');
            if (!presetNameInput) return;

            const presetName = presetNameInput.value.trim();

            // Validate preset name
            const validation = validatePresetName(presetName);
            if (!validation.valid) {
                showNotification(validation.error, 'error');
                return;
            }

            const config = getCurrentConfig();
            const presets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');

            presets[presetName] = {
                config: config,
                timestamp: new Date().toISOString()
            };

            const json = JSON.stringify(presets);
            if (safeLocalStorageSet('bladerfsensor_webserver_presets', json)) {
                refreshPresetList();
                showNotification(`Preset "${presetName}" saved successfully`, 'success');
                presetNameInput.value = '';
            }
        }

        // Load a preset configuration
        async function loadPreset() {
            const presetSelectElem = getElement('presetSelect');
            if (!presetSelectElem) return;

            const presetName = presetSelectElem.value;

            if (!presetName) {
                return;
            }

            const presets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');
            const preset = presets[presetName];

            if (!preset) {
                showNotification(`Preset "${presetName}" not found`, 'error');
                return;
            }

            await applyConfig(preset.config);
            showNotification(`Preset "${presetName}" loaded successfully`, 'success');
        }

        // Delete the currently selected preset
        function deletePreset() {
            const presetSelectElem = getElement('presetSelect');
            if (!presetSelectElem) return;

            const presetName = presetSelectElem.value;

            if (!presetName) {
                showNotification('Please select a preset to delete', 'warning');
                return;
            }

            if (!confirm(`Are you sure you want to delete the preset "${presetName}"?`)) {
                return;
            }

            const presets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');
            delete presets[presetName];

            const json = JSON.stringify(presets);
            if (safeLocalStorageSet('bladerfsensor_webserver_presets', json)) {
                refreshPresetList();
                showNotification(`Preset "${presetName}" deleted successfully`, 'success');
            }
        }

        // Export all presets to a JSON file
        function exportPresets() {
            const presets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');

            if (Object.keys(presets).length === 0) {
                showNotification('No presets to export', 'warning');
                return;
            }

            const dataStr = JSON.stringify(presets, null, 2);
            const dataBlob = new Blob([dataStr], { type: 'application/json' });
            const url = URL.createObjectURL(dataBlob);

            const link = document.createElement('a');
            link.href = url;
            link.download = 'bladerfsensor_webserver_presets_' + new Date().toISOString().split('T')[0] + '.json';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);

            showNotification(`Exported ${Object.keys(presets).length} preset(s)`, 'success');
        }

        // Trigger file input for importing presets
        function importPresets() {
            document.getElementById('presetImportFile').click();
        }

        // Handle imported preset file
        function handlePresetImport(event) {
            const file = event.target.files[0];
            if (!file) return;

            const reader = new FileReader();
            reader.onload = function(e) {
                try {
                    const importedPresets = JSON.parse(e.target.result);
                    const currentPresets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');

                    let importedCount = 0;
                    for (const name in importedPresets) {
                        if (importedPresets[name].config) {
                            currentPresets[name] = importedPresets[name];
                            importedCount++;
                        }
                    }

                    if (safeLocalStorageSet('bladerfsensor_webserver_presets', JSON.stringify(currentPresets))) {
                        refreshPresetList();
                        showNotification(`Imported ${importedCount} preset(s) successfully`, 'success');
                    }
                } catch (error) {
                    showNotification('Error importing presets: Invalid file format', 'error');
                }
            };
            reader.readAsText(file);

            // Reset the file input
            event.target.value = '';
        }

        // Refresh the preset dropdown list
        function refreshPresetList() {
            const presets = JSON.parse(localStorage.getItem('bladerfsensor_webserver_presets') || '{}');
            const select = document.getElementById('presetSelect');

            // Clear existing options except the first one
            select.innerHTML = '<option value="">-- Select --</option>';

            // Add presets sorted alphabetically
            const sortedNames = Object.keys(presets).sort();
            for (let i = 0; i < sortedNames.length; i++) {
                const name = sortedNames[i];
                const option = document.createElement('option');
                option.value = name;
                option.textContent = name;
                select.appendChild(option);
            }
        }

        // Export ALL application settings
        function exportAllSettings() {
            if (typeof Settings !== 'undefined') {
                Settings.downloadConfig();
            } else {
                showNotification('Settings module not loaded', 'error');
            }
        }

        // Trigger file input for importing settings
        function importAllSettings() {
            document.getElementById('settingsImportFile').click();
        }

        // Handle imported settings file
        function handleSettingsImport(event) {
            const file = event.target.files[0];
            if (!file) return;

            if (typeof Settings === 'undefined') {
                showNotification('Settings module not loaded', 'error');
                event.target.value = '';
                return;
            }

            Settings.uploadConfig(event.target).then(settings => {
                showNotification(`Imported ${Object.keys(settings).length} settings successfully. Reloading...`, 'success', 3000);
                // Reload page after 3 seconds to apply settings
                setTimeout(() => window.location.reload(), 3000);
            }).catch(err => {
                showNotification('Error importing settings: ' + err.message, 'error');
            });

            // Reset the file input
            event.target.value = '';
        }

        // Reset ALL settings to defaults
        function resetAllSettings() {
            if (!confirm('Are you sure you want to reset ALL settings to defaults? This cannot be undone.')) {
                return;
            }

            if (typeof Settings !== 'undefined') {
                Settings.clear();
                showNotification('Settings reset to defaults. Reloading...', 'success', 2000);
                setTimeout(() => window.location.reload(), 2000);
            } else {
                showNotification('Settings module not loaded', 'error');
            }
        }

        /**
         * Save UI state to localStorage
         */
        function saveUIState() {
            const state = {
                // Display settings
                waterfallIntensity: waterfallIntensity,
                waterfallContrast: waterfallContrast,
                spectrumMinDb: spectrumMinDb,
                spectrumMaxDb: spectrumMaxDb,
                showSpectrum: showSpectrum,
                showIQ: showIQ,
                showXCorr: showXCorr,

                // Zoom state (don't save actual zoom, just the fact that user had zoom capability)
                // Actual zoom depends on current frequency/bandwidth

                // Channel selection
                currentChannel: currentChannel,

                // Signal analysis settings
                persistenceMode: signalAnalysis.persistenceMode,
                persistenceDecayRate: signalAnalysis.persistenceDecayRate,
                colorPalette: signalAnalysis.colorPalette,

                // Timestamp
                savedAt: new Date().toISOString()
            };

            safeLocalStorageSet('bladerfsensor_ui_state', JSON.stringify(state));
        }

        /**
         * Restore UI state from localStorage
         */
        function restoreUIState() {
            try {
                const saved = localStorage.getItem('bladerfsensor_ui_state');
                if (!saved) return;

                const state = JSON.parse(saved);

                // Restore display settings
                if (state.waterfallIntensity !== undefined) {
                    waterfallIntensity = state.waterfallIntensity;
                    const intensitySlider = getElement('waterfallIntensity');
                    if (intensitySlider) {
                        intensitySlider.value = state.waterfallIntensity;
                        updateWaterfallIntensity(state.waterfallIntensity);
                    }
                }

                if (state.waterfallContrast !== undefined) {
                    waterfallContrast = state.waterfallContrast;
                    const contrastSlider = getElement('waterfallContrast');
                    if (contrastSlider) {
                        contrastSlider.value = state.waterfallContrast;
                        updateWaterfallContrast(state.waterfallContrast);
                    }
                }

                if (state.spectrumMinDb !== undefined) {
                    spectrumMinDb = state.spectrumMinDb;
                    const minInput = getElement('spectrumMin');
                    if (minInput) minInput.value = state.spectrumMinDb;
                }

                if (state.spectrumMaxDb !== undefined) {
                    spectrumMaxDb = state.spectrumMaxDb;
                    const maxInput = getElement('spectrumMax');
                    if (maxInput) maxInput.value = state.spectrumMaxDb;
                }

                // Restore visibility states
                if (state.showSpectrum !== undefined) showSpectrum = state.showSpectrum;
                if (state.showIQ !== undefined) showIQ = state.showIQ;
                if (state.showXCorr !== undefined) showXCorr = state.showXCorr;

                // Restore channel
                if (state.currentChannel !== undefined) {
                    currentChannel = state.currentChannel;
                    const channelSelect = getElement('channel_select');
                    if (channelSelect) channelSelect.value = state.currentChannel;
                }

                // Restore signal analysis settings
                if (state.persistenceMode !== undefined) {
                    signalAnalysis.persistenceMode = state.persistenceMode;
                }
                if (state.persistenceDecayRate !== undefined) {
                    signalAnalysis.persistenceDecayRate = state.persistenceDecayRate;
                }
                if (state.colorPalette !== undefined) {
                    signalAnalysis.colorPalette = state.colorPalette;
                }

                console.log('UI state restored from', state.savedAt);
                showNotification('UI settings restored', 'success', 2000);
            } catch (err) {
                console.error('Failed to restore UI state:', err);
            }
        }

        // ===== KEYBOARD SHORTCUTS =====
        document.addEventListener('keydown', (e) => {
            // Ignore if typing in input field
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

            const key = e.key.toLowerCase();

            switch(key) {
                case 'f': // Toggle filter mode
                    e.preventDefault();
                    document.getElementById('filter_mode_toggle')?.click();
                    break;

                case 'z': // Toggle zoom mode
                    e.preventDefault();
                    document.getElementById('zoom_mode_toggle')?.click();
                    break;

                case 'x': // Clear zoom
                    e.preventDefault();
                    if (zoomState.isZoomed) zoomOut();
                    break;

                case 's': // Toggle spectrum
                    e.preventDefault();
                    toggleSpectrum();
                    break;

                case 'i': // Toggle IQ constellation
                    e.preventDefault();
                    toggleIQ();
                    break;

                case 'c': // Toggle cross-correlation
                    e.preventDefault();
                    toggleXCorr();
                    break;

                case 'p': // Toggle peak hold
                    e.preventDefault();
                    const peakCheckbox = document.getElementById('peakHoldCheckbox');
                    if (peakCheckbox) {
                        peakCheckbox.checked = !peakCheckbox.checked;
                        togglePeakHold(peakCheckbox.checked);
                    }
                    break;

                case 'm': // Toggle min hold
                    e.preventDefault();
                    const minCheckbox = document.getElementById('minHoldCheckbox');
                    if (minCheckbox) {
                        minCheckbox.checked = !minCheckbox.checked;
                        toggleMinHold(minCheckbox.checked);
                    }
                    break;

                case 'r': // Toggle reference markers
                    e.preventDefault();
                    const refCheckbox = document.getElementById('refMarkersCheckbox');
                    if (refCheckbox) {
                        refCheckbox.checked = !refCheckbox.checked;
                        toggleRefMarkers(refCheckbox.checked);
                    }
                    break;

                case 'b': // Toggle bandwidth measure
                    e.preventDefault();
                    const bwCheckbox = document.getElementById('bwMeasureCheckbox');
                    if (bwCheckbox) {
                        bwCheckbox.checked = !bwCheckbox.checked;
                        toggleBandwidthMeasure(bwCheckbox.checked);
                    }
                    break;

                case 'arrowup': // Increase gain
                    e.preventDefault();
                    const gain1Input = document.getElementById('gain1Input');
                    if (gain1Input) {
                        gain1Input.value = Math.min(60, parseInt(gain1Input.value) + 3);
                        applyGain1();
                    }
                    break;

                case 'arrowdown': // Decrease gain
                    e.preventDefault();
                    const gain1Down = document.getElementById('gain1Input');
                    if (gain1Down) {
                        gain1Down.value = Math.max(0, parseInt(gain1Down.value) - 3);
                        applyGain1();
                    }
                    break;

                case 'arrowright': // Fine tune frequency up
                    if (e.shiftKey) {
                        e.preventDefault();
                        const freqInput = document.getElementById('freqInput');
                        if (freqInput) {
                            freqInput.value = (parseFloat(freqInput.value) + 0.001).toFixed(3);
                            applyFrequency();
                        }
                    }
                    break;

                case 'arrowleft': // Fine tune frequency down
                    if (e.shiftKey) {
                        e.preventDefault();
                        const freqDown = document.getElementById('freqInput');
                        if (freqDown) {
                            freqDown.value = (parseFloat(freqDown.value) - 0.001).toFixed(3);
                            applyFrequency();
                        }
                    }
                    break;

                case ' ': // Pause/resume (spacebar)
                    e.preventDefault();
                    // Could add pause functionality
                    showNotification('Pause/resume not yet implemented', 'info', 1500);
                    break;

                case 'h': // Show help
                case '?':
                    e.preventDefault();
                    showKeyboardHelp();
                    break;
            }
        });

        function showKeyboardHelp() {
            const helpText = `
<b>Keyboard Shortcuts:</b>

<b>Display Toggles:</b>
S - Toggle Spectrum
I - Toggle IQ Constellation
C - Toggle Cross-Correlation
P - Toggle Peak Hold
M - Toggle Min Hold
R - Toggle Reference Markers
B - Toggle Bandwidth Measurement

<b>Modes:</b>
F - Toggle Filter Mode
Z - Toggle Zoom Mode
X - Clear Zoom

<b>Controls:</b>
↑/↓ - Adjust Gain (±3dB)
Shift+←/→ - Fine tune frequency (±1kHz)

<b>Help:</b>
H or ? - Show this help
`;
            showNotification(helpText, 'info', 8000);
        }

        // Initialize preset list on page load
        window.addEventListener('load', function() {
            refreshPresetList();
            updateRecordMode();  // Initialize recording UI
            restoreUIState();    // Restore saved UI settings
            loadDisplaySettings();  // Load persistent display settings
            console.log('✓ Keyboard shortcuts enabled');
        });

        // Save UI state before page unload
        window.addEventListener('beforeunload', function() {
            saveUIState();
            saveDisplaySettings();  // Save persistent display settings
        });

        // ===== Recording Functions =====
        let isRecording = false;
        let recordedFrames = [];
        let recordingStartTime = null;
        let recordingTimer = null;
        let recordingConfig = {};

        function updateRecordMode() {
            const mode = document.getElementById('recordMode').value;
            const bandControls = document.getElementById('recordBandControls');
            if (mode === 'band') {
                bandControls.style.display = '';
            } else {
                bandControls.style.display = 'none';
            }
        }

        function toggleRecording() {
            if (isRecording) {
                stopRecording();
            } else {
                startRecording();
            }
        }

        function startRecording() {
            const duration = parseInt(document.getElementById('recordDuration').value);
            if (!duration || duration < 1 || duration > 300) {
                showNotification('Please set a valid duration (1-300 seconds)', 'warning');
                return;
            }

            const mode = document.getElementById('recordMode').value;
            const freq = parseFloat(document.getElementById('freqInput').value) * 1e6;
            const sr = parseFloat(document.getElementById('srInput').value) * 1e6;

            recordingConfig = {
                duration: duration,
                mode: mode,
                centerFreq: freq,
                sampleRate: sr,
                startTime: new Date().toISOString()
            };

            if (mode === 'band') {
                const centerMHz = parseFloat(document.getElementById('recordCenterFreq').value);
                const bwMHz = parseFloat(document.getElementById('recordBandwidth').value);
                if (!centerMHz || !bwMHz) {
                    showNotification('Please enter valid center frequency and bandwidth', 'warning');
                    return;
                }
                recordingConfig.recordCenterFreq = centerMHz * 1e6;
                recordingConfig.recordBandwidth = bwMHz * 1e6;
            }

            isRecording = true;
            recordedFrames = [];
            recordingStartTime = Date.now();

            document.getElementById('recordButton').textContent = '⏹ Stop Recording';
            document.getElementById('recordButton').style.background = '#ff0000';

            recordingTimer = setTimeout(() => {
                stopRecording();
            }, duration * 1000);

            updateRecordingStatus();
        }

        function stopRecording() {
            if (!isRecording) return;
            isRecording = false;

            if (recordingTimer) {
                clearTimeout(recordingTimer);
                recordingTimer = null;
            }

            document.getElementById('recordButton').textContent = '⏺ Start Recording';
            document.getElementById('recordButton').style.background = '';

            if (recordedFrames.length === 0) {
                showNotification('No data recorded', 'warning');
                document.getElementById('recordingStatus').textContent = 'Ready';
                return;
            }

            saveRecordingAsWAV();
            document.getElementById('recordingStatus').textContent = 'Ready';
            document.getElementById('recordingStatus').style.color = '#888';
        }

        function updateRecordingStatus() {
            if (!isRecording) return;
            const elapsed = (Date.now() - recordingStartTime) / 1000;
            document.getElementById('recordingStatus').textContent =
                'Recording... ' + elapsed.toFixed(1) + 's (' + recordedFrames.length + ' frames)';
            document.getElementById('recordingStatus').style.color = '#ff0000';
            setTimeout(updateRecordingStatus, 100);
        }

        function captureRecordingFrame(data) {
            if (!isRecording || !data) return;
            recordedFrames.push({
                timestamp: Date.now() - recordingStartTime,
                data: [...data]
            });
        }

        function saveRecordingAsWAV() {
            if (recordedFrames.length === 0) return;

            const audioSampleRate = 48000;
            const numChannels = 2;
            const bitsPerSample = 16;

            // Generate filename
            const timestamp = new Date().toISOString().replace(/[:.]/g, '-').substring(0, 19);
            let filename = 'bladerf-rec-' + timestamp;

            if (recordingConfig.mode === 'band') {
                const freqMHz = (recordingConfig.recordCenterFreq / 1e6).toFixed(1);
                const bwMHz = (recordingConfig.recordBandwidth / 1e6).toFixed(1);
                filename += '-' + freqMHz + 'MHz-BW' + bwMHz + 'MHz';
            } else {
                const freqMHz = (recordingConfig.centerFreq / 1e6).toFixed(1);
                filename += '-' + freqMHz + 'MHz-full';
            }

            filename += '.wav';

            // Create WAV file
            const wavData = createWAVFileWebServer(recordedFrames, audioSampleRate, numChannels, bitsPerSample);

            // Download
            const blob = new Blob([wavData], { type: 'audio/wav' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = filename;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);

            // Save metadata
            saveRecordingMetadataWebServer(filename.replace('.wav', '.json'));
        }

        function createWAVFileWebServer(frames, sampleRate, numChannels, bitsPerSample) {
            const samplesPerFrame = Math.floor(sampleRate / 20);
            const totalSamples = frames.length * samplesPerFrame;
            const fftBins = frames[0].data.length;
            const dataSize = totalSamples * numChannels * (bitsPerSample / 8);
            const buffer = new ArrayBuffer(44 + dataSize);
            const view = new DataView(buffer);

            // WAV Header
            writeStringWebServer(view, 0, 'RIFF');
            view.setUint32(4, 36 + dataSize, true);
            writeStringWebServer(view, 8, 'WAVE');
            writeStringWebServer(view, 12, 'fmt ');
            view.setUint32(16, 16, true);
            view.setUint16(20, 1, true);
            view.setUint16(22, numChannels, true);
            view.setUint32(24, sampleRate, true);
            view.setUint32(28, sampleRate * numChannels * (bitsPerSample / 8), true);
            view.setUint16(32, numChannels * (bitsPerSample / 8), true);
            view.setUint16(34, bitsPerSample, true);
            writeStringWebServer(view, 36, 'data');
            view.setUint32(40, dataSize, true);

            // Write audio data
            let offset = 44;
            for (let frameIdx = 0; frameIdx < frames.length; frameIdx++) {
                const frame = frames[frameIdx];
                for (let s = 0; s < samplesPerFrame; s++) {
                    const binIdx = Math.floor((s / samplesPerFrame) * fftBins);
                    const magnitude = frame.data[binIdx] || 0;
                    const audioSample = Math.floor((magnitude / 255) * 32767) - 16384;
                    view.setInt16(offset, audioSample, true);
                    offset += 2;
                    view.setInt16(offset, audioSample, true);
                    offset += 2;
                }
            }
            return buffer;
        }

        function writeStringWebServer(view, offset, string) {
            for (let i = 0; i < string.length; i++) {
                view.setUint8(offset + i, string.charCodeAt(i));
            }
        }

        function saveRecordingMetadataWebServer(filename) {
            const metadata = {
                recording: {
                    startTime: recordingConfig.startTime,
                    duration: recordingConfig.duration,
                    frames: recordedFrames.length,
                    mode: recordingConfig.mode
                },
                rf: {
                    centerFrequency: recordingConfig.centerFreq,
                    sampleRate: recordingConfig.sampleRate,
                    fftSize: FFT_SIZE
                },
                timestamp: new Date().toISOString()
            };

            if (recordingConfig.mode === 'band') {
                metadata.recording.recordCenterFreq = recordingConfig.recordCenterFreq;
                metadata.recording.recordBandwidth = recordingConfig.recordBandwidth;
            }

            const metadataStr = JSON.stringify(metadata, null, 2);
            const blob = new Blob([metadataStr], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = filename;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);
        }

        // Export spectrum data to CSV with metadata
        function exportSpectrumCSV() {
            if (!latestFFTData || latestFFTData.length === 0) {
                showNotification('No spectrum data available to export', 'warning');
                return;
            }

            const timestamp = new Date().toISOString();
            const dateStr = new Date().toLocaleString();

            // Get current RF settings
            const freq = parseFloat(document.getElementById('freqInput').value) * 1e6;
            const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
            const bw = parseFloat(document.getElementById('bwInput').value) * 1e6;
            const gain = parseInt(document.getElementById(currentChannel === 1 ? 'gain1Input' : 'gain2Input').value);

            // Build CSV with metadata header
            let csv = '# bladeRF Spectrum Data Export\n';
            csv += '# Export Date: ' + dateStr + '\n';
            csv += '# Timestamp: ' + timestamp + '\n';
            csv += '#\n';
            csv += '# Configuration:\n';
            csv += '# Center Frequency: ' + (freq / 1e6).toFixed(3) + ' MHz\n';
            csv += '# Sample Rate: ' + (sr / 1e6).toFixed(2) + ' MHz\n';
            csv += '# Bandwidth: ' + (bw / 1e6).toFixed(2) + ' MHz\n';
            csv += '# Channel: RX' + currentChannel + '\n';
            csv += '# Gain: ' + gain + ' dB\n';
            csv += '# FFT Size: ' + FFT_SIZE + '\n';
            csv += '#\n';
            csv += '# Display Settings:\n';
            csv += '# Waterfall Intensity: ' + waterfallIntensity.toFixed(1) + 'x\n';
            csv += '# Waterfall Contrast: ' + waterfallContrast.toFixed(1) + 'x\n';
            csv += '# Spectrum Range: ' + spectrumMinDb + ' to ' + spectrumMaxDb + ' dBFS\n';
            csv += '#\n';
            csv += '# Data Format:\n';
            csv += '# Frequency (Hz), Magnitude (raw), Power (dBFS)\n';
            csv += '#\n';

            // Column headers
            csv += 'Frequency_Hz,Magnitude_Raw,Power_dBFS\n';

            // Export data with both raw and dBFS values
            for (let i = 0; i < latestFFTData.length; i++) {
                const binFreq = freq - (sr / 2) + (i * sr / FFT_SIZE);
                const raw = latestFFTData[i];
                const dBFS = rawToDb(raw);
                csv += binFreq.toFixed(0) + ',' + raw.toFixed(2) + ',' + dBFS.toFixed(2) + '\n';
            }

            // Create download
            const blob = new Blob([csv], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = 'bladerf-spectrum-' + timestamp.replace(/[:.]/g, '-') + '.csv';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);

            showNotification(`Spectrum data exported (${latestFFTData.length} bins)`, 'success');
        }

        // Send control update to server
        async function sendControlUpdate(freq, sr, bw, gain1, gain2) {
            const payload = {};
            if (freq !== null) payload.freq = freq;
            if (sr !== null) payload.sr = sr;
            if (bw !== null) payload.bw = bw;
            if (gain1 !== null) payload.gain1 = gain1;
            if (gain2 !== null) payload.gain2 = gain2;

            console.log('Sending control update:', payload);

            try {
                const response = await fetch('/control', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });

                const result = await response.json();

                if (response.ok) {
                    console.log('Control update successful');
                    // Clear canvas for fresh data
                    ctx.fillStyle = '#000';
                    ctx.fillRect(0, 0, canvas.width, canvas.height);

                    // Update sample rate in eye diagram and waveform display modules
                    if (sr !== null) {
                        if (typeof EyeDiagram !== 'undefined' && EyeDiagram.setSampleRate) {
                            EyeDiagram.setSampleRate(sr);
                        }
                        if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.setSampleRate) {
                            WaveformDisplay.setSampleRate(sr);
                        }
                    }
                } else {
                    console.error('Control update failed:', result);
                    showNotification(`Error: ${result.error || 'Unknown error'}`, 'error');
                }
            } catch (err) {
                console.error('Control request error:', err);
                showNotification('Failed to send control update', 'error');
            }
        }

        // Continuous update loop using requestAnimationFrame
        let lastUpdateTime = 0;
        let isUpdating = false;

        // Intelligent throttling with adaptive performance monitoring
        const performanceMonitor = {
            frameTimes: [],
            maxSamples: 30,
            targetFPS: 30,
            minInterval: 100,  // 10 Hz minimum
            currentInterval: 100,
            consecutiveSlowFrames: 0,

            recordFrame(duration) {
                this.frameTimes.push(duration);
                if (this.frameTimes.length > this.maxSamples) {
                    this.frameTimes.shift();
                }

                // Calculate average frame time
                const avgFrameTime = this.frameTimes.reduce((a, b) => a + b, 0) / this.frameTimes.length;

                // If frames are taking too long, reduce update rate
                if (avgFrameTime > 50) {  // Slower than 20 FPS
                    this.consecutiveSlowFrames++;
                    if (this.consecutiveSlowFrames > 5) {
                        this.currentInterval = Math.min(this.currentInterval * 1.1, 200);  // Max 5 Hz
                        this.consecutiveSlowFrames = 0;
                    }
                } else if (avgFrameTime < 25) {  // Faster than 40 FPS
                    // Can handle faster updates
                    this.currentInterval = Math.max(this.currentInterval * 0.95, this.minInterval);
                    this.consecutiveSlowFrames = 0;
                }
            },

            getInterval() {
                return Math.floor(this.currentInterval);
            }
        };

        // Throttling flags to prevent overlapping requests
        let isUpdatingStatus = false;
        let isUpdatingIQ = false;
        let isUpdatingXCorr = false;
        let isUpdatingLinkQuality = false;

        function updateLoop(timestamp) {
            // Adaptive throttling based on performance
            const updateInterval = performanceMonitor.getInterval();

            // Only start new fetch if previous one finished
            if (!isUpdating && timestamp - lastUpdateTime >= updateInterval) {
                isUpdating = true;
                lastUpdateTime = timestamp;
                updateWaterfall();
                // DO NOT unlock here - fetch will unlock when complete
            }
            requestAnimationFrame(updateLoop);
        }

        // AbortController to cancel all pending fetches
        const abortController = new AbortController();
        const fetchSignal = abortController.signal;

        // Start the update loop
        let animationFrameId = requestAnimationFrame(updateLoop);

        // Store interval IDs for cleanup
        const intervals = [];
        intervals.push(setInterval(updateStatus, 2000));
        intervals.push(setInterval(updateIQData, 100));
        intervals.push(setInterval(updateLinkQuality, 1000));

        // XCorr uses requestAnimationFrame with adaptive throttling for better performance
        let xcorrLastUpdate = 0;
        let xcorrUpdateInterval = 500; // Start at 500ms, can adapt based on performance

        function xcorrAnimationLoop(timestamp) {
            if (!showXCorr) {
                requestAnimationFrame(xcorrAnimationLoop);
                return;
            }

            // Throttle updates based on interval
            if (timestamp - xcorrLastUpdate >= xcorrUpdateInterval) {
                xcorrLastUpdate = timestamp;
                updateXCorrData();

                // Adaptive rate: increase interval if updates are slow
                if (isUpdatingXCorr && xcorrUpdateInterval < 1000) {
                    xcorrUpdateInterval += 100; // Slow down if falling behind
                } else if (!isUpdatingXCorr && xcorrUpdateInterval > 200) {
                    xcorrUpdateInterval -= 50; // Speed up if keeping up
                }
            }

            requestAnimationFrame(xcorrAnimationLoop);
        }

        // Start XCorr animation loop
        requestAnimationFrame(xcorrAnimationLoop);

        // Cleanup function
        function cleanup() {
            console.log('RX page cleanup - aborting all requests');
            abortController.abort();  // Cancel all pending fetches
            intervals.forEach(id => clearInterval(id));
            if (animationFrameId) cancelAnimationFrame(animationFrameId);
        }

        // Cleanup intervals when leaving page
        window.addEventListener('beforeunload', cleanup);
        window.addEventListener('pagehide', cleanup);

        // CRITICAL: Stop all fetch loops when leaving
        window.addEventListener('visibilitychange', function() {
            if (document.hidden) {
                cleanup();
            }
        });

        // Initial updates
        updateStatus();
        updateLinkQuality();

        // Toggle button handlers
        document.getElementById('spectrum_toggle').addEventListener('click', toggleSpectrum);
        // Note: iq_toggle and xcorr_toggle removed in favor of workspace tabs
        // document.getElementById('iq_toggle').addEventListener('click', toggleIQ);
        // document.getElementById('xcorr_toggle').addEventListener('click', toggleXCorr);

        // Spectrum Y-axis scroll/zoom handler
        spectrumCanvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            console.log('Spectrum wheel event:', e.deltaY, 'Ctrl:', e.ctrlKey);

            if (e.ctrlKey) {
                // Ctrl+wheel = zoom
                const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;
                const oldRange = spectrumMaxDb - spectrumMinDb;
                const newRange = Math.max(20, Math.min(400, oldRange * zoomFactor)); // Min 20, max 400 dB range
                const center = (spectrumMinDb + spectrumMaxDb) / 2;

                spectrumMinDb = Math.max(-50, center - newRange / 2);
                spectrumMaxDb = Math.min(350, center + newRange / 2);
            } else {
                // Regular wheel = scroll (shift the window up/down)
                const scrollAmount = e.deltaY * 0.5;
                const range = spectrumMaxDb - spectrumMinDb;

                // Shift both min and max by the same amount
                let newMin = spectrumMinDb + scrollAmount;
                let newMax = spectrumMaxDb + scrollAmount;

                // Allow extended range for scrolling (data is 0-255, but allow viewing beyond)
                const minLimit = -50;   // Allow scrolling below 0 to see noise floor
                const maxLimit = 350;   // Allow scrolling above 255 to accommodate strong signals

                // Clamp to extended range while maintaining window size
                if (newMin < minLimit) {
                    newMin = minLimit;
                    newMax = minLimit + range;
                }
                if (newMax > maxLimit) {
                    newMax = maxLimit;
                    newMin = maxLimit - range;
                }

                spectrumMinDb = newMin;
                spectrumMaxDb = newMax;
            }

            console.log('New range:', spectrumMinDb, 'to', spectrumMaxDb);

            // Redraw spectrum with new range
            if (latestFFTData) {
                drawSpectrum(latestFFTData, latestFFTData2);
            }
        });

        console.log('Spectrum event listeners attached');

        // Add same event handlers for spectrum2
        spectrumCanvas2.addEventListener('wheel', (e) => {
            e.preventDefault();
            if (e.ctrlKey) {
                const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;
                const oldRange = spectrumMaxDb - spectrumMinDb;
                const newRange = Math.max(20, Math.min(400, oldRange * zoomFactor));
                const center = (spectrumMinDb + spectrumMaxDb) / 2;
                spectrumMinDb = Math.max(-50, center - newRange / 2);
                spectrumMaxDb = Math.min(350, center + newRange / 2);
            } else {
                const scrollAmount = e.deltaY * 0.5;
                const range = spectrumMaxDb - spectrumMinDb;
                let newMin = spectrumMinDb + scrollAmount;
                let newMax = spectrumMaxDb + scrollAmount;
                const minLimit = -50;
                const maxLimit = 350;
                if (newMin < minLimit) {
                    newMin = minLimit;
                    newMax = minLimit + range;
                }
                if (newMax > maxLimit) {
                    newMax = maxLimit;
                    newMin = maxLimit - range;
                }
                spectrumMinDb = newMin;
                spectrumMaxDb = newMax;
            }
            if (latestFFTData && latestFFTData2) {
                drawSpectrum(latestFFTData, latestFFTData2);
            }
        });

        // Double-click to reset spectrum Y-axis to default range
        spectrumCanvas.addEventListener('dblclick', () => {
            spectrumMinDb = -100;
            spectrumMaxDb = -10;
            console.log('Spectrum Y-axis reset to default range: -100 to -10 dB');
            if (latestFFTData) {
                drawSpectrum(latestFFTData, latestFFTData2);
            }
        });

        spectrumCanvas2.addEventListener('dblclick', () => {
            spectrumMinDb = -100;
            spectrumMaxDb = -10;
            if (latestFFTData && latestFFTData2) {
                drawSpectrum(latestFFTData, latestFFTData2);
            }
        });

        // Click handler for bandwidth measurement
        spectrumCanvas.addEventListener('click', (e) => {
            if (!bwMeasureEnabled) return;

            const rect = spectrumCanvas.getBoundingClientRect();
            const x = e.clientX - rect.left;

            // Calculate frequency at click position
            const currentSR = zoomState.fullBandwidth || 40000000;
            const currentCF = zoomState.centerFreq || 915000000;
            const binWidth = currentSR / FFT_SIZE;
            const fullStartFreq = currentCF - currentSR / 2;

            const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
            const displayedStartFreq = fullStartFreq + (zoomState.zoomStartBin * binWidth);
            const displayedBandwidth = zoomedBins * binWidth;

            const clickFreq = displayedStartFreq + (x / rect.width) * displayedBandwidth;

            // Store normalized position (0-1) instead of absolute pixels
            const normalizedX = x / rect.width;

            // Add point (max 2 points)
            if (bwMeasurePoints.length >= 2) {
                bwMeasurePoints = [];
            }

            bwMeasurePoints.push({
                normalizedX: normalizedX,
                freq: clickFreq,
                db: 0 // Could calculate from spectrum data if needed
            });

            // Redraw to show points
            if (latestFFTData) {
                drawSpectrum(latestFFTData, latestFFTData2);
            }
        });

        // Channel change handler
        document.getElementById('channel_select').addEventListener('change', () => {
            // Clear canvas when switching channels
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            if (canvas2) {
                ctx2.fillStyle = '#000';
                ctx2.fillRect(0, 0, canvas2.width, canvas2.height);
            }
            // Resize to handle dual-channel layout
            resizeCanvas();
        });

        // ===== INTERACTIVE ZOOM FUNCTIONALITY =====
        // Display-only zoom (no hardware reconfiguration)
        // Note: zoomState is declared at the top of the script

        // Update zoom state from server status
        function updateZoomState(freq, sr) {
            zoomState.centerFreq = freq;
            zoomState.fullBandwidth = sr;
        }

        // Update zoom level indicator in header
        function updateZoomIndicator() {
            const indicator = document.getElementById('zoom_indicator');
            const levelSpan = document.getElementById('zoom_level');

            if (!zoomState.isZoomed) {
                indicator.style.display = 'none';
            } else {
                indicator.style.display = 'inline';
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const zoomFactor = Math.round(FFT_SIZE / zoomedBins);
                levelSpan.textContent = zoomFactor + 'x';
            }
        }

        // Canvas mouse handlers for zoom and filter selection
        canvas.addEventListener('mousedown', (e) => {
            if (e.button === 0) {  // Left click
                if (filterState.enabled) {
                    // Filter selection mode
                    filterState.isSelecting = true;
                    filterState.startX = e.offsetX;
                    filterState.currentX = e.offsetX;
                } else {
                    // Zoom selection mode
                    zoomState.isSelecting = true;
                    zoomState.startX = e.offsetX;
                    zoomState.currentX = e.offsetX;
                }
            }
        });

        canvas.addEventListener('mousemove', (e) => {
            if (filterState.enabled && filterState.isSelecting) {
                filterState.currentX = e.offsetX;
                drawSelectionBox();
            } else if (zoomState.isSelecting) {
                zoomState.currentX = e.offsetX;
                drawSelectionBox();
            }
        });

        canvas.addEventListener('mouseup', (e) => {
            if (e.button === 0) {
                if (filterState.enabled && filterState.isSelecting) {
                    // Complete filter selection
                    filterState.isSelecting = false;
                    const x1 = Math.min(filterState.startX, filterState.currentX);
                    const x2 = Math.max(filterState.startX, filterState.currentX);

                    // Apply filter if selection is wide enough
                    if (x2 - x1 > 10) {
                        applyFilter(x1, x2);
                    }

                    // Clear selection box
                    drawSelectionBox(true);
                } else if (zoomState.isSelecting) {
                    // Complete zoom selection
                    zoomState.isSelecting = false;
                    const x1 = Math.min(zoomState.startX, zoomState.currentX);
                    const x2 = Math.max(zoomState.startX, zoomState.currentX);

                    // Zoom if selection is wide enough
                    if (x2 - x1 > 20) {
                        applyZoom(x1, x2);
                    }

                    // Clear selection box
                    drawSelectionBox(true);
                }
            }
        });

        canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            zoomOut();
            return false;
        });

        // Double-click to tune to frequency
        canvas.addEventListener('dblclick', (e) => {
            const rect = canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const canvasWidth = canvas.width;

            // Get current frequency and sample rate from display elements
            const freqElem = document.getElementById('freq');
            const srElem = document.getElementById('sr');

            if (!freqElem || !srElem) {
                console.warn('Frequency/samplerate elements not found');
                return;
            }

            const centerFreq = parseFloat(freqElem.textContent) || 915;
            const sampleRate = parseFloat(srElem.textContent) || 40;

            // Take zoom into account
            const zoomedRange = sampleRate * (zoomState.zoomEndBin - zoomState.zoomStartBin) / FFT_SIZE;
            const zoomedStart = centerFreq - sampleRate / 2 + (zoomState.zoomStartBin / FFT_SIZE) * sampleRate;

            const clickedFreq = zoomedStart + (x / canvasWidth) * zoomedRange;

            // Update frequency input element
            const freqInput = document.getElementById('freqInput');
            if (freqInput) {
                freqInput.value = clickedFreq.toFixed(3);
                applyFrequency();
            }

            // Show notification
            if (typeof showNotification === 'function') {
                showNotification(`Tuned to ${clickedFreq.toFixed(3)} MHz`, 'success', 2000);
            }
        });

        // Draw selection box overlay
        function drawSelectionBox(clear = false) {
            // This is a visual overlay - we'll use a second canvas
            let overlayCanvas = document.getElementById('overlay');
            if (!overlayCanvas) {
                overlayCanvas = document.createElement('canvas');
                overlayCanvas.id = 'overlay';
                overlayCanvas.style.position = 'fixed';
                overlayCanvas.style.top = '50px';
                overlayCanvas.style.left = '60px';
                overlayCanvas.style.pointerEvents = 'none';
                overlayCanvas.style.zIndex = '100';
                canvas.parentElement.appendChild(overlayCanvas);
            }

            overlayCanvas.width = canvas.width;
            overlayCanvas.height = canvas.height;
            const octx = overlayCanvas.getContext('2d');
            octx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);

            // Draw filter selection overlay (if active and not cleared)
            if (!clear && filterState.isFiltered) {
                const x1 = Math.floor((filterState.filterStartBin - zoomState.zoomStartBin) * canvas.width / (zoomState.zoomEndBin - zoomState.zoomStartBin + 1));
                const x2 = Math.floor((filterState.filterEndBin - zoomState.zoomStartBin) * canvas.width / (zoomState.zoomEndBin - zoomState.zoomStartBin + 1));
                octx.fillStyle = 'rgba(255, 165, 0, 0.15)';
                octx.fillRect(x1, 0, x2 - x1, canvas.height);
                octx.strokeStyle = '#fa0';
                octx.lineWidth = 2;
                octx.setLineDash([5, 5]);
                octx.strokeRect(x1, 0, x2 - x1, canvas.height);
                octx.setLineDash([]);
            }

            if (!clear && (zoomState.isSelecting || filterState.isSelecting)) {
                const isFilter = filterState.isSelecting;
                const state = isFilter ? filterState : zoomState;
                const x1 = Math.min(state.startX, state.currentX);
                const x2 = Math.max(state.startX, state.currentX);
                const width = x2 - x1;

                // Selection box colors
                const fillColor = isFilter ? 'rgba(255, 165, 0, 0.2)' : 'rgba(0, 255, 255, 0.1)';
                const borderColor = isFilter ? '#fa0' : '#0ff';
                const label = isFilter ? 'Filter' : 'Zoom';

                // Semi-transparent selection box
                octx.fillStyle = fillColor;
                octx.fillRect(x1, 0, width, canvas.height);

                // Border
                octx.strokeStyle = borderColor;
                octx.lineWidth = 2;
                octx.strokeRect(x1, 0, width, canvas.height);

                // Frequency labels on selection (respect current zoom)
                const currentSR = zoomState.fullBandwidth;
                const currentCF = zoomState.centerFreq;
                const fullStartFreq = currentCF - currentSR / 2;

                // Calculate frequency range currently displayed (respecting zoom)
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const binWidth = currentSR / FFT_SIZE;
                const displayedStartFreq = fullStartFreq + (zoomState.zoomStartBin * binWidth);
                const displayedBandwidth = zoomedBins * binWidth;

                const freq1 = displayedStartFreq + (x1 / canvas.width) * displayedBandwidth;
                const freq2 = displayedStartFreq + (x2 / canvas.width) * displayedBandwidth;
                const bw = freq2 - freq1;

                octx.fillStyle = borderColor;
                octx.font = '12px monospace';
                octx.fillText(`${label} BW: ${(bw / 1e6).toFixed(2)} MHz`, x1 + 5, 20);
            }
        }

        // Apply display zoom to selected region (no hardware reconfiguration)
        function applyZoom(x1, x2) {
            // Calculate FFT bin indices for the selected region
            // Map canvas X coordinates to FFT bins, respecting current zoom
            const currentZoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
            const bin1 = zoomState.zoomStartBin + Math.floor((x1 / canvas.width) * currentZoomedBins);
            const bin2 = zoomState.zoomStartBin + Math.floor((x2 / canvas.width) * currentZoomedBins);

            // Set zoom region
            zoomState.zoomStartBin = Math.max(0, bin1);
            zoomState.zoomEndBin = Math.min(FFT_SIZE - 1, bin2);
            zoomState.isZoomed = true;

            const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
            const zoomFactor = (FFT_SIZE / zoomedBins).toFixed(1);

            console.log(`Display zoom: bins ${zoomState.zoomStartBin}-${zoomState.zoomEndBin} (${zoomedBins} bins, ${zoomFactor}x)`);

            updateZoomIndicator();
            updateTimeAxis();  // Update frequency labels
        }

        // Zoom out to full spectrum
        function zoomOut() {
            if (!zoomState.isZoomed) {
                console.log('Already at full spectrum');
                return;
            }

            console.log('Zooming out to full spectrum');

            // Reset to full spectrum
            zoomState.zoomStartBin = 0;
            zoomState.zoomEndBin = FFT_SIZE - 1;
            zoomState.isZoomed = false;

            // Update display modules with new zoom state
            if (typeof WaterfallDisplay !== 'undefined') {
                WaterfallDisplay.updateZoomState(zoomState);
            }
            if (typeof SpectrumDisplay !== 'undefined') {
                SpectrumDisplay.updateZoomState(zoomState);
            }

            updateZoomIndicator();
            updateTimeAxis();  // Update frequency labels
        }

        // ========================================================================
        // ADVANCED SIGNAL ANALYSIS FEATURES
        // ========================================================================

        // Note: signalAnalysis is declared at the top of the script

        // Demodulator state
        let demodulator = {
            active: false,
            mode: 'off',
            audioCtx: null,
            oscillator: null,
            gainNode: null,
            scriptNode: null,
            sampleBuffer: [],
            phase: 0,
            lastSample: 0
        };

        // ===== COLOR PALETTE FUNCTIONS =====

        // Color palette implementations
        function getColorForValue(value, palette) {
            value = Math.max(0, Math.min(1, value));

            switch(palette) {
                case 'viridis':
                    return viridisColor(value);
                case 'plasma':
                    return plasmaColor(value);
                case 'inferno':
                    return infernoColor(value);
                case 'turbo':
                    return turboColor(value);
                case 'hot':
                    return hotColor(value);
                case 'cool':
                    return coolColor(value);
                case 'grayscale':
                    return grayscaleColor(value);
                case 'rainbow':
                    return rainbowColor(value);
                default:
                    return viridisColor(value);
            }
        }

        function plasmaColor(t) {
            const r = Math.floor(255 * (0.05 + 2.4*t - 5.1*t*t + 3.8*t*t*t));
            const g = Math.floor(255 * (0.02 + 1.7*t - 1.9*t*t));
            const b = Math.floor(255 * (0.6 + 1.3*t - 1.6*t*t));
            return [Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b))];
        }

        function infernoColor(t) {
            const r = Math.floor(255 * Math.min(1, 1.8*t));
            const g = Math.floor(255 * Math.max(0, 1.5*t - 0.5));
            const b = Math.floor(255 * Math.max(0, 3*t - 2));
            return [r, g, b];
        }

        function turboColor(t) {
            const r = Math.floor(255 * (0.13 + 1.5*t - 2.7*t*t + 2.2*t*t*t));
            const g = Math.floor(255 * (-0.01 + 2.6*t - 3.5*t*t + 1.9*t*t*t));
            const b = Math.floor(255 * (0.7 - 1.5*t + 2.3*t*t - 1.5*t*t*t));
            return [Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b))];
        }

        function hotColor(t) {
            let r, g, b;
            if (t < 0.4) {
                r = 255 * (t / 0.4);
                g = 0;
                b = 0;
            } else if (t < 0.7) {
                r = 255;
                g = 255 * ((t - 0.4) / 0.3);
                b = 0;
            } else {
                r = 255;
                g = 255;
                b = 255 * ((t - 0.7) / 0.3);
            }
            return [Math.floor(r), Math.floor(g), Math.floor(b)];
        }

        function coolColor(t) {
            const r = Math.floor(255 * t);
            const g = Math.floor(255 * (1 - t));
            const b = 255;
            return [r, g, b];
        }

        function grayscaleColor(t) {
            const v = Math.floor(255 * t);
            return [v, v, v];
        }

        function rainbowColor(t) {
            const h = t * 360;
            const s = 1;
            const v = 1;
            const c = v * s;
            const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
            const m = v - c;
            let r, g, b;
            if (h < 60) { r = c; g = x; b = 0; }
            else if (h < 120) { r = x; g = c; b = 0; }
            else if (h < 180) { r = 0; g = c; b = x; }
            else if (h < 240) { r = 0; g = x; b = c; }
            else if (h < 300) { r = x; g = 0; b = c; }
            else { r = c; g = 0; b = x; }
            return [Math.floor((r + m) * 255), Math.floor((g + m) * 255), Math.floor((b + m) * 255)];
        }

        function changeColorPalette() {
            signalAnalysis.colorPalette = document.getElementById('colorPalette').value;
            console.log('Changed color palette to:', signalAnalysis.colorPalette);

            // Update WaterfallDisplay module
            if (typeof WaterfallDisplay !== 'undefined') {
                WaterfallDisplay.setColorPalette(signalAnalysis.colorPalette);
            }
        }

        // ===== PERSISTENCE DISPLAY MODES =====

        function changePersistenceMode() {
            const mode = document.getElementById('persistence_mode').value;
            signalAnalysis.persistenceMode = mode;

            if (mode === 'none') {
                signalAnalysis.persistenceData = null;
                signalAnalysis.avgCount = 0;
            } else if (!signalAnalysis.persistenceData) {
                signalAnalysis.persistenceData = new Uint8Array(FFT_SIZE);
                if (mode === 'min') {
                    signalAnalysis.persistenceData.fill(255);
                }
            }

            console.log('Persistence mode:', mode);
        }

        function resetPersistence() {
            signalAnalysis.persistenceData = null;
            signalAnalysis.avgCount = 0;
            console.log('Persistence reset');
        }

        function applyPersistence(data) {
            if (signalAnalysis.persistenceMode === 'none') {
                return data;
            }

            if (!signalAnalysis.persistenceData) {
                signalAnalysis.persistenceData = new Uint8Array(data);
                return data;
            }

            const result = new Uint8Array(data.length);

            switch(signalAnalysis.persistenceMode) {
                case 'max':
                    for (let i = 0; i < data.length; i++) {
                        signalAnalysis.persistenceData[i] = Math.max(signalAnalysis.persistenceData[i], data[i]);
                        result[i] = signalAnalysis.persistenceData[i];
                    }
                    break;

                case 'min':
                    for (let i = 0; i < data.length; i++) {
                        signalAnalysis.persistenceData[i] = Math.min(signalAnalysis.persistenceData[i], data[i]);
                        result[i] = signalAnalysis.persistenceData[i];
                    }
                    break;

                case 'avg':
                    signalAnalysis.avgCount++;
                    for (let i = 0; i < data.length; i++) {
                        signalAnalysis.persistenceData[i] = (signalAnalysis.persistenceData[i] * (signalAnalysis.avgCount - 1) + data[i]) / signalAnalysis.avgCount;
                        result[i] = Math.floor(signalAnalysis.persistenceData[i]);
                    }
                    break;

                case 'decay':
                    for (let i = 0; i < data.length; i++) {
                        signalAnalysis.persistenceData[i] = Math.max(
                            signalAnalysis.persistenceData[i] * signalAnalysis.persistenceDecayRate,
                            data[i]
                        );
                        result[i] = Math.floor(signalAnalysis.persistenceData[i]);
                    }
                    break;
            }

            return result;
        }

        // ===== SIGNAL MEASUREMENT FUNCTIONS =====

        // NOTE: rawToDb already defined in main scope with correct formula
        // DO NOT redefine - use the global version: (raw / 255.0) * 120.0 - 100.0
        // This maps 0-255 to -100dB to +20dB (120dB range) matching C++ compute_magnitude_db()

        function updateMeasurements() {
            if (!latestFFTData || latestFFTData.length === 0) {
                console.log('No FFT data available');
                return;
            }

            const data = latestFFTData;
            let peakRaw = 0;
            let sumRaw = 0;

            // Find peak and calculate average
            for (let i = 0; i < data.length; i++) {
                peakRaw = Math.max(peakRaw, data[i]);
                sumRaw += data[i];
            }

            const avgRaw = sumRaw / data.length;
            const peakDb = rawToDb(peakRaw);
            const avgDb = rawToDb(avgRaw);

            // Estimate noise floor (lowest 10% of bins)
            const sorted = Array.from(data).sort((a, b) => a - b);
            const noiseFloorIdx = Math.floor(sorted.length * 0.1);
            const noiseFloorRaw = sorted[noiseFloorIdx];
            const noiseFloorDb = rawToDb(noiseFloorRaw);

            // Calculate SNR
            const snr = peakDb - noiseFloorDb;

            // Calculate occupied bandwidth (-3dB points)
            // Raw data is quantized dB: raw = (dB + 100) / 120 * 255
            // For -3dB: threshold_raw = peakRaw - (3 / 120 * 255) = peakRaw - 6.375
            const threshold3db = peakRaw - 6.375;
            let firstIdx = -1, lastIdx = -1;
            for (let i = 0; i < data.length; i++) {
                if (data[i] >= threshold3db) {
                    if (firstIdx === -1) firstIdx = i;
                    lastIdx = i;
                }
            }

            const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
            const obw = firstIdx >= 0 ? ((lastIdx - firstIdx) * sr / FFT_SIZE) / 1e6 : 0;

            // Update display
            document.getElementById('meas_peak_power').textContent = peakDb.toFixed(1);
            document.getElementById('meas_avg_power').textContent = avgDb.toFixed(1);
            document.getElementById('meas_noise_floor').textContent = noiseFloorDb.toFixed(1);
            document.getElementById('meas_snr').textContent = snr.toFixed(1);
            document.getElementById('meas_obw').textContent = obw.toFixed(3);

            console.log('Measurements updated:', { peakDb, avgDb, noiseFloorDb, snr, obw });
        }

        // ===== PEAK DETECTION =====

        function detectPeaks() {
            if (!latestFFTData || latestFFTData.length === 0) {
                showNotification('No FFT data available', 'warning');
                return;
            }

            const threshold = parseFloat(document.getElementById('peak_threshold').value);
            // Convert dB threshold to raw using correct 120 dB range formula
            const thresholdRaw = ((threshold + 100) / 120) * 255;
            const data = latestFFTData;
            const peaks = [];

            // Find peaks above threshold with local maxima detection
            for (let i = 10; i < data.length - 10; i++) {
                if (data[i] > thresholdRaw) {
                    // Check if local maximum
                    let isMax = true;
                    for (let j = i - 5; j <= i + 5; j++) {
                        if (j !== i && data[j] > data[i]) {
                            isMax = false;
                            break;
                        }
                    }

                    if (isMax) {
                        const freq = parseFloat(document.getElementById('freqInput').value) * 1e6;
                        const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
                        const binFreq = freq - (sr / 2) + (i * sr / FFT_SIZE);
                        const powerDb = rawToDb(data[i]);

                        peaks.push({ bin: i, freq: binFreq, power: powerDb });
                    }
                }
            }

            // Sort by power
            peaks.sort((a, b) => b.power - a.power);

            // Display top 20 peaks
            const listDiv = document.getElementById('peak_list');
            if (peaks.length === 0) {
                listDiv.innerHTML = '<div style="color: #888;">No peaks found above threshold</div>';
            } else {
                let html = '<table style="width: 100%; font-size: 10px;"><tr><th>Freq (MHz)</th><th>Power (dBFS)</th></tr>';
                for (let i = 0; i < Math.min(peaks.length, 20); i++) {
                    const p = peaks[i];
                    html += `<tr><td>${(p.freq / 1e6).toFixed(3)}</td><td>${p.power.toFixed(1)}</td></tr>`;
                }
                html += '</table>';
                listDiv.innerHTML = html;
            }

            signalAnalysis.peakMarkers = peaks.slice(0, 20);
            console.log('Detected', peaks.length, 'peaks');
        }

        // ===== BOOKMARK SYSTEM =====

        function addBookmark() {
            const name = document.getElementById('bookmark_name').value.trim();
            if (!name) {
                showNotification('Please enter a signal name', 'warning');
                return;
            }

            const freq = parseFloat(document.getElementById('freqInput').value);
            const bookmark = {
                name: name,
                freq: freq,
                timestamp: new Date().toISOString()
            };

            signalAnalysis.bookmarks.push(bookmark);
            localStorage.setItem('signal_bookmarks', JSON.stringify(signalAnalysis.bookmarks));
            updateBookmarkList();
            document.getElementById('bookmark_name').value = '';
            console.log('Bookmark added:', bookmark);
        }

        function updateBookmarkList() {
            const listDiv = document.getElementById('bookmark_list');
            if (!listDiv) return;  // Element doesn't exist, skip silently

            if (signalAnalysis.bookmarks.length === 0) {
                listDiv.innerHTML = '<div style="color: #888;">No bookmarks</div>';
                return;
            }

            let html = '';
            signalAnalysis.bookmarks.forEach((b, idx) => {
                html += `<div style="margin: 2px 0; padding: 3px; background: #111; border-radius: 2px;">
                    <div style="display: flex; justify-content: space-between;">
                        <span style="color: #0ff;">${b.name}</span>
                        <button onclick="deleteBookmark(${idx})" style="padding: 0 4px; font-size: 9px;">×</button>
                    </div>
                    <div style="color: #888; font-size: 9px;">${b.freq.toFixed(3)} MHz</div>
                </div>`;
            });
            listDiv.innerHTML = html;
        }

        function deleteBookmark(idx) {
            signalAnalysis.bookmarks.splice(idx, 1);
            localStorage.setItem('signal_bookmarks', JSON.stringify(signalAnalysis.bookmarks));
            updateBookmarkList();
        }

        // ===== MEASUREMENT CURSORS =====

        function toggleCursors() {
            signalAnalysis.cursorsEnabled = !signalAnalysis.cursorsEnabled;
            const btn = document.getElementById('cursor_toggle');
            btn.classList.toggle('active', signalAnalysis.cursorsEnabled);

            const overlay = document.getElementById('cursor_overlay');
            overlay.style.pointerEvents = signalAnalysis.cursorsEnabled ? 'auto' : 'none';

            if (!signalAnalysis.cursorsEnabled) {
                const ctx = overlay.getContext('2d');
                ctx.clearRect(0, 0, overlay.width, overlay.height);
            }

            console.log('Cursors:', signalAnalysis.cursorsEnabled ? 'enabled' : 'disabled');
        }

        // Setup cursor overlay
        const cursorOverlay = document.getElementById('cursor_overlay');
        cursorOverlay.width = window.innerWidth;
        cursorOverlay.height = window.innerHeight;

        cursorOverlay.addEventListener('mousemove', (e) => {
            if (!signalAnalysis.cursorsEnabled) return;

            signalAnalysis.cursorPos.x = e.clientX;
            signalAnalysis.cursorPos.y = e.clientY;

            drawCursors();
        });

        function drawCursors() {
            if (!signalAnalysis.cursorsEnabled || !signalAnalysis.cursorPos.x) return;

            const overlay = document.getElementById('cursor_overlay');
            const ctx = overlay.getContext('2d');
            ctx.clearRect(0, 0, overlay.width, overlay.height);

            // Draw crosshair
            ctx.strokeStyle = '#0ff';
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);

            // Vertical line
            ctx.beginPath();
            ctx.moveTo(signalAnalysis.cursorPos.x, 0);
            ctx.lineTo(signalAnalysis.cursorPos.x, overlay.height);
            ctx.stroke();

            // Horizontal line
            ctx.beginPath();
            ctx.moveTo(0, signalAnalysis.cursorPos.y);
            ctx.lineTo(overlay.width, signalAnalysis.cursorPos.y);
            ctx.stroke();

            ctx.setLineDash([]);

            // Calculate and display frequency
            const canvas = document.getElementById('waterfall');
            if (canvas) {
                const rect = canvas.getBoundingClientRect();
                const x = signalAnalysis.cursorPos.x - rect.left;

                if (x >= 0 && x <= rect.width) {
                    const freq = parseFloat(document.getElementById('freqInput').value) * 1e6;
                    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
                    const binFreq = freq - (sr / 2) + ((x / rect.width) * sr);

                    // Draw frequency label
                    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
                    ctx.fillRect(signalAnalysis.cursorPos.x + 5, signalAnalysis.cursorPos.y - 30, 150, 25);
                    ctx.fillStyle = '#0ff';
                    ctx.font = '12px monospace';
                    ctx.fillText(`${(binFreq / 1e6).toFixed(3)} MHz`, signalAnalysis.cursorPos.x + 10, signalAnalysis.cursorPos.y - 10);
                }
            }
        }

        window.addEventListener('resize', () => {
            cursorOverlay.width = window.innerWidth;
            cursorOverlay.height = window.innerHeight;
        });

        // ===== COMPREHENSIVE DEMODULATION ENGINE (Phase 3) =====

        // Enhanced demodulator state with audio processing chain
        let demodState = {
            active: false,
            mode: 'off',
            audioCtx: null,
            audioWorklet: null,
            gainNode: null,
            audioBuffer: [],
            sampleBuffer: 4096,
            audioSampleRate: 48000,

            // Demodulator state variables
            lastPhase: 0,
            dcI: 0,
            dcQ: 0,
            dcAlpha: 0.001,
            agcGain: 1.0,
            agcTarget: 0.3,
            agcAttack: 0.1,
            agcDecay: 0.001,

            // Audio filter (simple IIR lowpass)
            filterState: [0, 0],
            filterCoeffs: { a: [1, -0.95], b: [0.025, 0.025] },

            // Squelch
            squelchOpen: false,
            squelchLevel: -100,

            // Digital demod state
            symbolBuffer: [],
            bitBuffer: [],
            symbolRate: 1200,
            lastSymbolTime: 0,

            // Statistics
            audioLevel: 0,
            snrEstimate: 0,
            freqOffset: 0
        };

        // Multi-signal tracking system
        let signalTracker = {
            signals: [],  // Array of tracked signals
            maxSignals: 20,
            detectionThreshold: -80,  // dBm
            updateInterval: null,

            addSignal: function(freq, power, bandwidth) {
                const existing = this.signals.find(s => Math.abs(s.freq - freq) < bandwidth/2);
                if (existing) {
                    existing.power = power;
                    existing.lastSeen = Date.now();
                    existing.duration += 0.1;
                } else if (this.signals.length < this.maxSignals) {
                    this.signals.push({
                        id: Date.now(),
                        freq: freq,
                        power: power,
                        bandwidth: bandwidth,
                        firstSeen: Date.now(),
                        lastSeen: Date.now(),
                        duration: 0,
                        classification: 'Unknown'
                    });
                }
            },

            removeStale: function() {
                const now = Date.now();
                this.signals = this.signals.filter(s => (now - s.lastSeen) < 2000);
            },

            detectHopping: function() {
                // Detect frequency hopping by analyzing timing patterns
                const recentSignals = this.signals.filter(s => s.duration < 1.0);
                if (recentSignals.length >= 3) {
                    // Check if signals have similar timing patterns
                    const timeDiffs = [];
                    for (let i = 1; i < recentSignals.length; i++) {
                        timeDiffs.push(recentSignals[i].firstSeen - recentSignals[i-1].firstSeen);
                    }
                    const avgDiff = timeDiffs.reduce((a,b) => a+b, 0) / timeDiffs.length;
                    const variance = timeDiffs.reduce((sum, d) => sum + Math.pow(d - avgDiff, 2), 0) / timeDiffs.length;

                    if (variance < avgDiff * 0.2) {  // Low variance = regular hopping
                        return {
                            detected: true,
                            hopRate: 1000 / avgDiff,
                            channels: recentSignals.map(s => s.freq)
                        };
                    }
                }
                return { detected: false };
            }
        };

        // Interference analysis engine
        let interferenceAnalyzer = {
            harmonics: [],
            imdProducts: [],

            detectHarmonics: function(spectrum, fundamentalFreq, centerFreq, sampleRate) {
                this.harmonics = [];
                const binWidth = sampleRate / spectrum.length;
                const startFreq = centerFreq - sampleRate / 2;

                // Check up to 10th harmonic
                for (let n = 2; n <= 10; n++) {
                    const harmonicFreq = fundamentalFreq * n;
                    if (harmonicFreq > centerFreq + sampleRate/2) break;

                    const bin = Math.floor((harmonicFreq - startFreq) / binWidth);
                    if (bin >= 0 && bin < spectrum.length) {
                        const power = (spectrum[bin] / 255.0) * 120 - 100;
                        if (power > -80) {
                            this.harmonics.push({
                                order: n,
                                freq: harmonicFreq,
                                power: power,
                                bin: bin
                            });
                        }
                    }
                }
                return this.harmonics;
            },

            detectIMD: function(spectrum, signal1Freq, signal2Freq, centerFreq, sampleRate) {
                this.imdProducts = [];
                const binWidth = sampleRate / spectrum.length;
                const startFreq = centerFreq - sampleRate / 2;

                // Check 3rd order IMD products: 2*f1 - f2 and 2*f2 - f1
                const imd3_1 = 2 * signal1Freq - signal2Freq;
                const imd3_2 = 2 * signal2Freq - signal1Freq;

                for (const imdFreq of [imd3_1, imd3_2]) {
                    if (imdFreq < startFreq || imdFreq > centerFreq + sampleRate/2) continue;

                    const bin = Math.floor((imdFreq - startFreq) / binWidth);
                    if (bin >= 0 && bin < spectrum.length) {
                        const power = (spectrum[bin] / 255.0) * 120 - 100;
                        if (power > -80) {
                            this.imdProducts.push({
                                order: 3,
                                freq: imdFreq,
                                power: power,
                                signal1: signal1Freq,
                                signal2: signal2Freq
                            });
                        }
                    }
                }
                return this.imdProducts;
            },

            recommendMitigation: function() {
                const recommendations = [];

                if (this.harmonics.length > 0) {
                    recommendations.push({
                        type: 'Harmonics Detected',
                        severity: 'Medium',
                        suggestion: `${this.harmonics.length} harmonic(s) detected. Consider using harmonic filter or reducing drive level.`
                    });
                }

                if (this.imdProducts.length > 0) {
                    recommendations.push({
                        type: 'Intermodulation Detected',
                        severity: 'High',
                        suggestion: `IMD products detected. Reduce signal levels or increase frequency separation.`
                    });
                }

                return recommendations;
            }
        };

        // Protocol decoders
        let protocolDecoders = {
            // ADS-B decoder (1090 MHz aircraft transponder)
            adsb: {
                enabled: false,
                preamblePattern: [1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0],  // ADS-B preamble
                messageBuffer: [],

                decode: function(bits) {
                    // Look for preamble
                    for (let i = 0; i < bits.length - 112; i++) {
                        let preambleMatch = true;
                        for (let j = 0; j < this.preamblePattern.length; j++) {
                            if (bits[i+j] !== this.preamblePattern[j]) {
                                preambleMatch = false;
                                break;
                            }
                        }

                        if (preambleMatch) {
                            const message = bits.slice(i + 16, i + 128);  // 112 bits
                            return this.parseADSB(message);
                        }
                    }
                    return null;
                },

                parseADSB: function(bits) {
                    const df = this.bitsToInt(bits.slice(0, 5));  // Downlink Format

                    if (df === 17) {  // ADS-B message
                        const ca = this.bitsToInt(bits.slice(5, 8));
                        const icao = this.bitsToHex(bits.slice(8, 32));
                        const typeCode = this.bitsToInt(bits.slice(32, 37));

                        return {
                            type: 'ADS-B',
                            icao: icao,
                            typeCode: typeCode,
                            data: this.bitsToHex(bits.slice(32, 88))
                        };
                    }
                    return null;
                },

                bitsToInt: function(bits) {
                    return parseInt(bits.join(''), 2);
                },

                bitsToHex: function(bits) {
                    return this.bitsToInt(bits).toString(16).toUpperCase().padStart(bits.length/4, '0');
                }
            },

            // AIS decoder (VHF marine transponder)
            ais: {
                enabled: false,
                baudRate: 9600,
                messageBuffer: [],

                decode: function(bits) {
                    // AIS uses NRZI encoding and HDLC framing
                    // Look for HDLC flag: 01111110
                    const flag = [0,1,1,1,1,1,1,0];

                    for (let i = 0; i < bits.length - 8; i++) {
                        let flagMatch = true;
                        for (let j = 0; j < flag.length; j++) {
                            if (bits[i+j] !== flag[j]) {
                                flagMatch = false;
                                break;
                            }
                        }

                        if (flagMatch) {
                            // Extract message between flags (up to 256 bits)
                            const nextFlag = this.findNextFlag(bits, i + 8);
                            if (nextFlag > 0) {
                                const message = bits.slice(i + 8, nextFlag);
                                return this.parseAIS(message);
                            }
                        }
                    }
                    return null;
                },

                findNextFlag: function(bits, start) {
                    const flag = [0,1,1,1,1,1,1,0];
                    for (let i = start; i < Math.min(start + 256, bits.length - 8); i++) {
                        let match = true;
                        for (let j = 0; j < flag.length; j++) {
                            if (bits[i+j] !== flag[j]) {
                                match = false;
                                break;
                            }
                        }
                        if (match) return i;
                    }
                    return -1;
                },

                parseAIS: function(bits) {
                    if (bits.length < 168) return null;

                    const messageType = this.bitsToInt(bits.slice(0, 6));
                    const mmsi = this.bitsToInt(bits.slice(8, 38));

                    return {
                        type: 'AIS',
                        messageType: messageType,
                        mmsi: mmsi.toString(),
                        raw: this.bitsToHex(bits)
                    };
                },

                bitsToInt: function(bits) {
                    return parseInt(bits.join(''), 2);
                },

                bitsToHex: function(bits) {
                    return this.bitsToInt(bits).toString(16).toUpperCase();
                }
            }
        };

        // AM Demodulator: Envelope detection
        function demodulateAM(i_samples, q_samples) {
            const output = new Float32Array(i_samples.length);

            for (let n = 0; n < i_samples.length; n++) {
                // Normalize to float
                const i = i_samples[n] / 2048.0;
                const q = q_samples[n] / 2048.0;

                // DC removal
                demodState.dcI = demodState.dcAlpha * i + (1 - demodState.dcAlpha) * demodState.dcI;
                demodState.dcQ = demodState.dcAlpha * q + (1 - demodState.dcAlpha) * demodState.dcQ;

                const i_dc = i - demodState.dcI;
                const q_dc = q - demodState.dcQ;

                // Envelope detection: |I + jQ|
                const magnitude = Math.sqrt(i_dc * i_dc + q_dc * q_dc);

                // Simple DC blocker for audio (remove carrier)
                const audioSample = magnitude - 1.0;

                // Apply IIR lowpass filter (audio bandwidth limiting)
                const filtered = demodState.filterCoeffs.b[0] * audioSample +
                                demodState.filterCoeffs.b[1] * demodState.filterState[0] -
                                demodState.filterCoeffs.a[1] * demodState.filterState[1];

                demodState.filterState[0] = audioSample;
                demodState.filterState[1] = filtered;

                output[n] = filtered;
            }

            return output;
        }

        // FM Demodulator: Phase differentiation
        function demodulateFM(i_samples, q_samples, wideband = false) {
            const output = new Float32Array(i_samples.length);
            const deviation = wideband ? 75000 : 2500;  // WFM: 75kHz, NFM: 2.5kHz

            for (let n = 0; n < i_samples.length; n++) {
                const i = i_samples[n] / 2048.0;
                const q = q_samples[n] / 2048.0;

                // Calculate instantaneous phase
                const phase = Math.atan2(q, i);

                // Phase difference (frequency deviation)
                let phaseDiff = phase - demodState.lastPhase;

                // Unwrap phase (handle ±π discontinuities)
                while (phaseDiff > Math.PI) phaseDiff -= 2 * Math.PI;
                while (phaseDiff < -Math.PI) phaseDiff += 2 * Math.PI;

                demodState.lastPhase = phase;

                // Convert phase difference to audio sample
                // Normalize by deviation and sample rate
                const audioSample = phaseDiff / (2 * Math.PI * deviation / demodState.audioSampleRate);

                // Lowpass filter
                const filtered = demodState.filterCoeffs.b[0] * audioSample +
                                demodState.filterCoeffs.b[1] * demodState.filterState[0] -
                                demodState.filterCoeffs.a[1] * demodState.filterState[1];

                demodState.filterState[0] = audioSample;
                demodState.filterState[1] = filtered;

                output[n] = filtered;
            }

            return output;
        }

        // FSK Demodulator: Frequency shift keying bit extraction
        function demodulateFSK(i_samples, q_samples, baudRate = 1200) {
            const bits = [];
            const samplesPerSymbol = Math.floor(demodState.audioSampleRate / baudRate);

            for (let n = 0; n < i_samples.length - 1; n += samplesPerSymbol) {
                const i1 = i_samples[n] / 2048.0;
                const q1 = q_samples[n] / 2048.0;
                const i2 = i_samples[n + 1] / 2048.0;
                const q2 = q_samples[n + 1] / 2048.0;

                const phase1 = Math.atan2(q1, i1);
                const phase2 = Math.atan2(q2, i2);

                let phaseDiff = phase2 - phase1;
                while (phaseDiff > Math.PI) phaseDiff -= 2 * Math.PI;
                while (phaseDiff < -Math.PI) phaseDiff += 2 * Math.PI;

                // Positive phase change = '1', negative = '0'
                bits.push(phaseDiff > 0 ? 1 : 0);
            }

            return bits;
        }

        // PSK Demodulator: Phase shift keying bit extraction
        function demodulatePSK(i_samples, q_samples, baudRate = 1200) {
            const bits = [];
            const samplesPerSymbol = Math.floor(demodState.audioSampleRate / baudRate);

            for (let n = 0; n < i_samples.length; n += samplesPerSymbol) {
                const i = i_samples[n] / 2048.0;
                const q = q_samples[n] / 2048.0;

                const phase = Math.atan2(q, i);

                // BPSK: phase 0° = '1', phase 180° = '0'
                const bit = (Math.cos(phase) > 0) ? 1 : 0;
                bits.push(bit);
            }

            return bits;
        }

        // Apply AGC to audio samples
        function applyAGC(samples) {
            const output = new Float32Array(samples.length);

            for (let n = 0; n < samples.length; n++) {
                const sample = samples[n];

                // Measure signal level
                const level = Math.abs(sample);

                // Adjust gain
                if (level > demodState.agcTarget) {
                    demodState.agcGain *= (1 - demodState.agcAttack);
                } else {
                    demodState.agcGain *= (1 + demodState.agcDecay);
                }

                // Clamp gain
                demodState.agcGain = Math.max(0.1, Math.min(10.0, demodState.agcGain));

                // Apply gain
                output[n] = sample * demodState.agcGain;

                // Update audio level display
                demodState.audioLevel = 0.99 * demodState.audioLevel + 0.01 * level;
            }

            return output;
        }

        // Process demodulated audio and output to speakers
        function processAudio(audioSamples) {
            if (!demodState.audioCtx || !demodState.gainNode) return;

            // Apply AGC
            const agcSamples = applyAGC(audioSamples);

            // Create audio buffer
            const audioBuffer = demodState.audioCtx.createBuffer(1, agcSamples.length, demodState.audioSampleRate);
            const channelData = audioBuffer.getChannelData(0);

            for (let i = 0; i < agcSamples.length; i++) {
                channelData[i] = Math.max(-1, Math.min(1, agcSamples[i]));
            }

            // Create buffer source and play
            const source = demodState.audioCtx.createBufferSource();
            source.buffer = audioBuffer;
            source.connect(demodState.gainNode);
            source.start();
        }

        // Main demodulation processing loop
        async function processDemodulation() {
            if (!demodState.active) return;

            try {
                const response = await fetchWithTimeout('/iq_data?t=' + Date.now());
                const buffer = await response.arrayBuffer();
                    const int16View = new Int16Array(buffer);
                    const samplesPerChannel = int16View.length / 4;

                    const ch1_i = int16View.slice(0, samplesPerChannel);
                    const ch1_q = int16View.slice(samplesPerChannel, samplesPerChannel * 2);

                    let audioSamples = null;
                    let decodedBits = null;

                    switch(demodState.mode) {
                        case 'am':
                            audioSamples = demodulateAM(ch1_i, ch1_q);
                            processAudio(audioSamples);
                            break;

                        case 'fm':
                            audioSamples = demodulateFM(ch1_i, ch1_q, false);
                            processAudio(audioSamples);
                            break;

                        case 'wfm':
                            audioSamples = demodulateFM(ch1_i, ch1_q, true);
                            processAudio(audioSamples);
                            break;

                        case 'fsk':
                            decodedBits = demodulateFSK(ch1_i, ch1_q, 1200);
                            demodState.bitBuffer.push(...decodedBits);

                            // Try protocol decoders
                            if (protocolDecoders.ais.enabled && demodState.bitBuffer.length > 256) {
                                const aisMsg = protocolDecoders.ais.decode(demodState.bitBuffer);
                                if (aisMsg) {
                                    displayDecodedData(aisMsg);
                                    demodState.bitBuffer = [];
                                }
                            }
                            break;

                        case 'psk':
                            decodedBits = demodulatePSK(ch1_i, ch1_q, 1200);
                            demodState.bitBuffer.push(...decodedBits);

                            // Try protocol decoders
                            if (protocolDecoders.adsb.enabled && demodState.bitBuffer.length > 128) {
                                const adsbMsg = protocolDecoders.adsb.decode(demodState.bitBuffer);
                                if (adsbMsg) {
                                    displayDecodedData(adsbMsg);
                                    demodState.bitBuffer = [];
                                }
                            }
                            break;
                    }

                // Update statistics
                updateDemodStats();

                // Schedule next processing
                setTimeout(processDemodulation, 50);  // 20 Hz update rate
            } catch (err) {
                console.error('Demodulation error:', err);
                setTimeout(processDemodulation, 100);
            }
        }

        // Display decoded protocol data
        function displayDecodedData(message) {
            const output = document.getElementById('decoded_messages');
            if (!output) return;

            // Remove "no messages" text if present
            if (output.textContent.includes('No messages decoded yet')) {
                output.innerHTML = '';
            }

            const timestamp = new Date().toLocaleTimeString();
            let text = `[${timestamp}] ${message.type}: `;

            if (message.type === 'ADS-B') {
                text += `ICAO=${message.icao} Type=${message.typeCode} Data=${message.data}`;
            } else if (message.type === 'AIS') {
                text += `MMSI=${message.mmsi} MsgType=${message.messageType} Raw=${message.raw}`;
            }

            const line = document.createElement('div');
            line.textContent = text;
            line.style.color = '#0f0';
            line.style.marginBottom = '4px';
            line.style.padding = '4px';
            line.style.background = '#0a0a0a';
            line.style.borderLeft = '2px solid #0f0';
            line.style.borderRadius = '2px';
            output.appendChild(line);

            // Scroll to bottom
            output.scrollTop = output.scrollHeight;

            // Limit to 100 lines
            while (output.children.length > 100) {
                output.removeChild(output.firstChild);
            }
        }

        // Update demodulation statistics display
        function updateDemodStats() {
            const statusDiv = document.getElementById('demod_status');
            if (!statusDiv) return;

            let status = `Status: ${demodState.mode.toUpperCase()} Active\\n`;
            status += `Audio Level: ${(demodState.audioLevel * 100).toFixed(1)}%\\n`;
            status += `AGC Gain: ${demodState.agcGain.toFixed(2)}x`;

            if (demodState.mode === 'fsk' || demodState.mode === 'psk') {
                status += `\\nBits Buffered: ${demodState.bitBuffer.length}`;
            }

            statusDiv.textContent = status;
        }

        // UI Control Functions
        function toggleDemod() {
            const panel = document.getElementById('demod_panel');
            panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
        }

        function changeDemodMode() {
            const mode = document.getElementById('demod_mode').value;
            const decodeDiv = document.getElementById('demod_decode');

            if (mode === 'fsk' || mode === 'psk') {
                decodeDiv.style.display = 'block';
            } else {
                decodeDiv.style.display = 'none';
            }
        }

        function startDemod() {
            if (demodState.active) {
                stopDemod();
                return;
            }

            const mode = document.getElementById('demod_mode').value;
            if (mode === 'off') {
                showNotification('Please select a demodulation mode', 'warning');
                return;
            }

            // Initialize Web Audio API
            if (!demodState.audioCtx) {
                demodState.audioCtx = new (window.AudioContext || window.webkitAudioContext)();
                demodState.gainNode = demodState.audioCtx.createGain();
                demodState.gainNode.connect(demodState.audioCtx.destination);
            }

            // Reset state
            demodState.mode = mode;
            demodState.active = true;
            demodState.lastPhase = 0;
            demodState.dcI = 0;
            demodState.dcQ = 0;
            demodState.agcGain = 1.0;
            demodState.filterState = [0, 0];
            demodState.bitBuffer = [];

            // Set volume
            const volume = document.getElementById('demod_volume').value / 100.0;
            demodState.gainNode.gain.value = volume;

            // Enable protocol decoders based on mode
            if (mode === 'psk') {
                protocolDecoders.adsb.enabled = true;
            } else if (mode === 'fsk') {
                protocolDecoders.ais.enabled = true;
            }

            document.getElementById('demod_start_btn').textContent = 'Stop Demodulation';
            document.getElementById('demod_status').textContent = `Status: Starting ${mode.toUpperCase()}...`;

            console.log(`Demodulator started: ${mode.toUpperCase()} mode`);

            // Start processing
            processDemodulation();
        }

        function stopDemod() {
            demodState.active = false;

            if (demodState.audioCtx) {
                demodState.audioCtx.close();
                demodState.audioCtx = null;
                demodState.gainNode = null;
            }

            // Disable protocol decoders
            protocolDecoders.adsb.enabled = false;
            protocolDecoders.ais.enabled = false;

            document.getElementById('demod_start_btn').textContent = 'Start Demodulation';
            document.getElementById('demod_status').textContent = 'Status: Stopped';
            console.log('Demodulator stopped');
        }

        // ===== DEMOD UI CONTROL FUNCTIONS =====

        // Enhanced demodulator state
        let demodEnhanced = {
            tunedFreq: null,  // Hz
            bandwidth: 15000,  // Hz
            afcEnabled: false,
            freqOffset: 0,
            squelchLevel: -100,
            squelchOpen: true,
            recording: false,
            recordedChunks: [],
            recordStartTime: null,
            signalPower: -100,
            sMeterValue: 0
        };

        // Demod spectrum canvas
        // Initialize S-meter
        function initSMeter() {
            sMeterCanvas = document.getElementById('s_meter_canvas');
            if (!sMeterCanvas) return;

            sMeterCtx = sMeterCanvas.getContext('2d');
            const parent = sMeterCanvas.parentElement;
            sMeterCanvas.width = parent.clientWidth;
            sMeterCanvas.height = 80;

            console.log('S-meter initialized');
        }

        // Update S-meter display
        function updateSMeter() {
            if (!sMeterCtx) return;

            const canvas = sMeterCanvas;
            const ctx = sMeterCtx;
            const width = canvas.width;
            const height = canvas.height;

            // Clear
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, width, height);

            // Convert signal power to S-units
            // S9 = -73 dBm, each S-unit = 6 dB
            // S9+10 = -63 dBm, S9+20 = -53 dBm, etc.
            let sValue = 0;
            let sText = 'S0';
            const power = demodEnhanced.signalPower;

            if (power >= -73) {
                sValue = 9 + (power + 73) / 10;  // Above S9: +10dB steps
                if (sValue > 9) {
                    sText = `S9+${((sValue - 9) * 10).toFixed(0)}`;
                } else {
                    sText = 'S9';
                }
            } else {
                sValue = Math.max(0, 9 + (power + 73) / 6);  // Below S9: 6dB per S-unit
                sText = `S${Math.floor(sValue)}`;
            }

            // Draw meter background
            const meterWidth = width - 20;
            const meterHeight = height - 40;
            const meterX = 10;
            const meterY = 10;

            // Draw S-meter scale
            ctx.fillStyle = '#222';
            ctx.fillRect(meterX, meterY, meterWidth, meterHeight);

            // Draw segments
            const segments = 15;  // 9 S-units + 6 for +60dB
            const segmentWidth = meterWidth / segments;

            for (let i = 0; i < segments; i++) {
                const x = meterX + i * segmentWidth;
                const fillAmount = Math.max(0, Math.min(1, (sValue - i)));

                let color;
                if (i < 5) color = '#0f0';  // Green for S1-S5
                else if (i < 9) color = '#ff0';  // Yellow for S6-S9
                else color = '#f00';  // Red for S9+

                ctx.fillStyle = color;
                ctx.globalAlpha = fillAmount;
                ctx.fillRect(x + 2, meterY + 2, segmentWidth - 4, meterHeight - 4);
            }
            ctx.globalAlpha = 1.0;

            // Draw border
            ctx.strokeStyle = '#0ff';
            ctx.lineWidth = 2;
            ctx.strokeRect(meterX, meterY, meterWidth, meterHeight);

            // Update text display
            document.getElementById('s_meter_value').textContent = sText;
        }

        // Tune to specific frequency
        function tuneToFrequency(freqHz) {
            demodEnhanced.tunedFreq = freqHz;
            const freqMHz = freqHz / 1e6;

            document.getElementById('demod_tuned_freq').textContent = freqMHz.toFixed(3) + ' MHz';
            document.getElementById('demod_status_text').textContent = `Tuned to ${freqMHz.toFixed(3)} MHz`;

            console.log(`Tuned to ${freqMHz.toFixed(3)} MHz`);
        }

        // Signal presets
        const signalPresets = {
            broadcast_fm: { mode: 'wfm', bandwidth: 200000, freq: 100.0e6, name: 'FM Broadcast' },
            aircraft: { mode: 'am', bandwidth: 8000, freq: 121.5e6, name: 'Aircraft (121.5 MHz)' },
            marine: { mode: 'fm', bandwidth: 12500, freq: 156.8e6, name: 'Marine VHF Ch 16' },
            weather: { mode: 'fm', bandwidth: 12500, freq: 162.4e6, name: 'NOAA Weather Radio' }
        };

        function loadPreset(presetName) {
            const preset = signalPresets[presetName];
            if (!preset) return;

            // Set mode
            quickSelectMode(preset.mode);

            // Set bandwidth
            demodEnhanced.bandwidth = preset.bandwidth;
            document.getElementById('demod_bandwidth').textContent = (preset.bandwidth / 1000).toFixed(1) + ' kHz';

            // Tune to frequency
            tuneToFrequency(preset.freq);

            // Update RF parameters (would need to send to backend in real system)
            document.getElementById('demod_status_text').textContent = `Preset loaded: ${preset.name}`;

            console.log(`Loaded preset: ${preset.name}`);
        }

        // Squelch control
        document.getElementById('squelch_slider')?.addEventListener('input', (e) => {
            const squelchLevel = parseInt(e.target.value);
            demodEnhanced.squelchLevel = squelchLevel;

            if (squelchLevel >= -20) {
                document.getElementById('squelch_value').textContent = squelchLevel + ' dBm';
            } else {
                document.getElementById('squelch_value').textContent = 'OFF';
            }

            // Apply squelch to demodulator
            demodState.squelchLevel = squelchLevel;
        });

        // AFC toggle
        function toggleAFC() {
            demodEnhanced.afcEnabled = document.getElementById('afc_enable').checked;
            document.getElementById('afc_status').textContent = demodEnhanced.afcEnabled ? 'ON' : 'OFF';
            document.getElementById('afc_status').style.color = demodEnhanced.afcEnabled ? '#0f0' : '#888';

            console.log('AFC:', demodEnhanced.afcEnabled ? 'enabled' : 'disabled');
        }

        // AFC processing (runs periodically when enabled)
        function processAFC() {
            if (!demodEnhanced.afcEnabled || !demodState.active) return;

            // Detect frequency offset from phase differences
            // This is a simplified AFC - real implementation would track carrier offset
            if (Math.abs(demodEnhanced.freqOffset) > 500) {  // Hz
                // Adjust tuned frequency
                const correction = -demodEnhanced.freqOffset * 0.1;  // Slow correction
                demodEnhanced.tunedFreq += correction;
                document.getElementById('demod_tuned_freq').textContent = (demodEnhanced.tunedFreq / 1e6).toFixed(3) + ' MHz';
            }
        }

        setInterval(processAFC, 1000);  // Run AFC every second

        // Audio recording
        let audioRecorder = null;
        let audioRecordInterval = null;

        function toggleAudioRecording() {
            if (demodEnhanced.recording) {
                stopAudioRecording();
            } else {
                startAudioRecording();
            }
        }

        function startAudioRecording() {
            if (!demodState.audioCtx) {
                showNotification('Start demodulation first', 'warning');
                return;
            }

            demodEnhanced.recording = true;
            demodEnhanced.recordedChunks = [];
            demodEnhanced.recordStartTime = Date.now();

            document.getElementById('audio_record_btn').textContent = '⏹ Stop Recording';
            document.getElementById('audio_record_btn').style.borderColor = '#f00';
            document.getElementById('audio_record_time').style.display = 'inline';

            // Update record time display
            audioRecordInterval = setInterval(() => {
                const elapsed = Math.floor((Date.now() - demodEnhanced.recordStartTime) / 1000);
                const mins = Math.floor(elapsed / 60);
                const secs = elapsed % 60;
                document.getElementById('audio_record_time').textContent =
                    `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
            }, 1000);

            console.log('Audio recording started');
        }

        function stopAudioRecording() {
            if (!demodEnhanced.recording) return;

            demodEnhanced.recording = false;
            clearInterval(audioRecordInterval);

            document.getElementById('audio_record_btn').textContent = '⏺ Record Audio';
            document.getElementById('audio_record_btn').style.borderColor = '#666';
            document.getElementById('audio_record_time').style.display = 'none';

            // Export recorded audio as WAV
            if (demodEnhanced.recordedChunks.length > 0) {
                const blob = new Blob(demodEnhanced.recordedChunks, { type: 'audio/wav' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `demod_audio_${Date.now()}.wav`;
                a.click();
                URL.revokeObjectURL(url);
                console.log('Audio exported');
            }

            demodEnhanced.recordedChunks = [];
        }

        // Export decoded data
        function exportDecodedData() {
            const messages = document.getElementById('decoded_messages');
            if (!messages || messages.children.length === 0) {
                showNotification('No decoded data to export', 'warning');
                return;
            }

            // Build CSV
            let csv = 'Timestamp,Type,Data\n';
            for (const child of messages.children) {
                const text = child.textContent;
                csv += `"${text}"\n`;
            }

            // Download
            const blob = new Blob([csv], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `decoded_data_${Date.now()}.csv`;
            a.click();
            URL.revokeObjectURL(url);

            console.log('Decoded data exported');
        }

        // Quick select mode buttons
        let selectedMode = null;

        function quickSelectMode(mode) {
            // Update selected mode
            selectedMode = mode;

            // Update button styles
            document.querySelectorAll('.demod-mode-btn').forEach(btn => {
                btn.style.background = '#1a1a1a';
                btn.style.borderColor = '#666';
                btn.style.color = '#ccc';
            });

            const selectedBtn = document.getElementById('mode_' + mode);
            if (selectedBtn) {
                selectedBtn.style.background = '#0a5';
                selectedBtn.style.borderColor = '#0f0';
                selectedBtn.style.color = '#fff';
            }

            // Update display
            document.getElementById('demod_mode_display').textContent = mode.toUpperCase();
            document.getElementById('demod_status_text').textContent = `Mode: ${mode.toUpperCase()} selected - Click START to begin`;

            // If already running, switch mode
            if (demodState.active) {
                stopDemod();
                setTimeout(() => startDemodWithMode(mode), 100);
            }
        }

        // Toggle demodulation on/off
        function toggleDemodulation() {
            if (demodState.active) {
                stopDemod();
                document.getElementById('demod_start_stop').textContent = 'START';
                document.getElementById('demod_start_stop').style.background = '#0a5';
                document.getElementById('demod_start_stop').style.borderColor = '#0f0';
                document.getElementById('demod_status_indicator').style.background = '#666';
                document.getElementById('demod_status_text').textContent = 'Stopped - Select mode and click START';
            } else {
                if (!selectedMode) {
                    showNotification('Please select a demodulation mode first', 'warning');
                    return;
                }
                startDemodWithMode(selectedMode);
                document.getElementById('demod_start_stop').textContent = 'STOP';
                document.getElementById('demod_start_stop').style.background = '#a00';
                document.getElementById('demod_start_stop').style.borderColor = '#f00';
                document.getElementById('demod_status_indicator').style.background = '#0f0';
                document.getElementById('demod_status_indicator').style.boxShadow = '0 0 8px #0f0';
            }
        }

        // Start demodulation with specific mode
        function startDemodWithMode(mode) {
            // Initialize Web Audio API
            if (!demodState.audioCtx) {
                demodState.audioCtx = new (window.AudioContext || window.webkitAudioContext)();
                demodState.gainNode = demodState.audioCtx.createGain();
                demodState.gainNode.connect(demodState.audioCtx.destination);
            }

            // Reset state
            demodState.mode = mode;
            demodState.active = true;
            demodState.lastPhase = 0;
            demodState.dcI = 0;
            demodState.dcQ = 0;
            demodState.agcGain = 1.0;
            demodState.filterState = [0, 0];
            demodState.bitBuffer = [];

            // Set volume
            const volume = document.getElementById('demod_volume_slider').value / 100.0;
            demodState.gainNode.gain.value = volume;

            // Enable protocol decoders based on mode
            if (mode === 'psk') {
                protocolDecoders.adsb.enabled = true;
                document.getElementById('adsb_status').textContent = 'Active';
                document.getElementById('adsb_status').style.color = '#0f0';
            } else if (mode === 'fsk') {
                protocolDecoders.ais.enabled = true;
                document.getElementById('ais_status').textContent = 'Active';
                document.getElementById('ais_status').style.color = '#0f0';
            }

            document.getElementById('demod_status_text').textContent = `Running ${mode.toUpperCase()} demodulation - Audio output enabled`;
            console.log(`Demodulator started: ${mode.toUpperCase()} mode`);

            // Start processing
            processDemodulation();
        }

        // Volume slider update
        document.getElementById('demod_volume_slider')?.addEventListener('input', (e) => {
            const volume = e.target.value;
            document.getElementById('demod_volume_value').textContent = volume + '%';
            if (demodState.gainNode) {
                demodState.gainNode.gain.value = volume / 100.0;
            }
        });

        // Legacy volume control support
        document.getElementById('demod_volume')?.addEventListener('input', (e) => {
            if (demodState.gainNode) {
                demodState.gainNode.gain.value = e.target.value / 100.0;
            }
        });

        // Update audio level meter and AGC display
        setInterval(() => {
            if (demodState.active) {
                const level = Math.min(100, demodState.audioLevel * 100);
                document.getElementById('audio_level_bar').style.width = level + '%';
                document.getElementById('audio_level_text').textContent = level.toFixed(0) + '%';
                document.getElementById('agc_gain_display').textContent = demodState.agcGain.toFixed(1);
            }
        }, 100);

        // Panel toggle functions
        function toggleSignalTracker() {
            const panel = document.getElementById('signal_tracker_panel');
            panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
        }

        function toggleInterference() {
            const panel = document.getElementById('interference_panel');
            panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
        }

        function toggleDecoder() {
            const panel = document.getElementById('decoder_panel');
            panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
        }

        function clearSignalTracker() {
            signalTracker.signals = [];
            updateSignalTrackerDisplay();
        }

        function clearDecodedMessages() {
            const output = document.getElementById('decoded_messages');
            if (output) {
                output.innerHTML = '<div style="text-align: center; margin-top: 20px;">No messages decoded yet</div>';
            }
        }

        // Update signal tracker display
        function updateSignalTrackerDisplay() {
            const countSpan = document.getElementById('tracker_count');
            const listDiv = document.getElementById('signal_list');
            const hoppingDiv = document.getElementById('hopping_status');

            if (!countSpan || !listDiv) return;

            countSpan.textContent = signalTracker.signals.length;

            if (signalTracker.signals.length === 0) {
                listDiv.innerHTML = '<div style="color: #888;">No signals detected</div>';
                return;
            }

            // Sort by power (strongest first)
            const sorted = [...signalTracker.signals].sort((a, b) => b.power - a.power);

            let html = '<table style="width: 100%; border-collapse: collapse;">';
            html += '<tr style="color: #0ff; border-bottom: 1px solid #333;"><th>Freq (MHz)</th><th>Pwr</th><th>BW</th><th>Dur</th></tr>';

            for (const sig of sorted) {
                const age = ((Date.now() - sig.lastSeen) / 1000).toFixed(1);
                const color = age < 0.5 ? '#0f0' : '#ff0';
                html += `<tr style="color: ${color}; border-bottom: 1px solid #222;">`;
                html += `<td>${sig.freq.toFixed(3)}</td>`;
                html += `<td>${sig.power.toFixed(1)}</td>`;
                html += `<td>${sig.bandwidth.toFixed(1)}k</td>`;
                html += `<td>${sig.duration.toFixed(1)}s</td>`;
                html += '</tr>';
            }
            html += '</table>';

            listDiv.innerHTML = html;

            // Update hopping detection display
            const hoppingResult = signalTracker.detectHopping();
            if (hoppingResult.detected) {
                hoppingDiv.innerHTML = `
                    <div style="color: #ff0; font-weight: bold;">HOPPING DETECTED</div>
                    <div style="color: #0f0;">Rate: ${hoppingResult.hopRate.toFixed(1)} hops/sec</div>
                    <div style="color: #888;">Channels: ${hoppingResult.channels.length}</div>
                `;
            } else {
                hoppingDiv.innerHTML = '<div style="color: #888;">No hopping detected</div>';
            }
        }

        // Update interference analysis display
        function updateInterferenceDisplay() {
            const harmonicsList = document.getElementById('harmonics_list');
            const imdList = document.getElementById('imd_list');
            const recommendations = document.getElementById('interference_recommendations');

            if (!harmonicsList || !imdList || !recommendations) return;

            // Update harmonics
            if (interferenceAnalyzer.harmonics.length === 0) {
                harmonicsList.innerHTML = '<div style="color: #888;">No harmonics detected</div>';
            } else {
                let html = '<table style="width: 100%; border-collapse: collapse;">';
                html += '<tr style="color: #0ff; border-bottom: 1px solid #333;"><th>Order</th><th>Freq (MHz)</th><th>Power</th></tr>';

                for (const h of interferenceAnalyzer.harmonics) {
                    html += `<tr style="color: #ff0; border-bottom: 1px solid #222;">`;
                    html += `<td>${h.order}x</td>`;
                    html += `<td>${h.freq.toFixed(3)}</td>`;
                    html += `<td>${h.power.toFixed(1)} dBm</td>`;
                    html += '</tr>';
                }
                html += '</table>';
                harmonicsList.innerHTML = html;
            }

            // Update IMD products
            if (interferenceAnalyzer.imdProducts.length === 0) {
                imdList.innerHTML = '<div style="color: #888;">No IMD products detected</div>';
            } else {
                let html = '<table style="width: 100%; border-collapse: collapse;">';
                html += '<tr style="color: #0ff; border-bottom: 1px solid #333;"><th>Order</th><th>Freq (MHz)</th><th>Power</th></tr>';

                for (const imd of interferenceAnalyzer.imdProducts) {
                    html += `<tr style="color: #f80; border-bottom: 1px solid #222;">`;
                    html += `<td>IM${imd.order}</td>`;
                    html += `<td>${imd.freq.toFixed(3)}</td>`;
                    html += `<td>${imd.power.toFixed(1)} dBm</td>`;
                    html += '</tr>';
                }
                html += '</table>';
                imdList.innerHTML = html;
            }

            // Update recommendations
            const recs = interferenceAnalyzer.recommendMitigation();
            if (recs.length === 0) {
                recommendations.innerHTML = '<div style="color: #888;">No interference detected</div>';
            } else {
                let html = '';
                for (const rec of recs) {
                    const severityColor = rec.severity === 'High' ? '#f00' : '#ff0';
                    html += `<div style="margin-bottom: 8px; padding: 5px; background: #111; border-left: 3px solid ${severityColor}; border-radius: 2px;">`;
                    html += `<div style="color: ${severityColor}; font-weight: bold;">${rec.type}</div>`;
                    html += `<div style="color: #888; margin-top: 3px;">${rec.suggestion}</div>`;
                    html += '</div>';
                }
                recommendations.innerHTML = html;
            }
        }

        // Periodic UI updates for signal tracker and interference analysis
        setInterval(() => {
            updateSignalTrackerDisplay();
            updateInterferenceDisplay();
        }, 200);

        // Start signal tracking system
        setInterval(() => {
            if (!latestFFTData) return;

            signalTracker.removeStale();

            // Detect signals above threshold
            const centerFreq = parseFloat(document.getElementById('freq')?.textContent) || 915;  // MHz
            const sampleRate = parseFloat(document.getElementById('sample_rate')?.textContent) || 40;  // MHz
            const binWidth = sampleRate / latestFFTData.length;

            for (let i = 10; i < latestFFTData.length - 10; i++) {
                const power = (latestFFTData[i] / 255.0) * 120 - 100;

                if (power > signalTracker.detectionThreshold) {
                    // Check if local maximum
                    if (latestFFTData[i] > latestFFTData[i-1] && latestFFTData[i] > latestFFTData[i+1]) {
                        const freq = centerFreq - sampleRate/2 + i * binWidth;

                        // Estimate bandwidth (-3dB points)
                        const threshold3db = latestFFTData[i] - 6.375;
                        let bwLow = i, bwHigh = i;
                        while (bwLow > 0 && latestFFTData[bwLow] > threshold3db) bwLow--;
                        while (bwHigh < latestFFTData.length - 1 && latestFFTData[bwHigh] > threshold3db) bwHigh++;
                        const bandwidth = (bwHigh - bwLow) * binWidth * 1000;  // kHz

                        signalTracker.addSignal(freq, power, bandwidth);
                    }
                }
            }

            // Check for frequency hopping
            const hoppingResult = signalTracker.detectHopping();
            if (hoppingResult.detected) {
                console.log(`Frequency hopping detected: ${hoppingResult.hopRate.toFixed(1)} hops/sec across ${hoppingResult.channels.length} channels`);
            }
        }, 100);

        // Interference analysis update
        setInterval(() => {
            if (!latestFFTData || latestFFTData.length === 0) return;

            const centerFreq = parseFloat(document.getElementById('freq')?.textContent) || 915;  // MHz
            const sampleRate = parseFloat(document.getElementById('sample_rate')?.textContent) || 40;  // MHz

            // Find strongest signal for harmonic analysis
            let peakBin = 0, peakVal = 0;
            for (let i = 0; i < latestFFTData.length; i++) {
                if (latestFFTData[i] > peakVal) {
                    peakVal = latestFFTData[i];
                    peakBin = i;
                }
            }

            const binWidth = sampleRate / latestFFTData.length;
            const fundamentalFreq = centerFreq - sampleRate/2 + peakBin * binWidth;

            // Detect harmonics
            const harmonics = interferenceAnalyzer.detectHarmonics(latestFFTData, fundamentalFreq, centerFreq, sampleRate);

            // Detect IMD if we have multiple strong signals
            const strongSignals = [];
            for (let i = 0; i < latestFFTData.length; i++) {
                const power = (latestFFTData[i] / 255.0) * 120 - 100;
                if (power > -40 && latestFFTData[i] > latestFFTData[i-1] && latestFFTData[i] > latestFFTData[i+1]) {
                    strongSignals.push(centerFreq - sampleRate/2 + i * binWidth);
                }
            }

            if (strongSignals.length >= 2) {
                interferenceAnalyzer.detectIMD(latestFFTData, strongSignals[0], strongSignals[1], centerFreq, sampleRate);
            }
        }, 500);

        // ===== PANEL TOGGLE FUNCTIONS =====

        function toggleSignalAnalysis() {
            const panel = document.getElementById('signal_analysis');
            const isVisible = panel.style.display !== 'none';
            panel.style.display = isVisible ? 'none' : 'block';
            document.getElementById('analysis_toggle').classList.toggle('active', !isVisible);
            if (!isVisible) {
                updateBookmarkList();
            }
        }

        // ===== SIGNAL ACTIVITY TIMELINE =====

        let activityTimeline = {
            history: [],
            startTime: Date.now(),
            maxHistory: 300  // 300 samples (30 seconds at 10 Hz)
        };

        function toggleActivityTimeline() {
            const panel = document.getElementById('activity_timeline');
            panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
            document.getElementById('timeline_toggle').classList.toggle('active', panel.style.display !== 'none');
        }

        function toggleRecorder() {
            const panel = document.getElementById('recorder_panel');
            const isVisible = panel.style.display !== 'none';
            panel.style.display = isVisible ? 'none' : 'block';
            document.getElementById('recorder_toggle').classList.toggle('active', !isVisible);
        }

        function toggleDoA() {
            const panel = document.getElementById('doa_panel');
            const isVisible = panel.style.display !== 'none';
            panel.style.display = isVisible ? 'none' : 'block';
            const toggleBtn = document.getElementById('doa_toggle');
            if (toggleBtn) {
                toggleBtn.classList.toggle('active', !isVisible);
            }
            if (!isVisible) {
                initDoAPolar();
            }
        }

        // ===== GPS FUNCTIONS =====

        let gpsUpdateInterval = null;

        function toggleGPS() {
            const panel = document.getElementById('gps_panel');
            const isVisible = panel.style.display !== 'none';
            panel.style.display = isVisible ? 'none' : 'block';
            document.getElementById('gps_toggle').classList.toggle('active', !isVisible);

            if (!isVisible) {
                // Start live updates when panel opens
                updateGPSPanel();
                gpsUpdateInterval = setInterval(updateGPSPanel, 1000);  // Update every second
            } else {
                // Stop updates when panel closes
                if (gpsUpdateInterval) {
                    clearInterval(gpsUpdateInterval);
                    gpsUpdateInterval = null;
                }
            }
        }

        function connectGPSD() {
            // Enable GPS auto mode to connect to gpsd
            fetch('/set_gps_mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mode: 'auto'})
            }).then(response => response.json())
              .then(data => {
                  console.log('Connecting to gpsd...');
                  showNotification('Connecting to gpsd...', 'info', 2000);
                  setTimeout(updateGPSPanel, 1000);  // Update after connection attempt
              })
              .catch(err => {
                  console.error('Failed to connect to gpsd:', err);
                  showNotification('Failed to connect to gpsd', 'error');
              });
        }

        function updateGPSPanel() {
            fetch('/gps_position')
                .then(response => response.json())
                .then(data => {
                    // Update GPS panel
                    const statusElem = document.getElementById('gps_panel_status');
                    if (data.mode === 'auto') {
                        if (data.valid) {
                            statusElem.textContent = 'GPS FIX ✓';
                            statusElem.style.color = '#0f0';
                        } else {
                            statusElem.textContent = 'NO FIX (Searching...)';
                            statusElem.style.color = '#f80';
                        }
                    } else {
                        statusElem.textContent = 'NOT CONNECTED (Click Reconnect)';
                        statusElem.style.color = '#888';
                    }

                    if (data.valid) {
                        document.getElementById('gps_panel_position').innerHTML =
                            `${data.latitude.toFixed(6)}°, ${data.longitude.toFixed(6)}°`;
                        document.getElementById('gps_panel_position').style.color = '#0f0';
                        document.getElementById('gps_panel_altitude').textContent = data.altitude_m.toFixed(1) + ' m';
                        document.getElementById('gps_panel_altitude').style.color = '#0f0';
                    } else {
                        document.getElementById('gps_panel_position').textContent = '--';
                        document.getElementById('gps_panel_position').style.color = '#888';
                        document.getElementById('gps_panel_altitude').textContent = '-- m';
                        document.getElementById('gps_panel_altitude').style.color = '#888';
                    }

                    document.getElementById('gps_panel_sats').textContent = data.satellites;
                    document.getElementById('gps_panel_sats').style.color = data.satellites > 3 ? '#0f0' : '#f80';
                    document.getElementById('gps_panel_hdop').textContent = data.hdop.toFixed(1);

                    // Update header status bar
                    updateGPSStatusBar(data);

                    // Update Stream Out modal if open
                    updateStreamOutGPS(data);
                })
                .catch(err => console.error('Failed to update GPS:', err));
        }

        function updateGPSStatusBar(data) {
            const statusElem = document.getElementById('gps_mode_bar');
            if (data.mode === 'auto') {
                if (data.valid) {
                    statusElem.textContent = `${data.satellites} sats`;
                    statusElem.style.color = '#0f0';
                } else {
                    statusElem.textContent = 'SEARCHING';
                    statusElem.style.color = '#f80';
                }
            } else {
                statusElem.textContent = 'OFF';
                statusElem.style.color = '#888';
            }
        }

        function updateStreamOutGPS(data) {
            const gpsInfo = document.getElementById('gps_position_info');
            if (!gpsInfo || gpsInfo.style.display === 'none') return;

            const statusElem = document.getElementById('streamout_gps_status');
            if (data.mode === 'auto') {
                if (data.valid) {
                    statusElem.textContent = 'GPS FIX ✓';
                    statusElem.style.color = '#0f0';
                    document.getElementById('streamout_gps_position').innerHTML =
                        `Lat: ${data.latitude.toFixed(6)}°<br>Lon: ${data.longitude.toFixed(6)}°<br>Alt: ${data.altitude_m.toFixed(1)} m`;
                    document.getElementById('streamout_gps_sats').textContent = data.satellites;
                } else {
                    statusElem.textContent = 'NO FIX (Searching...)';
                    statusElem.style.color = '#f80';
                    document.getElementById('streamout_gps_position').textContent = '--';
                    document.getElementById('streamout_gps_sats').textContent = '0';
                }
            } else {
                statusElem.textContent = 'GPS Not Connected';
                statusElem.style.color = '#888';
                document.getElementById('streamout_gps_position').innerHTML = 'Open GPS Monitor and click Reconnect';
                document.getElementById('streamout_gps_sats').textContent = '0';
            }
        }

        // Poll GPS status periodically for header
        setInterval(() => {
            if (!gpsUpdateInterval) {  // Only update if panel not open
                fetch('/gps_position')
                    .then(response => response.json())
                    .then(data => updateGPSStatusBar(data))
                    .catch(err => {});
            }
        }, 3000);

        function updateActivityTimeline(data) {
            if (!data || data.length === 0) return;

            // Calculate peak power for this frame
            let peak = 0;
            for (let i = 0; i < data.length; i++) {
                peak = Math.max(peak, data[i]);
            }

            activityTimeline.history.push({
                time: (Date.now() - activityTimeline.startTime) / 1000,
                peak: peak
            });

            // Limit history size
            if (activityTimeline.history.length > activityTimeline.maxHistory) {
                activityTimeline.history.shift();
            }

            // Update display if panel is visible
            const panel = document.getElementById('activity_timeline');
            if (panel.style.display !== 'none') {
                drawActivityTimeline();
            }

            // Update duration display
            document.getElementById('activity_duration').textContent =
                Math.floor((Date.now() - activityTimeline.startTime) / 1000);
        }

        function drawActivityTimeline() {
            const canvas = document.getElementById('timeline_canvas');
            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;

            // Clear
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, width, height);

            if (activityTimeline.history.length < 2) return;

            // Draw grid
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;
            for (let i = 0; i <= 5; i++) {
                const y = (height / 5) * i;
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(width, y);
                ctx.stroke();
            }

            // Draw activity line
            ctx.strokeStyle = '#0ff';
            ctx.lineWidth = 2;
            ctx.beginPath();

            const history = activityTimeline.history;
            for (let i = 0; i < history.length; i++) {
                const x = (i / activityTimeline.maxHistory) * width;
                const y = height - (history[i].peak / 255.0) * height;

                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.stroke();

            // Draw labels
            ctx.fillStyle = '#888';
            ctx.font = '10px monospace';
            ctx.fillText('0 dB', 5, height - 5);
            ctx.fillText('-100 dB', 5, 12);
        }

        function clearActivityHistory() {
            activityTimeline.history = [];
            activityTimeline.startTime = Date.now();
            drawActivityTimeline();
        }

        // ===== SPECTRUM MASK TESTING =====

        let spectrumMask = {
            points: [],
            enabled: false
        };

        function switchMeasTab(tabName) {
            // Hide all tabs
            document.querySelectorAll('.meas-content').forEach(tab => {
                tab.style.display = 'none';
            });
            document.querySelectorAll('.meas-tab').forEach(tab => {
                tab.classList.remove('active');
            });

            // Show selected tab
            document.getElementById('meas-content-' + tabName).style.display = 'block';
            document.getElementById('tab-' + tabName).classList.add('active');
        }

        function loadMaskTemplate() {
            const template = document.getElementById('mask_template').value;
            spectrumMask.points = [];

            const centerFreq = parseFloat(document.getElementById('freq').textContent) || 915;

            if (template === 'fcc_part15') {
                // FCC Part 15 ISM 915 MHz mask
                spectrumMask.points = [
                    { freq: centerFreq - 2.0, level: -50 },
                    { freq: centerFreq - 0.5, level: 0 },
                    { freq: centerFreq + 0.5, level: 0 },
                    { freq: centerFreq + 2.0, level: -50 }
                ];
            } else if (template === 'etsi_300220') {
                // ETSI 300 220 (868 MHz)
                spectrumMask.points = [
                    { freq: centerFreq - 0.2, level: -30 },
                    { freq: centerFreq - 0.1, level: 0 },
                    { freq: centerFreq + 0.1, level: 0 },
                    { freq: centerFreq + 0.2, level: -30 }
                ];
            }

            updateMaskTable();
        }

        function createMaskPoint() {
            const freq = prompt('Frequency (MHz):');
            const level = prompt('Level (dBm):');
            if (freq && level) {
                spectrumMask.points.push({
                    freq: parseFloat(freq),
                    level: parseFloat(level)
                });
                spectrumMask.points.sort((a, b) => a.freq - b.freq);
                updateMaskTable();
            }
        }

        function clearMask() {
            spectrumMask.points = [];
            updateMaskTable();
        }

        function updateMaskTable() {
            const tbody = document.getElementById('mask_points_body');
            if (spectrumMask.points.length === 0) {
                tbody.innerHTML = '<tr><td colspan="3" style="text-align: center; color: #888; padding: 10px;">No mask defined</td></tr>';
                return;
            }

            tbody.innerHTML = '';
            spectrumMask.points.forEach((point, idx) => {
                const row = tbody.insertRow();
                row.innerHTML = `
                    <td>${point.freq.toFixed(3)}</td>
                    <td style="text-align: right;">${point.level.toFixed(1)}</td>
                    <td style="text-align: center;">
                        <span onclick="deleteMaskPoint(${idx})" style="cursor: pointer; color: #f00;">&times;</span>
                    </td>
                `;
            });
        }

        function deleteMaskPoint(idx) {
            spectrumMask.points.splice(idx, 1);
            updateMaskTable();
        }

        function testMask() {
            if (!latestFFTData || spectrumMask.points.length === 0) return;

            const sampleRate = 40000000; // 40 MHz
            const centerFreq = parseFloat(document.getElementById('freq').textContent) * 1e6 || 915e6;

            let violations = 0;
            let maxMargin = -999;
            let minMargin = 999;

            // Test each FFT bin against mask
            for (let i = 0; i < latestFFTData.length; i++) {
                const freq = (centerFreq - sampleRate / 2 + (i * sampleRate / FFT_SIZE)) / 1e6;
                // Convert raw to dB using correct 120 dB range (-100 to +20 dB)
                const power = latestFFTData[i] / 255.0 * 120 - 100;

                // Find mask limit at this frequency (linear interpolation)
                let maskLimit = -999;
                for (let j = 0; j < spectrumMask.points.length - 1; j++) {
                    const p1 = spectrumMask.points[j];
                    const p2 = spectrumMask.points[j + 1];
                    if (freq >= p1.freq && freq <= p2.freq) {
                        const ratio = (freq - p1.freq) / (p2.freq - p1.freq);
                        maskLimit = p1.level + ratio * (p2.level - p1.level);
                        break;
                    }
                }

                if (maskLimit > -999) {
                    const margin = maskLimit - power;
                    if (margin < 0) violations++;
                    maxMargin = Math.max(maxMargin, margin);
                    minMargin = Math.min(minMargin, margin);
                }
            }

            // Update display
            document.getElementById('mask_status').textContent = violations === 0 ? 'PASS' : 'FAIL';
            document.getElementById('mask_status').style.color = violations === 0 ? '#0f0' : '#f00';
            document.getElementById('mask_violations').textContent = violations;
            document.getElementById('mask_max_margin').textContent = maxMargin.toFixed(1) + ' dB';
            document.getElementById('mask_min_margin').textContent = minMargin.toFixed(1) + ' dB';
        }

        function toggleMaskOverlay() {
            spectrumMask.enabled = !spectrumMask.enabled;
            console.log('Mask overlay:', spectrumMask.enabled ? 'enabled' : 'disabled');
        }

        // ===== SIGNAL RECORDER =====

        let recorder = {
            recording: false,
            startTime: 0,
            samples: 0,
            filename: '',
            duration: 0,
            statusInterval: null,
            stopTimeout: null
        };

        async function startRecording() {
            if (recorder.recording) return;

            const format = document.getElementById('record_format').value;
            const duration = parseInt(document.getElementById('record_duration').value);
            const trigger = document.getElementById('record_trigger').value;

            // Generate filename with timestamp and format
            const timestamp = new Date().toISOString().replace(/[:.]/g, '-').substring(0, 19);
            const freq = parseFloat(document.getElementById('freq')?.textContent) || 915;
            recorder.filename = `bladerf_${freq}MHz_${timestamp}.bin`;

            // Call backend to start recording
            try {
                const response = await fetchWithTimeout('/start_recording', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        filename: recorder.filename
                    })
                });
                const data = await response.json();

                if (data.status === 'ok') {
                    recorder.recording = true;
                    recorder.startTime = Date.now();
                    recorder.duration = duration;

                    setElementText('record_state', 'Recording...');
                    const stateElem = getElement('record_state');
                    if (stateElem) stateElem.style.color = '#f00';
                    setElementText('record_filename', recorder.filename);

                    console.log('Recording started:', recorder.filename);

                    // Poll recording status
                    recorder.statusInterval = setInterval(updateRecordingStatus, 100);

                    // Auto-stop after duration
                    recorder.stopTimeout = setTimeout(() => {
                        stopRecording();
                    }, duration * 1000);
                } else {
                    showNotification(`Failed to start recording: ${data.error || 'Unknown error'}`, 'error');
                }
            } catch (err) {
                console.error('Recording start error:', err);
                showNotification('Failed to start recording', 'error');
            }
        }

        async function stopRecording() {
            if (!recorder.recording) return;

            // Clear timers
            if (recorder.statusInterval) {
                clearInterval(recorder.statusInterval);
                recorder.statusInterval = null;
            }
            if (recorder.stopTimeout) {
                clearTimeout(recorder.stopTimeout);
                recorder.stopTimeout = null;
            }

            // Call backend to stop recording
            try {
                const response = await fetchWithTimeout('/stop_recording', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'}
                });
                const data = await response.json();
                recorder.recording = false;
                setElementText('record_state', 'Stopped');
                const stateElem = getElement('record_state');
                if (stateElem) stateElem.style.color = '#888';

                console.log('Recording stopped:', recorder.filename);

                // Add to recordings list
                const list = getElement('recordings_list');
                if (list) {
                    if (list.innerHTML.includes('No recordings')) {
                        list.innerHTML = '';
                    }
                    const entry = document.createElement('div');
                    entry.style.cssText = 'padding: 5px; border-bottom: 1px solid #222; cursor: pointer;';
                    entry.innerHTML = `
                        <div style="color: #0ff;">${recorder.filename}</div>
                        <div style="color: #888; font-size: 8px;">${recorder.samples.toLocaleString()} samples, ${((recorder.samples * 4) / 1024 / 1024).toFixed(2)} MB</div>
                    `;
                    list.insertBefore(entry, list.firstChild);
                }
            } catch (err) {
                console.error('Recording stop error:', err);
                recorder.recording = false;
            }
        }

        async function updateRecordingStatus() {
            if (!recorder.recording) return;

            try {
                const response = await fetchWithTimeout('/recording_status');
                const data = await response.json();

                if (data.recording) {
                    recorder.samples = data.samples;
                    const sizeBytes = data.samples * 4; // 2 channels * 2 bytes per sample
                    const sizeMB = sizeBytes / 1024 / 1024;

                    setElementText('record_samples', data.samples.toLocaleString());
                    setElementText('record_size', sizeMB.toFixed(2) + ' MB');
                } else {
                    // Recording stopped unexpectedly
                    if (recorder.recording) {
                        stopRecording();
                    }
                }
            } catch (err) {
                console.error('Status poll error:', err);
            }
        }

        // ===== DIRECTION FINDING (DoA) =====

        let doaCanvas = null;
        let doaCtx = null;

        function initDoAPolar() {
            doaCanvas = document.getElementById('doa_polar');
            if (!doaCanvas) return;
            doaCtx = doaCanvas.getContext('2d');
            drawDoAPolar();
        }

        function drawDoAPolar(azimuth = null) {
            if (!doaCtx) return;

            const width = doaCanvas.width;
            const height = doaCanvas.height;
            const centerX = width / 2;
            const centerY = height / 2;
            const radius = Math.min(width, height) / 2 - 20;

            // Clear
            doaCtx.fillStyle = '#0a0a0a';
            doaCtx.fillRect(0, 0, width, height);

            // Draw polar grid
            doaCtx.strokeStyle = '#333';
            doaCtx.lineWidth = 1;

            // Circles
            for (let r = radius / 4; r <= radius; r += radius / 4) {
                doaCtx.beginPath();
                doaCtx.arc(centerX, centerY, r, 0, 2 * Math.PI);
                doaCtx.stroke();
            }

            // Radial lines
            for (let angle = 0; angle < 360; angle += 30) {
                const rad = angle * Math.PI / 180;
                doaCtx.beginPath();
                doaCtx.moveTo(centerX, centerY);
                doaCtx.lineTo(centerX + radius * Math.cos(rad - Math.PI / 2),
                            centerY + radius * Math.sin(rad - Math.PI / 2));
                doaCtx.stroke();
            }

            // Labels
            doaCtx.fillStyle = '#888';
            doaCtx.font = '10px monospace';
            doaCtx.fillText('0°', centerX - 5, centerY - radius - 5);
            doaCtx.fillText('90°', centerX + radius + 5, centerY + 5);
            doaCtx.fillText('180°', centerX - 10, centerY + radius + 15);
            doaCtx.fillText('270°', centerX - radius - 25, centerY + 5);

            // Draw azimuth indicator if provided
            if (azimuth !== null) {
                const rad = azimuth * Math.PI / 180;
                doaCtx.strokeStyle = '#0ff';
                doaCtx.lineWidth = 3;
                doaCtx.beginPath();
                doaCtx.moveTo(centerX, centerY);
                doaCtx.lineTo(centerX + radius * 0.8 * Math.cos(rad - Math.PI / 2),
                            centerY + radius * 0.8 * Math.sin(rad - Math.PI / 2));
                doaCtx.stroke();

                // Draw confidence cone (±10 degrees)
                doaCtx.fillStyle = 'rgba(0, 255, 255, 0.2)';
                doaCtx.beginPath();
                doaCtx.moveTo(centerX, centerY);
                doaCtx.arc(centerX, centerY, radius * 0.8,
                          (azimuth - 10) * Math.PI / 180 - Math.PI / 2,
                          (azimuth + 10) * Math.PI / 180 - Math.PI / 2);
                doaCtx.closePath();
                doaCtx.fill();
            }
        }

        async function updateDoA() {
            try {
                // Fetch IQ data from both channels and calculate phase difference
                const response = await fetchWithTimeout('/iq_data?t=' + Date.now());
                const buffer = await response.arrayBuffer();
                const view = new DataView(buffer);
                const samplesPerChannel = 256;

                // Extract CH1 and CH2 I/Q
                let ch1_i = [], ch1_q = [], ch2_i = [], ch2_q = [];
                let offset = 0;

                for (let i = 0; i < samplesPerChannel; i++) {
                    ch1_i.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch1_q.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch2_i.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch2_q.push(view.getInt16(offset, true));
                    offset += 2;
                }

                // Calculate average phase difference
                let phaseDiffSum = 0;
                let count = 0;

                for (let i = 0; i < samplesPerChannel; i++) {
                    const phase1 = Math.atan2(ch1_q[i], ch1_i[i]);
                    const phase2 = Math.atan2(ch2_q[i], ch2_i[i]);
                    let diff = phase2 - phase1;

                    // Wrap to [-π, π]
                    while (diff > Math.PI) diff -= 2 * Math.PI;
                    while (diff < -Math.PI) diff += 2 * Math.PI;

                    phaseDiffSum += diff;
                    count++;
                }

                const avgPhaseDiff = phaseDiffSum / count;
                const phaseDiffDeg = avgPhaseDiff * 180 / Math.PI;

                // Calculate azimuth from phase difference
                const spacing = parseFloat(document.getElementById('doa_spacing').value);
                const wavelength = 1.0; // Normalized
                const azimuth = Math.asin(avgPhaseDiff / (2 * Math.PI * spacing)) * 180 / Math.PI;

                // Update display
                document.getElementById('doa_azimuth').textContent = azimuth.toFixed(1) + '°';
                document.getElementById('doa_phase').textContent = phaseDiffDeg.toFixed(1) + '°';
                document.getElementById('doa_confidence').textContent = '75%';
                document.getElementById('doa_coherence').textContent = '0.85';

                // Draw polar display
                drawDoAPolar(azimuth);
            } catch (err) {
                console.error('DoA update failed:', err);
            }
        }

        function calibrateDoA() {
            console.log('DoA calibration started');
            showNotification('Place a known signal source at 0° and click OK', 'info', 5000);
            updateDoA();
        }

        // ===== DIRECTION FINDING WORKSPACE =====

        let directionFinding = {
            spectrumCanvas: null,
            spectrumCtx: null,
            polarCanvas: null,
            polarCtx: null,
            timelineCanvas: null,
            timelineCtx: null,

            selection: {
                active: false,
                centerFreq: 0,
                bandwidth: 0,
                leftCursor: 0,  // Percentage 0-100
                rightCursor: 0,
                dragging: null,  // 'left', 'right', 'selecting', or null
                startX: 0
            },

            verticalOffset: 0,  // Vertical pan offset in dB
            running: false,
            frozen: false,  // Freeze display updates
            frozenData: null,  // Store frozen display state
            updateInterval: null,
            history: [],
            maxHistory: 200,

            calibration: {
                phaseOffset: 0,
                gainImbalance: 1.0
            }
        };

        function initDirectionSpectrum() {
            console.log('initDirectionSpectrum() called');

            directionFinding.spectrumCanvas = document.getElementById('direction_spectrum');
            if (!directionFinding.spectrumCanvas) {
                console.error('Direction spectrum canvas not found!');
                return;
            }

            console.log('Direction spectrum canvas found:', directionFinding.spectrumCanvas);

            directionFinding.spectrumCtx = directionFinding.spectrumCanvas.getContext('2d');
            console.log('Got 2D context:', directionFinding.spectrumCtx);

            const parent = directionFinding.spectrumCanvas.parentElement;
            directionFinding.spectrumCanvas.width = parent.clientWidth;
            directionFinding.spectrumCanvas.height = parent.clientHeight;

            console.log('Canvas size set to:', directionFinding.spectrumCanvas.width, 'x', directionFinding.spectrumCanvas.height);

            // Draw test pattern to verify canvas works
            directionFinding.spectrumCtx.fillStyle = '#f00';
            directionFinding.spectrumCtx.fillRect(10, 10, 100, 100);
            console.log('Drew test red rectangle at 10,10 100x100');

            setupDirectionSpectrumMouseHandlers();

            console.log('Direction spectrum initialized successfully');
        }

        function setupDirectionSpectrumMouseHandlers() {
            const canvas = directionFinding.spectrumCanvas;
            if (!canvas) {
                console.error('Direction spectrum canvas not found for mouse handlers');
                return;
            }

            console.log('Setting up direction spectrum mouse handlers');

            const leftCursor = document.getElementById('doa_cursor_left');
            const rightCursor = document.getElementById('doa_cursor_right');

            if (!leftCursor || !rightCursor) {
                console.error('Direction cursors not found');
                return;
            }

            // Remove any existing handlers by cloning
            const canvasClone = canvas.cloneNode(true);
            canvas.parentNode.replaceChild(canvasClone, canvas);
            directionFinding.spectrumCanvas = canvasClone;

            // CRITICAL: Get new context from cloned canvas!
            directionFinding.spectrumCtx = canvasClone.getContext('2d');
            console.log('Canvas cloned, new context obtained');

            // Also clone cursors to remove their handlers
            const leftClone = leftCursor.cloneNode(true);
            leftCursor.parentNode.replaceChild(leftClone, leftCursor);
            const rightClone = rightCursor.cloneNode(true);
            rightCursor.parentNode.replaceChild(rightClone, rightCursor);

            // Canvas selection handlers
            canvasClone.addEventListener('mousedown', (e) => {
                console.log('Canvas mousedown at', e.clientX, e.clientY);
                const rect = canvasClone.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const pct = (x / rect.width) * 100;

                console.log('Starting selection at', pct.toFixed(1), '%');

                directionFinding.selection.dragging = 'selecting';
                directionFinding.selection.startX = pct;
                directionFinding.selection.leftCursor = pct;
                directionFinding.selection.rightCursor = pct;
                directionFinding.selection.active = true;

                updateDoACursors();
            });

            canvasClone.addEventListener('mousemove', (e) => {
                if (directionFinding.selection.dragging !== 'selecting') return;

                const rect = canvasClone.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const pct = Math.max(0, Math.min(100, (x / rect.width) * 100));

                console.log('Selection dragging to', pct.toFixed(1), '%');

                if (pct < directionFinding.selection.startX) {
                    directionFinding.selection.leftCursor = pct;
                    directionFinding.selection.rightCursor = directionFinding.selection.startX;
                } else {
                    directionFinding.selection.leftCursor = directionFinding.selection.startX;
                    directionFinding.selection.rightCursor = pct;
                }

                updateDoACursors();
                updateDoASelectionInfo();
            });

            canvasClone.addEventListener('mouseup', (e) => {
                console.log('Canvas mouseup, dragging was:', directionFinding.selection.dragging);
                if (directionFinding.selection.dragging === 'selecting') {
                    directionFinding.selection.dragging = null;

                    console.log('Selection complete:', directionFinding.selection.leftCursor.toFixed(1), '-', directionFinding.selection.rightCursor.toFixed(1), '%');

                    updateDoACursors();
                    updateDoASelectionInfo();
                    hideDoAInstructions();
                }
            });

            // Set up cursor dragging handlers
            leftClone.addEventListener('mousedown', (e) => {
                console.log('Left cursor mousedown');
                e.stopPropagation();
                e.preventDefault();
                directionFinding.selection.dragging = 'left';
                leftClone.style.background = '#ff0';
            });

            rightClone.addEventListener('mousedown', (e) => {
                console.log('Right cursor mousedown');
                e.stopPropagation();
                e.preventDefault();
                directionFinding.selection.dragging = 'right';
                rightClone.style.background = '#ff0';
            });

            // Global handlers for cursor dragging (add only once)
            if (!window.doaGlobalHandlersSetup) {
                window.doaGlobalHandlersSetup = true;

                document.addEventListener('mousemove', (e) => {
                    if (directionFinding.selection.dragging === 'left' || directionFinding.selection.dragging === 'right') {
                        const canvas = directionFinding.spectrumCanvas;
                        if (!canvas) return;

                        const rect = canvas.getBoundingClientRect();
                        const x = e.clientX - rect.left;
                        const pct = Math.max(0, Math.min(100, (x / rect.width) * 100));

                        console.log('Cursor drag to', pct.toFixed(1), '%');

                        if (directionFinding.selection.dragging === 'left') {
                            directionFinding.selection.leftCursor = Math.min(pct, directionFinding.selection.rightCursor - 1);
                        } else {
                            directionFinding.selection.rightCursor = Math.max(pct, directionFinding.selection.leftCursor + 1);
                        }

                        updateDoACursors();
                        updateDoASelectionInfo();
                    }
                });

                document.addEventListener('mouseup', (e) => {
                    console.log('Global mouseup, dragging was:', directionFinding.selection.dragging);
                    if (directionFinding.selection.dragging === 'left' || directionFinding.selection.dragging === 'right') {
                        console.log('Cursor drag ended');
                        const leftCursor = document.getElementById('doa_cursor_left');
                        const rightCursor = document.getElementById('doa_cursor_right');
                        if (leftCursor) leftCursor.style.background = '#0ff';
                        if (rightCursor) rightCursor.style.background = '#0ff';
                        directionFinding.selection.dragging = null;
                    }
                });
            }

            // Add wheel event listener for Y-axis zoom/scroll (same as LIVE tab)
            canvasClone.addEventListener('wheel', (e) => {
                e.preventDefault();

                if (e.ctrlKey) {
                    // Ctrl+wheel = zoom Y-axis
                    const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;
                    const oldRange = spectrumMaxDb - spectrumMinDb;
                    const newRange = Math.max(20, Math.min(400, oldRange * zoomFactor));
                    const center = (spectrumMinDb + spectrumMaxDb) / 2;

                    spectrumMinDb = Math.max(-50, center - newRange / 2);
                    spectrumMaxDb = Math.min(350, center + newRange / 2);
                } else {
                    // Regular wheel = scroll Y-axis up/down
                    const scrollAmount = e.deltaY * 0.5;
                    const range = spectrumMaxDb - spectrumMinDb;

                    let newMin = spectrumMinDb + scrollAmount;
                    let newMax = spectrumMaxDb + scrollAmount;

                    // Clamp to extended range
                    const minLimit = -50;
                    const maxLimit = 350;

                    if (newMin < minLimit) {
                        newMin = minLimit;
                        newMax = minLimit + range;
                    }
                    if (newMax > maxLimit) {
                        newMax = maxLimit;
                        newMin = maxLimit - range;
                    }

                    spectrumMinDb = newMin;
                    spectrumMaxDb = newMax;
                }

                // Redraw direction spectrum with new range
                updateDirectionSpectrum();
            });

            // Double-click to reset Y-axis to default range
            canvasClone.addEventListener('dblclick', () => {
                spectrumMinDb = 75;
                spectrumMaxDb = 225;
                updateDirectionSpectrum();
            });

            console.log('Direction spectrum mouse handlers setup complete');
        }

        function hideDoAInstructions() {
            const instructions = document.getElementById('doa_instructions');
            if (instructions && directionFinding.selection.active) {
                instructions.style.display = 'none';
            }
        }

        function showDoAInstructions() {
            const instructions = document.getElementById('doa_instructions');
            if (instructions && !directionFinding.selection.active) {
                instructions.style.display = 'block';
            }
        }

        function adjustSpectrumOffset(delta) {
            directionFinding.verticalOffset += delta;
            console.log('Spectrum vertical offset:', directionFinding.verticalOffset);
        }

        function resetSpectrumOffset() {
            directionFinding.verticalOffset = 0;
            console.log('Spectrum vertical offset reset');
        }

        function updateDirectionSpectrum() {
            if (!directionFinding.spectrumCtx) {
                console.log('Direction spectrum: no context');
                return;
            }
            if (!latestFFTData || latestFFTData.length === 0) {
                console.log('Direction spectrum: no FFT data');
                return;
            }

            const canvas = directionFinding.spectrumCanvas;
            const ctx = directionFinding.spectrumCtx;
            const data = latestFFTData;
            const width = canvas.width;
            const height = canvas.height;

            // Clear with dark background
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, width, height);

            // Draw enhanced grid with dB labels
            const dbRange = spectrumMaxDb - spectrumMinDb;

            // Horizontal grid lines with dB scale
            ctx.font = '10px monospace';
            ctx.textAlign = 'left';
            for (let i = 0; i <= 10; i++) {
                const y = (height / 10) * i;
                const dbValue = Math.floor(spectrumMaxDb - (i / 10) * dbRange);

                // Emphasize major gridlines (every 20 dB)
                if (dbValue % 20 === 0) {
                    ctx.strokeStyle = 'rgba(100, 100, 100, 0.4)';
                    ctx.lineWidth = 1;
                    ctx.fillStyle = '#aaa';
                } else {
                    ctx.strokeStyle = 'rgba(60, 60, 60, 0.2)';
                    ctx.lineWidth = 0.5;
                    ctx.fillStyle = '#666';
                }

                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(width, y);
                ctx.stroke();

                // Draw dB labels on left side
                ctx.fillText(dbValue + ' dB', 5, y - 2);
            }

            // Vertical grid lines with frequency markers
            const sampleRate = 40000000;
            const centerFreq = parseFloat(document.getElementById('freq').textContent) * 1e6 || 915e6;

            ctx.textAlign = 'center';
            ctx.font = '9px monospace';
            for (let i = 0; i <= 10; i++) {
                const x = (width / 10) * i;

                // Calculate frequency at this position
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const fftIdx = zoomState.zoomStartBin + (i / 10) * zoomedBins;
                const freqOffset = (fftIdx / data.length - 0.5) * sampleRate;
                const freq = centerFreq + freqOffset;

                // Major gridlines every 2 divisions
                if (i % 2 === 0) {
                    ctx.strokeStyle = 'rgba(80, 80, 80, 0.3)';
                    ctx.lineWidth = 1;
                    ctx.fillStyle = '#888';

                    // Frequency label at bottom
                    ctx.fillText((freq / 1e6).toFixed(1) + ' MHz', x, height - 3);
                } else {
                    ctx.strokeStyle = 'rgba(50, 50, 50, 0.15)';
                    ctx.lineWidth = 0.5;
                }

                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, height);
                ctx.stroke();
            }

            // Enable smoothing
            ctx.imageSmoothingEnabled = true;
            ctx.imageSmoothingQuality = 'high';

            // Store path for gradient fill (dbRange already calculated above)
            ctx.beginPath();
            ctx.moveTo(0, height);

            // Draw spectrum line with gradient color
            const points = [];
            for (let x = 0; x < width; x++) {
                // Map canvas X to FFT bin, respecting zoom
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
                const raw = data[fftIdx];
                const magDb = rawToDb(raw);

                // Map magnitude to visible range
                const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
                const y = height - (normalizedMag * height);
                points.push({ x: x, y: y, mag: normalizedMag });

                ctx.lineTo(x, y);
            }

            // Complete the path for fill
            ctx.lineTo(width, height);
            ctx.closePath();

            // Create gradient fill (green-yellow gradient like SDR++)
            const gradient = ctx.createLinearGradient(0, 0, 0, height);
            gradient.addColorStop(0, 'rgba(255, 255, 0, 0.4)');
            gradient.addColorStop(0.3, 'rgba(0, 255, 100, 0.3)');
            gradient.addColorStop(0.7, 'rgba(0, 255, 200, 0.2)');
            gradient.addColorStop(1, 'rgba(0, 100, 255, 0.1)');

            ctx.fillStyle = gradient;
            ctx.fill();

            // Draw spectrum line with variable color
            ctx.lineJoin = 'round';
            ctx.lineCap = 'round';
            ctx.lineWidth = 1.5;

            for (let i = 0; i < points.length - 1; i++) {
                const p1 = points[i];
                const p2 = points[i + 1];

                // Color based on magnitude
                const mag = (p1.mag + p2.mag) / 2;
                if (mag > 0.8) {
                    ctx.strokeStyle = '#ffff00';  // Yellow for strong signals
                } else if (mag > 0.5) {
                    ctx.strokeStyle = '#88ff00';  // Yellow-green
                } else if (mag > 0.3) {
                    ctx.strokeStyle = '#00ff88';  // Green-cyan
                } else {
                    ctx.strokeStyle = '#00ffff';  // Cyan for weak signals
                }

                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }

            // Find and mark peaks for signal identification
            const peaks = [];
            const minPeakSpacing = width / 40;  // Minimum 40 pixels between peaks
            const noiseThreshold = 0.3;  // Peaks must be above 30% of range

            for (let i = 5; i < points.length - 5; i++) {
                const p = points[i];

                // Check if this is a local maximum
                let isPeak = true;
                for (let j = -5; j <= 5; j++) {
                    if (j !== 0 && points[i + j].mag >= p.mag) {
                        isPeak = false;
                        break;
                    }
                }

                // Check if peak is significant enough
                if (isPeak && p.mag > noiseThreshold) {
                    // Check spacing from other peaks
                    let tooClose = false;
                    for (const peak of peaks) {
                        if (Math.abs(peak.x - p.x) < minPeakSpacing) {
                            // Keep higher peak
                            if (p.mag > peak.mag) {
                                peaks.splice(peaks.indexOf(peak), 1);
                            } else {
                                tooClose = true;
                            }
                            break;
                        }
                    }

                    if (!tooClose) {
                        // Calculate frequency for this peak
                        const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                        const fftIdx = zoomState.zoomStartBin + Math.floor((p.x / width) * zoomedBins);
                        const freqOffset = (fftIdx / data.length - 0.5) * sampleRate;
                        const freq = centerFreq + freqOffset;

                        const magDb = spectrumMinDb + (p.mag * dbRange);

                        peaks.push({ x: p.x, y: p.y, mag: p.mag, freq: freq, power: magDb });
                    }
                }
            }

            // Draw peak markers and labels
            peaks.slice(0, 10).forEach((peak, idx) => {  // Limit to top 10 peaks
                // Draw peak marker (vertical line with circle)
                ctx.strokeStyle = '#ff00ff';
                ctx.fillStyle = '#ff00ff';
                ctx.lineWidth = 1;

                // Vertical line
                ctx.beginPath();
                ctx.moveTo(peak.x, peak.y);
                ctx.lineTo(peak.x, peak.y + 15);
                ctx.stroke();

                // Circle at peak
                ctx.beginPath();
                ctx.arc(peak.x, peak.y, 3, 0, 2 * Math.PI);
                ctx.fill();

                // Label with frequency and power
                ctx.fillStyle = '#ff00ff';
                ctx.font = '9px monospace';
                ctx.textAlign = 'center';
                ctx.fillText((peak.freq / 1e6).toFixed(2) + ' MHz', peak.x, peak.y - 10);
                ctx.fillText(peak.power.toFixed(1) + ' dB', peak.x, peak.y - 20);
            });

            // Draw noise floor line if available
            if (typeof latestNoiseFloor !== 'undefined' && latestNoiseFloor > 0) {
                const noiseDb = rawToDb(latestNoiseFloor);
                const noiseY = height - ((noiseDb - spectrumMinDb) / dbRange) * height;

                if (noiseY > 0 && noiseY < height) {
                    ctx.strokeStyle = 'rgba(255, 100, 100, 0.5)';
                    ctx.lineWidth = 1;
                    ctx.setLineDash([5, 5]);
                    ctx.beginPath();
                    ctx.moveTo(0, noiseY);
                    ctx.lineTo(width, noiseY);
                    ctx.stroke();
                    ctx.setLineDash([]);

                    // Label
                    ctx.fillStyle = '#ff6666';
                    ctx.font = '9px monospace';
                    ctx.textAlign = 'left';
                    ctx.fillText('Noise Floor: ' + noiseDb.toFixed(1) + ' dB', 60, noiseY - 3);
                }
            }

            // Draw selection region with enhanced visuals
            if (directionFinding.selection.active) {
                const left = (directionFinding.selection.leftCursor / 100) * width;
                const right = (directionFinding.selection.rightCursor / 100) * width;

                // Highlighted region background
                ctx.fillStyle = 'rgba(0, 255, 255, 0.15)';
                ctx.fillRect(left, 0, right - left, height);

                // Selection borders with glow
                ctx.shadowBlur = 10;
                ctx.shadowColor = '#0ff';
                ctx.strokeStyle = '#0ff';
                ctx.lineWidth = 3;
                ctx.beginPath();
                ctx.moveTo(left, 0);
                ctx.lineTo(left, height);
                ctx.moveTo(right, 0);
                ctx.lineTo(right, height);
                ctx.stroke();
                ctx.shadowBlur = 0;

                // Redraw spectrum in selection region with highlighted color
                const leftIdx = Math.floor((directionFinding.selection.leftCursor / 100) * data.length);
                const rightIdx = Math.ceil((directionFinding.selection.rightCursor / 100) * data.length);

                for (let i = leftIdx; i < rightIdx - 1; i++) {
                    const x1 = Math.floor((i / data.length) * width);
                    const x2 = Math.floor(((i + 1) / data.length) * width);

                    const raw1 = data[i];
                    const raw2 = data[i + 1];
                    const magDb1 = rawToDb(raw1);
                    const magDb2 = rawToDb(raw2);

                    const normalizedMag1 = Math.max(0, Math.min(1, (magDb1 - spectrumMinDb) / dbRange));
                    const normalizedMag2 = Math.max(0, Math.min(1, (magDb2 - spectrumMinDb) / dbRange));

                    const y1 = height - (normalizedMag1 * height);
                    const y2 = height - (normalizedMag2 * height);

                    // Brighter color for selection
                    ctx.strokeStyle = '#00ff00';  // Bright green
                    ctx.lineWidth = 2;
                    ctx.beginPath();
                    ctx.moveTo(x1, y1);
                    ctx.lineTo(x2, y2);
                    ctx.stroke();
                }
            }

            // Debug: Log successful render (throttled)
            if (!window.directionSpectrumRenderCount) window.directionSpectrumRenderCount = 0;
            if (++window.directionSpectrumRenderCount % 100 === 0) {
                console.log('Direction spectrum rendered',  window.directionSpectrumRenderCount, 'times. Canvas:', canvas.width, 'x', canvas.height, 'Data points:', data.length);
            }
        }

        function updateDoACursors() {
            const leftCursor = document.getElementById('doa_cursor_left');
            const rightCursor = document.getElementById('doa_cursor_right');

            if (!leftCursor || !rightCursor) return;

            // Show cursors if we have a selection
            if (directionFinding.selection.leftCursor !== 0 || directionFinding.selection.rightCursor !== 0) {
                leftCursor.style.display = 'block';
                rightCursor.style.display = 'block';
                leftCursor.style.left = directionFinding.selection.leftCursor + '%';
                rightCursor.style.left = directionFinding.selection.rightCursor + '%';
            } else {
                leftCursor.style.display = 'none';
                rightCursor.style.display = 'none';
            }
        }

        function updateDoASelectionInfo() {
            const sampleRate = 40000000; // 40 MHz
            const centerFreq = parseFloat(document.getElementById('freq').textContent) * 1e6 || 915e6;

            const leftFreq = centerFreq - sampleRate / 2 + (directionFinding.selection.leftCursor / 100) * sampleRate;
            const rightFreq = centerFreq - sampleRate / 2 + (directionFinding.selection.rightCursor / 100) * sampleRate;

            directionFinding.selection.centerFreq = (leftFreq + rightFreq) / 2;
            directionFinding.selection.bandwidth = rightFreq - leftFreq;

            document.getElementById('doa_sel_center').textContent = (directionFinding.selection.centerFreq / 1e6).toFixed(3) + ' MHz';
            document.getElementById('doa_sel_bw').textContent = (directionFinding.selection.bandwidth / 1e3).toFixed(1) + ' kHz';
        }

        function initDoAPolarMain() {
            directionFinding.polarCanvas = document.getElementById('doa_polar_main');
            if (!directionFinding.polarCanvas) return;

            directionFinding.polarCtx = directionFinding.polarCanvas.getContext('2d');

            const parent = directionFinding.polarCanvas.parentElement;
            directionFinding.polarCanvas.width = parent.clientWidth - 20;
            directionFinding.polarCanvas.height = parent.clientHeight - 80;

            drawDoAPolarMain();
        }

        // Bearing history for trail display
        const bearingHistory = {
            bearings: [],
            maxLength: 20,
            enabled: true
        };

        // Confidence alert state
        const confidenceAlert = {
            enabled: true,
            threshold: 50,
            consecutiveLowCount: 0,
            lastAlertTime: 0,
            alertCooldown: 10000  // 10 seconds between alerts
        };

        function addBearingToHistory(azimuth, confidence) {
            if (!bearingHistory.enabled) return;

            bearingHistory.bearings.push({
                azimuth: azimuth,
                confidence: confidence,
                timestamp: Date.now()
            });

            // Keep only last N bearings
            if (bearingHistory.bearings.length > bearingHistory.maxLength) {
                bearingHistory.bearings.shift();
            }
        }

        function drawDoAPolarMain(azimuth = null, backAzimuth = null, sources = []) {
            if (!directionFinding.polarCtx) return;

            const canvas = directionFinding.polarCanvas;
            const ctx = directionFinding.polarCtx;
            const width = canvas.width;
            const height = canvas.height;
            const centerX = width / 2;
            const centerY = height / 2;
            const radius = Math.min(width, height) / 2 - 30;

            // Clear
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, width, height);

            // Draw polar grid
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;

            // Circles with confidence zone colors
            for (let i = 1; i <= 4; i++) {
                const r = (radius / 4) * i;

                // Color code the zones based on typical confidence ranges
                // Outer ring (75-100%) = Green, Mid (50-75%) = Yellow, Inner (<50%) = Red
                if (i === 4) {
                    ctx.strokeStyle = 'rgba(0, 255, 0, 0.3)'; // Green - high confidence
                    ctx.fillStyle = 'rgba(0, 255, 0, 0.05)';
                } else if (i === 3) {
                    ctx.strokeStyle = 'rgba(255, 255, 0, 0.3)'; // Yellow - medium confidence
                    ctx.fillStyle = 'rgba(255, 255, 0, 0.05)';
                } else {
                    ctx.strokeStyle = 'rgba(255, 100, 0, 0.3)'; // Orange/Red - low confidence
                    ctx.fillStyle = 'rgba(255, 100, 0, 0.05)';
                }

                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.arc(centerX, centerY, r, 0, 2 * Math.PI);
                ctx.fill();
                ctx.stroke();
            }

            // Add confidence zone labels
            ctx.fillStyle = '#888';
            ctx.font = '9px monospace';
            ctx.textAlign = 'right';
            ctx.fillText('Low Conf', width - 5, centerY - radius * 0.25);
            ctx.fillText('Med Conf', width - 5, centerY - radius * 0.625);
            ctx.fillText('High Conf', width - 5, centerY - radius * 0.875);

            // Radial lines with angle labels
            ctx.font = '10px monospace';
            for (let angle = 0; angle < 360; angle += 30) {
                const rad = angle * Math.PI / 180;
                const x1 = centerX + radius * Math.cos(rad - Math.PI / 2);
                const y1 = centerY + radius * Math.sin(rad - Math.PI / 2);

                ctx.strokeStyle = '#333';
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(x1, y1);
                ctx.stroke();

                // Angle labels
                const labelDist = radius + 15;
                const lx = centerX + labelDist * Math.cos(rad - Math.PI / 2);
                const ly = centerY + labelDist * Math.sin(rad - Math.PI / 2);

                ctx.fillStyle = '#888';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(angle + '°', lx, ly);
            }

            // Draw bearing history trail
            if (bearingHistory.enabled && bearingHistory.bearings.length > 1) {
                for (let i = 0; i < bearingHistory.bearings.length; i++) {
                    const bearing = bearingHistory.bearings[i];
                    const rad = bearing.azimuth * Math.PI / 180;

                    // Fade older bearings
                    const alpha = (i + 1) / bearingHistory.bearings.length;
                    const opacity = alpha * 0.5; // Max 50% opacity

                    // Draw small marker
                    const markerDist = radius * 0.7;
                    const mx = centerX + markerDist * Math.cos(rad - Math.PI / 2);
                    const my = centerY + markerDist * Math.sin(rad - Math.PI / 2);

                    ctx.fillStyle = `rgba(0, 255, 255, ${opacity})`;
                    ctx.beginPath();
                    ctx.arc(mx, my, 2 + (alpha * 2), 0, 2 * Math.PI);
                    ctx.fill();

                    // Connect with lines
                    if (i > 0) {
                        const prevBearing = bearingHistory.bearings[i - 1];
                        const prevRad = prevBearing.azimuth * Math.PI / 180;
                        const prevX = centerX + markerDist * Math.cos(prevRad - Math.PI / 2);
                        const prevY = centerY + markerDist * Math.sin(prevRad - Math.PI / 2);

                        ctx.strokeStyle = `rgba(0, 255, 255, ${opacity * 0.3})`;
                        ctx.lineWidth = 1;
                        ctx.beginPath();
                        ctx.moveTo(prevX, prevY);
                        ctx.lineTo(mx, my);
                        ctx.stroke();
                    }
                }
            }

            // Draw BACK azimuth indicator (180° ambiguity) - dotted/dim
            if (backAzimuth !== null) {
                const rad = backAzimuth * Math.PI / 180;

                // Back beam (dashed, dimmer)
                ctx.strokeStyle = '#f80'; // Orange for back azimuth
                ctx.lineWidth = 3;
                ctx.setLineDash([5, 5]);
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(centerX + radius * 0.75 * Math.cos(rad - Math.PI / 2),
                          centerY + radius * 0.75 * Math.sin(rad - Math.PI / 2));
                ctx.stroke();
                ctx.setLineDash([]);

                // Back cone (dimmer)
                ctx.fillStyle = 'rgba(255, 128, 0, 0.15)';
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.arc(centerX, centerY, radius * 0.75,
                      (backAzimuth - 5) * Math.PI / 180 - Math.PI / 2,
                      (backAzimuth + 5) * Math.PI / 180 - Math.PI / 2);
                ctx.closePath();
                ctx.fill();

                // End point marker
                const endX = centerX + radius * 0.75 * Math.cos(rad - Math.PI / 2);
                const endY = centerY + radius * 0.75 * Math.sin(rad - Math.PI / 2);
                ctx.fillStyle = '#f80';
                ctx.beginPath();
                ctx.arc(endX, endY, 5, 0, 2 * Math.PI);
                ctx.fill();
            }

            // Draw PRIMARY azimuth indicator (solid, bright)
            if (azimuth !== null) {
                const rad = azimuth * Math.PI / 180;

                // Main beam
                ctx.strokeStyle = '#0ff';
                ctx.lineWidth = 4;
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(centerX + radius * 0.85 * Math.cos(rad - Math.PI / 2),
                          centerY + radius * 0.85 * Math.sin(rad - Math.PI / 2));
                ctx.stroke();

                // Confidence cone (±5 degrees)
                ctx.fillStyle = 'rgba(0, 255, 255, 0.25)';
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.arc(centerX, centerY, radius * 0.85,
                      (azimuth - 5) * Math.PI / 180 - Math.PI / 2,
                      (azimuth + 5) * Math.PI / 180 - Math.PI / 2);
                ctx.closePath();
                ctx.fill();

                // End point marker
                const endX = centerX + radius * 0.85 * Math.cos(rad - Math.PI / 2);
                const endY = centerY + radius * 0.85 * Math.sin(rad - Math.PI / 2);
                ctx.fillStyle = '#0f0';
                ctx.beginPath();
                ctx.arc(endX, endY, 6, 0, 2 * Math.PI);
                ctx.fill();
            }

            // Draw multiple sources if detected
            sources.forEach((src, idx) => {
                const rad = src.azimuth * Math.PI / 180;
                const color = ['#f00', '#ff0', '#0f0', '#0ff'][idx % 4];

                ctx.strokeStyle = color;
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(centerX + radius * 0.7 * Math.cos(rad - Math.PI / 2),
                          centerY + radius * 0.7 * Math.sin(rad - Math.PI / 2));
                ctx.stroke();

                const endX = centerX + radius * 0.7 * Math.cos(rad - Math.PI / 2);
                const endY = centerY + radius * 0.7 * Math.sin(rad - Math.PI / 2);
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(endX, endY, 4, 0, 2 * Math.PI);
                ctx.fill();
            });
        }

        function initDoATimeline() {
            directionFinding.timelineCanvas = document.getElementById('doa_timeline');
            if (!directionFinding.timelineCanvas) return;

            directionFinding.timelineCtx = directionFinding.timelineCanvas.getContext('2d');

            const parent = directionFinding.timelineCanvas.parentElement;
            directionFinding.timelineCanvas.width = parent.clientWidth - 20;
            directionFinding.timelineCanvas.height = parent.clientHeight - 60;

            drawDoATimeline();
        }

        function drawDoATimeline() {
            if (!directionFinding.timelineCtx || directionFinding.history.length === 0) return;

            const canvas = directionFinding.timelineCanvas;
            const ctx = directionFinding.timelineCtx;
            const width = canvas.width;
            const height = canvas.height;

            // Clear
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, width, height);

            // Draw grid
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;

            for (let i = 0; i <= 4; i++) {
                const y = (height / 4) * i;
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(width, y);
                ctx.stroke();
            }

            // Draw azimuth history
            ctx.strokeStyle = '#0ff';
            ctx.lineWidth = 2;
            ctx.beginPath();

            const history = directionFinding.history;
            for (let i = 0; i < history.length; i++) {
                const x = (i / directionFinding.maxHistory) * width;
                const y = height - ((history[i].azimuth + 180) / 360) * height;

                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();

            // Draw labels
            ctx.fillStyle = '#888';
            ctx.font = '10px monospace';
            ctx.fillText('+180°', 5, 12);
            ctx.fillText('0°', 5, height / 2 + 5);
            ctx.fillText('-180°', 5, height - 5);
        }

        function startDoA() {
            if (directionFinding.running) return;
            if (!directionFinding.selection.active) {
                showNotification('Please select a frequency range by clicking and dragging on the spectrum', 'info');
                return;
            }

            directionFinding.running = true;
            document.getElementById('doa_status_live').textContent = 'Running';
            document.getElementById('doa_status_live').style.color = '#0f0';

            const updateRate = parseInt(document.getElementById('doa_update_rate').value);
            directionFinding.updateInterval = setInterval(performDoAUpdate, updateRate);

            console.log('Direction finding started');
        }

        function stopDoA() {
            if (!directionFinding.running) return;

            directionFinding.running = false;
            clearInterval(directionFinding.updateInterval);

            document.getElementById('doa_status_live').textContent = 'Stopped';
            document.getElementById('doa_status_live').style.color = '#888';

            console.log('Direction finding stopped');
        }

        function toggleDoAFreeze() {
            directionFinding.frozen = !directionFinding.frozen;

            const btn = document.getElementById('doa_freeze_btn');
            if (!btn) return;

            if (directionFinding.frozen) {
                // Freeze mode - change button appearance
                btn.style.background = '#046';
                btn.style.borderColor = '#0af';
                btn.innerHTML = '▶ Resume';

                // Show notification
                showNotification('Direction finding display frozen', 'info', 2000);

                // Store current display state
                console.log('Direction finding frozen');
            } else {
                // Resume mode - restore button appearance
                btn.style.background = '#1a1a1a';
                btn.style.borderColor = '#666';
                btn.innerHTML = '❄ Freeze';

                // Clear frozen data
                directionFinding.frozenData = null;

                showNotification('Direction finding resumed', 'success', 2000);
                console.log('Direction finding resumed');
            }

            // Save freeze state
            if (typeof Settings !== 'undefined') {
                Settings.set('direction_frozen', directionFinding.frozen);
            }
        }

        function showBearingExportDialog() {
            if (!directionFinding.history || directionFinding.history.length === 0) {
                showNotification('No bearing history to export. Run direction finding first.', 'warning', 3000);
                return;
            }

            const options = `
                <div style="padding: 20px;">
                    <h3 style="margin: 0 0 15px 0; color: #0ff;">Export Bearing Data</h3>
                    <p style="margin-bottom: 15px; color: #888;">
                        ${directionFinding.history.length} bearing measurements ready to export
                    </p>
                    <div style="display: flex; flex-direction: column; gap: 10px;">
                        <button onclick="exportBearingCSV(); closeDialog();" style="padding: 12px; background: #046; border: none; color: #fff; cursor: pointer; border-radius: 4px; font-size: 14px;">
                            📊 Export as CSV (Spreadsheet)
                        </button>
                        <button onclick="exportBearingJSON(); closeDialog();" style="padding: 12px; background: #046; border: none; color: #fff; cursor: pointer; border-radius: 4px; font-size: 14px;">
                            📄 Export as JSON
                        </button>
                        <button onclick="exportBearingKML(); closeDialog();" style="padding: 12px; background: #046; border: none; color: #fff; cursor: pointer; border-radius: 4px; font-size: 14px;">
                            🌍 Export as KML (Google Earth)
                        </button>
                        <button onclick="closeDialog();" style="padding: 10px; background: #222; border: 1px solid #666; color: #ccc; cursor: pointer; border-radius: 4px; font-size: 13px; margin-top: 10px;">
                            Cancel
                        </button>
                    </div>
                </div>
            `;

            showDialog(options);
        }

        function exportBearingCSV() {
            if (!directionFinding.history || directionFinding.history.length === 0) {
                showNotification('No data to export', 'warning');
                return;
            }

            // Build CSV content
            let csv = 'Timestamp,Azimuth,Back Azimuth,Has Ambiguity,Confidence (%),Frequency (MHz),Bandwidth (Hz),SNR (dB),Phase Diff (deg),Phase Std (deg),Coherence\n';

            directionFinding.history.forEach(item => {
                const timestamp = new Date(item.timestamp).toISOString();
                const freqMHz = (item.frequency / 1e6).toFixed(6);
                const bwHz = item.bandwidth.toFixed(0);

                csv += `${timestamp},${item.azimuth.toFixed(2)},${item.backAzimuth.toFixed(2)},${item.hasAmbiguity},${item.confidence.toFixed(1)},${freqMHz},${bwHz},${item.snr.toFixed(2)},${item.phaseDiff.toFixed(2)},${item.phaseStd.toFixed(3)},${item.coherence.toFixed(3)}\n`;
            });

            // Download CSV file
            if (typeof Utils !== 'undefined' && Utils.downloadFile) {
                const filename = `bearing_data_${new Date().toISOString().split('T')[0]}.csv`;
                Utils.downloadFile(csv, filename, 'text/csv');
                showNotification(`Exported ${directionFinding.history.length} bearings to ${filename}`, 'success', 3000);
            } else {
                console.error('Utils.downloadFile not available');
            }
        }

        function exportBearingJSON() {
            if (!directionFinding.history || directionFinding.history.length === 0) {
                showNotification('No data to export', 'warning');
                return;
            }

            const data = {
                exportTime: new Date().toISOString(),
                sensorLocation: {
                    latitude: parseFloat(document.getElementById('sensor_lat')?.value || 0),
                    longitude: parseFloat(document.getElementById('sensor_lon')?.value || 0),
                    altitude: parseFloat(document.getElementById('sensor_alt')?.value || 0)
                },
                bearings: directionFinding.history.map(item => ({
                    timestamp: new Date(item.timestamp).toISOString(),
                    azimuth: item.azimuth,
                    backAzimuth: item.backAzimuth,
                    hasAmbiguity: item.hasAmbiguity,
                    confidence: item.confidence,
                    frequencyMHz: item.frequency / 1e6,
                    bandwidthHz: item.bandwidth,
                    snr: item.snr,
                    phaseDiff: item.phaseDiff,
                    phaseStd: item.phaseStd,
                    coherence: item.coherence
                }))
            };

            const json = JSON.stringify(data, null, 2);
            const filename = `bearing_data_${new Date().toISOString().split('T')[0]}.json`;

            if (typeof Utils !== 'undefined' && Utils.downloadFile) {
                Utils.downloadFile(json, filename, 'application/json');
                showNotification(`Exported ${directionFinding.history.length} bearings to ${filename}`, 'success', 3000);
            }
        }

        function exportBearingKML() {
            if (!directionFinding.history || directionFinding.history.length === 0) {
                showNotification('No data to export', 'warning');
                return;
            }

            // Get sensor location from settings
            const sensorLat = parseFloat(document.getElementById('sensor_lat')?.value || 37.7749);
            const sensorLon = parseFloat(document.getElementById('sensor_lon')?.value || -122.4194);
            const sensorAlt = parseFloat(document.getElementById('sensor_alt')?.value || 10);

            // Build KML content
            let kml = '<?xml version="1.0" encoding="UTF-8"?>\n';
            kml += '<kml xmlns="http://www.opengis.net/kml/2.2">\n';
            kml += '<Document>\n';
            kml += '  <name>BladeRF Direction Finding Data</name>\n';
            kml += '  <description>Bearing measurements exported from BladeRF Sensor</description>\n\n';

            // Add sensor location
            kml += '  <Placemark>\n';
            kml += '    <name>Sensor Location</name>\n';
            kml += '    <description>BladeRF Antenna Array</description>\n';
            kml += '    <Point>\n';
            kml += `      <coordinates>${sensorLon},${sensorLat},${sensorAlt}</coordinates>\n`;
            kml += '    </Point>\n';
            kml += '  </Placemark>\n\n';

            // Add bearing lines (draw lines from sensor in detected directions)
            directionFinding.history.forEach((item, index) => {
                const timestamp = new Date(item.timestamp).toISOString();
                const freqMHz = (item.frequency / 1e6).toFixed(3);

                // Calculate endpoint 10km away in bearing direction
                const bearingRad = item.azimuth * Math.PI / 180;
                const distance = 10; // 10 km
                const R = 6371; // Earth radius in km

                const lat1 = sensorLat * Math.PI / 180;
                const lon1 = sensorLon * Math.PI / 180;

                const lat2 = Math.asin(Math.sin(lat1) * Math.cos(distance / R) +
                                      Math.cos(lat1) * Math.sin(distance / R) * Math.cos(bearingRad));
                const lon2 = lon1 + Math.atan2(Math.sin(bearingRad) * Math.sin(distance / R) * Math.cos(lat1),
                                              Math.cos(distance / R) - Math.sin(lat1) * Math.sin(lat2));

                const endLat = lat2 * 180 / Math.PI;
                const endLon = lon2 * 180 / Math.PI;

                // Color code by confidence
                let color = 'ff00ff00'; // Green (AABBGGRR format)
                if (item.confidence < 70) color = 'ff00ffff'; // Yellow
                if (item.confidence < 40) color = 'ff0088ff'; // Orange

                kml += '  <Placemark>\n';
                kml += `    <name>Bearing ${index + 1}: ${item.azimuth.toFixed(1)}°</name>\n`;
                kml += `    <description>Time: ${timestamp}\\nFrequency: ${freqMHz} MHz\\nConfidence: ${item.confidence.toFixed(0)}%\\nSNR: ${item.snr.toFixed(1)} dB</description>\n`;
                kml += '    <Style>\n';
                kml += '      <LineStyle>\n';
                kml += `        <color>${color}</color>\n`;
                kml += '        <width>2</width>\n';
                kml += '      </LineStyle>\n';
                kml += '    </Style>\n';
                kml += '    <LineString>\n';
                kml += '      <coordinates>\n';
                kml += `        ${sensorLon},${sensorLat},${sensorAlt}\n`;
                kml += `        ${endLon},${endLat},${sensorAlt}\n`;
                kml += '      </coordinates>\n';
                kml += '    </LineString>\n';
                kml += '  </Placemark>\n\n';

                // Add back azimuth if ambiguity exists
                if (item.hasAmbiguity) {
                    const backBearingRad = item.backAzimuth * Math.PI / 180;
                    const backLat2 = Math.asin(Math.sin(lat1) * Math.cos(distance / R) +
                                              Math.cos(lat1) * Math.sin(distance / R) * Math.cos(backBearingRad));
                    const backLon2 = lon1 + Math.atan2(Math.sin(backBearingRad) * Math.sin(distance / R) * Math.cos(lat1),
                                                      Math.cos(distance / R) - Math.sin(lat1) * Math.sin(backLat2));

                    const backEndLat = backLat2 * 180 / Math.PI;
                    const backEndLon = backLon2 * 180 / Math.PI;

                    kml += '  <Placemark>\n';
                    kml += `    <name>Bearing ${index + 1} (Back): ${item.backAzimuth.toFixed(1)}°</name>\n`;
                    kml += `    <description>Ambiguous bearing (180° from primary)\\nTime: ${timestamp}\\nFrequency: ${freqMHz} MHz</description>\n`;
                    kml += '    <Style>\n';
                    kml += '      <LineStyle>\n';
                    kml += `        <color>88${color.substring(2)}</color>\n`; // Semi-transparent
                    kml += '        <width>1</width>\n';
                    kml += '      </LineStyle>\n';
                    kml += '    </Style>\n';
                    kml += '    <LineString>\n';
                    kml += '      <coordinates>\n';
                    kml += `        ${sensorLon},${sensorLat},${sensorAlt}\n`;
                    kml += `        ${backEndLon},${backEndLat},${sensorAlt}\n`;
                    kml += '      </coordinates>\n';
                    kml += '    </LineString>\n';
                    kml += '  </Placemark>\n\n';
                }
            });

            kml += '</Document>\n';
            kml += '</kml>\n';

            // Download KML file
            const filename = `bearing_data_${new Date().toISOString().split('T')[0]}.kml`;

            if (typeof Utils !== 'undefined' && Utils.downloadFile) {
                Utils.downloadFile(kml, filename, 'application/vnd.google-earth.kml+xml');
                showNotification(`Exported ${directionFinding.history.length} bearings to ${filename}`, 'success', 3000);
            }
        }

        /**
         * Validate DoA result data
         * @param {object} result - DoA result from backend
         * @returns {object} {valid: boolean, error: string}
         */
        function validateDoAResult(result) {
            if (!result) {
                return {valid: false, error: 'No DoA data received'};
            }

            // Check for required fields
            const requiredFields = ['azimuth', 'backAzimuth', 'confidence', 'snr', 'phaseDiff', 'phaseStd', 'coherence'];
            for (const field of requiredFields) {
                if (result[field] === undefined || result[field] === null) {
                    return {valid: false, error: `Missing field: ${field}`};
                }
            }

            // Check for NaN or Infinity
            if (!isFinite(result.azimuth) || !isFinite(result.backAzimuth)) {
                return {valid: false, error: 'Invalid azimuth values (NaN or Infinity)'};
            }

            if (!isFinite(result.confidence) || !isFinite(result.snr)) {
                return {valid: false, error: 'Invalid quality metrics (NaN or Infinity)'};
            }

            // Check value ranges
            if (result.azimuth < 0 || result.azimuth > 360) {
                return {valid: false, error: `Azimuth out of range: ${result.azimuth}°`};
            }

            if (result.confidence < 0 || result.confidence > 100) {
                return {valid: false, error: `Confidence out of range: ${result.confidence}%`};
            }

            if (result.phaseStd < 0) {
                return {valid: false, error: `Negative phase std dev: ${result.phaseStd}°`};
            }

            // Check for suspiciously low confidence
            if (result.confidence < 10) {
                console.warn('Very low DF confidence:', result.confidence);
            }

            return {valid: true};
        }

        async function performDoAUpdate() {
            // Skip update if frozen
            if (directionFinding.frozen) {
                return;
            }

            try {
                // Calculate bin range from selection cursors
                const fftSize = 4096;
                const startBin = Math.floor((directionFinding.selection.leftCursor / 100) * fftSize);
                const endBin = Math.floor((directionFinding.selection.rightCursor / 100) * fftSize);

                // Fetch DoA result calculated by backend (C++ phase-based interferometry)
                // Send bin range so backend only processes selected spectrum region
                const url = `/doa_result?start_bin=${startBin}&end_bin=${endBin}&t=` + Date.now();
                const response = await fetchWithTimeout(url);
                const result = await response.json();

                // Validate result data
                const validation = validateDoAResult(result);
                if (!validation.valid) {
                    console.error('DoA validation failed:', validation.error);
                    showNotification(`DF Error: ${validation.error}`, 'error');
                    return;
                }

                // Show warning if confidence is low
                if (result.confidence < 30) {
                    console.warn('Low DF confidence:', result.confidence + '%');
                    // Don't spam notifications, just log
                }

                // Update displays with backend-calculated result
                updateDoADisplays(result);

                // Add to history with full signal data
                directionFinding.history.push({
                    timestamp: Date.now(),
                    azimuth: result.azimuth,
                    backAzimuth: result.backAzimuth,
                    hasAmbiguity: result.hasAmbiguity,
                    confidence: result.confidence,
                    frequency: directionFinding.selection.centerFreq,
                    bandwidth: directionFinding.selection.bandwidth,
                    snr: result.snr,
                    phaseDiff: result.phaseDiff,
                    phaseStd: result.phaseStd,
                    coherence: result.coherence
                });

                if (directionFinding.history.length > directionFinding.maxHistory) {
                    directionFinding.history.shift();
                }

                drawDoATimeline();
            } catch (err) {
                if (err.name !== 'AbortError') {
                    console.error('DoA update failed:', err);
                    if (directionFinding.running) {
                        showNotification('DF update failed: Connection timeout', 'error');
                    }
                }
            }
        }

        function updateDoADisplays(result) {
            // Display primary azimuth with ambiguity warning
            const azimuthElem = getElement('doa_azimuth_main');
            if (azimuthElem) {
                let displayText = result.azimuth.toFixed(1) + '°';
                if (result.hasAmbiguity) {
                    displayText += ' / ' + result.backAzimuth.toFixed(1) + '°';
                }
                azimuthElem.textContent = displayText;
                azimuthElem.title = result.hasAmbiguity ?
                    `Primary: ${result.azimuth.toFixed(1)}° | Back: ${result.backAzimuth.toFixed(1)}° (180° ambiguity)` :
                    '';

                // Color code by confidence
                if (result.confidence > 70) {
                    azimuthElem.style.color = '#0f0';  // Green - good
                } else if (result.confidence > 40) {
                    azimuthElem.style.color = '#ff0';  // Yellow - fair
                } else {
                    azimuthElem.style.color = '#f80';  // Orange - poor
                }
            }

            const confidenceElem = getElement('doa_confidence_main');
            if (confidenceElem) {
                confidenceElem.textContent = result.confidence.toFixed(0) + '%';
                // Color code confidence
                if (result.confidence > 70) {
                    confidenceElem.style.color = '#0f0';
                } else if (result.confidence > 40) {
                    confidenceElem.style.color = '#ff0';
                } else {
                    confidenceElem.style.color = '#f80';
                }
            }

            setElementText('doa_snr', result.snr.toFixed(1) + ' dB');
            setElementText('doa_phase_diff', result.phaseDiff.toFixed(1) + '°');
            setElementText('doa_phase_unwrap', result.phaseDiff.toFixed(1) + '°');

            const phaseStdElem = getElement('doa_phase_std');
            if (phaseStdElem) {
                phaseStdElem.textContent = result.phaseStd.toFixed(2) + '°';
                // Color code by phase stability (lower is better)
                if (result.phaseStd < 5) {
                    phaseStdElem.style.color = '#0f0';  // Stable
                } else if (result.phaseStd < 15) {
                    phaseStdElem.style.color = '#ff0';  // Moderate
                } else {
                    phaseStdElem.style.color = '#f80';  // Unstable
                }
            }

            setElementText('doa_coherence_mag', result.coherence.toFixed(3));

            // Update quality with ambiguity warning and detailed tooltip
            let quality = result.confidence > 70 ? 'Good' : result.confidence > 40 ? 'Fair' : 'Poor';
            if (result.hasAmbiguity) quality += ' ⚠';

            const qualityElem = getElement('doa_quality');
            if (qualityElem) {
                qualityElem.textContent = quality;
                qualityElem.title = result.hasAmbiguity ?
                    `2-channel DF has 180° ambiguity\nConfidence: ${result.confidence.toFixed(0)}%\nPhase Std: ${result.phaseStd.toFixed(2)}°\nSNR: ${result.snr.toFixed(1)} dB` :
                    `Confidence: ${result.confidence.toFixed(0)}%\nPhase Std: ${result.phaseStd.toFixed(2)}°\nSNR: ${result.snr.toFixed(1)} dB`;
            }

            setElementText('doa_samples', String(directionFinding.history.length));

            // Add to bearing history
            if (typeof addBearingToHistory === 'function') {
                addBearingToHistory(result.azimuth, result.confidence);
            }

            // Check confidence and trigger alert if needed
            if (confidenceAlert.enabled) {
                const now = Date.now();

                if (result.confidence < confidenceAlert.threshold) {
                    confidenceAlert.consecutiveLowCount++;

                    // Trigger alert after 3 consecutive low readings and cooldown has passed
                    if (confidenceAlert.consecutiveLowCount >= 3 &&
                        (now - confidenceAlert.lastAlertTime) > confidenceAlert.alertCooldown) {

                        showNotification(
                            `⚠ Low DF Confidence: ${result.confidence.toFixed(0)}% (threshold: ${confidenceAlert.threshold}%)`,
                            'warning',
                            5000
                        );

                        console.warn(`Low DF confidence alert: ${result.confidence}% (SNR: ${result.snr} dB, Phase Std: ${result.phaseStd}°)`);

                        confidenceAlert.lastAlertTime = now;
                        confidenceAlert.consecutiveLowCount = 0; // Reset after alert
                    }
                } else {
                    // Reset counter when confidence is good
                    confidenceAlert.consecutiveLowCount = 0;
                }
            }

            // Update polar display with both bearings
            drawDoAPolarMain(result.azimuth, result.backAzimuth, result.sources);
        }

        async function calibrateDoAMain() {
            // Use confirm for now since we need user acknowledgment before starting
            // TODO: Could create a custom modal in future
            if (!confirm('Place a known signal source directly at 0° (north) and click OK to calibrate')) {
                return;
            }

            showNotification('Performing DF calibration...', 'info', 3000);

            try {
                // Perform calibration measurement
                const response = await fetchWithTimeout('/iq_data?t=' + Date.now());
                const buffer = await response.arrayBuffer();

                // Validate buffer size
                const expectedSize = IQ_SAMPLES * 4 * 2; // 4 channels (I,Q × 2) × 2 bytes per sample
                if (buffer.byteLength < expectedSize) {
                    throw new Error(`Insufficient IQ data: got ${buffer.byteLength} bytes, expected ${expectedSize}`);
                }

                const view = new DataView(buffer);
                const samplesPerChannel = IQ_SAMPLES;

                let ch1_i = [], ch1_q = [], ch2_i = [], ch2_q = [];
                let offset = 0;

                // Parse IQ samples
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch1_i.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch1_q.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch2_i.push(view.getInt16(offset, true));
                    offset += 2;
                }
                for (let i = 0; i < samplesPerChannel; i++) {
                    ch2_q.push(view.getInt16(offset, true));
                    offset += 2;
                }

                // Validate signal strength
                const ch1Power = Math.sqrt(ch1_i.reduce((sum, val, idx) => sum + val*val + ch1_q[idx]*ch1_q[idx], 0) / samplesPerChannel);
                const ch2Power = Math.sqrt(ch2_i.reduce((sum, val, idx) => sum + val*val + ch2_q[idx]*ch2_q[idx], 0) / samplesPerChannel);

                if (ch1Power < 100 || ch2Power < 100) {
                    showNotification('Warning: Weak signal detected. Calibration may be inaccurate', 'warning', 5000);
                    console.warn('Low signal power during calibration:', {ch1: ch1Power, ch2: ch2Power});
                }

                // Calculate phase offset
                let phaseDiffSum = 0;
                const phaseValues = [];

                for (let i = 0; i < samplesPerChannel; i++) {
                    const phase1 = Math.atan2(ch1_q[i], ch1_i[i]);
                    const phase2 = Math.atan2(ch2_q[i], ch2_i[i]);
                    let diff = phase2 - phase1;

                    // Wrap to [-π, π]
                    while (diff > Math.PI) diff -= 2 * Math.PI;
                    while (diff < -Math.PI) diff += 2 * Math.PI;

                    phaseDiffSum += diff;
                    phaseValues.push(diff);
                }

                const avgPhaseOffset = phaseDiffSum / samplesPerChannel;

                // Calculate standard deviation to assess calibration quality
                const variance = phaseValues.reduce((sum, val) => sum + Math.pow(val - avgPhaseOffset, 2), 0) / samplesPerChannel;
                const stdDev = Math.sqrt(variance);
                const stdDevDeg = stdDev * 180 / Math.PI;

                // Store calibration
                directionFinding.calibration.phaseOffset = avgPhaseOffset;

                // Report results
                const offsetDeg = (avgPhaseOffset * 180 / Math.PI).toFixed(2);
                console.log('DoA calibration complete:', {
                    phaseOffset: avgPhaseOffset,
                    phaseOffsetDeg: offsetDeg,
                    stdDevDeg: stdDevDeg.toFixed(2),
                    ch1Power: ch1Power.toFixed(1),
                    ch2Power: ch2Power.toFixed(1)
                });

                if (stdDevDeg > 10) {
                    showNotification(`Calibration complete but unstable (σ=${stdDevDeg.toFixed(1)}°). Phase offset: ${offsetDeg}°`, 'warning', 5000);
                } else {
                    showNotification(`Calibration successful! Phase offset: ${offsetDeg}° (σ=${stdDevDeg.toFixed(1)}°)`, 'success', 5000);
                }

            } catch (err) {
                console.error('Calibration failed:', err);
                showNotification(`Calibration failed: ${err.message}`, 'error');
            }
        }

        function clearDoAHistory() {
            directionFinding.history = [];
            drawDoATimeline();
        }

        function exportDoAData() {
            if (directionFinding.history.length === 0) {
                showNotification('No data to export', 'warning');
                return;
            }

            let csv = 'Timestamp,Azimuth (deg),Confidence (%)\n';
            directionFinding.history.forEach(entry => {
                csv += entry.timestamp + ',' + entry.azimuth.toFixed(2) + ',' + entry.confidence.toFixed(1) + '\n';
            });

            const blob = new Blob([csv], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'doa_data_' + Date.now() + '.csv';
            a.click();
            URL.revokeObjectURL(url);
        }

        // ===== STREAM OUT FUNCTIONALITY =====

        // MGRS to Lat/Lon converter
        function mgrsToLatLon(mgrs) {
            // Remove spaces
            mgrs = mgrs.replace(/\s+/g, '').toUpperCase();

            // Parse MGRS: GridZone(2-3) + 100kmSquare(2) + Easting(variable) + Northing(variable)
            const match = mgrs.match(/^(\d{1,2})([C-X])([A-Z]{2})(\d+)(\d+)$/);
            if (!match) {
                throw new Error('Invalid MGRS format');
            }

            const zone = parseInt(match[1]);
            const latBand = match[2];
            const square100km = match[3];
            const easting = match[4];
            const northing = match[5];

            // Ensure easting and northing have same length
            if (easting.length !== northing.length) {
                throw new Error('Easting and Northing must have same precision');
            }

            const precision = easting.length;
            const factor = Math.pow(10, 5 - precision);

            // Convert to meters within 100km square
            let eastingM = parseInt(easting) * factor;
            let northingM = parseInt(northing) * factor;

            // Add 100km square offsets (simplified approximation)
            const square1 = square100km.charCodeAt(0) - 65; // A=0, B=1, etc
            const square2 = square100km.charCodeAt(1) - 65;

            // Simplified 100km square calculation (this is an approximation)
            eastingM += (square1 % 8) * 100000;
            northingM += Math.floor(square2 / 2) * 100000;

            // Convert UTM to Lat/Lon (simplified)
            const k0 = 0.9996;
            const a = 6378137; // WGS84 semi-major axis
            const e = 0.081819190842622; // WGS84 eccentricity
            const e1sq = e * e / (1 - e * e);

            const x = eastingM - 500000;
            const y = northingM;

            const M = y / k0;
            const mu = M / (a * (1 - e * e / 4 - 3 * e * e * e * e / 64));

            const phi1 = mu + (3 * e1sq / 2 - 27 * e1sq * e1sq * e1sq / 32) * Math.sin(2 * mu);
            const lat = phi1 * 180 / Math.PI;

            const centralMeridian = (zone - 1) * 6 - 180 + 3;
            const lon = centralMeridian + (x / (a * k0)) * 180 / Math.PI;

            return { lat, lon };
        }

        // Protobuf encoder for DF bearing message
        function encodeProtobuf(doaData) {
            // Simple protobuf-like binary format
            // Field 1: timestamp (uint64)
            // Field 2: sensor_uid (string)
            // Field 3: lat (double)
            // Field 4: lon (double)
            // Field 5: alt (float)
            // Field 6: azimuth (float)
            // Field 7: confidence (float)

            const encoder = new DataView(new ArrayBuffer(1024));
            let offset = 0;

            // Helper functions
            function writeVarint(value) {
                while (value >= 128) {
                    encoder.setUint8(offset++, (value & 127) | 128);
                    value >>>= 7;
                }
                encoder.setUint8(offset++, value & 127);
            }

            function writeFieldHeader(fieldNum, wireType) {
                writeVarint((fieldNum << 3) | wireType);
            }

            function writeString(fieldNum, str) {
                writeFieldHeader(fieldNum, 2); // Wire type 2 = length-delimited
                const bytes = new TextEncoder().encode(str);
                writeVarint(bytes.length);
                for (let i = 0; i < bytes.length; i++) {
                    encoder.setUint8(offset++, bytes[i]);
                }
            }

            function writeDouble(fieldNum, value) {
                writeFieldHeader(fieldNum, 1); // Wire type 1 = 64-bit
                encoder.setFloat64(offset, value, true); // little-endian
                offset += 8;
            }

            function writeFloat(fieldNum, value) {
                writeFieldHeader(fieldNum, 5); // Wire type 5 = 32-bit
                encoder.setFloat32(offset, value, true); // little-endian
                offset += 4;
            }

            function writeUint64(fieldNum, value) {
                writeFieldHeader(fieldNum, 0); // Wire type 0 = varint
                // Simplified for values < 2^32
                writeVarint(Math.floor(value));
            }

            // Encode fields
            writeUint64(1, doaData.timestamp); // timestamp
            writeString(2, streamOutConfig.sensorUid); // sensor_uid
            writeDouble(3, streamOutConfig.position.lat); // lat
            writeDouble(4, streamOutConfig.position.lon); // lon
            writeFloat(5, streamOutConfig.position.alt); // alt
            writeFloat(6, doaData.azimuth); // azimuth
            writeFloat(7, doaData.confidence); // confidence

            // Return as Uint8Array
            return new Uint8Array(encoder.buffer, 0, offset);
        }

        let streamOutConfig = {
            active: false,
            interval: null,
            platformInterval: null,
            protocol: 'http',
            endpoint: '',
            port: 8089,
            format: 'cot',
            rate: 500,
            sensorUid: 'DF-SENSOR-001',
            platformType: 'ugv',  // ugv, uav-fixed, uav-rotary, usv, ground-station
            lobRange: 10000,  // LoB range in meters (default 10km)
            position: {
                mode: 'static',
                lat: 37.7749,
                lon: -122.4194,
                alt: 10,
                mgrs: ''
            },
            stats: {
                sent: 0,
                errors: 0
            }
        };

        function openStreamOutConfig() {
            const modal = document.getElementById('streamout_modal');
            if (modal) {
                modal.style.display = 'flex';

                // Load current config into form
                document.getElementById('stream_endpoint').value = streamOutConfig.endpoint || '127.0.0.1';
                document.getElementById('stream_port').value = streamOutConfig.port;
                document.getElementById('stream_format').value = streamOutConfig.format;
                document.getElementById('stream_rate').value = streamOutConfig.rate;
                document.getElementById('sensor_uid').value = streamOutConfig.sensorUid;
                document.getElementById('platform_type').value = streamOutConfig.platformType;
                document.getElementById('lob_range').value = streamOutConfig.lobRange;
                document.getElementById('sensor_lat').value = streamOutConfig.position.lat;
                document.getElementById('sensor_lon').value = streamOutConfig.position.lon;
                document.getElementById('sensor_alt').value = streamOutConfig.position.alt;
            }
        }

        function closeStreamOutConfig() {
            const modal = document.getElementById('streamout_modal');
            if (modal) {
                modal.style.display = 'none';
            }
        }

        function togglePositionMode() {
            const mode = document.querySelector('input[name="position_mode"]:checked').value;
            const staticInputs = document.getElementById('static_position_inputs');
            const gpsInfo = document.getElementById('gps_position_info');

            if (mode === 'static') {
                staticInputs.style.display = 'block';
                gpsInfo.style.display = 'none';
            } else if (mode === 'gps') {
                staticInputs.style.display = 'none';
                gpsInfo.style.display = 'block';
                // Trigger GPS status update
                fetch('/gps_position')
                    .then(response => response.json())
                    .then(data => updateStreamOutGPS(data))
                    .catch(err => console.error('Failed to get GPS:', err));
            }
        }

        function toggleCoordFormat() {
            const format = document.getElementById('coord_format').value;
            const latlonInputs = document.getElementById('latlon_inputs');
            const mgrsInputs = document.getElementById('mgrs_inputs');

            if (format === 'latlon') {
                latlonInputs.style.display = 'block';
                mgrsInputs.style.display = 'none';
            } else {
                latlonInputs.style.display = 'none';
                mgrsInputs.style.display = 'block';
            }
        }

        function toggleStreamOut() {
            if (streamOutConfig.active) {
                stopStreamOut();
            } else {
                startStreamOut();
            }
        }

        function startStreamOut() {
            // Read configuration from form
            streamOutConfig.protocol = document.getElementById('stream_protocol').value;
            streamOutConfig.endpoint = document.getElementById('stream_endpoint').value;
            streamOutConfig.port = parseInt(document.getElementById('stream_port').value);
            streamOutConfig.format = document.getElementById('stream_format').value;
            streamOutConfig.rate = parseInt(document.getElementById('stream_rate').value);
            streamOutConfig.sensorUid = document.getElementById('sensor_uid').value;
            streamOutConfig.platformType = document.getElementById('platform_type').value;
            streamOutConfig.lobRange = parseInt(document.getElementById('lob_range').value);

            const posMode = document.querySelector('input[name="position_mode"]:checked').value;
            streamOutConfig.position.mode = posMode;

            if (posMode === 'static') {
                const coordFormat = document.getElementById('coord_format').value;
                if (coordFormat === 'latlon') {
                    streamOutConfig.position.lat = parseFloat(document.getElementById('sensor_lat').value);
                    streamOutConfig.position.lon = parseFloat(document.getElementById('sensor_lon').value);
                    streamOutConfig.position.alt = parseFloat(document.getElementById('sensor_alt').value);
                } else {
                    // Convert MGRS to Lat/Lon
                    streamOutConfig.position.mgrs = document.getElementById('sensor_mgrs').value;
                    streamOutConfig.position.alt = parseFloat(document.getElementById('sensor_alt_mgrs').value);

                    try {
                        const converted = mgrsToLatLon(streamOutConfig.position.mgrs);
                        streamOutConfig.position.lat = converted.lat;
                        streamOutConfig.position.lon = converted.lon;
                        console.log('MGRS converted:', streamOutConfig.position.mgrs, '→', converted.lat.toFixed(6), converted.lon.toFixed(6));
                    } catch (e) {
                        showNotification(`Invalid MGRS coordinate: ${e.message}`, 'error');
                        return;
                    }
                }
            }

            // Validate
            if (!streamOutConfig.endpoint || streamOutConfig.port < 1 || streamOutConfig.port > 65535) {
                showNotification('Please enter valid endpoint and port', 'warning');
                return;
            }

            // Start streaming
            streamOutConfig.active = true;
            streamOutConfig.stats.sent = 0;
            streamOutConfig.stats.errors = 0;

            // Send platform position immediately and every 10 seconds
            sendPlatformPosition();
            streamOutConfig.platformInterval = setInterval(sendPlatformPosition, 10000);

            // Send LoB updates at configured rate
            streamOutConfig.interval = setInterval(() => {
                if (directionFinding.history.length > 0) {
                    const latest = directionFinding.history[directionFinding.history.length - 1];
                    sendDoAStream(latest);
                }
            }, streamOutConfig.rate);

            // Update UI
            document.getElementById('stream_toggle_btn').textContent = 'Stop Streaming';
            document.getElementById('stream_toggle_btn').style.background = '#3a0a0a';
            document.getElementById('stream_toggle_btn').style.borderColor = '#f00';
            document.getElementById('stream_toggle_btn').style.color = '#f00';
            document.getElementById('doa_stream_status').textContent = 'Active';
            document.getElementById('doa_stream_status').style.color = '#0f0';

            console.log('Stream Out started:', streamOutConfig.endpoint + ':' + streamOutConfig.port, 'format:', streamOutConfig.format);
            closeStreamOutConfig();
        }

        function stopStreamOut() {
            if (streamOutConfig.interval) {
                clearInterval(streamOutConfig.interval);
                streamOutConfig.interval = null;
            }
            if (streamOutConfig.platformInterval) {
                clearInterval(streamOutConfig.platformInterval);
                streamOutConfig.platformInterval = null;
            }

            streamOutConfig.active = false;

            // Update UI
            document.getElementById('stream_toggle_btn').textContent = 'Start Streaming';
            document.getElementById('stream_toggle_btn').style.background = '#0a3a0a';
            document.getElementById('stream_toggle_btn').style.borderColor = '#0ff';
            document.getElementById('stream_toggle_btn').style.color = '#0ff';
            document.getElementById('doa_stream_status').textContent = 'Off';
            document.getElementById('doa_stream_status').style.color = '#888';

            console.log('Stream Out stopped. Sent:', streamOutConfig.stats.sent, 'Errors:', streamOutConfig.stats.errors);
        }

        function sendTargetRegistration() {
            const now = new Date();
            const time = now.toISOString();
            const stale = new Date(now.getTime() + 300000).toISOString(); // 5 min stale

            const lat = streamOutConfig.position.lat;
            const lon = streamOutConfig.position.lon;
            const alt = Math.round(streamOutConfig.position.alt);
            const uid = streamOutConfig.sensorUid;

            // Target registration message
            const registrationCot = `<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<event version='2.0' uid='${uid}-target-reg' type='b-t-f' time='${time}' start='${time}' stale='${stale}' how='m-g'>
    <point lat='${lat}' lon='${lon}' hae='${alt}' ce='10' le='10'/>
    <detail>
        <contact callsign='RF-Target'/>
        <__registrationStatus deviceType='bladeRF xA9' signalType='RF' unitId='${uid}'/>
        <remarks>RF Target Registration for LoB</remarks>
    </detail>
</event>`;

            // Send registration
            if (streamOutConfig.protocol === 'udp') {
                (async () => {
                    try {
                        await fetchWithTimeout('/stream_udp_relay', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({
                                endpoint: streamOutConfig.endpoint,
                                port: streamOutConfig.port,
                                data: registrationCot,
                                format: 'cot'
                            })
                        });
                        console.log('Target registration sent');
                    } catch (err) {
                        console.error('Target registration send failed:', err.message);
                    }
                })();
            }
        }

        function sendPlatformPosition() {
            const now = new Date();
            const time = now.toISOString();
            const stale = new Date(now.getTime() + 300000).toISOString(); // 5 min stale

            const lat = streamOutConfig.position.lat;
            const lon = streamOutConfig.position.lon;
            const alt = Math.round(streamOutConfig.position.alt);
            const uid = streamOutConfig.sensorUid;

            // Platform type to CoT type mapping
            const platformTypes = {
                'ugv': 'a-f-G-E-V',
                'uav-fixed': 'a-f-A-M-F-Q',
                'uav-rotary': 'a-f-A-M-H-Q',
                'usv': 'a-f-S-E-V',
                'ground-station': 'a-f-G-E-S'
            };
            const cotType = platformTypes[streamOutConfig.platformType] || 'a-f-G-E-S';

            const platformNames = {
                'ugv': 'UGV',
                'uav-fixed': 'UAV (Fixed)',
                'uav-rotary': 'UAV (Rotary)',
                'usv': 'USV',
                'ground-station': 'Ground Station'
            };
            const platformName = platformNames[streamOutConfig.platformType] || 'Sensor';

            // Platform Position CoT Event
            const platformCot = `<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<event version='2.0' uid='${uid}' type='${cotType}' time='${time}' start='${time}' stale='${stale}' how='m-g'>
    <point lat='${lat}' lon='${lon}' hae='${alt}' ce='10' le='10'/>
    <detail>
        <contact callsign='${platformName}-DF'/>
        <remarks>Direction Finding Sensor Platform</remarks>
        <track speed='0' course='0'/>
        <precisionlocation altsrc='GPS' geopointsrc='GPS'/>
    </detail>
</event>`;

            // Send platform position
            if (streamOutConfig.protocol === 'udp') {
                (async () => {
                    try {
                        await fetchWithTimeout('/stream_udp_relay', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({
                                endpoint: streamOutConfig.endpoint,
                                port: streamOutConfig.port,
                                data: platformCot,
                                format: 'cot'
                            })
                        });
                    } catch (err) {
                        console.error('Platform position send failed:', err.message);
                    }
                })();
            }
        }

        function sendDoAStream(doaData) {
            let payload;
            let contentType;
            let isBinary = false;

            // Format the data based on selected format
            switch (streamOutConfig.format) {
                case 'cot':
                    payload = formatCoT(doaData);
                    contentType = 'application/xml';
                    break;
                case 'json':
                    payload = formatJSON(doaData);
                    contentType = 'application/json';
                    break;
                case 'proto':
                    payload = formatProtobuf(doaData);
                    contentType = 'application/x-protobuf';
                    isBinary = true;
                    break;
                case 'csv':
                    payload = formatCSV(doaData);
                    contentType = 'text/csv';
                    break;
                case 'nmea':
                    payload = formatNMEA(doaData);
                    contentType = 'text/plain';
                    break;
                default:
                    console.error('Unknown format:', streamOutConfig.format);
                    return;
            }

            // Send based on protocol
            if (streamOutConfig.protocol === 'udp') {
                // Send via UDP relay endpoint on server
                (async () => {
                    try {
                        const response = await fetchWithTimeout('/stream_udp_relay', {
                            method: 'POST',
                            headers: {
                                'Content-Type': 'application/json'
                            },
                            body: JSON.stringify({
                                endpoint: streamOutConfig.endpoint,
                                port: streamOutConfig.port,
                                data: isBinary ? Array.from(payload) : payload,
                                format: streamOutConfig.format
                            })
                        });

                        streamOutConfig.stats.sent++;
                        if (streamOutConfig.stats.sent % 10 === 0) {
                            console.log('Streamed', streamOutConfig.stats.sent, 'DF reports via UDP');
                        }
                    } catch (err) {
                        streamOutConfig.stats.errors++;
                        if (streamOutConfig.stats.errors % 10 === 0) {
                            console.error('UDP relay failed:', err.message, '(', streamOutConfig.stats.errors, 'errors)');
                        }
                    }
                })();
            } else {
                // Send via HTTP
                const url = 'http://' + streamOutConfig.endpoint + ':' + streamOutConfig.port;

                (async () => {
                    try {
                        await fetchWithTimeout(url, {
                            method: 'POST',
                            headers: {
                                'Content-Type': contentType
                            },
                            body: payload
                        });

                        streamOutConfig.stats.sent++;
                        if (streamOutConfig.stats.sent % 10 === 0) {
                            console.log('Streamed', streamOutConfig.stats.sent, 'DF reports via HTTP');
                        }
                    } catch (err) {
                        streamOutConfig.stats.errors++;
                        if (streamOutConfig.stats.errors % 10 === 0) {
                            console.error('HTTP stream failed:', err.message, '(', streamOutConfig.stats.errors, 'errors)');
                        }
                    }
                })();
            }
        }

        function formatCoT(doaData) {
            const now = new Date();
            const time = now.toISOString();
            const stale = new Date(now.getTime() + 86400000).toISOString(); // 24 hour stale
            const deviceTime = now.toISOString();

            const lat = streamOutConfig.position.lat;
            const lon = streamOutConfig.position.lon;
            const alt = Math.round(streamOutConfig.position.alt);
            const uid = streamOutConfig.sensorUid;
            const bearing = doaData.azimuth.toFixed(1);
            const confidence = doaData.confidence ? doaData.confidence.toFixed(0) : '50';

            // Get actual frequency from DoA data (must be integer for TAKX-RF)
            const frequency = Math.round(doaData.frequency || 100000000);
            const bandwidth = Math.round(doaData.bandwidth || 1000000);

            // Calculate RSSI from SNR if available
            const snr = doaData.snr || 10;
            const rssi = (-100 + snr).toFixed(1);

            // Calculate error estimates
            const phaseStd = doaData.phaseStd || 5.0;
            const coherence = doaData.coherence || 0.8;

            // Error radius based on phase standard deviation (degrees to meters at range)
            const lobRange = streamOutConfig.lobRange;
            const errorAngleDeg = phaseStd;
            const errorRadius = Math.round(lobRange * Math.tan(errorAngleDeg * Math.PI / 180));

            // Platform type to CoT type mapping
            const platformTypes = {
                'ugv': 'a-f-G-E-V',           // Friendly Ground Equipment Vehicle
                'uav-fixed': 'a-f-A-M-F-Q',   // Friendly Air Military Fixed-wing UAV
                'uav-rotary': 'a-f-A-M-H-Q',  // Friendly Air Military Rotary-wing UAV
                'usv': 'a-f-S-E-V',           // Friendly Surface Equipment Vehicle
                'ground-station': 'a-f-G-E-S' // Friendly Ground Equipment Sensor
            };
            const cotType = platformTypes[streamOutConfig.platformType] || 'a-f-G-E-S';

            // Platform display name
            const platformNames = {
                'ugv': 'UGV',
                'uav-fixed': 'UAV (Fixed)',
                'uav-rotary': 'UAV (Rotary)',
                'usv': 'USV',
                'ground-station': 'Ground Station'
            };
            const platformName = platformNames[streamOutConfig.platformType] || 'Sensor';

            // TAKX-RF LoB format - just send LoB, platform icon from deviceType
            return `<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
<event version='2.0' uid='${uid}-lob-${Date.now()}' type='b-d' time='${time}' start='${time}' stale='${stale}' how='m-p'>
    <point lat='0.0' lon='0.0' hae='9999999.0' ce='9999999.0' le='9999999.0'/>
    <detail>
        <__lob deviceType='bladeRF xA9' rssi='${rssi}' confidence='${confidence}' unitId='${uid}' azimuth='${bearing}' family='com.nuand.bladerf' deviceTime='${deviceTime}' elevationAngle='0' frequency='${frequency}' range='${lobRange}' strokeColor='-16711936' strokeWeight='2.0' signalType='RF'>
            <startLocation ce='0' le='0' lon='${lon}' hae='${alt}' lat='${lat}'/>
            <__rf rssi='${rssi}' uplinkRssi='${rssi}' comments='2-ch DF SNR:${snr.toFixed(1)}dB Coh:${(coherence*100).toFixed(0)}% BW:${(bandwidth/1e3).toFixed(1)}kHz' downlinkRssi='-999.0' tag='${platformName} - ${(frequency/1e6).toFixed(3)} MHz' errorRadius='${errorRadius}' frequency='${frequency}'/>
        </__lob>
        <contact callsign='${platformName}-DF-${bearing}°'/>
    </detail>
</event>`;
        }

        function formatJSON(doaData) {
            return JSON.stringify({
                timestamp: doaData.timestamp,
                sensor: {
                    uid: streamOutConfig.sensorUid,
                    position: {
                        lat: streamOutConfig.position.lat,
                        lon: streamOutConfig.position.lon,
                        alt: streamOutConfig.position.alt
                    }
                },
                bearing: {
                    azimuth: doaData.azimuth,
                    confidence: doaData.confidence,
                    unit: 'degrees_true'
                }
            });
        }

        function formatCSV(doaData) {
            return `${doaData.timestamp},${streamOutConfig.sensorUid},${streamOutConfig.position.lat},${streamOutConfig.position.lon},${streamOutConfig.position.alt},${doaData.azimuth.toFixed(2)},${doaData.confidence.toFixed(1)}`;
        }

        function formatNMEA(doaData) {
            // Custom NMEA-like sentence for DF bearing
            // $DFBR,timestamp,azimuth,confidence,lat,lon*checksum
            const lat = Math.abs(streamOutConfig.position.lat).toFixed(6);
            const latDir = streamOutConfig.position.lat >= 0 ? 'N' : 'S';
            const lon = Math.abs(streamOutConfig.position.lon).toFixed(6);
            const lonDir = streamOutConfig.position.lon >= 0 ? 'E' : 'W';
            const azimuth = doaData.azimuth.toFixed(1);
            const confidence = doaData.confidence.toFixed(1);

            const sentence = `DFBR,${Date.now()},${azimuth},${confidence},${lat},${latDir},${lon},${lonDir}`;
            const checksum = sentence.split('').reduce((acc, char) => acc ^ char.charCodeAt(0), 0).toString(16).toUpperCase().padStart(2, '0');

            return `$${sentence}*${checksum}`;
        }

        function formatProtobuf(doaData) {
            return encodeProtobuf(doaData);
        }


        // ===== ENHANCED EXPORT FUNCTIONS =====

        function exportSigMF() {
            if (!latestFFTData || latestFFTData.length === 0) {
                showNotification('No spectrum data available to export', 'warning');
                return;
            }

            const timestamp = new Date().toISOString();
            const freq = parseFloat(document.getElementById('freqInput').value) * 1e6;
            const sr = parseFloat(document.getElementById('srInput').value) * 1e6;

            // Create SigMF metadata
            const sigmf = {
                global: {
                    "core:datatype": "rf32_le",
                    "core:sample_rate": sr,
                    "core:version": "1.0.0",
                    "core:description": "bladeRF spectrum capture",
                    "core:author": "bladeRF Sensor System",
                    "core:recorder": "bladeRF xA9",
                    "core:hw": "bladeRF xA9"
                },
                captures: [{
                    "core:sample_start": 0,
                    "core:frequency": freq,
                    "core:datetime": timestamp
                }],
                annotations: []
            };

            // Add bookmarks as annotations
            signalAnalysis.bookmarks.forEach(b => {
                sigmf.annotations.push({
                    "core:sample_start": 0,
                    "core:sample_count": FFT_SIZE,
                    "core:freq_lower_edge": b.freq * 1e6 - 10000,
                    "core:freq_upper_edge": b.freq * 1e6 + 10000,
                    "core:label": b.name
                });
            });

            const sigmfStr = JSON.stringify(sigmf, null, 2);
            const blob = new Blob([sigmfStr], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = `spectrum_${Date.now()}.sigmf-meta`;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);

            console.log('Exported SigMF metadata');
        }

        // Hook up new toggle buttons
        // Note: analysis_toggle, demod_toggle, timeline_toggle removed in favor of workspace tabs
        // document.getElementById('analysis_toggle').addEventListener('click', toggleSignalAnalysis);
        // document.getElementById('demod_toggle').addEventListener('click', toggleDemod);
        document.getElementById('cursor_toggle').addEventListener('click', toggleCursors);
        // document.getElementById('timeline_toggle').addEventListener('click', toggleActivityTimeline);

        // ========================================================================
        // INITIALIZATION
        // ========================================================================

        // Initialize bookmarks on load
        updateBookmarkList();

        // Update activity timeline on each frame
        setInterval(() => {
            if (latestFFTData) {
                updateActivityTimeline(latestFFTData);
            }
        }, 100);

        console.log('Advanced signal analysis features loaded');
        console.log('Activity timeline ready');

        // ========================================================================
        // ADVANCED ANALYTICS - Burst, Hopping, Interference Detection
        // ========================================================================

        let advancedAnalysisState = {
            burstHistory: [],
            freqHoppingHistory: [],
            interferenceDetected: [],
            lastAnalysisTime: 0
        };

        function runAdvancedAnalysis() {
            if (!latestFFTData || latestFFTData.length === 0) {
                showNotification('No spectrum data available', 'warning');
                return;
            }

            const data = latestFFTData;
            const threshold = parseFloat(document.getElementById('burst_threshold').value);
            const minDuration = parseFloat(document.getElementById('burst_min_duration').value);

            // Burst Detection
            detectBursts(data, threshold, minDuration);

            // Frequency Hopping Detection
            if (document.getElementById('fh_detect_enable').checked) {
                detectFrequencyHopping(data);
            }

            // Interference Analysis
            analyzeInterference(data);

            console.log('Advanced analysis complete');
        }

        function detectBursts(data, thresholdDb, minDurationMs) {
            const avgPower = data.reduce((a, b) => a + b, 0) / data.length;
            const thresholdRaw = avgPower + ((thresholdDb + 100) / 120 * 255);

            let bursts = [];
            let inBurst = false;
            let burstStart = 0;
            let burstSum = 0;

            for (let i = 0; i < data.length; i++) {
                if (data[i] > thresholdRaw && !inBurst) {
                    inBurst = true;
                    burstStart = i;
                    burstSum = 0;
                } else if (data[i] < thresholdRaw && inBurst) {
                    inBurst = false;
                    const burstLength = i - burstStart;
                    if (burstLength > 0) {
                        bursts.push({
                            start: burstStart,
                            length: burstLength,
                            avgPower: burstSum / burstLength
                        });
                    }
                }

                if (inBurst) {
                    burstSum += data[i];
                }
            }

            // Update UI
            document.getElementById('burst_count').textContent = bursts.length;

            if (bursts.length > 0) {
                const avgDuration = bursts.reduce((a, b) => a + b.length, 0) / bursts.length;
                const binTime = 1000.0 / (40e6 / 4096); // Approximate
                document.getElementById('burst_avg_duration').textContent = (avgDuration * binTime).toFixed(2) + ' ms';

                // Calculate burst rate (assuming data is from recent time period)
                const burstRate = bursts.length / 0.1; // Assume 100ms observation window
                document.getElementById('burst_rate').textContent = burstRate.toFixed(1) + ' Hz';
            }

            advancedAnalysisState.burstHistory.push({
                timestamp: Date.now(),
                count: bursts.length
            });

            // Keep only last 100 entries
            if (advancedAnalysisState.burstHistory.length > 100) {
                advancedAnalysisState.burstHistory.shift();
            }
        }

        function detectFrequencyHopping(data) {
            const historyLength = parseInt(document.getElementById('fh_history').value);

            // Find peak frequency
            let peakIdx = 0;
            let peakVal = 0;
            for (let i = 0; i < data.length; i++) {
                if (data[i] > peakVal) {
                    peakVal = data[i];
                    peakIdx = i;
                }
            }

            const centerFreq = parseFloat(document.getElementById('freqInput').value) * 1e6;
            const sampleRate = parseFloat(document.getElementById('srInput').value) * 1e6;
            const peakFreq = centerFreq - (sampleRate / 2) + (peakIdx * sampleRate / FFT_SIZE);

            advancedAnalysisState.freqHoppingHistory.push({
                timestamp: Date.now(),
                frequency: peakFreq,
                power: peakVal
            });

            // Keep only specified history
            while (advancedAnalysisState.freqHoppingHistory.length > historyLength) {
                advancedAnalysisState.freqHoppingHistory.shift();
            }

            // Analyze hopping pattern
            if (advancedAnalysisState.freqHoppingHistory.length > 10) {
                const history = advancedAnalysisState.freqHoppingHistory;
                const recentHistory = history.slice(-20);

                // Count unique channels
                const channels = new Set(recentHistory.map(h => Math.round(h.frequency / 1e6)));
                document.getElementById('fh_channels').textContent = channels.size;

                // Calculate hop rate
                let hops = 0;
                for (let i = 1; i < recentHistory.length; i++) {
                    const freqDiff = Math.abs(recentHistory[i].frequency - recentHistory[i-1].frequency);
                    if (freqDiff > 100e3) { // 100 kHz threshold
                        hops++;
                    }
                }

                const timeSpan = (recentHistory[recentHistory.length-1].timestamp - recentHistory[0].timestamp) / 1000;
                const hopRate = hops / timeSpan;
                document.getElementById('fh_hop_rate').textContent = hopRate.toFixed(1);

                // Determine pattern type
                if (channels.size > 15) {
                    document.getElementById('fh_pattern').textContent = 'Pseudo-Random';
                } else if (channels.size > 5) {
                    document.getElementById('fh_pattern').textContent = 'Sequential';
                } else if (channels.size > 1) {
                    document.getElementById('fh_pattern').textContent = 'Alternating';
                } else {
                    document.getElementById('fh_pattern').textContent = 'Fixed Freq';
                }
            }
        }

        function analyzeInterference(data) {
            const avgPower = data.reduce((a, b) => a + b, 0) / data.length;
            const threshold = avgPower * 1.5;

            // Detect narrowband interference (single bin spikes)
            let narrowbandCount = 0;
            for (let i = 1; i < data.length - 1; i++) {
                if (data[i] > threshold && data[i] > data[i-1] * 2 && data[i] > data[i+1] * 2) {
                    narrowbandCount++;
                }
            }

            document.getElementById('interf_narrowband').textContent = narrowbandCount;

            // Detect wideband interference (wide elevated floor)
            let wideCount = 0;
            let windowSize = 50;
            for (let i = 0; i < data.length - windowSize; i++) {
                let windowAvg = 0;
                for (let j = 0; j < windowSize; j++) {
                    windowAvg += data[i + j];
                }
                windowAvg /= windowSize;

                if (windowAvg > threshold) {
                    wideCount++;
                }
            }

            const widePercent = (wideCount / (data.length - windowSize) * 100).toFixed(1);
            document.getElementById('interf_wideband').textContent = widePercent + '%';

            // Calculate duty cycle (% time signal is present)
            let activeCount = 0;
            for (let i = 0; i < data.length; i++) {
                if (data[i] > avgPower * 1.2) {
                    activeCount++;
                }
            }

            const dutyCycle = (activeCount / data.length * 100).toFixed(1);
            document.getElementById('interf_duty_cycle').textContent = dutyCycle + '%';
        }

        // ========================================================================
        // LIVE WORKSPACE DISPLAYS
        // ========================================================================

        let liveStatsInterval = null;

        function updateLiveStats() {
            if (!latestFFTData || latestFFTData.length === 0) return;

            const data = latestFFTData;

            // Find peak
            let peakIdx = 0;
            let peakVal = 0;
            let sum = 0;
            for (let i = 0; i < data.length; i++) {
                if (data[i] > peakVal) {
                    peakVal = data[i];
                    peakIdx = i;
                }
                sum += data[i];
            }

            const avgVal = sum / data.length;

            // Noise floor (10th percentile)
            const sorted = Array.from(data).sort((a, b) => a - b);
            const noiseFloor = sorted[Math.floor(sorted.length * 0.1)];

            // Convert to dBm (assuming 50 ohm, 0dBFS = 0dBm for simplicity)
            const peakDbm = rawToDb(peakVal);
            const avgDbm = rawToDb(avgVal);
            const floorDbm = rawToDb(noiseFloor);

            document.getElementById('live_peak_power').textContent = peakDbm.toFixed(1) + ' dBm';
            document.getElementById('live_avg_power').textContent = avgDbm.toFixed(1) + ' dBm';
            document.getElementById('live_noise_floor').textContent = floorDbm.toFixed(1) + ' dBm';

            // Update signal strength meter (based on peak power)
            // Map -120 dBm to 0%, 0 dBm to 100%
            const signalPercent = Math.max(0, Math.min(100, ((peakDbm + 120) / 120) * 100));
            const signalBar = document.getElementById('signal_strength_bar');
            const signalText = document.getElementById('signal_strength_text');
            if (signalBar) {
                signalBar.style.width = signalPercent.toFixed(1) + '%';
            }
            if (signalText) {
                signalText.textContent = peakDbm.toFixed(0) + ' dBm';
            }

            // FPS is already updated elsewhere
            const fpsElement = document.getElementById('fps');
            if (fpsElement) {
                document.getElementById('live_fps_display').textContent = fpsElement.textContent;
            }
        }
        // Start update intervals when page loads
        setTimeout(() => {
            // Try initial canvas setup
            initSMeter();

            // Update live stats in LIVE workspace
            setInterval(updateLiveStats, 500);

            // Update direction finding spectrum
            let directionUpdateCount = 0;
            let directionIntervalStarted = false;
            setInterval(() => {
                const workspace = document.getElementById('workspace-direction');

                // Log first time interval runs
                if (!directionIntervalStarted) {
                    directionIntervalStarted = true;
                    console.log('Direction spectrum update interval started');
                    console.log('workspace-direction element:', workspace);
                    console.log('workspace active?', workspace ? workspace.classList.contains('active') : 'N/A');
                    console.log('latestFFTData available?', latestFFTData ? latestFFTData.length + ' points' : 'null');
                }

                if (workspace && workspace.classList.contains('active')) {
                    directionUpdateCount++;

                    // Log every 10 updates instead of 100 for better visibility
                    if (directionUpdateCount % 10 === 0) {
                        console.log('Direction spectrum update #' + directionUpdateCount + ', latestFFTData:', latestFFTData ? latestFFTData.length + ' points' : 'null');
                    }

                    if (!directionFinding.spectrumCtx) {
                        console.log('No spectrum context, initializing...');
                        initDirectionSpectrum();
                    }
                    updateDirectionSpectrum();
                }
            }, 100);

            console.log('Live workspace displays initialized');
        }, 1000);

        // Reinitialize canvases on window resize
        window.addEventListener('resize', () => {
            if (document.getElementById('workspace-direction').classList.contains('active')) {
                initDirectionSpectrum();
                initDoAPolarMain();
                initDoATimeline();
            }
        });

    </script>

    <!-- Load modular JavaScript components -->
    <!-- Core utilities (load first) -->
    <script src="/js/utils.js"></script>
    <script src="/js/settings.js"></script>

    <!-- Colormap utilities -->
    <script src="/js/utils/colormap.js"></script>

    <!-- Display modules -->
    <script src="/js/displays/waterfall.js"></script>
    <script src="/js/displays/spectrum.js"></script>
    <script src="/js/displays/iq_constellation_enhanced.js"></script>
    <script src="/js/displays/cross_correlation_enhanced.js"></script>
    <script src="/js/displays/eye_diagram.js"></script>
    <script src="/js/displays/waveform_display.js"></script>

    <!-- Feature modules -->
    <script src="/js/signal_filters.js"></script>
    <script src="/js/rf_measurements.js"></script>
    <script src="/js/marker_system.js"></script>
    <script src="/js/vsa_analysis.js"></script>
    <script src="/js/statistics.js"></script>

    <!-- Attach event listeners AFTER modules load -->
    <script>
        // Initialize display modules
        (function() {
            console.log('Initializing display modules...');

            // Get canvas elements
            const wfCanvas = document.getElementById('waterfall');
            const wfCanvas2 = document.getElementById('waterfall2');
            const specCanvas = document.getElementById('spectrum');
            const specCanvas2 = document.getElementById('spectrum2');
            const iqCanvas = document.getElementById('iq_canvas');
            const xcorrCanvas = document.getElementById('xcorr_canvas');
            const iqFullscreenCanvas = document.getElementById('iq_fullscreen');
            const xcorrFullscreenCanvas = document.getElementById('xcorr_fullscreen');

            // Initialize modules with their canvases
            if (typeof WaterfallDisplay !== 'undefined') {
                WaterfallDisplay.init(wfCanvas, wfCanvas2, window.zoomState || {
                    zoomStartBin: 0,
                    zoomEndBin: 4095,
                    centerFreq: 915000000,
                    fullBandwidth: 40000000
                });
                console.log('✓ WaterfallDisplay initialized');
            }

            if (typeof SpectrumDisplay !== 'undefined') {
                SpectrumDisplay.init(specCanvas, specCanvas2, window.zoomState);
                console.log('✓ SpectrumDisplay initialized');
            }

            if (typeof IQConstellationEnhanced !== 'undefined') {
                IQConstellationEnhanced.init(iqCanvas, iqFullscreenCanvas);
                console.log('✓ IQConstellationEnhanced initialized');
            }

            if (typeof CrossCorrelationEnhanced !== 'undefined') {
                CrossCorrelationEnhanced.init(xcorrCanvas, xcorrFullscreenCanvas);
                console.log('✓ CrossCorrelationEnhanced initialized');
            }

            // Note: Eye Diagram and Waveform Display will be initialized when IQ workspace is activated
            // (deferred initialization to ensure canvas dimensions are computed)

            console.log('Display modules initialization complete');
        })();

        // Draggable panel system
        function makePanelDraggable(panel) {
            const header = panel.querySelector('.panel-header');
            if (!header) return;

            let isDragging = false;
            let currentX, currentY, initialX, initialY;
            let xOffset = 0, yOffset = 0;

            header.addEventListener('mousedown', dragStart);
            document.addEventListener('mousemove', drag);
            document.addEventListener('mouseup', dragEnd);

            function dragStart(e) {
                if (e.target.classList.contains('panel-close')) return;

                initialX = e.clientX - xOffset;
                initialY = e.clientY - yOffset;

                if (e.target === header || e.target.classList.contains('panel-title')) {
                    isDragging = true;
                    panel.classList.add('active');
                }
            }

            function drag(e) {
                if (isDragging) {
                    e.preventDefault();
                    currentX = e.clientX - initialX;
                    currentY = e.clientY - initialY;

                    xOffset = currentX;
                    yOffset = currentY;

                    setTranslate(currentX, currentY, panel);
                }
            }

            function dragEnd(e) {
                initialX = currentX;
                initialY = currentY;
                isDragging = false;
                panel.classList.remove('active');
            }

            function setTranslate(xPos, yPos, el) {
                el.style.transform = `translate3d(${xPos}px, ${yPos}px, 0)`;
            }
        }

        // Event listeners for modular panel toggles (now that functions are defined)
        // Note: marker_toggle, vsa_toggle, stats_toggle removed in favor of workspace tabs
        // document.getElementById('marker_toggle').addEventListener('click', toggleMarkerPanel);
        // document.getElementById('vsa_toggle').addEventListener('click', toggleVSA);
        // document.getElementById('stats_toggle').addEventListener('click', toggleStatsPanel);

        // Event listeners for panel close buttons
        document.getElementById('marker_panel_close').addEventListener('click', toggleMarkerPanel);
        document.getElementById('vsa_panel_close').addEventListener('click', toggleVSA);
        document.getElementById('stats_panel_close').addEventListener('click', toggleStatsPanel);

        console.log('Modular panel event listeners attached');

        // Make ALL panels draggable (including those from external modules and in workspaces)
        document.querySelectorAll('.draggable-panel').forEach(panel => {
            makePanelDraggable(panel);
        });

        // Make panel headers in workspace slots also draggable (to drag OUT of slots)
        document.querySelectorAll('.workspace-panel-slot .panel-header').forEach(header => {
            header.style.cursor = 'move';
        });

        console.log('Draggable panels initialized');

        // ========================================================================
        // WORKSPACE TAB SWITCHING
        // ========================================================================

        function switchWorkspace(tabName) {
            // Save workspace selection
            if (typeof Settings !== 'undefined') {
                Settings.set('last_workspace', tabName);
            }

            // Hide all workspace content
            document.querySelectorAll('.workspace-content').forEach(content => {
                content.classList.remove('active');
            });

            // Remove active class from all tabs
            document.querySelectorAll('.workspace-tab').forEach(tab => {
                tab.classList.remove('active');
            });

            // Show selected workspace
            const workspace = document.getElementById('workspace-' + tabName);
            if (workspace) {
                workspace.classList.add('active');
            }

            // Activate selected tab
            const tab = document.querySelector(`.workspace-tab[data-tab="${tabName}"]`);
            if (tab) {
                tab.classList.add('active');
            }

            // Move panels to their workspace slots
            if (tabName === 'live') {
                // Live workspace - hide all panels
                document.getElementById('signal_analysis').style.display = 'none';
                document.getElementById('marker_panel').style.display = 'none';
                document.getElementById('stats_panel').style.display = 'none';
                document.getElementById('activity_timeline').style.display = 'none';
                document.getElementById('iq_constellation').style.display = 'none';
                document.getElementById('xcorr_display').style.display = 'none';
                document.getElementById('vsa_panel').style.display = 'none';
                document.getElementById('demod_panel').style.display = 'none';

                // Disable IQ and cross-correlation updates
                showIQ = false;
                showXCorr = false;
            } else if (tabName === 'measurements') {
                // Move measurement panels into their slots
                const slot1 = document.getElementById('measurements-slot-1');
                const slot2 = document.getElementById('measurements-slot-2');
                const slot3 = document.getElementById('measurements-slot-3');
                const slot4 = document.getElementById('measurements-slot-4');

                const rfPanel = document.getElementById('signal_analysis');
                const markerPanel = document.getElementById('marker_panel');
                const statsPanel = document.getElementById('stats_panel');
                const timelinePanel = document.getElementById('activity_timeline');

                // Clear slots first
                slot1.innerHTML = '';
                slot2.innerHTML = '';
                slot3.innerHTML = '';
                slot4.innerHTML = '';

                // Move panels into slots and show them
                slot1.appendChild(rfPanel);
                slot2.appendChild(markerPanel);
                slot3.appendChild(statsPanel);
                slot4.appendChild(timelinePanel);

                rfPanel.style.display = 'flex';
                rfPanel.style.flexDirection = 'column';
                rfPanel.style.position = 'relative';
                rfPanel.style.width = '100%';
                rfPanel.style.height = '100%';
                rfPanel.style.top = 'auto';
                rfPanel.style.left = 'auto';
                rfPanel.style.right = 'auto';
                rfPanel.style.bottom = 'auto';

                markerPanel.style.display = 'flex';
                markerPanel.style.flexDirection = 'column';
                markerPanel.style.position = 'relative';
                markerPanel.style.width = '100%';
                markerPanel.style.height = '100%';
                markerPanel.style.top = 'auto';
                markerPanel.style.left = 'auto';
                markerPanel.style.right = 'auto';
                markerPanel.style.bottom = 'auto';

                statsPanel.style.display = 'flex';
                statsPanel.style.flexDirection = 'column';
                statsPanel.style.position = 'relative';
                statsPanel.style.width = '100%';
                statsPanel.style.height = '100%';
                statsPanel.style.top = 'auto';
                statsPanel.style.left = 'auto';
                statsPanel.style.right = 'auto';
                statsPanel.style.bottom = 'auto';

                timelinePanel.style.display = 'flex';
                timelinePanel.style.flexDirection = 'column';
                timelinePanel.style.position = 'relative';
                timelinePanel.style.width = '100%';
                timelinePanel.style.height = '100%';
                timelinePanel.style.top = 'auto';
                timelinePanel.style.left = 'auto';
                timelinePanel.style.right = 'auto';
                timelinePanel.style.bottom = 'auto';

                // Hide demod panels
                document.getElementById('iq_constellation').style.display = 'none';
                document.getElementById('xcorr_display').style.display = 'none';
                document.getElementById('vsa_panel').style.display = 'none';
                document.getElementById('demod_panel').style.display = 'none';
                document.getElementById('stats_panel').style.display = 'none';

                // Disable IQ and cross-correlation updates
                showIQ = false;
                showXCorr = false;
            } else if (tabName === 'demod') {
                // Move demod panels into their slots (Phase 3 enhanced layout)
                const slotTracker = document.getElementById('demod-slot-tracker');
                const slotInterference = document.getElementById('demod-slot-interference');
                const slotDecoder = document.getElementById('demod-slot-decoder');

                const trackerPanel = document.getElementById('signal_tracker_panel');
                const interferencePanel = document.getElementById('interference_panel');
                const decoderPanel = document.getElementById('decoder_panel');

                // Clear slots first
                slotTracker.innerHTML = '';
                slotInterference.innerHTML = '';
                slotDecoder.innerHTML = '';

                // Move panels into slots: Signal Tracker | Interference | Decoder
                slotTracker.appendChild(trackerPanel);
                slotInterference.appendChild(interferencePanel);
                slotDecoder.appendChild(decoderPanel);

                // Style tracker panel (spans 2 rows)
                trackerPanel.style.display = 'flex';
                trackerPanel.style.flexDirection = 'column';
                trackerPanel.style.position = 'relative';
                trackerPanel.style.width = '100%';
                trackerPanel.style.height = '100%';
                trackerPanel.style.top = 'auto';
                trackerPanel.style.left = 'auto';
                trackerPanel.style.right = 'auto';
                trackerPanel.style.bottom = 'auto';

                // Style interference panel
                interferencePanel.style.display = 'flex';
                interferencePanel.style.flexDirection = 'column';
                interferencePanel.style.position = 'relative';
                interferencePanel.style.width = '100%';
                interferencePanel.style.height = '100%';
                interferencePanel.style.top = 'auto';
                interferencePanel.style.left = 'auto';
                interferencePanel.style.right = 'auto';
                interferencePanel.style.bottom = 'auto';

                // Style decoder panel
                decoderPanel.style.display = 'flex';
                decoderPanel.style.flexDirection = 'column';
                decoderPanel.style.position = 'relative';
                decoderPanel.style.width = '100%';
                decoderPanel.style.height = '100%';
                decoderPanel.style.top = 'auto';
                decoderPanel.style.left = 'auto';
                decoderPanel.style.right = 'auto';
                decoderPanel.style.bottom = 'auto';

                // Enable IQ and cross-correlation updates for demod workspace
                showIQ = true;
                showXCorr = true;

                // Hide measurement panels and duplicate IQ panel
                document.getElementById('signal_analysis').style.display = 'none';
                document.getElementById('marker_panel').style.display = 'none';
                document.getElementById('activity_timeline').style.display = 'none';
                document.getElementById('iq_constellation').style.display = 'none';
            } else if (tabName === 'direction') {
                // Direction finding workspace - no panels to move, just hide all
                document.getElementById('signal_analysis').style.display = 'none';
                document.getElementById('marker_panel').style.display = 'none';
                document.getElementById('stats_panel').style.display = 'none';
                document.getElementById('activity_timeline').style.display = 'none';
                document.getElementById('iq_constellation').style.display = 'none';
                document.getElementById('xcorr_display').style.display = 'none';
                document.getElementById('vsa_panel').style.display = 'none';
                document.getElementById('demod_panel').style.display = 'none';

                // Disable IQ and cross-correlation panel updates
                showIQ = false;
                showXCorr = false;

                // Initialize direction finding canvases
                setTimeout(() => {
                    initDirectionSpectrum();
                    initDoAPolarMain();
                    initDoATimeline();
                    showDoAInstructions();
                }, 100);
            } else if (tabName === 'iq') {
                // IQ Constellation fullscreen workspace
                document.getElementById('signal_analysis').style.display = 'none';
                document.getElementById('marker_panel').style.display = 'none';
                document.getElementById('stats_panel').style.display = 'none';
                document.getElementById('activity_timeline').style.display = 'none';
                document.getElementById('iq_constellation').style.display = 'none';
                document.getElementById('xcorr_display').style.display = 'none';
                document.getElementById('vsa_panel').style.display = 'none';
                document.getElementById('demod_panel').style.display = 'none';

                // Enable IQ updates for fullscreen canvas
                showIQ = true;
                showXCorr = false;

                // Initialize IQ workspace canvases with proper DPI scaling
                setTimeout(() => {
                    const iqFullscreenCanvas = document.getElementById('iq_fullscreen');
                    const iqSpectrumCanvas = document.getElementById('iq_spectrum');
                    const dpr = window.devicePixelRatio || 1;

                    if (iqFullscreenCanvas) {
                        const container = document.getElementById('iq_fullscreen_container');
                        const rect = container.getBoundingClientRect();

                        // Set canvas resolution based on display size and device pixel ratio
                        iqFullscreenCanvas.width = rect.width * dpr;
                        iqFullscreenCanvas.height = rect.height * dpr;

                        // Scale context to match device pixel ratio for crisp rendering
                        const ctx = iqFullscreenCanvas.getContext('2d');
                        ctx.scale(dpr, dpr);

                        console.log(`IQ fullscreen canvas: display=${rect.width}x${rect.height}, resolution=${iqFullscreenCanvas.width}x${iqFullscreenCanvas.height}, dpr=${dpr}`);

                        // Initialize IQ constellation module with both canvases
                        if (typeof IQConstellationEnhanced !== 'undefined') {
                            const smallCanvas = document.getElementById('iq_canvas');
                            IQConstellationEnhanced.init(smallCanvas, iqFullscreenCanvas);
                        }
                    }

                    if (iqSpectrumCanvas) {
                        const container = iqSpectrumCanvas.parentElement;
                        const rect = container.getBoundingClientRect();

                        iqSpectrumCanvas.width = rect.width * dpr;
                        iqSpectrumCanvas.height = rect.height * dpr;

                        const ctx = iqSpectrumCanvas.getContext('2d');
                        ctx.scale(dpr, dpr);

                        console.log(`IQ spectrum canvas: display=${rect.width}x${rect.height}, resolution=${iqSpectrumCanvas.width}x${iqSpectrumCanvas.height}`);

                        // Add mouse handlers for spectrum selection
                        iqSpectrumCanvas.style.cursor = 'crosshair';

                        iqSpectrumCanvas.addEventListener('mousedown', (e) => {
                            const rect = iqSpectrumCanvas.getBoundingClientRect();
                            const x = e.clientX - rect.left;
                            const pct = (x / rect.width) * 100;

                            iqSelection.dragging = 'selecting';
                            iqSelection.leftPercent = pct;
                            iqSelection.rightPercent = pct;
                            iqSelection.active = true;
                        });

                        iqSpectrumCanvas.addEventListener('mousemove', (e) => {
                            if (iqSelection.dragging !== 'selecting') return;

                            const rect = iqSpectrumCanvas.getBoundingClientRect();
                            const x = e.clientX - rect.left;
                            const pct = Math.max(0, Math.min(100, (x / rect.width) * 100));

                            if (pct < iqSelection.leftPercent) {
                                iqSelection.rightPercent = iqSelection.leftPercent;
                                iqSelection.leftPercent = pct;
                            } else {
                                iqSelection.rightPercent = pct;
                            }
                        });

                        iqSpectrumCanvas.addEventListener('mouseup', async (e) => {
                            if (iqSelection.dragging === 'selecting') {
                                iqSelection.dragging = null;
                                console.log(`IQ spectrum selection: ${iqSelection.leftPercent.toFixed(1)}% - ${iqSelection.rightPercent.toFixed(1)}%`);

                                // Tune to the selected region
                                console.log(`[Debug] window.tuneToSelection type: ${typeof window.tuneToSelection}`);
                                if (typeof window.filterToSelection === 'function') {
                                    console.log('[Debug] Calling window.filterToSelection...');
                                    window.filterToSelection(iqSelection);
                                    console.log('[Debug] filterToSelection completed successfully');
                                } else {
                                    console.warn('[Debug] window.filterToSelection is not a function!');
                                }
                            }
                        });
                    }

                    // Initialize Eye Diagram and Waveform Display (deferred until canvases are sized)
                    // Only initialize once to avoid re-initialization on tab switches
                    if (typeof EyeDiagram !== 'undefined' && !window.eyeDiagramInitialized) {
                        EyeDiagram.init('eye_diagram_canvas');
                        window.eyeDiagramInitialized = true;
                        console.log('✓ EyeDiagram initialized (deferred)');
                    }

                    if (typeof WaveformDisplay !== 'undefined' && !window.waveformDisplayInitialized) {
                        WaveformDisplay.init('waveform_canvas');
                        window.waveformDisplayInitialized = true;
                        console.log('✓ WaveformDisplay initialized (deferred)');
                    }

                    // Trigger resize to ensure proper canvas dimensions
                    if (typeof EyeDiagram !== 'undefined' && EyeDiagram.resize) {
                        EyeDiagram.resize();
                    }
                    if (typeof WaveformDisplay !== 'undefined' && WaveformDisplay.resize) {
                        WaveformDisplay.resize();
                    }
                }, 100);
            } else if (tabName === 'xcorr') {
                // Cross-Correlation fullscreen workspace
                document.getElementById('signal_analysis').style.display = 'none';
                document.getElementById('marker_panel').style.display = 'none';
                document.getElementById('stats_panel').style.display = 'none';
                document.getElementById('activity_timeline').style.display = 'none';
                document.getElementById('iq_constellation').style.display = 'none';
                document.getElementById('xcorr_display').style.display = 'none';
                document.getElementById('vsa_panel').style.display = 'none';
                document.getElementById('demod_panel').style.display = 'none';

                // Enable cross-correlation updates for fullscreen canvas
                showIQ = false;
                showXCorr = true;

                // Initialize XCORR workspace canvases with proper DPI scaling
                setTimeout(() => {
                    const xcorrFullscreenCanvas = document.getElementById('xcorr_fullscreen');
                    const xcorrSpectrumCanvas = document.getElementById('xcorr_spectrum');
                    const dpr = window.devicePixelRatio || 1;

                    if (xcorrFullscreenCanvas) {
                        const container = document.getElementById('xcorr_fullscreen_container');
                        const rect = container.getBoundingClientRect();

                        // Set canvas resolution based on display size and device pixel ratio
                        xcorrFullscreenCanvas.width = rect.width * dpr;
                        xcorrFullscreenCanvas.height = rect.height * dpr;

                        // Scale context to match device pixel ratio for crisp rendering
                        const ctx = xcorrFullscreenCanvas.getContext('2d');
                        ctx.scale(dpr, dpr);

                        console.log(`XCORR fullscreen canvas: display=${rect.width}x${rect.height}, resolution=${xcorrFullscreenCanvas.width}x${xcorrFullscreenCanvas.height}, dpr=${dpr}`);

                        // Initialize cross-correlation module with both canvases
                        if (typeof CrossCorrelationEnhanced !== 'undefined') {
                            const smallCanvas = document.getElementById('xcorr_canvas');
                            CrossCorrelationEnhanced.init(smallCanvas, xcorrFullscreenCanvas);
                        }
                    }

                    if (xcorrSpectrumCanvas) {
                        const container = xcorrSpectrumCanvas.parentElement;
                        const rect = container.getBoundingClientRect();

                        xcorrSpectrumCanvas.width = rect.width * dpr;
                        xcorrSpectrumCanvas.height = rect.height * dpr;

                        const ctx = xcorrSpectrumCanvas.getContext('2d');
                        ctx.scale(dpr, dpr);

                        console.log(`XCORR spectrum canvas: display=${rect.width}x${rect.height}, resolution=${xcorrSpectrumCanvas.width}x${xcorrSpectrumCanvas.height}`);

                        // Add mouse handlers for spectrum selection
                        xcorrSpectrumCanvas.style.cursor = 'crosshair';

                        xcorrSpectrumCanvas.addEventListener('mousedown', (e) => {
                            const rect = xcorrSpectrumCanvas.getBoundingClientRect();
                            const x = e.clientX - rect.left;
                            const pct = (x / rect.width) * 100;

                            xcorrSelection.dragging = 'selecting';
                            xcorrSelection.leftPercent = pct;
                            xcorrSelection.rightPercent = pct;
                            xcorrSelection.active = true;
                        });

                        xcorrSpectrumCanvas.addEventListener('mousemove', (e) => {
                            if (xcorrSelection.dragging !== 'selecting') return;

                            const rect = xcorrSpectrumCanvas.getBoundingClientRect();
                            const x = e.clientX - rect.left;
                            const pct = Math.max(0, Math.min(100, (x / rect.width) * 100));

                            if (pct < xcorrSelection.leftPercent) {
                                xcorrSelection.rightPercent = xcorrSelection.leftPercent;
                                xcorrSelection.leftPercent = pct;
                            } else {
                                xcorrSelection.rightPercent = pct;
                            }
                        });

                        xcorrSpectrumCanvas.addEventListener('mouseup', async (e) => {
                            if (xcorrSelection.dragging === 'selecting') {
                                xcorrSelection.dragging = null;
                                console.log(`XCORR spectrum selection: ${xcorrSelection.leftPercent.toFixed(1)}% - ${xcorrSelection.rightPercent.toFixed(1)}%`);

                                // Tune to the selected region
                                console.log(`[Debug] window.tuneToSelection type: ${typeof window.tuneToSelection}`);
                                if (typeof window.filterToSelection === 'function') {
                                    console.log('[Debug] Calling window.filterToSelection...');
                                    window.filterToSelection(xcorrSelection);
                                    console.log('[Debug] filterToSelection completed successfully');
                                } else {
                                    console.warn('[Debug] window.filterToSelection is not a function!');
                                }
                            }
                        });
                    }
                }, 100);
            }

            console.log('Switched to workspace:', tabName);
        }

        // Detach panel from workspace slot back to floating mode
        function detachPanel(panelId) {
            const panel = document.getElementById(panelId);
            if (!panel) return;

            // Move panel to body (out of workspace slot)
            document.body.appendChild(panel);

            // Restore floating styles
            panel.style.position = 'fixed';
            panel.style.display = 'block';
            panel.style.width = '450px';
            panel.style.height = 'auto';
            panel.style.maxHeight = '80vh';

            // Position in center-ish of screen
            panel.style.top = '100px';
            panel.style.left = '50%';
            panel.style.transform = 'translateX(-50%)';
            panel.style.right = 'auto';
            panel.style.bottom = 'auto';

            // Restore border and shadow
            panel.style.border = '1px solid #333';
            panel.style.borderRadius = '4px';
            panel.style.boxShadow = '0 4px 6px rgba(0, 0, 0, 0.5)';

            console.log('Detached panel:', panelId);
        }

        // Attach click handlers to workspace tabs
        document.querySelectorAll('.workspace-tab').forEach(tab => {
            tab.addEventListener('click', () => {
                const tabName = tab.getAttribute('data-tab');
                switchWorkspace(tabName);

                // Reinitialize canvases when switching tabs
                setTimeout(() => {
                    if (tabName === 'measurements') {
                        initMeasurementsSpectrum();
                    } else if (tabName === 'demod') {
                        initDemodDisplays();
                        initDemodSpectrum();
                        initSMeter();
                    } else if (tabName === 'direction') {
                        initDirectionSpectrum();
                        initDoAPolarMain();
                        initDoATimeline();
                        showDoAInstructions();
                    }
                }, 100);
            });
        });

        // Initialize LIVE workspace as default (already active in HTML)
        console.log('Workspace tab system initialized - LIVE workspace active by default');

        // ===== INITIALIZE ENHANCED MODULES =====
        setTimeout(() => {
            // Restore last workspace
            const lastWorkspace = Settings.get('last_workspace', 'live');
            if (lastWorkspace && lastWorkspace !== 'live') {
                switchWorkspace(lastWorkspace);
            }

            console.log('✓ Enhanced modules initialized');
        }, 500);

    </script>

</body>
</html>
)HTMLDELIM";

// Helper function to read JavaScript files
static std::string read_js_file(const char* filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "JS file not found: " << filepath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// HTTP request handler
void web_server_handler(struct mg_connection *c, int ev, void *ev_data) {
#ifdef USE_MONGOOSE
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;

        // Serve main HTML page
        if (mg_strcmp(hm->uri, mg_str("/")) == 0) {
            mg_http_reply(c, 200,
                "Content-Type: text/html\r\n"
                "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                "Pragma: no-cache\r\n"
                "Expires: 0\r\n",
                "%s", html_page);
        }
        // FFT data request
        else if (mg_strcmp(hm->uri, mg_str("/fft")) == 0) {
            char channel_str[8] = "1";
            mg_http_get_var(&hm->query, "ch", channel_str, sizeof(channel_str));
            int channel = atoi(channel_str);

            std::lock_guard<std::mutex> lock(g_waterfall.mutex);
            const auto& history = (channel == 1) ? g_waterfall.ch1_history : g_waterfall.ch2_history;
            int latest_idx = (g_waterfall.write_index - 1 + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;

            // Send raw magnitude data as binary
            mg_printf(c, "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/octet-stream\r\n"
                        "Cache-Control: no-cache\r\n"
                        "Content-Length: %d\r\n"
                        "\r\n", WATERFALL_WIDTH);
            mg_send(c, history[latest_idx].data(), WATERFALL_WIDTH);
            g_http_bytes_sent.fetch_add(WATERFALL_WIDTH);
            c->is_draining = 1;
        }
        // Serve status JSON
        else if (mg_strcmp(hm->uri, mg_str("/status")) == 0) {
            char json[384];

            // Get noise floor values (0-255 scale)
            float nf_ch1, nf_ch2;
            get_noise_floor(g_noise_floor, nf_ch1, nf_ch2);

            snprintf(json, sizeof(json),
                    "{\"freq\":%llu,\"sr\":%u,\"bw\":%u,\"g1\":%u,\"g2\":%u,\"nf1\":%.1f,\"nf2\":%.1f}",
                    (unsigned long long)g_center_freq.load(),
                    g_sample_rate.load(),
                    g_bandwidth.load(),
                    g_gain_rx1.load(),
                    g_gain_rx2.load(),
                    nf_ch1,
                    nf_ch2);
            mg_http_reply(c, 200,
                "Content-Type: application/json\r\n",
                "%s", json);
            g_telemetry.http_requests.fetch_add(1);
        }
        // Serve telemetry/stats JSON
        else if (mg_strcmp(hm->uri, mg_str("/stats")) == 0) {
            std::string telemetry_json = get_telemetry_json();
            mg_http_reply(c, 200,
                "Content-Type: application/json\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", telemetry_json.c_str());
            g_telemetry.http_requests.fetch_add(1);
        }
        // Serve IQ constellation data
        else if (mg_strcmp(hm->uri, mg_str("/iq_data")) == 0) {
            std::lock_guard<std::mutex> lock(g_iq_data.mutex);

            // Parse optional filter parameters
            char start_bin_str[32] = "0";
            char end_bin_str[32] = "";
            mg_http_get_var(&hm->query, "start_bin", start_bin_str, sizeof(start_bin_str));
            mg_http_get_var(&hm->query, "end_bin", end_bin_str, sizeof(end_bin_str));

            const size_t sample_bytes = IQ_SAMPLES * sizeof(int16_t);
            const size_t total_bytes = sample_bytes * 4;

            // Check if filtering is requested and FFT data is available
            const bool filter_requested = (end_bin_str[0] != '\0');
            const bool fft_available = !g_iq_data.ch1_fft.empty() && !g_iq_data.ch2_fft.empty();

            if (filter_requested && fft_available) {
                // Perform frequency-domain bandpass filtering
                const size_t fft_size = g_iq_data.ch1_fft.size();
                size_t start_bin = std::atoi(start_bin_str);
                size_t end_bin = std::atoi(end_bin_str);

                // Clamp to valid range
                start_bin = std::min(start_bin, fft_size - 1);
                end_bin = std::min(end_bin, fft_size - 1);
                if (start_bin > end_bin) std::swap(start_bin, end_bin);

                // Create FFTW plans for IFFT (reuse for both channels)
                static fftwf_plan ifft_plan = nullptr;
                static fftwf_complex* ifft_in = nullptr;
                static fftwf_complex* ifft_out = nullptr;
                static size_t plan_size = 0;

                if (!ifft_plan || plan_size != fft_size) {
                    if (ifft_plan) {
                        fftwf_destroy_plan(ifft_plan);
                        fftwf_free(ifft_in);
                        fftwf_free(ifft_out);
                    }
                    ifft_in = fftwf_alloc_complex(fft_size);
                    ifft_out = fftwf_alloc_complex(fft_size);
                    ifft_plan = fftwf_plan_dft_1d(fft_size, ifft_in, ifft_out, FFTW_BACKWARD, FFTW_ESTIMATE);
                    plan_size = fft_size;
                }

                int16_t filtered_ch1_i[IQ_SAMPLES], filtered_ch1_q[IQ_SAMPLES];
                int16_t filtered_ch2_i[IQ_SAMPLES], filtered_ch2_q[IQ_SAMPLES];

                // Process CH1
                for (size_t i = 0; i < fft_size; i++) {
                    if (i >= start_bin && i <= end_bin) {
                        ifft_in[i][0] = g_iq_data.ch1_fft[i].real();
                        ifft_in[i][1] = g_iq_data.ch1_fft[i].imag();
                    } else {
                        ifft_in[i][0] = 0.0f;
                        ifft_in[i][1] = 0.0f;
                    }
                }
                fftwf_execute(ifft_plan);

                // Decimate IFFT output to 256 samples
                const int decimation_step = fft_size / IQ_SAMPLES;
                const float scale = 1.0f / fft_size;  // FFTW doesn't normalize IFFT
                for (int i = 0; i < IQ_SAMPLES; i++) {
                    const size_t idx = i * decimation_step;
                    filtered_ch1_i[i] = static_cast<int16_t>(ifft_out[idx][0] * scale * 32767.0f);
                    filtered_ch1_q[i] = static_cast<int16_t>(ifft_out[idx][1] * scale * 32767.0f);
                }

                // Process CH2
                for (size_t i = 0; i < fft_size; i++) {
                    if (i >= start_bin && i <= end_bin) {
                        ifft_in[i][0] = g_iq_data.ch2_fft[i].real();
                        ifft_in[i][1] = g_iq_data.ch2_fft[i].imag();
                    } else {
                        ifft_in[i][0] = 0.0f;
                        ifft_in[i][1] = 0.0f;
                    }
                }
                fftwf_execute(ifft_plan);

                // Decimate IFFT output to 256 samples
                for (int i = 0; i < IQ_SAMPLES; i++) {
                    const size_t idx = i * decimation_step;
                    filtered_ch2_i[i] = static_cast<int16_t>(ifft_out[idx][0] * scale * 32767.0f);
                    filtered_ch2_q[i] = static_cast<int16_t>(ifft_out[idx][1] * scale * 32767.0f);
                }

                // Send filtered data
                mg_printf(c, "HTTP/1.1 200 OK\r\n"
                            "Content-Type: application/octet-stream\r\n"
                            "Cache-Control: no-cache\r\n"
                            "Content-Length: %zu\r\n"
                            "\r\n", total_bytes);

                mg_send(c, filtered_ch1_i, sample_bytes);
                mg_send(c, filtered_ch1_q, sample_bytes);
                mg_send(c, filtered_ch2_i, sample_bytes);
                mg_send(c, filtered_ch2_q, sample_bytes);
                g_http_bytes_sent.fetch_add(total_bytes);
                c->is_draining = 1;
            } else {
                // Send unfiltered data
                mg_printf(c, "HTTP/1.1 200 OK\r\n"
                            "Content-Type: application/octet-stream\r\n"
                            "Cache-Control: no-cache\r\n"
                            "Content-Length: %zu\r\n"
                            "\r\n", total_bytes);

                mg_send(c, g_iq_data.ch1_i, sample_bytes);
                mg_send(c, g_iq_data.ch1_q, sample_bytes);
                mg_send(c, g_iq_data.ch2_i, sample_bytes);
                mg_send(c, g_iq_data.ch2_q, sample_bytes);
                g_http_bytes_sent.fetch_add(total_bytes);
                c->is_draining = 1;
            }
        }
        // Serve cross-correlation data
        else if (mg_strcmp(hm->uri, mg_str("/xcorr_data")) == 0) {
            std::lock_guard<std::mutex> lock(g_xcorr_data.mutex);

            // Parse optional filter parameters
            char start_bin_str[32] = "0";
            char end_bin_str[32] = "";
            mg_http_get_var(&hm->query, "start_bin", start_bin_str, sizeof(start_bin_str));
            mg_http_get_var(&hm->query, "end_bin", end_bin_str, sizeof(end_bin_str));

            const size_t array_size = g_xcorr_data.magnitude.size();
            size_t start_bin = std::atoi(start_bin_str);
            size_t end_bin = (end_bin_str[0] != '\0') ? std::atoi(end_bin_str) : array_size - 1;

            // Clamp to valid range
            start_bin = std::min(start_bin, array_size - 1);
            end_bin = std::min(end_bin, array_size - 1);
            if (start_bin > end_bin) std::swap(start_bin, end_bin);

            // Calculate filtered data size
            const size_t filtered_size = end_bin - start_bin + 1;
            const size_t mag_bytes = filtered_size * sizeof(float);
            const size_t total_bytes = mag_bytes * 2;

            // Send headers with exact Content-Length
            mg_printf(c, "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/octet-stream\r\n"
                        "Cache-Control: no-cache\r\n"
                        "Content-Length: %zu\r\n"
                        "\r\n", total_bytes);

            // Send binary data - magnitude then phase (filtered range)
            mg_send(c, g_xcorr_data.magnitude.data() + start_bin, mag_bytes);
            mg_send(c, g_xcorr_data.phase.data() + start_bin, mag_bytes);
            g_http_bytes_sent.fetch_add(total_bytes);
            c->is_draining = 1;
        }
        // Serve Direction of Arrival result as JSON
        else if (mg_strcmp(hm->uri, mg_str("/doa_result")) == 0) {
            // Parse optional bin range parameters for DF filtering
            char start_bin_str[32] = "0";
            char end_bin_str[32] = "0";
            mg_http_get_var(&hm->query, "start_bin", start_bin_str, sizeof(start_bin_str));
            mg_http_get_var(&hm->query, "end_bin", end_bin_str, sizeof(end_bin_str));

            const uint32_t start_bin = static_cast<uint32_t>(atoi(start_bin_str));
            const uint32_t end_bin = static_cast<uint32_t>(atoi(end_bin_str));

            // Update global DF bin range (0, 0 means use entire spectrum)
            g_df_start_bin.store(start_bin);
            g_df_end_bin.store(end_bin);

            std::lock_guard<std::mutex> lock(g_doa_result.mutex);

            // Format DoA result as JSON
            char json[512];
            snprintf(json, sizeof(json),
                    "{\"azimuth\":%.2f,"
                    "\"backAzimuth\":%.2f,"
                    "\"hasAmbiguity\":%s,"
                    "\"phaseDiff\":%.2f,"
                    "\"phaseStd\":%.2f,"
                    "\"confidence\":%.1f,"
                    "\"snr\":%.1f,"
                    "\"coherence\":%.3f}",
                    g_doa_result.azimuth,
                    g_doa_result.back_azimuth,
                    g_doa_result.has_ambiguity ? "true" : "false",
                    g_doa_result.phase_diff_deg,
                    g_doa_result.phase_std_deg,
                    g_doa_result.confidence,
                    g_doa_result.snr_db,
                    g_doa_result.coherence);

            mg_http_reply(c, 200,
                         "Content-Type: application/json\r\n"
                         "Cache-Control: no-cache\r\n",
                         "%s", json);
            g_http_bytes_sent.fetch_add(strlen(json));
        }
        // Serve link quality metrics as JSON
        else if (mg_strcmp(hm->uri, mg_str("/link_quality")) == 0) {
            std::lock_guard<std::mutex> lock(g_link_quality.mutex);

            // Calculate bandwidth in kbps (bytes_sent is already per-second from update)
            float bandwidth_kbps = (g_link_quality.bytes_sent.load() * 8.0f) / 1000.0f;

            char json[256];
            snprintf(json, sizeof(json),
                    "{\"rtt_ms\":%.1f,\"packet_loss\":%.3f,\"fps\":%.1f,\"bandwidth_kbps\":%.1f}",
                    g_link_quality.rtt_ms.load(),
                    g_link_quality.packet_loss.load(),
                    g_link_quality.fps.load(),
                    bandwidth_kbps);

            mg_http_reply(c, 200,
                "Content-Type: application/json\r\n",
                "%s", json);
        }
        // Handle control commands (zoom and parameter changes)
        else if (mg_strcmp(hm->uri, mg_str("/control")) == 0) {
            // Parse JSON body using mg_json_get_long
            double freq_val = mg_json_get_long(hm->body, "$.freq", -1);
            double sr_val = mg_json_get_long(hm->body, "$.sr", -1);
            double bw_val = mg_json_get_long(hm->body, "$.bw", -1);
            double gain1_val = mg_json_get_long(hm->body, "$.gain1", -1);
            double gain2_val = mg_json_get_long(hm->body, "$.gain2", -1);

            // Only log significant parameter changes
            if (freq_val >= 0 || sr_val >= 0 || bw_val >= 0) {
                std::cout << "RF: ";
                if (freq_val >= 0) std::cout << freq_val << "MHz ";
                if (sr_val >= 0) std::cout << "SR:" << sr_val << "M ";
                if (bw_val >= 0) std::cout << "BW:" << bw_val << "M ";
                if (gain1_val >= 0 || gain2_val >= 0) std::cout << "G:" << gain1_val << "/" << gain2_val << "dB";
                std::cout << std::endl;
            }

            bool has_update = false;
            bool valid = true;
            std::string update_msg = "Updated: ";

            std::lock_guard<std::mutex> lock(g_config_mutex);

            // Update frequency if provided
            if (freq_val > 0) {
                uint64_t new_freq = (uint64_t)freq_val;
                if (new_freq >= 47000000ULL && new_freq <= 6000000000ULL) {
                    g_center_freq.store(new_freq);
                    has_update = true;
                    update_msg += "freq=" + std::to_string(new_freq / 1e6) + "MHz ";
                } else {
                    valid = false;
                }
            }

            // Update sample rate if provided
            if (sr_val > 0) {
                uint32_t new_sr = (uint32_t)sr_val;
                if (new_sr >= 520000 && new_sr <= 61440000) {
                    g_sample_rate.store(new_sr);
                    has_update = true;
                    update_msg += "sr=" + std::to_string(new_sr / 1e6) + "MHz ";
                } else {
                    valid = false;
                }
            }

            // Update bandwidth if provided
            if (bw_val > 0) {
                uint32_t new_bw = (uint32_t)bw_val;
                if (new_bw >= 520000 && new_bw <= 61440000) {
                    g_bandwidth.store(new_bw);
                    has_update = true;
                    update_msg += "bw=" + std::to_string(new_bw / 1e6) + "MHz ";
                } else {
                    valid = false;
                }
            }

            // Update gain RX1 if provided
            if (gain1_val >= 0) {
                uint32_t new_gain = (uint32_t)gain1_val;
                if (new_gain <= 60) {
                    g_gain_rx1.store(new_gain);
                    has_update = true;
                    update_msg += "RX1=" + std::to_string(new_gain) + "dB ";
                } else {
                    valid = false;
                }
            }

            // Update gain RX2 if provided
            if (gain2_val >= 0) {
                uint32_t new_gain = (uint32_t)gain2_val;
                if (new_gain <= 60) {
                    g_gain_rx2.store(new_gain);
                    has_update = true;
                    update_msg += "RX2=" + std::to_string(new_gain) + "dB ";
                } else {
                    valid = false;
                }
            }

            if (has_update && valid) {
                g_params_changed.store(true);
                mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                             "{\"status\":\"ok\"}");
            } else if (!valid) {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Invalid parameters\"}");
            } else {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"No parameters provided\"}");
            }
        }
        // Start Recording Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/start_recording")) == 0) {
            // Parse JSON body for filename
            char *filename_str = mg_json_get_str(hm->body, "$.filename");

            if (!filename_str) {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Missing filename\"}");
                return;
            }

            // Start recording
            bool success = start_recording(filename_str, g_center_freq.load(), g_sample_rate.load(),
                                          BANDWIDTH, g_gain_rx1.load(), g_gain_rx2.load());
            free(filename_str);

            if (success) {
                char json_buf[256];
                snprintf(json_buf, sizeof(json_buf),
                        "{\"status\":\"ok\",\"recording\":true,\"samples\":0}");
                mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_buf);
            } else {
                mg_http_reply(c, 500, "Content-Type: application/json\r\n",
                             "{\"error\":\"Failed to start recording\"}");
            }
        }
        // Stop Recording Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/stop_recording")) == 0) {
            stop_recording();
            mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                         "{\"status\":\"ok\",\"recording\":false}");
        }
        // Get Recording Status Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/recording_status")) == 0) {
            uint64_t samples_written = 0;
            bool active = get_recording_status(samples_written);
            char json_buf[256];
            snprintf(json_buf, sizeof(json_buf),
                    "{\"recording\":%s,\"samples\":%llu,\"duration_sec\":%.1f}",
                    active ? "true" : "false",
                    (unsigned long long)samples_written,
                    samples_written / (float)g_sample_rate.load());
            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_buf);
        }
        // Get GPS Position Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/gps_position")) == 0) {
            std::lock_guard<std::mutex> lock(g_gps_position.mutex);

            char json_buf[512];
            snprintf(json_buf, sizeof(json_buf),
                    "{\"mode\":\"%s\","
                    "\"valid\":%s,"
                    "\"latitude\":%.8f,"
                    "\"longitude\":%.8f,"
                    "\"altitude_m\":%.2f,"
                    "\"satellites\":%u,"
                    "\"hdop\":%.1f,"
                    "\"timestamp_ms\":%llu}",
                    (g_gps_position.mode == GPSPosition::Mode::GPS_AUTO) ? "auto" : "manual",
                    g_gps_position.valid ? "true" : "false",
                    g_gps_position.latitude,
                    g_gps_position.longitude,
                    g_gps_position.altitude_m,
                    g_gps_position.satellites,
                    g_gps_position.hdop,
                    (unsigned long long)g_gps_position.timestamp_ms);

            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_buf);
        }
        // Set GPS Mode Endpoint (auto/manual)
        else if (mg_strcmp(hm->uri, mg_str("/set_gps_mode")) == 0) {
            char *mode_str = mg_json_get_str(hm->body, "$.mode");
            if (!mode_str) {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Missing mode parameter\"}");
                return;
            }

            if (strcmp(mode_str, "auto") == 0) {
                set_gps_mode(true);
                mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                             "{\"status\":\"ok\",\"mode\":\"auto\"}");
            } else if (strcmp(mode_str, "manual") == 0) {
                set_gps_mode(false);
                mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                             "{\"status\":\"ok\",\"mode\":\"manual\"}");
            } else {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Invalid mode (use 'auto' or 'manual')\"}");
            }
            free(mode_str);
        }
        // Set Manual Position Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/set_manual_position")) == 0) {
            double lat = mg_json_get_num(hm->body, "$.latitude", 0);
            double lon = mg_json_get_num(hm->body, "$.longitude", 0);
            double alt = mg_json_get_num(hm->body, "$.altitude_m", 0);

            // Basic validation
            if (lat < -90 || lat > 90 || lon < -180 || lon > 180) {
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Invalid coordinates\"}");
                return;
            }

            set_manual_position(lat, lon, alt);

            mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                         "{\"status\":\"ok\",\"latitude\":%.8f,\"longitude\":%.8f,\"altitude_m\":%.2f}",
                         lat, lon, alt);
        }
        // UDP Stream Relay Endpoint
        else if (mg_strcmp(hm->uri, mg_str("/stream_udp_relay")) == 0) {
            // Parse JSON body using mg_json_get
            char *endpoint_str = mg_json_get_str(hm->body, "$.endpoint");
            long port_val = mg_json_get_long(hm->body, "$.port", 0);
            char *data_str = mg_json_get_str(hm->body, "$.data");

            if (!endpoint_str || port_val <= 0 || !data_str) {
                if (endpoint_str) free(endpoint_str);
                if (data_str) free(data_str);
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Missing endpoint, port, or data\"}");
                return;
            }

            // Create UDP socket
            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                free(endpoint_str);
                free(data_str);
                mg_http_reply(c, 500, "Content-Type: application/json\r\n",
                             "{\"error\":\"Failed to create UDP socket\"}");
                return;
            }

            struct sockaddr_in dest_addr;
            memset(&dest_addr, 0, sizeof(dest_addr));
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons((uint16_t)port_val);

            if (inet_pton(AF_INET, endpoint_str, &dest_addr.sin_addr) <= 0) {
                close(sockfd);
                free(endpoint_str);
                free(data_str);
                mg_http_reply(c, 400, "Content-Type: application/json\r\n",
                             "{\"error\":\"Invalid IP address\"}");
                return;
            }

            // Send data via UDP
            ssize_t sent = sendto(sockfd, data_str, strlen(data_str), 0,
                                  (struct sockaddr*)&dest_addr, sizeof(dest_addr));

            close(sockfd);
            // Only log errors, not every successful send
            if (sent < 0) {
                std::cerr << "UDP send failed to " << endpoint_str << ":" << port_val << std::endl;
            }
            free(endpoint_str);
            free(data_str);

            if (sent < 0) {
                mg_http_reply(c, 500, "Content-Type: application/json\r\n",
                             "{\"error\":\"UDP send failed\"}");
            } else {
                mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                             "{\"status\":\"ok\",\"sent\":%d}", (int)sent);
            }
        }
        // Serve JavaScript modules from web_assets/js/
        else if (mg_strcmp(hm->uri, mg_str("/js/rf_measurements.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/rf_measurements.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/marker_system.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/marker_system.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/vsa_analysis.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/vsa_analysis.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/statistics.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/statistics.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/utils.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/utils.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/settings.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/settings.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/signal_filters.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/signal_filters.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/marker_system.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/marker_system.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/rf_measurements.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/rf_measurements.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/vsa_analysis.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/vsa_analysis.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/signal_classifier.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/signal_classifier.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        // Display modules
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/waterfall.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/waterfall.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/spectrum.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/spectrum.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/iq_constellation_enhanced.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/iq_constellation_enhanced.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/cross_correlation_enhanced.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/cross_correlation_enhanced.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/eye_diagram.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/eye_diagram.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else if (mg_strcmp(hm->uri, mg_str("/js/displays/waveform_display.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/displays/waveform_display.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        // Utility modules
        else if (mg_strcmp(hm->uri, mg_str("/js/utils/colormap.js")) == 0) {
            std::string js_content = read_js_file("server/web_assets/js/utils/colormap.js");
            mg_http_reply(c, 200,
                "Content-Type: text/javascript; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n",
                "%s", js_content.c_str());
        }
        else {
            mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "Not Found");
        }
    }
#else
    (void)c; (void)ev; (void)ev_data;
#endif
}

// Removed TX endpoints - keeping code minimal and RX-focused

void start_web_server() {
#ifdef USE_MONGOOSE
    g_web_running = true;

    // Start web server thread
    g_web_thread = std::thread([]() {
        // Reduce mongoose logging verbosity (only show errors)
        mg_log_set(MG_LL_ERROR);

        mg_mgr_init(&g_mgr);

        char url[64];
        snprintf(url, sizeof(url), "http://0.0.0.0:%d", WEB_SERVER_PORT);

        if (mg_http_listen(&g_mgr, url, web_server_handler, nullptr) == nullptr) {
            std::cerr << "Web server failed to start on port " << WEB_SERVER_PORT << std::endl;
            g_web_running = false;
            return;
        }

        std::cout << "Web server ready: http://localhost:" << WEB_SERVER_PORT << std::endl;

        while (g_web_running) {
            mg_mgr_poll(&g_mgr, 100);  // Poll every 100ms
        }

        mg_mgr_free(&g_mgr);
    });
#else
    std::cout << "Web server disabled (USE_MONGOOSE not defined)" << std::endl;
#endif
}

void stop_web_server() {
    // Stop GPS thread if running
    if (g_gps_running) {
        g_gps_running = false;
        if (g_gps_thread.joinable()) {
            g_gps_thread.join();
        }
    }

    // Stop web server thread
    if (g_web_running) {
        g_web_running = false;
        if (g_web_thread.joinable()) {
            g_web_thread.join();
        }
    }
}

