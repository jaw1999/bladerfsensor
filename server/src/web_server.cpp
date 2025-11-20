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

// Helper function to get MIME type from file extension
static const char* get_mime_type(const char* path) {
    const char* ext = strrchr(path, '.');
    if (!ext) return "application/octet-stream";

    if (strcmp(ext, ".html") == 0 || strcmp(ext, ".htm") == 0) return "text/html; charset=utf-8";
    if (strcmp(ext, ".js") == 0) return "application/javascript; charset=utf-8";
    if (strcmp(ext, ".css") == 0) return "text/css; charset=utf-8";
    if (strcmp(ext, ".json") == 0) return "application/json; charset=utf-8";
    if (strcmp(ext, ".png") == 0) return "image/png";
    if (strcmp(ext, ".jpg") == 0 || strcmp(ext, ".jpeg") == 0) return "image/jpeg";
    if (strcmp(ext, ".gif") == 0) return "image/gif";
    if (strcmp(ext, ".svg") == 0) return "image/svg+xml";
    if (strcmp(ext, ".ico") == 0) return "image/x-icon";
    if (strcmp(ext, ".txt") == 0) return "text/plain; charset=utf-8";

    return "application/octet-stream";
}

// Helper function to read static files from web_assets directory
static std::string read_static_file(const char* filepath) {
    // Try to read from web_assets directory relative to executable
    std::string full_path = std::string("web_assets/") + filepath;

    std::ifstream file(full_path, std::ios::binary);
    if (!file.is_open()) {
        // Try alternative path (relative to server directory)
        full_path = std::string("server/web_assets/") + filepath;
        file.open(full_path, std::ios::binary);

        if (!file.is_open()) {
            std::cerr << "Static file not found: " << filepath << std::endl;
            return "";
        }
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// HTML page is now served from web_assets/index.html via read_static_file()


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
            std::string html_content = read_static_file("index.html");
            if (!html_content.empty()) {
                mg_http_reply(c, 200,
                    "Content-Type: text/html; charset=utf-8\r\n"
                    "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                    "Pragma: no-cache\r\n"
                    "Expires: 0\r\n",
                    "%s", html_content.c_str());
            } else {
                mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "404 Not Found");
            }
        }
        // FFT data request (uncompressed)
        else if (mg_strcmp(hm->uri, mg_str("/fft")) == 0) {
            char channel_str[8] = "1";
            mg_http_get_var(&hm->query, "ch", channel_str, sizeof(channel_str));
            int channel = atoi(channel_str);

            std::lock_guard<std::mutex> lock(g_waterfall.mutex);
            const auto& history = (channel == 1) ? g_waterfall.ch1_history : g_waterfall.ch2_history;
            int latest_idx = (g_waterfall.write_index - 1 + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;

            // Send raw uncompressed data
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
        // Generic static file serving for all other requests
        else {
            // Extract URI path (remove leading slash)
            std::string uri_path(hm->uri.buf, hm->uri.len);
            if (uri_path[0] == '/') {
                uri_path = uri_path.substr(1);
            }

            // Try to serve the file
            std::string file_content = read_static_file(uri_path.c_str());
            if (!file_content.empty()) {
                const char* mime_type = get_mime_type(uri_path.c_str());
                char content_type_header[256];
                snprintf(content_type_header, sizeof(content_type_header),
                        "Content-Type: %s\r\n"
                        "Cache-Control: no-cache\r\n", mime_type);
                mg_http_reply(c, 200, content_type_header, "%s", file_content.c_str());
            } else {
                mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "404 Not Found");
            }
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

