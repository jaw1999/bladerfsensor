#ifndef ARRAY_CALIBRATION_H
#define ARRAY_CALIBRATION_H

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>

// Array calibration point
struct CalibrationPoint {
    uint64_t frequency;                // Frequency in Hz
    float phase_correction_deg;        // Phase correction in degrees
    float known_azimuth_deg;           // Known true azimuth used for calibration
    uint64_t timestamp;                // Calibration timestamp (UNIX seconds)
};

// Array calibration state
struct ArrayCalibration {
    bool enabled;                      // Calibration correction enabled
    std::vector<CalibrationPoint> points;  // Calibration points
    float antenna_spacing_actual;      // Actual measured spacing (wavelengths at ref freq)
};

// Global calibration state (thread-safe access via mutex)
extern ArrayCalibration g_array_cal;
extern std::mutex g_calibration_mutex;

// Add or update a calibration point
void add_calibration_point(uint64_t frequency, float measured_phase_diff_deg, float known_azimuth_deg);

// Get phase correction for a given frequency (interpolated if needed)
float get_phase_correction(uint64_t frequency);

// Save calibration data to file
void save_calibration(const std::string& filename);

// Load calibration data from file
void load_calibration(const std::string& filename);

#endif // ARRAY_CALIBRATION_H
