#include "array_calibration.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

// Global calibration state
ArrayCalibration g_array_cal = {false, {}, 0.5f};
std::mutex g_calibration_mutex;

void add_calibration_point(uint64_t frequency, float measured_phase_diff_deg, float known_azimuth_deg) {
    std::lock_guard<std::mutex> lock(g_calibration_mutex);

    // Expected phase difference for this azimuth at this frequency
    // Using interferometer equation: Δφ = (2π * d * sin(θ)) / λ
    // With d = 0.5λ: Δφ = π * sin(θ)
    const float theta_rad = known_azimuth_deg * M_PI / 180.0f;
    const float expected_phase_rad = M_PI * std::sin(theta_rad);
    const float expected_phase_deg = expected_phase_rad * 180.0f / M_PI;

    // Phase correction is the difference between expected and measured
    const float correction_deg = expected_phase_deg - measured_phase_diff_deg;

    // Check if we already have a calibration point at this frequency
    bool found = false;
    for (auto& point : g_array_cal.points) {
        if (point.frequency == frequency) {
            // Update existing point
            point.phase_correction_deg = correction_deg;
            point.known_azimuth_deg = known_azimuth_deg;
            point.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            found = true;
            std::cout << "Updated calibration at " << (frequency / 1e6) << " MHz: "
                      << "correction = " << correction_deg << "°" << std::endl;
            break;
        }
    }

    if (!found) {
        // Add new calibration point
        CalibrationPoint new_point;
        new_point.frequency = frequency;
        new_point.phase_correction_deg = correction_deg;
        new_point.known_azimuth_deg = known_azimuth_deg;
        new_point.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        g_array_cal.points.push_back(new_point);

        // Sort points by frequency for interpolation
        std::sort(g_array_cal.points.begin(), g_array_cal.points.end(),
                  [](const CalibrationPoint& a, const CalibrationPoint& b) {
                      return a.frequency < b.frequency;
                  });

        std::cout << "Added calibration at " << (frequency / 1e6) << " MHz: "
                  << "correction = " << correction_deg << "°" << std::endl;
    }
}

float get_phase_correction(uint64_t frequency) {
    std::lock_guard<std::mutex> lock(g_calibration_mutex);

    if (!g_array_cal.enabled || g_array_cal.points.empty()) {
        return 0.0f;  // No correction
    }

    // If only one calibration point, use it for all frequencies
    if (g_array_cal.points.size() == 1) {
        return g_array_cal.points[0].phase_correction_deg;
    }

    // Find surrounding calibration points for interpolation
    CalibrationPoint* lower = nullptr;
    CalibrationPoint* upper = nullptr;

    for (size_t i = 0; i < g_array_cal.points.size(); i++) {
        if (g_array_cal.points[i].frequency <= frequency) {
            lower = &g_array_cal.points[i];
        }
        if (g_array_cal.points[i].frequency >= frequency && upper == nullptr) {
            upper = &g_array_cal.points[i];
        }
    }

    // Exact match
    if (lower && lower->frequency == frequency) {
        return lower->phase_correction_deg;
    }
    if (upper && upper->frequency == frequency) {
        return upper->phase_correction_deg;
    }

    // Interpolate between two points
    if (lower && upper) {
        const float freq_frac = static_cast<float>(frequency - lower->frequency) /
                                static_cast<float>(upper->frequency - lower->frequency);
        return lower->phase_correction_deg +
               freq_frac * (upper->phase_correction_deg - lower->phase_correction_deg);
    }

    // Extrapolate from nearest point
    if (lower) {
        return lower->phase_correction_deg;
    }
    if (upper) {
        return upper->phase_correction_deg;
    }

    return 0.0f;
}

void save_calibration(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_calibration_mutex);

    FILE* f = fopen(filename.c_str(), "w");
    if (!f) {
        std::cerr << "Failed to save calibration to: " << filename << std::endl;
        return;
    }

    fprintf(f, "# Array Calibration Data\n");
    fprintf(f, "# Frequency(Hz), PhaseCorrection(deg), KnownAzimuth(deg), Timestamp\n");

    for (const auto& point : g_array_cal.points) {
        fprintf(f, "%lu,%.3f,%.2f,%lu\n",
                point.frequency,
                point.phase_correction_deg,
                point.known_azimuth_deg,
                point.timestamp);
    }

    fclose(f);
    std::cout << "Calibration saved to: " << filename << " ("
              << g_array_cal.points.size() << " points)" << std::endl;
}

void load_calibration(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_calibration_mutex);

    FILE* f = fopen(filename.c_str(), "r");
    if (!f) {
        std::cerr << "Failed to load calibration from: " << filename << std::endl;
        return;
    }

    g_array_cal.points.clear();
    char line[256];

    // Skip header lines
    while (fgets(line, sizeof(line), f)) {
        if (line[0] != '#') {
            // Parse calibration point
            CalibrationPoint point;
            if (sscanf(line, "%lu,%f,%f,%lu",
                      &point.frequency,
                      &point.phase_correction_deg,
                      &point.known_azimuth_deg,
                      &point.timestamp) == 4) {
                g_array_cal.points.push_back(point);
            }
        }
    }

    fclose(f);

    // Sort by frequency
    std::sort(g_array_cal.points.begin(), g_array_cal.points.end(),
              [](const CalibrationPoint& a, const CalibrationPoint& b) {
                  return a.frequency < b.frequency;
              });

    std::cout << "Calibration loaded from: " << filename << " ("
              << g_array_cal.points.size() << " points)" << std::endl;
}
