#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================================
// SIGNAL PROCESSING CONFIGURATION
// These constants control FFT processing, windowing, and magnitude scaling
// ============================================================================

// FFT magnitude to dB conversion constants
// These map FFT power values to a 0-255 display range covering 120 dB dynamic range
namespace FFTConfig {
    constexpr float DB_SCALE = 10.0f;           // Logarithmic scaling factor (10*log10 for power)
    constexpr float DB_OFFSET = 100.0f;         // Offset to handle negative dB values
    constexpr float DB_RANGE = 120.0f;          // Total dynamic range in dB (typ. -120 to 0 dBm)
    constexpr float NORM_SCALE = 255.0f / DB_RANGE;  // Normalize to 0-255 range
    constexpr float MIN_POWER = 1e-20f;         // Minimum power floor (prevents log(0))
}

// DC offset removal configuration
// Controls spectral leakage mitigation at center frequency
namespace DCConfig {
    constexpr int DC_INTERPOLATION_BINS = 2;   // Bins around DC to interpolate (±2 bins)
    constexpr float DC_ALPHA_FAST = 0.5f;      // Fast convergence alpha (first 20 frames)
    constexpr float DC_ALPHA_SLOW = 0.1f;      // Slow tracking alpha (after convergence)
    constexpr int DC_CONVERGENCE_FRAMES = 20;   // Frames until convergence complete
}

// Noise floor estimation configuration
// Percentile-based noise floor for adaptive CFAR and SNR calculation
namespace NoiseFloorConfig {
    constexpr float DEFAULT_PERCENTILE = 15.0f;  // Use 15th percentile as noise floor
    constexpr float SMOOTHING_ALPHA = 0.1f;      // EWMA smoothing factor (0.1 = slow tracking)
    constexpr int UPDATE_INTERVAL_FRAMES = 10;   // Update every 10 frames to reduce CPU load
}

// ============================================================================
// DIRECTION FINDING CONFIGURATION
// Phase-based interferometry parameters for 2-channel DF
// ============================================================================

namespace DFConfig {
    // Antenna array geometry
    constexpr float ANTENNA_SPACING_WAVELENGTHS = 0.5f;  // λ/2 spacing (typ. for 915 MHz)
    // At 915 MHz: λ = c/f = 0.328m, so 0.5λ = 0.164m = 164mm

    // Signal detection thresholds
    constexpr size_t MIN_BINS_FOR_DF = 3;        // Minimum bins needed for reliable DF
    constexpr float MIN_CONFIDENCE_THRESHOLD = 20.0f;  // Minimum confidence to report bearing (%)

    // Phase stability metrics
    constexpr float PHASE_CONFIDENCE_DECAY = 25.0f;  // Decay rate for phase std dev
    constexpr float SNR_BOOST_THRESHOLD = 20.0f;     // SNR above which confidence increases
    constexpr float SNR_BOOST_SCALE = 40.0f;          // Scale factor for SNR boost
    constexpr float MAX_SNR_BOOST = 1.3f;              // Maximum SNR confidence multiplier
    constexpr float AMBIGUITY_PENALTY = 0.9f;          // Confidence penalty for 180° ambiguity

    // Coherence calculation
    constexpr float COHERENCE_DECAY = 10.0f;     // Decay rate for coherence metric
}

// Kalman filter configuration for bearing tracking
namespace KalmanConfig {
    constexpr float PROCESS_NOISE_AZIMUTH = 0.5f;    // Process noise for azimuth (deg^2)
    constexpr float PROCESS_NOISE_VELOCITY = 0.1f;   // Process noise for velocity ((deg/s)^2)
    constexpr float INITIAL_VELOCITY_UNCERTAINTY = 10.0f;  // Initial velocity covariance
}

// ============================================================================
// CFAR DETECTOR CONFIGURATION
// Constant False Alarm Rate detection parameters
// ============================================================================

namespace CFARConfig {
    // CA-CFAR (Cell-Averaging CFAR) - general purpose
    constexpr int CA_GUARD_CELLS = 4;           // Guard cells on each side of CUT
    constexpr int CA_TRAINING_CELLS = 16;       // Training cells for noise estimation
    constexpr float CA_THRESHOLD_FACTOR = 3.0f; // Threshold = mean + 3*std (typ. 10-15 dB)

    // OS-CFAR (Ordered Statistic CFAR) - better in cluttered environments
    constexpr int OS_GUARD_CELLS = 4;
    constexpr int OS_TRAINING_CELLS = 24;
    constexpr float OS_PERCENTILE = 0.75f;      // Use 75th percentile (k = 0.75*N)
    constexpr float OS_THRESHOLD_FACTOR = 2.5f;

    // GO-CFAR (Greatest-Of CFAR) - better in multiple target scenarios
    constexpr float GO_THRESHOLD_FACTOR = 3.5f;

    // SO-CFAR (Smallest-Of CFAR) - better in clutter edges
    constexpr float SO_THRESHOLD_FACTOR = 3.0f;
}

// ============================================================================
// USB ERROR RECOVERY CONFIGURATION
// Exponential backoff and device reset parameters
// ============================================================================

namespace USBConfig {
    constexpr uint32_t INITIAL_BACKOFF_MS = 100;      // Initial backoff delay
    constexpr uint32_t MAX_BACKOFF_MS = 5000;         // Maximum backoff delay (5 seconds)
    constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 10;   // Errors before attempting reset
    constexpr uint32_t RESET_SETTLE_TIME_MS = 1000;   // Wait after reset before retry
}

// ============================================================================
// WATCHDOG CONFIGURATION
// RX thread health monitoring parameters
// ============================================================================

namespace WatchdogConfig {
    constexpr uint32_t CHECK_INTERVAL_SEC = 1;        // Check heartbeat every second
    constexpr uint32_t STALL_THRESHOLD_SEC = 3;       // Alert after 3 seconds of stall
    constexpr uint32_t CRITICAL_STALL_SEC = 10;       // Force shutdown after 10 second stall
}

// ============================================================================
// HTTP SERVER CONFIGURATION
// Web interface and data streaming parameters
// ============================================================================

namespace HTTPConfig {
    constexpr int WATERFALL_RATE_LIMIT_DIVISOR = 1;  // Send every Nth waterfall update
    constexpr int XCORR_RATE_LIMIT_DIVISOR = 5;      // Send xcorr at 1/5 rate (2 Hz)
    constexpr int IQ_RATE_LIMIT_DIVISOR = 1;         // Send IQ data every update
}

#endif // CONFIG_H
