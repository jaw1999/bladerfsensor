# bladeRF Sensor System

Advanced RF spectrum monitoring and direction finding system using bladeRF xA9 for coherent dual-channel acquisition

## Overview

This system provides real-time RF spectrum monitoring, signal analysis, and 2-channel direction finding using a bladeRF xA9 SDR. The server runs on a LattePanda Sigma or similar platform, performs coherent dual-channel sampling at 40 MHz, FFT processing, cross-correlation analysis, and serves an advanced web interface with multiple workspaces over HTTP. Features include TAKX-RF integration for tactical systems, real-time spectrum analysis, and recording capabilities.

## System Architecture

```
┌────────────────────────────────────────────────────┐
│  LattePanda Sigma / Aircraft / Mobile Platform     │
│  ┌──────────────────────────────┐                  │
│  │  bladeRF xA9 (Dual RX)       │                  │
│  │  • RX1 @ 40 MHz (coherent)   │                  │
│  │  • RX2 @ 40 MHz (coherent)   │                  │
│  │  • Phase-locked for DF       │                  │
│  └──────────┬───────────────────┘                  │
│             │ USB 3.0                              │
│  ┌──────────▼─────────────────────────────────┐    │
│  │  C++ Server (bladerf_server)               │    │
│  │  • Dual-channel acquisition (40 MHz)       │    │
│  │  • 4096-point FFT (FFTW)                   │    │
│  │  • Cross-correlation & DF processing       │    │
│  │  • Direction finding (phase-based)         │    │
│  │  • Mongoose HTTP/SSE server                │    │
│  │  • Binary data streaming                   │    │
│  │  • CoT message generation (TAKX-RF)        │    │
│  └──────────┬─────────────────────────────────┘    │
└─────────────┼──────────────────────────────────────┘
              │
              ├─ HTTP/SSE (~0.9 Mbps)
              │  Tactical IP Link
              │  (Silvus/Doodle Labs/etc)
              │
┌─────────────▼──────────────────────────────────────┐
│  Ground Station - Web Browser UI                   │
│  ┌──────────────────────────────────────────────┐  │
│  │  LIVE: Waterfall + Spectrum + IQ + Controls  │  │
│  │  MEASUREMENTS: Signal Analysis + Mask Tests  │  │
│  │  DIRECTION: Phase-Based DF + CoT Streaming   │  │
│  │  DEMOD: IQ Visualization (Placeholder)       │  │
│  └──────────────────────────────────────────────┘  │
└─────────────┬──────────────────────────────────────┘
              │
              │ UDP CoT Messages
              │ (LoB + Platform Position)
              │
┌─────────────▼──────────────────────────────────────┐
│  TAKX-RF / TAK Client                              │
│  • Platform icon on map                            │
│  • Line of bearing (LoB) display                   │
│  • Real-time bearing updates                       │
└────────────────────────────────────────────────────┘
```

## Features

### Hardware
- **Coherent dual-channel RX** from bladeRF xA9 (MIMO mode)
- **40 MHz sample rate**, 40 MHz bandwidth
- **Configurable center frequency** (70 MHz - 6 GHz)
- **Phase-coherent sampling** for direction finding

### Signal Processing
- **4096-point FFT** on both channels with FFTW
- **Cross-correlation analysis** (frequency domain)
- **DC offset removal** with EWMA filtering
- **Configurable window functions** (Hamming, Hanning, Blackman, Kaiser)
- **Magnitude-to-dB conversion** with 120 dB dynamic range
- **Real-time averaging** and persistence modes
- **10-20 Hz update rate**

### Direction Finding
- **2-Channel Phase-Based DF** using interferometry
- **Real-time azimuth calculation** with phase unwrapping
- **180° ambiguity detection** (inherent to 2-element arrays)
- **Polar plots** with bearing timeline history
- **Configurable antenna spacing** (wavelength-based)
- **SNR and coherence metrics**
- **TAKX-RF CoT streaming** for TAK integration
- **Multiple platform types** (UAV, UGV, USV, Ground Station)

### Web Interface - Multiple Workspaces

#### LIVE Tab
- **Real-time waterfall display** with color gradients
- **Spectrum analyzer** with gradient fill and color-coded traces
- **Dual-channel display** (RX1/RX2/Both)
- **Interactive zoom** (mouse selection, scroll/zoom)
- **IQ constellation plots**
- **RF parameter controls** (frequency, gain, sample rate, bandwidth)

#### MEASUREMENTS Tab
- **Signal measurements**: Peak, Average, Noise Floor, OBUE
- **Occupied bandwidth** (-3dB calculation)
- **SNR calculations**
- **Peak detection** with threshold control
- **Spectrum analyzer** with peak markers

#### DIRECTION Tab
- **Interactive spectrum display** with frequency selection
- **Real-time polar azimuth plot** with dual ambiguity display
- **Bearing timeline** showing history
- **Phase difference metrics** (unwrapped, std deviation)
- **Confidence and quality indicators**
- **CoT streaming configuration** (UDP/TCP)
- **Platform position** (static or MGRS)
- **Calibration controls**

#### DEMOD Tab
- **Live IQ display**
- **Waveform visualization**
- **Placeholder for future demodulation** (AM/FM/PSK/FSK)

### Recording & Playback
- **Spectrum recording** to WAV format
- **Full-band or selective bandwidth** recording
- **Metadata export** (JSON with RF parameters)
- **Audio conversion** from FFT magnitude

### Signal Analysis
- **Spectrum mask testing** with violation detection
- **Signal classification** (narrowband, wideband, OFDM)
- **Activity timeline** tracking signal presence
- **Bookmark system** for signals of interest
- **CSV export** for offline analysis

### TAKX-RF Integration
- **Cursor on Target (CoT)** message generation
- **Line of Bearing (LoB)** format compliance
- **Platform position events** (separate from LoB)
- **UDP/TCP streaming** to TAK clients
- **MIL-STD-2525 symbology** support
- **Real-time bearing updates** at configurable rates

### Bandwidth Optimization
- **Binary data format** (no JSON overhead)
- **Direct HTTP streaming** via Server-Sent Events
- **8-bit magnitude compression** (120 dB dynamic range mapped to 0-255)
- **Configurable update rates** (1-20 Hz)
- **Target bandwidth**: <1 Mbps for full operation

## Quick Start

### Prerequisites

**On LattePanda:**
- Ubuntu/Debian Linux
- bladeRF xA9 with FPGA loaded
- Network connection

### Build and Run

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y cmake build-essential pkg-config \
    libfftw3-dev libbladerf-dev libbladerf2

# Build server
cd server
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run server
./bladerf_server        # Uses default 915 MHz
# or
./bladerf_server 2450000000  # Specify frequency in Hz
```

### Access Web UI

Open your browser to:
```
http://<latte-panda-ip>:8080
```

You should see:
- Real-time waterfall display
- Frequency spectrum
- RF parameter controls

## Configuration

### Server Configuration

Edit `server/include/bladerf_sensor.h`:

```cpp
constexpr uint32_t SAMPLE_RATE = 40000000;      // 40 MHz
constexpr uint32_t BANDWIDTH = 40000000;        // 40 MHz
constexpr uint64_t CENTER_FREQ = 915000000;     // 915 MHz
constexpr uint32_t FFT_SIZE = 4096;             // FFT points
constexpr uint32_t UPDATE_RATE_HZ = 10;         // Updates/sec
constexpr uint32_t GAIN_RX1 = 40;               // RX1 gain (dB)
constexpr uint32_t GAIN_RX2 = 40;               // RX2 gain (dB)
constexpr int WEB_SERVER_PORT = 8080;           // Web interface port
```

After changes: `cd build && make`

### Bandwidth Optimization

For tactical radio links reduce bandwidth:

**Option 1: Reduce FFT size**
```cpp
constexpr uint32_t FFT_SIZE = 2048;  // Half the bins
```
Bandwidth: ~0.5 Mbps

**Option 2: Reduce update rate**
```cpp
constexpr uint32_t UPDATE_RATE_HZ = 5;  // Half the rate
```
Bandwidth: ~0.45 Mbps

**Option 3: Both**
Combine for ultra-low bandwidth (~0.25 Mbps)

## Network Requirements

- **Bandwidth**: ~0.9 Mbps (configurable down to 0.1 Mbps)
- **Latency**: Works over high-latency tactical links (100-500ms)
- **Connection**: TCP/IP over any IP backhaul
- **Ports**:
  - Web UI: 8080 (HTTP/SSE)
  - Data port: 5555 (TCP, for external clients)

## Troubleshooting

### Server Issues

**Device not found:**
```bash
bladeRF-cli -p  # Verify device is detected
sudo dmesg | grep -i blade  # Check USB connection
```

**Permission denied:**
```bash
sudo usermod -a -G plugdev $USER
# Log out and back in
```

**FPGA not loaded:**
```bash
bladeRF-cli -l /path/to/hostedxA9.rbf
```

### Client Issues

**Cannot connect to web UI:**
```bash
ping <latte-panda-ip>  # Test connectivity
telnet <latte-panda-ip> 8080  # Test port
# Check firewall on LattePanda
```

**No data in browser:**
- Open browser console (F12) and check for errors
- Verify server is running
- Check that bladeRF is detected

### Performance Issues

**Choppy display over tactical link:**
- Reduce FFT_SIZE to 2048 or 1024
- Reduce UPDATE_RATE_HZ to 5 or 2
- Enable link quality monitoring

**High CPU on server:**
- Reduce FFT_SIZE
- Reduce UPDATE_RATE_HZ
- Enable performance CPU governor

## Project Structure

```
bladerfsensor/
├── README.md              # This file
├── deploy.sh              # Automated deployment script
└── server/                # C++ server
    ├── CMakeLists.txt
    ├── README.md
    ├── include/
    │   ├── bladerf_sensor.h
    │   └── web_server.h
    └── src/
        ├── main.cpp
        ├── web_server.cpp
        ├── mongoose.c         # Embedded web server
        └── mongoose.h
```

## Technical Details

### Data Flow

1. **Acquisition**: bladeRF xA9 samples at 40 MHz on both RX channels in MIMO mode
2. **Deinterleaving**: Server separates interleaved I/Q samples (I1,Q1,I2,Q2)
3. **DC Offset Removal**: EWMA filter removes DC bias
4. **Window Function**: Applied to reduce spectral leakage (Hamming default)
5. **FFT**: 4096-point FFT computed on both channels using FFTW
6. **Compression**: Convert complex FFT to 8-bit magnitude (120 dB range → 0-255)
7. **Cross-correlation**: Computed in frequency domain for DF
8. **Streaming**: Binary data sent via HTTP/SSE to browsers
9. **Visualization**: JavaScript canvas rendering with WebGL acceleration

### Direction Finding Mathematics

The 2-channel direction finding uses **phase interferometry**:

**Phase Difference Calculation:**
```
Δφ = phase(CH2) - phase(CH1)
   = atan2(Q₂, I₂) - atan2(Q₁, I₁)
```

**Interferometer Equation:**
```
sin(θ) = (Δφ × λ) / (2π × d)

Where:
  θ  = angle of arrival
  Δφ = measured phase difference (radians)
  λ  = wavelength (meters)
  d  = antenna spacing (meters)
```

**180° Ambiguity:**

For a 2-element array, `sin(θ) = sin(180° - θ)`, creating two possible solutions:
- Primary bearing: θ
- Ambiguous bearing: 180° - θ

**Example:**
```
Antenna spacing: 0.5λ (half-wavelength)
Measured phase: 45°
sin(θ) = (45° × π/180 × λ) / (2π × 0.5λ) = 0.25
θ₁ = arcsin(0.25) = 14.5°
θ₂ = 180° - 14.5° = 165.5°
```

Both bearings are displayed on the polar plot. Requires 3+ channels to resolve ambiguity.

**Confidence Metrics:**
- **Phase Std Dev**: Lower = more stable signal
- **Coherence**: Measure of phase consistency (0-1)
- **SNR**: Signal-to-noise ratio in dB
- **Confidence**: Combined metric (0-100%)

### Mathematical Accuracy

All mathematical calculations have been validated and corrected:

**dB Conversion:**
```
dB = (raw / 255.0) × 120.0 - 100.0
Range: -100 dBFS to +20 dBFS (120 dB dynamic range)
```

**Occupied Bandwidth (-3dB):**
```
threshold_raw = peak_raw - 6.375
(Correct for quantized dB space, not linear amplitude)
```

**Cross-Correlation Phase:**
```
xcorr = conj(CH1) × CH2
Gives: phase(CH2) - phase(CH1)
(Critical for correct DF bearing direction)
```

**IQ Power:**
```
power = sqrt((I² + Q²) / N)
(Both I and Q components, not just I alone)
```

**Zoom Coordinate Mapping:**
```
bin = zoomStartBin + (canvasX / canvasWidth) × zoomedBins
(Respects current zoom state for nested zooming)
```

All formulas verified against theory and tested with real signals.

### Bandwidth Calculation

**Spectrum data per frame:**
- CH1 magnitude: 4096 bytes (8-bit)
- CH2 magnitude: 4096 bytes (8-bit)
- Total: 8192 bytes per frame

**At 10 Hz update rate:**
- Data rate: 81920 bytes/sec
- With HTTP overhead: ~90 KB/s
- Bandwidth: 0.87 Mbps

### Web Server Architecture

Uses Mongoose embedded HTTP server:
- Single-threaded event loop
- Server-Sent Events for real-time updates
- Binary data endpoints
- RESTful control API
- Embedded HTML/CSS/JavaScript

### Link Degradation Strategy
As link quality degrades:
1. Reduce FFT size (4096 → 2048 → 1024)
2. Reduce update rate (10 → 5 → 2 Hz)
3. Drop to waterfall-only mode
4. Buffer data during brief outages

## Usage Examples

### Basic Spectrum Monitoring
1. Open web interface at `http://<device-ip>:8080`
2. Navigate to **LIVE** tab
3. Adjust frequency, gain, and sample rate as needed
4. Enable **Spectrum** view for real-time analysis
5. Use mouse to **zoom into signals** (drag to select, right-click to zoom out)
6. Use **Ctrl+Scroll** to zoom Y-axis, **Scroll** to pan

### Direction Finding
1. Navigate to **DIRECTION** tab
2. Click and drag on spectrum to **select frequency range**
3. Click **Start** to begin direction finding
4. View **azimuth on polar plot** (shows both ambiguous bearings)
5. Monitor **confidence, SNR, and coherence** metrics
6. Configure **Stream Out** for TAK integration

### TAKX-RF Integration
1. Open **Stream Out Configuration**
2. Set **Endpoint IP** and **Port** (default: 8089 for TAKX-RF)
3. Select **Protocol**: UDP
4. Choose **Format**: CoT
5. Enter **Platform Position** (Lat/Lon or MGRS)
6. Select **Platform Type** (UAV, UGV, USV, etc.)
7. Click **Start Streaming**
8. Platform icon and bearing lines appear in TAKX-RF

### Recording Signals
1. Click **Record** button in header
2. Choose **Mode**: Full Band or Selective Band
3. For selective, specify center frequency and bandwidth
4. Click **Start Recording**
5. Recording saved as WAV file with metadata JSON

## Known Limitations

- **2-Channel DF**: Inherent 180° ambiguity (requires 3+ channels to resolve)
- **MGRS Conversion**: Uses simplified approximation (errors of 100m-1km depending on location)
- **Demodulation**: UI placeholder only, no actual AM/FM/PSK/FSK demodulation implemented
- **Multi-client**: Single client recommended for best performance

## Future Enhancements

- **Automatic signal classification** using ML
- **GPS integration** for dynamic platform positioning
- **Real-time demodulation** (AM/FM/PSK/FSK)
- **Adaptive streaming** based on link quality
- **Multi-client support** with bandwidth sharing
- **CUDA/OpenCL FFT** acceleration
- **Persistent storage** of spectrum recordings
- **Replay mode** from recorded files

## References

- [bladeRF Documentation](https://github.com/Nuand/bladeRF)
- [FFTW Library](http://www.fftw.org/)
- [Mongoose Web Server](https://github.com/cesanta/mongoose)
