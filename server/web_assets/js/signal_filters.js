/**
 * Signal Filtering Module
 * Advanced DSP filtering for spectrum data
 */

const SignalFilters = {
    // Filter state
    state: {
        notchFilters: [],  // Array of {freq: MHz, width: kHz, enabled: bool}
        noiseBlankerEnabled: false,
        noiseBlankerThreshold: 20,  // dB above noise floor
        anrEnabled: false,  // Adaptive Noise Reduction
        anrStrength: 0.5,   // 0-1
        iqCorrectionEnabled: false,
        iqGainImbalance: 1.0,
        iqPhaseImbalance: 0.0
    },

    /**
     * Apply notch filter to remove interference
     * @param {Uint8Array} fftData - FFT magnitude data
     * @param {number} centerFreqMHz - Center frequency
     * @param {number} sampleRateMHz - Sample rate
     * @returns {Uint8Array} Filtered data
     */
    applyNotchFilters(fftData, centerFreqMHz, sampleRateMHz) {
        if (!this.state.notchFilters.length) return fftData;

        const output = new Uint8Array(fftData);
        const binWidth = (sampleRateMHz * 1e6) / fftData.length; // Hz per bin

        this.state.notchFilters.forEach(notch => {
            if (!notch.enabled) return;

            // Calculate frequency range for this notch
            const notchFreqHz = notch.freq * 1e6;
            const notchWidthHz = notch.width * 1e3;
            const startFreqHz = centerFreqMHz * 1e6 - (sampleRateMHz * 1e6) / 2;

            // Calculate affected bins
            const notchCenterBin = Math.floor((notchFreqHz - startFreqHz) / binWidth);
            const notchWidthBins = Math.ceil(notchWidthHz / binWidth);

            // Apply notch (zero out affected bins with some tapering)
            const startBin = Math.max(0, notchCenterBin - notchWidthBins);
            const endBin = Math.min(fftData.length - 1, notchCenterBin + notchWidthBins);

            for (let i = startBin; i <= endBin; i++) {
                // Calculate distance from center
                const dist = Math.abs(i - notchCenterBin);
                const normalized = dist / notchWidthBins;

                // Apply Hann window taper
                const attenuation = normalized < 1.0 ? Math.cos(normalized * Math.PI / 2) : 0;
                output[i] = Math.floor(output[i] * attenuation);
            }
        });

        return output;
    },

    /**
     * Noise blanker - removes impulsive noise
     * @param {Uint8Array} fftData - FFT magnitude data
     * @returns {Uint8Array} Filtered data
     */
    applyNoiseBlanker(fftData) {
        if (!this.state.noiseBlankerEnabled) return fftData;

        const output = new Uint8Array(fftData);

        // Estimate noise floor (median of lower 25% of values)
        const sorted = Array.from(fftData).sort((a, b) => a - b);
        const noiseFloor = sorted[Math.floor(sorted.length * 0.25)];
        const threshold = noiseFloor + this.state.noiseBlankerThreshold;

        // Blank samples that are significantly above threshold
        for (let i = 0; i < fftData.length; i++) {
            // Check if this is an isolated spike
            const left = i > 0 ? fftData[i - 1] : fftData[i];
            const right = i < fftData.length - 1 ? fftData[i + 1] : fftData[i];
            const neighbors = (left + right) / 2;

            // If current sample is much higher than neighbors and above threshold, it's likely noise
            if (fftData[i] > threshold && fftData[i] > neighbors * 1.5) {
                // Replace with interpolated value
                output[i] = Math.floor(neighbors);
            }
        }

        return output;
    },

    /**
     * Adaptive Noise Reduction - smooths out noise while preserving signals
     * @param {Uint8Array} fftData - FFT magnitude data
     * @param {Uint8Array} previousData - Previous frame for temporal smoothing
     * @returns {Uint8Array} Filtered data
     */
    applyANR(fftData, previousData) {
        if (!this.state.anrEnabled || !previousData) return fftData;

        const output = new Uint8Array(fftData);
        const alpha = this.state.anrStrength; // Smoothing factor

        // Temporal smoothing - reduces random noise
        for (let i = 0; i < fftData.length; i++) {
            // Exponential moving average
            output[i] = Math.floor(alpha * fftData[i] + (1 - alpha) * previousData[i]);
        }

        // Spatial smoothing - moving average
        const windowSize = 3;
        const temp = new Uint8Array(output);

        for (let i = windowSize; i < fftData.length - windowSize; i++) {
            let sum = 0;
            for (let j = -windowSize; j <= windowSize; j++) {
                sum += temp[i + j];
            }
            output[i] = Math.floor(sum / (2 * windowSize + 1));
        }

        return output;
    },

    /**
     * Add a notch filter
     * @param {number} freqMHz - Notch center frequency
     * @param {number} widthKHz - Notch width
     */
    addNotchFilter(freqMHz, widthKHz) {
        this.state.notchFilters.push({
            freq: freqMHz,
            width: widthKHz,
            enabled: true,
            id: Date.now()
        });

        console.log(`Added notch filter at ${freqMHz} MHz, ${widthKHz} kHz wide`);
        return this.state.notchFilters[this.state.notchFilters.length - 1];
    },

    /**
     * Remove a notch filter
     */
    removeNotchFilter(id) {
        this.state.notchFilters = this.state.notchFilters.filter(n => n.id !== id);
    },

    /**
     * Clear all notch filters
     */
    clearNotchFilters() {
        this.state.notchFilters = [];
    },

    /**
     * Apply all enabled filters
     * @param {Uint8Array} fftData - Input FFT data
     * @param {number} centerFreqMHz - Center frequency
     * @param {number} sampleRateMHz - Sample rate
     * @param {Uint8Array} previousData - Previous frame (for ANR)
     * @returns {Uint8Array} Filtered data
     */
    applyAll(fftData, centerFreqMHz, sampleRateMHz, previousData) {
        let filtered = fftData;

        // Apply filters in order
        filtered = this.applyNotchFilters(filtered, centerFreqMHz, sampleRateMHz);
        filtered = this.applyNoiseBlanker(filtered);
        filtered = this.applyANR(filtered, previousData);

        return filtered;
    }
};

// Make available globally
window.SignalFilters = SignalFilters;

console.log('✓ Signal Filters module loaded');
