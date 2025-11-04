/**
 * Signal Classification Module
 * Analyzes RF signals to detect modulation type, bandwidth, and characteristics
 */

const SignalClassifier = {
    // Classification state
    state: {
        enabled: false,
        selectedSignal: null,
        classificationHistory: [],
        maxHistory: 100
    },

    /**
     * Estimate signal bandwidth from FFT data
     * @param {Uint8Array} fftData - FFT magnitude data
     * @param {number} centerBin - Center bin of signal
     * @param {number} binWidth - Frequency width per bin (Hz)
     * @returns {object} {bandwidth: number, startBin: number, endBin: number}
     */
    estimateBandwidth(fftData, centerBin, binWidth) {
        if (!fftData || centerBin < 0) return null;

        // Find signal edges using threshold method
        const centerPower = fftData[centerBin];
        const threshold = centerPower * 0.5; // -6dB from peak

        // Search left for edge
        let leftBin = centerBin;
        while (leftBin > 0 && fftData[leftBin] > threshold) {
            leftBin--;
        }

        // Search right for edge
        let rightBin = centerBin;
        while (rightBin < fftData.length - 1 && fftData[rightBin] > threshold) {
            rightBin++;
        }

        const bw = (rightBin - leftBin) * binWidth;

        return {
            bandwidth: bw,
            startBin: leftBin,
            endBin: rightBin,
            occupied3dB: (rightBin - leftBin)
        };
    },

    /**
     * Classify modulation type from IQ samples
     * @param {Array} iData - In-phase samples
     * @param {Array} qData - Quadrature samples
     * @returns {object} Classification result
     */
    classifyModulation(iData, qData) {
        if (!iData || !qData || iData.length < 100) {
            return { type: 'UNKNOWN', confidence: 0 };
        }

        // Calculate signal characteristics
        const envelope = this.calculateEnvelope(iData, qData);
        const phase = this.calculatePhase(iData, qData);
        const freq = this.calculateInstantaneousFreq(phase);

        // Feature extraction
        const envStdDev = this.stdDev(envelope);
        const envMean = this.mean(envelope);
        const envVariance = envStdDev / (envMean + 1e-10);

        const phaseStdDev = this.stdDev(phase);
        const freqStdDev = this.stdDev(freq);

        // Classification logic
        let type = 'UNKNOWN';
        let confidence = 0;

        // AM: High envelope variance, low phase variance
        if (envVariance > 0.3 && phaseStdDev < 0.5) {
            type = 'AM';
            confidence = Math.min(95, 50 + envVariance * 100);
        }
        // FM: Low envelope variance, high freq deviation
        else if (envVariance < 0.2 && freqStdDev > 0.5) {
            type = 'FM';
            confidence = Math.min(95, 50 + freqStdDev * 50);
        }
        // PSK/Digital: Clustered phase states
        else if (this.detectPhaseClusters(phase) > 1) {
            const clusters = this.detectPhaseClusters(phase);
            if (clusters === 2) {
                type = 'BPSK';
            } else if (clusters === 4) {
                type = 'QPSK';
            } else {
                type = 'PSK';
            }
            confidence = 70;
        }
        // FSK: Distinct frequency clusters
        else if (this.detectFreqClusters(freq) >= 2) {
            type = 'FSK';
            confidence = 75;
        }
        // SSB: Moderate envelope variance, some phase variation
        else if (envVariance > 0.15 && envVariance < 0.4) {
            type = 'SSB';
            confidence = 60;
        }
        // CW: Very stable envelope and phase
        else if (envVariance < 0.05 && phaseStdDev < 0.1) {
            type = 'CW';
            confidence = 80;
        }

        return {
            type: type,
            confidence: confidence,
            features: {
                envelopeVariance: envVariance.toFixed(3),
                phaseStdDev: phaseStdDev.toFixed(3),
                freqStdDev: freqStdDev.toFixed(3)
            }
        };
    },

    /**
     * Calculate signal envelope from IQ
     */
    calculateEnvelope(iData, qData) {
        const envelope = [];
        for (let i = 0; i < iData.length; i++) {
            envelope.push(Math.sqrt(iData[i] * iData[i] + qData[i] * qData[i]));
        }
        return envelope;
    },

    /**
     * Calculate instantaneous phase from IQ
     */
    calculatePhase(iData, qData) {
        const phase = [];
        for (let i = 0; i < iData.length; i++) {
            phase.push(Math.atan2(qData[i], iData[i]));
        }
        return phase;
    },

    /**
     * Calculate instantaneous frequency from phase
     */
    calculateInstantaneousFreq(phase) {
        const freq = [];
        for (let i = 1; i < phase.length; i++) {
            let diff = phase[i] - phase[i - 1];
            // Unwrap phase
            if (diff > Math.PI) diff -= 2 * Math.PI;
            if (diff < -Math.PI) diff += 2 * Math.PI;
            freq.push(diff);
        }
        return freq;
    },

    /**
     * Detect number of phase clusters (for PSK detection)
     */
    detectPhaseClusters(phase, tolerance = 0.3) {
        // Simple histogram-based clustering
        const bins = 8;
        const histogram = new Array(bins).fill(0);

        phase.forEach(p => {
            // Normalize to [0, 2π]
            let normalized = p;
            while (normalized < 0) normalized += 2 * Math.PI;
            while (normalized >= 2 * Math.PI) normalized -= 2 * Math.PI;

            const bin = Math.floor((normalized / (2 * Math.PI)) * bins);
            histogram[bin]++;
        });

        // Count peaks in histogram
        const mean = histogram.reduce((a, b) => a + b, 0) / bins;
        const threshold = mean * 1.5;
        let clusters = 0;

        for (let i = 0; i < bins; i++) {
            if (histogram[i] > threshold) {
                clusters++;
            }
        }

        return clusters;
    },

    /**
     * Detect number of frequency clusters (for FSK detection)
     */
    detectFreqClusters(freq, numBins = 10) {
        if (freq.length === 0) return 0;

        const min = Math.min(...freq);
        const max = Math.max(...freq);
        const range = max - min;

        if (range < 1e-6) return 1;

        const histogram = new Array(numBins).fill(0);

        freq.forEach(f => {
            const bin = Math.floor(((f - min) / range) * (numBins - 1));
            histogram[Math.max(0, Math.min(numBins - 1, bin))]++;
        });

        // Count peaks
        const mean = histogram.reduce((a, b) => a + b, 0) / numBins;
        const threshold = mean * 1.5;
        let clusters = 0;

        for (let i = 0; i < numBins; i++) {
            if (histogram[i] > threshold) {
                clusters++;
            }
        }

        return clusters;
    },

    /**
     * Calculate mean of array
     */
    mean(arr) {
        if (arr.length === 0) return 0;
        return arr.reduce((a, b) => a + b, 0) / arr.length;
    },

    /**
     * Calculate standard deviation of array
     */
    stdDev(arr) {
        if (arr.length === 0) return 0;
        const avg = this.mean(arr);
        const squareDiffs = arr.map(value => Math.pow(value - avg, 2));
        return Math.sqrt(this.mean(squareDiffs));
    },

    /**
     * Classify modulation from FFT characteristics (when IQ not available)
     * @param {Uint8Array} fftData - FFT magnitude data
     * @param {number} peakBin - Peak bin index
     * @param {object} bwInfo - Bandwidth information
     * @returns {object} Classification result
     */
    classifyFromFFT(fftData, peakBin, bwInfo) {
        if (!fftData || !bwInfo) {
            return { type: 'UNKNOWN', confidence: 0 };
        }

        const bandwidth = bwInfo.bandwidth;
        const peakPower = fftData[peakBin];

        // Calculate spectral characteristics
        const regionData = fftData.slice(bwInfo.startBin, bwInfo.endBin + 1);
        const regionMean = this.mean(Array.from(regionData));
        const regionStd = this.stdDev(Array.from(regionData));
        const spectralVariance = regionStd / (regionMean + 1e-10);

        let type = 'UNKNOWN';
        let confidence = 0;

        // Classification based on FFT characteristics
        if (bandwidth < 100) {
            // Very narrow bandwidth
            type = 'CW/Carrier';
            confidence = 85;
        } else if (bandwidth < 5000) {
            // Narrow bandwidth (< 5 kHz)
            if (spectralVariance < 0.15) {
                type = 'CW/Carrier';
                confidence = 75;
            } else {
                type = 'SSB/Narrowband';
                confidence = 60;
            }
        } else if (bandwidth < 15000) {
            // Medium bandwidth (5-15 kHz) - typical voice
            if (spectralVariance > 0.3) {
                type = 'AM';
                confidence = 65;
            } else {
                type = 'NFM';
                confidence = 70;
            }
        } else if (bandwidth < 50000) {
            // Wide bandwidth (15-50 kHz)
            type = 'FM';
            confidence = 75;
        } else if (bandwidth < 200000) {
            // Very wide bandwidth (50-200 kHz)
            if (spectralVariance < 0.2) {
                type = 'Digital/OFDM';
                confidence = 60;
            } else {
                type = 'WFM';
                confidence = 70;
            }
        } else {
            // Extremely wide
            type = 'Wideband/Noise';
            confidence = 50;
        }

        return {
            type: type,
            confidence: confidence,
            features: {
                bandwidth: (bandwidth / 1000).toFixed(1) + ' kHz',
                spectralVariance: spectralVariance.toFixed(3)
            }
        };
    },

    /**
     * Classify a signal region
     * @param {Uint8Array} fftData - Full FFT data
     * @param {number} peakBin - Peak bin index
     * @param {number} centerFreqMHz - Center frequency
     * @param {number} sampleRateMHz - Sample rate
     * @param {object} iqData - {i: Array, q: Array} IQ samples (optional)
     */
    classifySignal(fftData, peakBin, centerFreqMHz, sampleRateMHz, iqData) {
        const binWidth = (sampleRateMHz * 1e6) / fftData.length;

        // Estimate bandwidth
        const bwInfo = this.estimateBandwidth(fftData, peakBin, binWidth);

        // Calculate signal frequency
        const binFreq = centerFreqMHz - sampleRateMHz / 2 + (peakBin / fftData.length) * sampleRateMHz;

        // Classify modulation
        let modulation;
        if (iqData && iqData.i && iqData.q) {
            // Use IQ data if available (more accurate)
            modulation = this.classifyModulation(iqData.i, iqData.q);
        } else {
            // Fall back to FFT-based classification
            modulation = this.classifyFromFFT(fftData, peakBin, bwInfo);
        }

        const result = {
            timestamp: Date.now(),
            frequency: binFreq,
            bandwidth: bwInfo ? bwInfo.bandwidth : 0,
            power: fftData[peakBin],
            modulation: modulation.type,
            confidence: modulation.confidence,
            features: modulation.features || {}
        };

        // Add to history
        this.state.classificationHistory.push(result);
        if (this.state.classificationHistory.length > this.state.maxHistory) {
            this.state.classificationHistory.shift();
        }

        return result;
    },

    /**
     * Export classification history
     */
    exportHistory() {
        if (this.state.classificationHistory.length === 0) {
            return null;
        }

        let csv = 'Timestamp,Frequency (MHz),Bandwidth (Hz),Power,Modulation,Confidence (%)\n';
        this.state.classificationHistory.forEach(item => {
            const timestamp = new Date(item.timestamp).toISOString();
            csv += `${timestamp},${item.frequency.toFixed(6)},${item.bandwidth.toFixed(0)},${item.power},${item.modulation},${item.confidence.toFixed(0)}\n`;
        });

        return csv;
    }
};

// Make available globally
window.SignalClassifier = SignalClassifier;

console.log('✓ Signal Classifier module loaded');
