/**
 * Cross-Correlation Display Module
 * Three-plot visualization: coherence spectrum, magnitude, and group delay
 */

const CrossCorrelation = (function() {
    'use strict';

    let xcorrCanvas, xcorrCtx;
    const XCORR_SIZE = 512;

    function init(canvas) {
        xcorrCanvas = canvas;
        xcorrCtx = canvas.getContext('2d');
    }

    function unwrapPhase(phase) {
        const unwrapped = new Float32Array(phase.length);
        unwrapped[0] = phase[0];

        for (let i = 1; i < phase.length; i++) {
            let diff = phase[i] - unwrapped[i - 1];
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            unwrapped[i] = unwrapped[i - 1] + diff;
        }

        return unwrapped;
    }

    function draw(correlation, ch1_fft, ch2_fft) {
        if (!correlation || !ch1_fft || !ch2_fft) return;

        const width = xcorrCanvas.width;
        const height = xcorrCanvas.height;

        // Clear
        xcorrCtx.fillStyle = '#0a0a0a';
        xcorrCtx.fillRect(0, 0, width, height);

        // Three plots stacked vertically
        const plotHeight = height / 3;

        // Calculate metrics
        const fftCorr = new Float32Array(XCORR_SIZE);
        const fftPhase = new Float32Array(XCORR_SIZE);

        for (let i = 0; i < XCORR_SIZE; i++) {
            const ch1_re = ch1_fft[i * 2];
            const ch1_im = ch1_fft[i * 2 + 1];
            const ch2_re = ch2_fft[i * 2];
            const ch2_im = ch2_fft[i * 2 + 1];

            const ch1_mag = Math.sqrt(ch1_re * ch1_re + ch1_im * ch1_im);
            const ch2_mag = Math.sqrt(ch2_re * ch2_re + ch2_im * ch2_im);

            // Cross-spectrum
            const cross_re = ch1_re * ch2_re + ch1_im * ch2_im;
            const cross_im = ch1_im * ch2_re - ch1_re * ch2_im;
            const cross_mag = Math.sqrt(cross_re * cross_re + cross_im * cross_im);

            // Coherence
            fftCorr[i] = (ch1_mag > 0 && ch2_mag > 0) ? cross_mag / (ch1_mag * ch2_mag) : 0;
            fftPhase[i] = Math.atan2(cross_im, cross_re);
        }

        const unwrappedPhase = unwrapPhase(fftPhase);

        // Group delay (derivative of phase)
        const freqStep = 1.0 / XCORR_SIZE;
        const groupDelay = new Float32Array(unwrappedPhase.length - 1);
        for (let i = 0; i < groupDelay.length; i++) {
            const phaseDiff = unwrappedPhase[i + 1] - unwrappedPhase[i];
            groupDelay[i] = -phaseDiff / (2 * Math.PI * freqStep) * 1e9; // Convert to ns
        }

        // Find peaks and stats
        let maxCorr = Math.max(...fftCorr);
        let avgCorr = fftCorr.reduce((a, b) => a + b, 0) / fftCorr.length;
        let peakIdx = 0;
        for (let i = 0; i < fftCorr.length; i++) {
            if (fftCorr[i] === maxCorr) {
                peakIdx = i;
                break;
            }
        }

        // === PLOT 1: Coherence Spectrum ===
        drawCoherenceSpectrum(fftCorr, plotHeight, width);

        // === PLOT 2: Cross-Correlation Magnitude ===
        drawMagnitude(correlation, plotHeight, width, peakIdx);

        // === PLOT 3: Group Delay ===
        drawGroupDelay(groupDelay, plotHeight * 2, plotHeight, width);

        // Update metrics display
        updateMetrics(maxCorr, avgCorr, peakIdx);
    }

    function drawCoherenceSpectrum(fftCorr, yOffset, width) {
        const plotHeight = xcorrCanvas.height / 3;

        // Grid
        xcorrCtx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        xcorrCtx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yOffset + (plotHeight / 5) * i;
            xcorrCtx.beginPath();
            xcorrCtx.moveTo(0, y);
            xcorrCtx.lineTo(width, y);
            xcorrCtx.stroke();
        }

        // Coherence threshold line at 0.7
        xcorrCtx.strokeStyle = 'rgba(255, 255, 0, 0.5)';
        xcorrCtx.setLineDash([5, 5]);
        const thresholdY = yOffset + plotHeight * (1 - 0.7);
        xcorrCtx.beginPath();
        xcorrCtx.moveTo(0, thresholdY);
        xcorrCtx.lineTo(width, thresholdY);
        xcorrCtx.stroke();
        xcorrCtx.setLineDash([]);

        // Draw coherence spectrum with gradient
        for (let x = 0; x < width - 1; x++) {
            const idx = Math.floor((x / width) * fftCorr.length);
            const idx2 = Math.floor(((x + 1) / width) * fftCorr.length);
            const coherence1 = Math.min(1, fftCorr[idx]);
            const coherence2 = Math.min(1, fftCorr[idx2]);

            const y1 = yOffset + plotHeight * (1 - coherence1);
            const y2 = yOffset + plotHeight * (1 - coherence2);

            // Color based on coherence value
            const avgCoherence = (coherence1 + coherence2) / 2;
            if (avgCoherence > 0.7) xcorrCtx.strokeStyle = '#00ff00';
            else if (avgCoherence > 0.4) xcorrCtx.strokeStyle = '#ffff00';
            else xcorrCtx.strokeStyle = '#ff0000';

            xcorrCtx.lineWidth = 1.5;
            xcorrCtx.beginPath();
            xcorrCtx.moveTo(x, y1);
            xcorrCtx.lineTo(x + 1, y2);
            xcorrCtx.stroke();
        }

        // Label
        xcorrCtx.fillStyle = '#0ff';
        xcorrCtx.font = 'bold 11px monospace';
        xcorrCtx.fillText('Coherence Spectrum', 5, yOffset + 15);
    }

    function drawMagnitude(correlation, yOffset, width, peakIdx) {
        const plotHeight = xcorrCanvas.height / 3;
        const halfLen = correlation.length / 2;

        // Grid
        xcorrCtx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        xcorrCtx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yOffset + (plotHeight / 5) * i;
            xcorrCtx.beginPath();
            xcorrCtx.moveTo(0, y);
            xcorrCtx.lineTo(width, y);
            xcorrCtx.stroke();
        }

        // Find max for normalization
        let maxVal = Math.max(...correlation);

        // Draw magnitude
        xcorrCtx.strokeStyle = '#00ffff';
        xcorrCtx.lineWidth = 1.5;
        xcorrCtx.beginPath();

        for (let x = 0; x < width; x++) {
            const idx = Math.floor((x / width) * correlation.length);
            const normalized = correlation[idx] / maxVal;
            const y = yOffset + plotHeight * (1 - normalized);

            if (x === 0) xcorrCtx.moveTo(x, y);
            else xcorrCtx.lineTo(x, y);
        }
        xcorrCtx.stroke();

        // Mark peak
        const peakX = (peakIdx / correlation.length) * width;
        const peakY = yOffset + plotHeight * (1 - correlation[peakIdx] / maxVal);
        xcorrCtx.fillStyle = '#ffff00';
        xcorrCtx.beginPath();
        xcorrCtx.arc(peakX, peakY, 4, 0, 2 * Math.PI);
        xcorrCtx.fill();

        // Label
        xcorrCtx.fillStyle = '#0ff';
        xcorrCtx.font = 'bold 11px monospace';
        xcorrCtx.fillText('Cross-Correlation', 5, yOffset + 15);
    }

    function drawGroupDelay(groupDelay, yOffset, plotHeight, width) {
        // Auto-scale
        let minDelay = Math.min(...groupDelay);
        let maxDelay = Math.max(...groupDelay);
        const range = maxDelay - minDelay;
        const padding = range * 0.1;
        minDelay -= padding;
        maxDelay += padding;

        // Grid
        xcorrCtx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        xcorrCtx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yOffset + (plotHeight / 5) * i;
            xcorrCtx.beginPath();
            xcorrCtx.moveTo(0, y);
            xcorrCtx.lineTo(width, y);
            xcorrCtx.stroke();
        }

        // Zero reference line
        if (minDelay < 0 && maxDelay > 0) {
            const zeroY = yOffset + plotHeight * (maxDelay / (maxDelay - minDelay));
            xcorrCtx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            xcorrCtx.setLineDash([3, 3]);
            xcorrCtx.beginPath();
            xcorrCtx.moveTo(0, zeroY);
            xcorrCtx.lineTo(width, zeroY);
            xcorrCtx.stroke();
            xcorrCtx.setLineDash([]);
        }

        // Draw group delay
        xcorrCtx.strokeStyle = '#ff00ff';
        xcorrCtx.lineWidth = 1.5;
        xcorrCtx.beginPath();

        for (let x = 0; x < width; x++) {
            const idx = Math.floor((x / width) * groupDelay.length);
            const normalized = (groupDelay[idx] - minDelay) / (maxDelay - minDelay);
            const y = yOffset + plotHeight * (1 - normalized);

            if (x === 0) xcorrCtx.moveTo(x, y);
            else xcorrCtx.lineTo(x, y);
        }
        xcorrCtx.stroke();

        // Label
        xcorrCtx.fillStyle = '#0ff';
        xcorrCtx.font = 'bold 11px monospace';
        xcorrCtx.fillText('Group Delay (ns)', 5, yOffset + 15);
    }

    function updateMetrics(maxCorr, avgCorr, peakIdx) {
        const coherenceEl = document.getElementById('xcorr_coherence');
        const delayEl = document.getElementById('xcorr_delay');
        const phaseEl = document.getElementById('xcorr_phase');

        if (coherenceEl) coherenceEl.textContent = maxCorr.toFixed(3);
        if (delayEl) delayEl.textContent = `${peakIdx} samples`;
        if (phaseEl) phaseEl.textContent = `${(peakIdx * 360 / XCORR_SIZE).toFixed(1)}°`;
    }

    return {
        init,
        draw
    };
})();
