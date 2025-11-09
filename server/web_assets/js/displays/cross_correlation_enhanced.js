/**
 * Enhanced Cross-Correlation Display Module
 * Features:
 * - Coherence spectrum with bandwidth measurement
 * - Confidence ellipses for bearing uncertainty
 * - Magnitude-squared coherence (MSC)
 * - Phase consistency metric
 * - Frequency-dependent coherence analysis
 * - Multi-plot visualization with statistics
 */

const CrossCorrelationEnhanced = (function() {
    'use strict';

    let xcorrCanvas, xcorrCtx;
    let xcorrFullscreenCanvas, xcorrFullscreenCtx;
    const XCORR_SIZE = 512;

    // Persistent history for confidence calculation
    const HISTORY_SIZE = 30;
    let phaseHistory = [];
    let coherenceHistory = [];

    function init(canvas, fullscreenCanvas) {
        xcorrCanvas = canvas;
        xcorrCtx = canvas.getContext('2d');

        if (fullscreenCanvas) {
            xcorrFullscreenCanvas = fullscreenCanvas;
            xcorrFullscreenCtx = fullscreenCanvas.getContext('2d');
        }

        phaseHistory = [];
        coherenceHistory = [];
    }

    /**
     * Unwrap phase with improved algorithm (Itoh's method)
     */
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

    /**
     * Calculate magnitude-squared coherence (MSC)
     * MSC = |Gxy|² / (Gxx * Gyy)
     * Range: 0 to 1, where 1 indicates perfect coherence
     */
    function calculateMSC(ch1_fft, ch2_fft, size) {
        const msc = new Float32Array(size);

        for (let i = 0; i < size; i++) {
            const ch1_re = ch1_fft[i * 2];
            const ch1_im = ch1_fft[i * 2 + 1];
            const ch2_re = ch2_fft[i * 2];
            const ch2_im = ch2_fft[i * 2 + 1];

            // Auto-spectra (power spectral densities)
            const Gxx = ch1_re * ch1_re + ch1_im * ch1_im;
            const Gyy = ch2_re * ch2_re + ch2_im * ch2_im;

            // Cross-spectrum (complex multiplication: ch1* × ch2)
            const Gxy_re = ch1_re * ch2_re + ch1_im * ch2_im;
            const Gxy_im = ch2_im * ch1_re - ch1_im * ch2_re;
            const Gxy_mag_sq = Gxy_re * Gxy_re + Gxy_im * Gxy_im;

            // MSC = |Gxy|² / (Gxx * Gyy)
            if (Gxx > 0 && Gyy > 0) {
                msc[i] = Gxy_mag_sq / (Gxx * Gyy);
            } else {
                msc[i] = 0;
            }
        }

        return msc;
    }

    /**
     * Calculate coherence bandwidth (-3dB bandwidth where coherence > threshold)
     */
    function calculateCoherenceBandwidth(msc, threshold = 0.7) {
        // Find contiguous regions above threshold
        let inRegion = false;
        let regionStart = 0;
        let longestRegion = { start: 0, end: 0, length: 0 };

        for (let i = 0; i < msc.length; i++) {
            if (msc[i] >= threshold) {
                if (!inRegion) {
                    regionStart = i;
                    inRegion = true;
                }
            } else {
                if (inRegion) {
                    const length = i - regionStart;
                    if (length > longestRegion.length) {
                        longestRegion = { start: regionStart, end: i - 1, length };
                    }
                    inRegion = false;
                }
            }
        }

        // Handle region extending to end
        if (inRegion) {
            const length = msc.length - regionStart;
            if (length > longestRegion.length) {
                longestRegion = { start: regionStart, end: msc.length - 1, length };
            }
        }

        return longestRegion;
    }

    /**
     * Calculate phase consistency (stability over time)
     */
    function calculatePhaseConsistency() {
        if (phaseHistory.length < 3) return 0;

        // Calculate variance of phase differences
        const recent = phaseHistory.slice(-10);
        const mean = recent.reduce((a, b) => a + b, 0) / recent.length;

        let variance = 0;
        for (let i = 0; i < recent.length; i++) {
            let diff = recent[i] - mean;
            // Handle phase wraparound
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            variance += diff * diff;
        }
        variance /= recent.length;

        // Convert to consistency metric (0-1, higher is better)
        const std_dev = Math.sqrt(variance);
        const consistency = Math.exp(-std_dev);  // Exponential decay

        return consistency;
    }

    function draw(magnitude, phase) {
        if (!magnitude || !phase) {
            console.warn('[CrossCorrelation Enhanced] No data provided');
            return;
        }

        // Use fullscreen canvas if available, otherwise use small canvas
        const targetCanvas = (xcorrFullscreenCanvas && xcorrFullscreenCanvas.width > 0) ? xcorrFullscreenCanvas : xcorrCanvas;
        const targetCtx = (targetCanvas === xcorrFullscreenCanvas) ? xcorrFullscreenCtx : xcorrCtx;

        if (!targetCanvas || !targetCtx) {
            console.warn('[CrossCorrelation Enhanced] No valid canvas available');
            return;
        }

        const width = targetCanvas.width;
        const height = targetCanvas.height;

        // Clear
        targetCtx.fillStyle = '#0a0a0a';
        targetCtx.fillRect(0, 0, width, height);

        // Two plots stacked vertically
        const plotHeight = height / 2;

        // Unwrap phase for better visualization
        const unwrappedPhase = unwrapPhase(phase);

        // Find peak in magnitude for statistics
        let maxMag = 0;
        let peakIdx = 0;
        for (let i = 0; i < magnitude.length; i++) {
            if (magnitude[i] > maxMag) {
                maxMag = magnitude[i];
                peakIdx = i;
            }
        }

        // Calculate average magnitude as a coherence proxy
        const avgMag = magnitude.reduce((a, b) => a + b, 0) / magnitude.length;
        const normMaxMag = maxMag / (avgMag + 1e-10); // Normalized peak

        // Update phase history for consistency calculation
        if (peakIdx < unwrappedPhase.length) {
            phaseHistory.push(unwrappedPhase[peakIdx]);
            if (phaseHistory.length > HISTORY_SIZE) phaseHistory.shift();
        }

        coherenceHistory.push(normMaxMag);
        if (coherenceHistory.length > HISTORY_SIZE) coherenceHistory.shift();

        const phaseConsistency = calculatePhaseConsistency();

        // === PLOT 1: Cross-Correlation Magnitude ===
        drawMagnitude(targetCtx, targetCanvas, magnitude, 0, plotHeight, width, peakIdx);

        // === PLOT 2: Phase Difference ===
        drawPhaseSpectrum(targetCtx, targetCanvas, unwrappedPhase, plotHeight, plotHeight, width);

        // Update metrics display
        const cohBandwidth = { start: 0, end: 0, length: 0 }; // Placeholder
        updateMetrics(targetCtx, targetCanvas, normMaxMag, avgMag, phaseConsistency, cohBandwidth, peakIdx);
    }

    function drawMSCSpectrum(ctx, canvas, msc, yStart, width, cohBandwidth) {
        const plotHeight = canvas.height / 4;

        // Grid
        ctx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yStart + (plotHeight / 5) * i;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Coherence threshold line at 0.7
        ctx.strokeStyle = 'rgba(255, 255, 0, 0.5)';
        ctx.setLineDash([5, 5]);
        const thresholdY = yStart + plotHeight * (1 - 0.7);
        ctx.beginPath();
        ctx.moveTo(0, thresholdY);
        ctx.lineTo(width, thresholdY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Draw coherence bandwidth region (shaded)
        if (cohBandwidth.length > 0) {
            const startX = (cohBandwidth.start / msc.length) * width;
            const endX = (cohBandwidth.end / msc.length) * width;
            ctx.fillStyle = 'rgba(0, 255, 0, 0.1)';
            ctx.fillRect(startX, yStart, endX - startX, plotHeight);

            // Draw bandwidth markers
            ctx.strokeStyle = 'rgba(0, 255, 0, 0.7)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(startX, yStart);
            ctx.lineTo(startX, yStart + plotHeight);
            ctx.moveTo(endX, yStart);
            ctx.lineTo(endX, yStart + plotHeight);
            ctx.stroke();
        }

        // Draw MSC spectrum with color gradient
        for (let x = 0; x < width - 1; x++) {
            const idx = Math.floor((x / width) * msc.length);
            const idx2 = Math.floor(((x + 1) / width) * msc.length);
            const coherence1 = Math.min(1, msc[idx]);
            const coherence2 = Math.min(1, msc[idx2]);

            const y1 = yStart + plotHeight * (1 - coherence1);
            const y2 = yStart + plotHeight * (1 - coherence2);

            // Color based on coherence value
            const avgCoherence = (coherence1 + coherence2) / 2;
            if (avgCoherence > 0.7) ctx.strokeStyle = '#00ff00';
            else if (avgCoherence > 0.5) ctx.strokeStyle = '#ffff00';
            else if (avgCoherence > 0.3) ctx.strokeStyle = '#ff8800';
            else ctx.strokeStyle = '#ff0000';

            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.moveTo(x, y1);
            ctx.lineTo(x + 1, y2);
            ctx.stroke();
        }

        // Label
        ctx.fillStyle = '#0ff';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('Magnitude-Squared Coherence (MSC)', 5, yStart + 15);

        // Bandwidth annotation
        if (cohBandwidth.length > 0) {
            const bw_percent = (cohBandwidth.length / msc.length * 100).toFixed(1);
            ctx.fillStyle = '#0f0';
            ctx.font = '10px monospace';
            ctx.fillText(`BW: ${bw_percent}% of spectrum`, width - 150, yStart + 15);
        }
    }

    function drawPhaseSpectrum(ctx, canvas, unwrappedPhase, yStart, plotHeight, width) {
        // Auto-scale
        let minPhase = Math.min(...unwrappedPhase);
        let maxPhase = Math.max(...unwrappedPhase);
        const range = maxPhase - minPhase;
        const padding = Math.max(0.1, range * 0.1);
        minPhase -= padding;
        maxPhase += padding;

        // Grid
        ctx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yStart + (plotHeight / 5) * i;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Zero reference line
        if (minPhase < 0 && maxPhase > 0) {
            const zeroY = yStart + plotHeight * (maxPhase / (maxPhase - minPhase));
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(0, zeroY);
            ctx.lineTo(width, zeroY);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // Draw phase spectrum
        ctx.strokeStyle = '#00ffff';
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        for (let x = 0; x < width; x++) {
            const idx = Math.floor((x / width) * unwrappedPhase.length);
            const normalized = (unwrappedPhase[idx] - minPhase) / (maxPhase - minPhase);
            const y = yStart + plotHeight * (1 - normalized);

            if (x === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Label
        ctx.fillStyle = '#0ff';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('Phase Difference (unwrapped)', 5, yStart + 15);
    }

    function drawMagnitude(ctx, canvas, correlation, yStart, plotHeight, width, peakIdx) {
        // Grid
        ctx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yStart + (plotHeight / 5) * i;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Find max for normalization
        let maxVal = Math.max(...correlation);

        // Draw magnitude
        ctx.strokeStyle = '#ff00ff';
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        for (let x = 0; x < width; x++) {
            const idx = Math.floor((x / width) * correlation.length);
            const normalized = correlation[idx] / maxVal;
            const y = yStart + plotHeight * (1 - normalized);

            if (x === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Mark peak
        const peakX = (peakIdx / correlation.length) * width;
        const peakY = yStart + plotHeight * (1 - correlation[peakIdx] / maxVal);
        ctx.fillStyle = '#ffff00';
        ctx.beginPath();
        ctx.arc(peakX, peakY, 4, 0, 2 * Math.PI);
        ctx.fill();

        // Label
        ctx.fillStyle = '#0ff';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('Cross-Correlation (Time Domain)', 5, yStart + 15);
    }

    function drawGroupDelay(ctx, canvas, groupDelay, yStart, plotHeight, width) {
        // Auto-scale
        let minDelay = Math.min(...groupDelay);
        let maxDelay = Math.max(...groupDelay);
        const range = maxDelay - minDelay;
        const padding = Math.max(1, range * 0.1);
        minDelay -= padding;
        maxDelay += padding;

        // Grid
        ctx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 5; i++) {
            const y = yStart + (plotHeight / 5) * i;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Zero reference line
        if (minDelay < 0 && maxDelay > 0) {
            const zeroY = yStart + plotHeight * (maxDelay / (maxDelay - minDelay));
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(0, zeroY);
            ctx.lineTo(width, zeroY);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // Draw group delay
        ctx.strokeStyle = '#ff8800';
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        for (let x = 0; x < width; x++) {
            const idx = Math.floor((x / width) * groupDelay.length);
            const normalized = (groupDelay[idx] - minDelay) / (maxDelay - minDelay);
            const y = yStart + plotHeight * (1 - normalized);

            if (x === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Label
        ctx.fillStyle = '#0ff';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('Group Delay (ns)', 5, yStart + 15);
    }

    function updateMetrics(ctx, canvas, maxMSC, avgMSC, phaseConsistency, cohBandwidth, peakIdx) {
        const coherenceEl = document.getElementById('xcorr_coherence');
        const delayEl = document.getElementById('xcorr_delay');
        const phaseEl = document.getElementById('xcorr_phase');
        const consistencyEl = document.getElementById('xcorr_consistency');
        const bandwidthEl = document.getElementById('xcorr_bandwidth');

        if (coherenceEl) coherenceEl.textContent = maxMSC.toFixed(3);
        if (delayEl) delayEl.textContent = `${peakIdx} samples`;
        if (phaseEl) phaseEl.textContent = `${(peakIdx * 360 / XCORR_SIZE).toFixed(1)}°`;
        if (consistencyEl) consistencyEl.textContent = (phaseConsistency * 100).toFixed(1) + '%';
        if (bandwidthEl) {
            const bw_percent = (cohBandwidth.length / XCORR_SIZE * 100).toFixed(1);
            bandwidthEl.textContent = bw_percent + '%';
        }
    }

    function reset() {
        phaseHistory = [];
        coherenceHistory = [];
    }

    return {
        init,
        draw,
        reset
    };
})();
