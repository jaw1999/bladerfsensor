/**
 * IQ Constellation Display Module
 * Enhanced visualization with density heatmap and EVM metrics
 */

const IQConstellation = (function() {
    'use strict';

    const IQ_SAMPLES = 256;
    let iqCanvas, iqCtx;
    let iqOffscreen, iqOffscreenCtx;

    /**
     * Heat colormap for density visualization
     * @param {number} intensity - Value between 0 and 1
     * @returns {string} RGBA color string
     */
    function heatColor(intensity) {
        intensity = Math.max(0, Math.min(1, intensity));
        const alpha = intensity * 0.8;

        // Heat colormap: blue -> cyan -> green -> yellow -> red
        let r, g, b;
        if (intensity < 0.25) {
            r = 0;
            g = intensity * 4 * 255;
            b = 255;
        } else if (intensity < 0.5) {
            r = 0;
            g = 255;
            b = (0.5 - intensity) * 4 * 255;
        } else if (intensity < 0.75) {
            r = (intensity - 0.5) * 4 * 255;
            g = 255;
            b = 0;
        } else {
            r = 255;
            g = (1 - intensity) * 4 * 255;
            b = 0;
        }

        return `rgba(${Math.floor(r)},${Math.floor(g)},${Math.floor(b)},${alpha})`;
    }

    function init(canvas) {
        iqCanvas = canvas;
        iqCtx = canvas.getContext('2d');

        // Create offscreen canvas for double buffering
        iqOffscreen = document.createElement('canvas');
        iqOffscreenCtx = iqOffscreen.getContext('2d');
        iqOffscreen.width = iqCanvas.width;
        iqOffscreen.height = iqCanvas.height;
    }

    function draw(ch1_i, ch1_q, ch2_i, ch2_q) {
        if (!ch1_i || !ch1_q) return;

        const width = iqOffscreen.width;
        const height = iqOffscreen.height;

        // Clear
        iqOffscreenCtx.fillStyle = '#0a0a0a';
        iqOffscreenCtx.fillRect(0, 0, width, height);

        const centerX = width / 2;
        const centerY = height / 2;
        const scale = Math.min(width, height) * 0.35;

        // Draw grid and axes
        iqOffscreenCtx.strokeStyle = 'rgba(80, 80, 80, 0.3)';
        iqOffscreenCtx.lineWidth = 1;

        // Circles for amplitude reference (polar grid)
        for (let r = 0.25; r <= 1.0; r += 0.25) {
            iqOffscreenCtx.beginPath();
            iqOffscreenCtx.arc(centerX, centerY, scale * r, 0, 2 * Math.PI);
            iqOffscreenCtx.stroke();

            // dB labels
            const dbValue = 20 * Math.log10(r);
            iqOffscreenCtx.fillStyle = '#666';
            iqOffscreenCtx.font = '9px monospace';
            iqOffscreenCtx.fillText(`${dbValue.toFixed(0)}dB`, centerX + scale * r + 3, centerY - 2);
        }

        // I and Q axes
        iqOffscreenCtx.strokeStyle = 'rgba(0, 255, 255, 0.5)';
        iqOffscreenCtx.lineWidth = 1.5;
        iqOffscreenCtx.beginPath();
        iqOffscreenCtx.moveTo(0, centerY);
        iqOffscreenCtx.lineTo(width, centerY);
        iqOffscreenCtx.moveTo(centerX, 0);
        iqOffscreenCtx.lineTo(centerX, height);
        iqOffscreenCtx.stroke();

        // Calculate density heatmap for CH1
        const gridSize = 64;
        const density1 = new Uint32Array(gridSize * gridSize);

        for (let i = 0; i < IQ_SAMPLES; i++) {
            const gridX = Math.floor((ch1_i[i] + 1.0) / 2.0 * (gridSize - 1));
            const gridY = Math.floor((1.0 - ch1_q[i]) / 2.0 * (gridSize - 1));
            if (gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize) {
                density1[gridY * gridSize + gridX]++;
            }
        }

        const maxDensity = Math.max(...density1);

        // Draw density heatmap
        for (let gy = 0; gy < gridSize; gy++) {
            for (let gx = 0; gx < gridSize; gx++) {
                const d = density1[gy * gridSize + gx];
                if (d > 0) {
                    const intensity = Math.log1p(d) / Math.log1p(maxDensity);
                    const x = centerX + (gx / (gridSize - 1) * 2 - 1) * scale;
                    const y = centerY - ((gridSize - 1 - gy) / (gridSize - 1) * 2 - 1) * scale;
                    const size = Math.max(2, width / gridSize);

                    iqOffscreenCtx.fillStyle = heatColor(intensity);
                    iqOffscreenCtx.fillRect(x - size/2, y - size/2, size, size);
                }
            }
        }

        // Calculate statistics for CH1
        let sum_i = 0, sum_q = 0, sum_i2 = 0, sum_q2 = 0;
        for (let i = 0; i < IQ_SAMPLES; i++) {
            sum_i += ch1_i[i];
            sum_q += ch1_q[i];
            sum_i2 += ch1_i[i] * ch1_i[i];
            sum_q2 += ch1_q[i] * ch1_q[i];
        }

        const mean_i = sum_i / IQ_SAMPLES;
        const mean_q = sum_q / IQ_SAMPLES;
        const variance_i = (sum_i2 / IQ_SAMPLES) - (mean_i * mean_i);
        const variance_q = (sum_q2 / IQ_SAMPLES) - (mean_q * mean_q);
        const std_i = Math.sqrt(Math.max(0, variance_i));
        const std_q = Math.sqrt(Math.max(0, variance_q));

        // EVM calculation
        let evm_sum = 0;
        const avg_power = Math.sqrt(mean_i * mean_i + mean_q * mean_q);
        for (let i = 0; i < IQ_SAMPLES; i++) {
            const err_i = ch1_i[i] - mean_i;
            const err_q = ch1_q[i] - mean_q;
            evm_sum += err_i * err_i + err_q * err_q;
        }
        const evm_rms = Math.sqrt(evm_sum / IQ_SAMPLES);
        const evm_percent = avg_power > 0 ? (evm_rms / avg_power * 100) : 0;

        // Draw mean vector for CH1
        const mean1X = centerX + mean_i * scale;
        const mean1Y = centerY - mean_q * scale;
        iqOffscreenCtx.strokeStyle = '#00ff00';
        iqOffscreenCtx.lineWidth = 2;
        iqOffscreenCtx.beginPath();
        iqOffscreenCtx.moveTo(centerX, centerY);
        iqOffscreenCtx.lineTo(mean1X, mean1Y);
        iqOffscreenCtx.stroke();

        // Draw CH2 if available
        if (ch2_i && ch2_q) {
            iqOffscreenCtx.fillStyle = 'rgba(255, 165, 0, 0.4)';
            for (let i = 0; i < IQ_SAMPLES; i++) {
                const x = centerX + ch2_i[i] * scale;
                const y = centerY - ch2_q[i] * scale;
                iqOffscreenCtx.fillRect(x - 0.5, y - 0.5, 1, 1);
            }
        }

        // Statistics panel
        iqOffscreenCtx.fillStyle = 'rgba(0, 0, 0, 0.7)';
        iqOffscreenCtx.fillRect(5, 5, 180, 110);
        iqOffscreenCtx.strokeStyle = '#0ff';
        iqOffscreenCtx.lineWidth = 1;
        iqOffscreenCtx.strokeRect(5, 5, 180, 110);

        iqOffscreenCtx.fillStyle = '#0ff';
        iqOffscreenCtx.font = 'bold 11px monospace';
        iqOffscreenCtx.fillText('CH1 Statistics', 10, 20);

        iqOffscreenCtx.fillStyle = '#fff';
        iqOffscreenCtx.font = '10px monospace';
        let yPos = 35;
        iqOffscreenCtx.fillText(`Mean I: ${mean_i.toFixed(3)}`, 10, yPos); yPos += 12;
        iqOffscreenCtx.fillText(`Mean Q: ${mean_q.toFixed(3)}`, 10, yPos); yPos += 12;
        iqOffscreenCtx.fillText(`Std I:  ${std_i.toFixed(3)}`, 10, yPos); yPos += 12;
        iqOffscreenCtx.fillText(`Std Q:  ${std_q.toFixed(3)}`, 10, yPos); yPos += 12;
        iqOffscreenCtx.fillStyle = '#ff0';
        iqOffscreenCtx.fillText(`EVM:    ${evm_percent.toFixed(2)}%`, 10, yPos); yPos += 12;
        iqOffscreenCtx.fillStyle = '#0f0';
        iqOffscreenCtx.fillText(`Power:  ${(avg_power * 100).toFixed(1)}%`, 10, yPos);

        // Labels
        iqOffscreenCtx.fillStyle = '#0ff';
        iqOffscreenCtx.font = 'bold 12px monospace';
        iqOffscreenCtx.fillText('I', width - 15, centerY - 5);
        iqOffscreenCtx.fillText('Q', centerX + 5, 15);

        // Copy to visible canvas
        iqCtx.drawImage(iqOffscreen, 0, 0);
    }

    function resize(width, height) {
        if (iqOffscreen.width !== width || iqOffscreen.height !== height) {
            iqOffscreen.width = width;
            iqOffscreen.height = height;
        }
    }

    return {
        init,
        draw,
        resize
    };
})();
