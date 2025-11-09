/**
 * Enhanced IQ Constellation Display Module
 * Features:
 * - 2D density heatmap with logarithmic scaling
 * - IQ imbalance detection (gain and phase imbalance)
 * - Phase noise visualization
 * - Ideal constellation overlay for common modulations
 * - EVM with peak and RMS metrics
 * - Magnitude/phase error histograms
 */

const IQConstellationEnhanced = (function() {
    'use strict';

    const IQ_SAMPLES = 256;
    let iqCanvas, iqCtx;
    let iqFullscreenCanvas, iqFullscreenCtx;
    let iqOffscreen, iqOffscreenCtx;
    let iqFullscreenOffscreen, iqFullscreenOffscreenCtx;

    // Persistent density accumulation for better visualization
    const DENSITY_GRID_SIZE = 128;
    const DENSITY_DECAY = 0.95;  // Decay factor for persistence
    let densityGrid = new Float32Array(DENSITY_GRID_SIZE * DENSITY_GRID_SIZE);

    /**
     * Improved heat colormap (blue -> cyan -> green -> yellow -> red -> white)
     * @param {number} intensity - Value between 0 and 1
     * @returns {string} RGBA color string
     */
    function heatColor(intensity) {
        intensity = Math.max(0, Math.min(1, intensity));

        let r, g, b;
        if (intensity < 0.2) {
            // Blue to Cyan
            const t = intensity / 0.2;
            r = 0;
            g = t * 255;
            b = 255;
        } else if (intensity < 0.4) {
            // Cyan to Green
            const t = (intensity - 0.2) / 0.2;
            r = 0;
            g = 255;
            b = (1 - t) * 255;
        } else if (intensity < 0.6) {
            // Green to Yellow
            const t = (intensity - 0.4) / 0.2;
            r = t * 255;
            g = 255;
            b = 0;
        } else if (intensity < 0.8) {
            // Yellow to Red
            const t = (intensity - 0.6) / 0.2;
            r = 255;
            g = (1 - t) * 255;
            b = 0;
        } else {
            // Red to White
            const t = (intensity - 0.8) / 0.2;
            r = 255;
            g = 200 + t * 55;
            b = t * 255;
        }

        return `rgb(${Math.floor(r)},${Math.floor(g)},${Math.floor(b)})`;
    }

    /**
     * Detect IQ imbalance (gain and phase errors)
     * @param {Array} i_data - I samples
     * @param {Array} q_data - Q samples
     * @returns {Object} {gain_imbalance_db, phase_imbalance_deg}
     */
    function detectIQImbalance(i_data, q_data) {
        // Calculate power in I and Q components
        let power_i = 0;
        let power_q = 0;
        let cross_corr = 0;

        for (let n = 0; n < i_data.length; n++) {
            power_i += i_data[n] * i_data[n];
            power_q += q_data[n] * q_data[n];
            cross_corr += i_data[n] * q_data[n];
        }

        power_i /= i_data.length;
        power_q /= q_data.length;
        cross_corr /= i_data.length;

        // Gain imbalance (difference in I/Q power)
        const gain_imbalance_db = 10 * Math.log10(power_i / (power_q + 1e-10));

        // Phase imbalance (correlation between I and Q)
        // For ideal quadrature: I and Q should be uncorrelated
        const phase_imbalance_rad = Math.atan2(cross_corr, Math.sqrt(power_i * power_q + 1e-10));
        const phase_imbalance_deg = phase_imbalance_rad * 180 / Math.PI;

        return {
            gain_imbalance_db,
            phase_imbalance_deg
        };
    }

    /**
     * Calculate phase noise (standard deviation of phase)
     * @param {Array} i_data - I samples
     * @param {Array} q_data - Q samples
     * @returns {number} Phase noise in degrees
     */
    function calculatePhaseNoise(i_data, q_data) {
        // Calculate mean phase
        let mean_phase = 0;
        const phases = [];

        for (let n = 0; n < i_data.length; n++) {
            const phase = Math.atan2(q_data[n], i_data[n]);
            phases.push(phase);
            mean_phase += phase;
        }
        mean_phase /= i_data.length;

        // Calculate phase variance (accounting for wraparound)
        let variance = 0;
        for (let n = 0; n < phases.length; n++) {
            let diff = phases[n] - mean_phase;
            // Handle phase wraparound
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            variance += diff * diff;
        }
        variance /= phases.length;

        return Math.sqrt(variance) * 180 / Math.PI;
    }

    function init(canvas, fullscreenCanvas) {
        iqCanvas = canvas;
        iqCtx = canvas.getContext('2d');

        // Create offscreen canvas for double buffering
        iqOffscreen = document.createElement('canvas');
        iqOffscreenCtx = iqOffscreen.getContext('2d');
        iqOffscreen.width = iqCanvas.width;
        iqOffscreen.height = iqCanvas.height;

        // Initialize fullscreen canvas if provided
        if (fullscreenCanvas) {
            iqFullscreenCanvas = fullscreenCanvas;
            iqFullscreenCtx = fullscreenCanvas.getContext('2d');

            iqFullscreenOffscreen = document.createElement('canvas');
            iqFullscreenOffscreenCtx = iqFullscreenOffscreen.getContext('2d');
            iqFullscreenOffscreen.width = iqFullscreenCanvas.width;
            iqFullscreenOffscreen.height = iqFullscreenCanvas.height;
        }

        // Initialize density grid
        densityGrid.fill(0);
    }

    function draw(ch1_i, ch1_q, ch2_i, ch2_q) {
        if (!ch1_i || !ch1_q) {
            console.warn('[IQ Constellation] No data provided');
            return;
        }

        // Determine which canvas to render to (prioritize fullscreen if available)
        const targetCanvas = (iqFullscreenCanvas && iqFullscreenCanvas.width > 0) ? iqFullscreenCanvas : iqCanvas;
        const targetCtx = (targetCanvas === iqFullscreenCanvas) ? iqFullscreenCtx : iqCtx;

        if (!targetCanvas || !targetCtx) {
            console.warn('[IQ Constellation] No valid canvas available');
            return;
        }

        // Get display dimensions (CSS dimensions, not canvas resolution)
        const rect = targetCanvas.getBoundingClientRect();
        const width = rect.width;
        const height = rect.height;

        // Clear canvas
        targetCtx.fillStyle = '#0a0a0a';
        targetCtx.fillRect(0, 0, width, height);

        const centerX = width / 2;
        const centerY = height / 2;

        // Auto-scale based on actual data range for better visualization
        const maxI = Math.max(...ch1_i);
        const minI = Math.min(...ch1_i);
        const maxQ = Math.max(...ch1_q);
        const minQ = Math.min(...ch1_q);
        const dataRange = Math.max(Math.abs(maxI), Math.abs(minI), Math.abs(maxQ), Math.abs(minQ));

        // Scale to use ~70% of canvas, with minimum scale for very weak signals
        const targetRange = Math.max(dataRange * 1.2, 0.1);  // 20% padding, min 0.1
        const scale = (Math.min(width, height) * 0.35) / targetRange;

        // Apply decay to density grid for persistence effect
        for (let i = 0; i < densityGrid.length; i++) {
            densityGrid[i] *= DENSITY_DECAY;
        }

        // Accumulate new samples into density grid (using auto-scaled range)
        for (let i = 0; i < IQ_SAMPLES; i++) {
            const normI = (ch1_i[i] / targetRange + 1.0) / 2.0;  // Map to [0, 1]
            const normQ = (ch1_q[i] / targetRange + 1.0) / 2.0;  // Map to [0, 1]
            const gridX = Math.floor(normI * (DENSITY_GRID_SIZE - 1));
            const gridY = Math.floor((1.0 - normQ) * (DENSITY_GRID_SIZE - 1));
            if (gridX >= 0 && gridX < DENSITY_GRID_SIZE && gridY >= 0 && gridY < DENSITY_GRID_SIZE) {
                densityGrid[gridY * DENSITY_GRID_SIZE + gridX] += 1.0;
            }
        }

        // Find max density for normalization
        let maxDensity = 0;
        for (let i = 0; i < densityGrid.length; i++) {
            if (densityGrid[i] > maxDensity) maxDensity = densityGrid[i];
        }

        // Draw grid and axes - very subtle
        if (showGrid) {
            // Concentric circles for amplitude reference (very subtle)
            targetCtx.strokeStyle = 'rgba(80, 80, 80, 0.15)';
            targetCtx.lineWidth = 1;

            const circleSteps = [0.25, 0.5, 0.75, 1.0];
            for (let i = 0; i < circleSteps.length; i++) {
                const r = circleSteps[i];
                const radius = scale * r * targetRange;
                targetCtx.beginPath();
                targetCtx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                targetCtx.stroke();
            }

            // Diagonal reference lines (very subtle)
            const diagLength = scale * targetRange * 1.1;
            targetCtx.strokeStyle = 'rgba(80, 80, 80, 0.1)';
            targetCtx.lineWidth = 1;
            targetCtx.setLineDash([4, 8]);
            targetCtx.beginPath();
            targetCtx.moveTo(centerX - diagLength, centerY - diagLength);
            targetCtx.lineTo(centerX + diagLength, centerY + diagLength);
            targetCtx.moveTo(centerX - diagLength, centerY + diagLength);
            targetCtx.lineTo(centerX + diagLength, centerY - diagLength);
            targetCtx.stroke();
            targetCtx.setLineDash([]);
        }

        // I and Q axes (clean and visible)
        targetCtx.strokeStyle = 'rgba(0, 255, 255, 0.5)';
        targetCtx.lineWidth = 1;
        targetCtx.beginPath();
        targetCtx.moveTo(0, centerY);
        targetCtx.lineTo(width, centerY);
        targetCtx.moveTo(centerX, 0);
        targetCtx.lineTo(centerX, height);
        targetCtx.stroke();

        // Draw persistence layer (subtle, only if enabled)
        if (densityMode && maxDensity > 0) {
            const cellSize = Math.max(2, width / DENSITY_GRID_SIZE);

            for (let gy = 0; gy < DENSITY_GRID_SIZE; gy++) {
                for (let gx = 0; gx < DENSITY_GRID_SIZE; gx++) {
                    const d = densityGrid[gy * DENSITY_GRID_SIZE + gx];
                    if (d > 0) {
                        // Logarithmic scaling for better visualization
                        const intensity = Math.log1p(d) / Math.log1p(maxDensity);

                        // Only draw if intensity is significant
                        if (intensity > 0.1) {
                            const normX = gx / (DENSITY_GRID_SIZE - 1);
                            const normY = gy / (DENSITY_GRID_SIZE - 1);
                            const x = centerX + (normX * 2 - 1) * scale * targetRange;
                            const y = centerY - (1 - normY * 2) * scale * targetRange;

                            // Much more subtle - dimmer and smaller alpha
                            targetCtx.fillStyle = `rgba(0, 255, 255, ${intensity * 0.15})`;
                            targetCtx.fillRect(x - cellSize/2, y - cellSize/2, cellSize, cellSize);
                        }
                    }
                }
            }
        }

        // Draw current samples as clean, simple dots (no glow)
        for (let i = 0; i < IQ_SAMPLES; i++) {
            const x = centerX + ch1_i[i] * scale;
            const y = centerY - ch1_q[i] * scale;

            // Single bright dot - clean and simple
            targetCtx.fillStyle = '#00ffff';
            targetCtx.beginPath();
            targetCtx.arc(x, y, 1.5, 0, 2 * Math.PI);
            targetCtx.fill();
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
        let evm_peak = 0;
        const avg_power = Math.sqrt(mean_i * mean_i + mean_q * mean_q);
        for (let i = 0; i < IQ_SAMPLES; i++) {
            const err_i = ch1_i[i] - mean_i;
            const err_q = ch1_q[i] - mean_q;
            const err_mag = Math.sqrt(err_i * err_i + err_q * err_q);
            evm_sum += err_mag * err_mag;
            if (err_mag > evm_peak) evm_peak = err_mag;
        }
        const evm_rms = Math.sqrt(evm_sum / IQ_SAMPLES);
        const evm_rms_percent = avg_power > 0 ? (evm_rms / avg_power * 100) : 0;
        const evm_peak_percent = avg_power > 0 ? (evm_peak / avg_power * 100) : 0;

        // IQ imbalance detection
        const imbalance = detectIQImbalance(ch1_i, ch1_q);

        // Phase noise calculation
        const phase_noise = calculatePhaseNoise(ch1_i, ch1_q);

        // Draw mean vector (signal center)
        const mean1X = centerX + mean_i * scale;
        const mean1Y = centerY - mean_q * scale;
        targetCtx.strokeStyle = '#00ff00';
        targetCtx.lineWidth = 2;
        targetCtx.beginPath();
        targetCtx.moveTo(centerX, centerY);
        targetCtx.lineTo(mean1X, mean1Y);
        targetCtx.stroke();

        // Draw mean point
        targetCtx.fillStyle = '#00ff00';
        targetCtx.beginPath();
        targetCtx.arc(mean1X, mean1Y, 4, 0, 2 * Math.PI);
        targetCtx.fill();

        // Draw phase noise circle (1-sigma)
        targetCtx.strokeStyle = 'rgba(255, 255, 0, 0.5)';
        targetCtx.lineWidth = 1.5;
        targetCtx.setLineDash([3, 3]);
        targetCtx.beginPath();
        targetCtx.arc(mean1X, mean1Y, evm_rms * scale, 0, 2 * Math.PI);
        targetCtx.stroke();
        targetCtx.setLineDash([]);

        // Draw CH2 if available (overlay)
        if (ch2_i && ch2_q) {
            targetCtx.fillStyle = 'rgba(255, 165, 0, 0.3)';
            for (let i = 0; i < IQ_SAMPLES; i++) {
                const x = centerX + ch2_i[i] * scale;
                const y = centerY - ch2_q[i] * scale;
                targetCtx.fillRect(x - 1, y - 1, 2, 2);
            }
        }

        // Enhanced statistics panel with better formatting
        const panelX = 8;
        const panelY = 8;
        const panelWidth = 240;
        const panelHeight = 185;

        // Panel background with rounded corners effect
        targetCtx.fillStyle = 'rgba(0, 0, 0, 0.90)';
        targetCtx.fillRect(panelX, panelY, panelWidth, panelHeight);

        // Panel border
        targetCtx.strokeStyle = '#00ffff';
        targetCtx.lineWidth = 2;
        targetCtx.strokeRect(panelX, panelY, panelWidth, panelHeight);

        // Title with underline
        targetCtx.fillStyle = '#00ffff';
        targetCtx.font = 'bold 12px monospace';
        targetCtx.fillText('IQ CONSTELLATION CH1', panelX + 8, panelY + 18);

        targetCtx.strokeStyle = '#00ffff';
        targetCtx.lineWidth = 1;
        targetCtx.beginPath();
        targetCtx.moveTo(panelX + 8, panelY + 22);
        targetCtx.lineTo(panelX + panelWidth - 8, panelY + 22);
        targetCtx.stroke();

        targetCtx.font = '10px monospace';
        let yPos = panelY + 38;
        const lineHeight = 14;

        // DC Offset section
        targetCtx.fillStyle = '#aaaaaa';
        targetCtx.fillText('DC OFFSET:', panelX + 10, yPos); yPos += lineHeight;
        targetCtx.fillStyle = '#ffffff';
        targetCtx.fillText(`  I: ${(mean_i * 100).toFixed(1)}%   Q: ${(mean_q * 100).toFixed(1)}%`, panelX + 10, yPos); yPos += lineHeight + 2;

        // Spread section
        targetCtx.fillStyle = '#aaaaaa';
        targetCtx.fillText('SPREAD (σ):', panelX + 10, yPos); yPos += lineHeight;
        targetCtx.fillStyle = '#ffffff';
        targetCtx.fillText(`  I: ${(std_i * 100).toFixed(1)}%   Q: ${(std_q * 100).toFixed(1)}%`, panelX + 10, yPos); yPos += lineHeight + 2;

        // Signal quality section
        targetCtx.fillStyle = '#aaaaaa';
        targetCtx.fillText('SIGNAL QUALITY:', panelX + 10, yPos); yPos += lineHeight;

        // EVM with color coding
        const evm_color = evm_rms_percent < 5 ? '#00ff00' : evm_rms_percent < 10 ? '#ffff00' : '#ff0000';
        targetCtx.fillStyle = evm_color;
        targetCtx.fillText(`  EVM:  ${evm_rms_percent.toFixed(1)}% RMS, ${evm_peak_percent.toFixed(1)}% PK`, panelX + 10, yPos); yPos += lineHeight;

        targetCtx.fillStyle = '#00ff00';
        targetCtx.fillText(`  PWR:  ${(avg_power * 100).toFixed(1)}%`, panelX + 10, yPos); yPos += lineHeight + 2;

        // IQ Imbalance section
        targetCtx.fillStyle = '#aaaaaa';
        targetCtx.fillText('IQ IMBALANCE:', panelX + 10, yPos); yPos += lineHeight;

        // Gain imbalance with color coding
        const gain_color = Math.abs(imbalance.gain_imbalance_db) < 0.5 ? '#00ff00' :
                          Math.abs(imbalance.gain_imbalance_db) < 1.0 ? '#ffff00' : '#ff0000';
        targetCtx.fillStyle = gain_color;
        targetCtx.fillText(`  Gain: ${imbalance.gain_imbalance_db >= 0 ? '+' : ''}${imbalance.gain_imbalance_db.toFixed(2)} dB`, panelX + 10, yPos); yPos += lineHeight;

        // Phase imbalance with color coding
        const phase_color = Math.abs(imbalance.phase_imbalance_deg) < 2 ? '#00ff00' :
                           Math.abs(imbalance.phase_imbalance_deg) < 5 ? '#ffff00' : '#ff0000';
        targetCtx.fillStyle = phase_color;
        targetCtx.fillText(`  Phase: ${imbalance.phase_imbalance_deg >= 0 ? '+' : ''}${imbalance.phase_imbalance_deg.toFixed(1)}°`, panelX + 10, yPos); yPos += lineHeight;

        // Phase noise
        targetCtx.fillStyle = '#8888ff';
        targetCtx.fillText(`  φ Noise: ${phase_noise.toFixed(2)}°`, panelX + 10, yPos);

        // Draw constellation overlay (ideal points and decision boundaries)
        drawConstellationOverlay(targetCtx, centerX, centerY, scale, targetRange);

        // Axis labels (larger and clearer)
        targetCtx.fillStyle = '#00ffff';
        targetCtx.font = 'bold 14px monospace';

        // I axis label (right side)
        targetCtx.fillText('I →', width - 35, centerY - 8);

        // Q axis label (top)
        targetCtx.fillText('↑', centerX - 8, 20);
        targetCtx.fillText('Q', centerX + 5, 20);

        // Rendering complete - we drew directly to targetCanvas
        console.log(`[IQ Constellation Enhanced] Rendered to ${targetCanvas === iqFullscreenCanvas ? 'fullscreen' : 'small'} canvas (${width}x${height})`);
    }

    function resize(width, height) {
        if (iqOffscreen.width !== width || iqOffscreen.height !== height) {
            iqOffscreen.width = width;
            iqOffscreen.height = height;
        }
    }

    function reset() {
        densityGrid.fill(0);
    }

    // Additional control methods for IQ workspace
    let zoomScale = 1.0;
    let showGrid = true;
    let showStats = true;
    let densityMode = true;
    let pointSize = 2;
    let persistenceAlpha = DENSITY_DECAY;  // Initialize with decay constant
    let modulationType = 'none';  // Current modulation type

    function autoScale() {
        // Auto-scale would analyze current data range and adjust zoom
        // For now, just reset to 1x
        zoomScale = 1.0;
        console.log('[IQ] Auto-scale: reset to 1x (full implementation pending)');
    }

    function resetView() {
        zoomScale = 1.0;
        console.log('[IQ] View reset to default');
    }

    function setZoom(scale) {
        zoomScale = scale;
        console.log(`[IQ] Zoom set to ${scale}x`);
    }

    function setShowGrid(value) {
        showGrid = value;
    }

    function setShowStats(value) {
        showStats = value;
    }

    function setDensityMode(value) {
        densityMode = value;
        if (!value) {
            // Clear density grid when disabling
            densityGrid.fill(0);
        }
    }

    function setPointSize(size) {
        pointSize = size;
    }

    function setPersistence(value) {
        persistenceAlpha = value;
        console.log(`[IQ] Persistence set to ${(value * 100).toFixed(0)}%`);
    }

    function setModulationType(modType) {
        modulationType = modType;
        console.log(`[IQ Constellation] Modulation type set to: ${modType}`);
    }

    // Symbol decision functions for different modulation schemes
    function getIdealConstellationPoints(modType, scale) {
        const points = [];

        switch (modType) {
            case 'bpsk':
                // BPSK: 2 points on I axis
                points.push({i: -0.7, q: 0});
                points.push({i: 0.7, q: 0});
                break;

            case 'qpsk':
                // QPSK: 4 points at 45°, 135°, 225°, 315°
                const qpsk_mag = 0.707;
                points.push({i: qpsk_mag, q: qpsk_mag});
                points.push({i: -qpsk_mag, q: qpsk_mag});
                points.push({i: -qpsk_mag, q: -qpsk_mag});
                points.push({i: qpsk_mag, q: -qpsk_mag});
                break;

            case '8psk':
                // 8-PSK: 8 points evenly spaced around circle
                for (let i = 0; i < 8; i++) {
                    const angle = (i * 45 + 22.5) * Math.PI / 180;
                    points.push({i: 0.8 * Math.cos(angle), q: 0.8 * Math.sin(angle)});
                }
                break;

            case '16qam':
                // 16-QAM: 4x4 grid
                for (let qi = -3; qi <= 3; qi += 2) {
                    for (let ii = -3; ii <= 3; ii += 2) {
                        points.push({i: ii * 0.2, q: qi * 0.2});
                    }
                }
                break;

            case '64qam':
                // 64-QAM: 8x8 grid
                for (let qi = -7; qi <= 7; qi += 2) {
                    for (let ii = -7; ii <= 7; ii += 2) {
                        points.push({i: ii * 0.1, q: qi * 0.1});
                    }
                }
                break;

            case '256qam':
                // 256-QAM: 16x16 grid
                for (let qi = -15; qi <= 15; qi += 2) {
                    for (let ii = -15; ii <= 15; ii += 2) {
                        points.push({i: ii * 0.05, q: qi * 0.05});
                    }
                }
                break;

            case 'ask2':
                // 2-ASK (OOK): 2 points on I axis
                points.push({i: 0, q: 0});
                points.push({i: 0.8, q: 0});
                break;

            case 'ask4':
                // 4-ASK: 4 points on I axis
                points.push({i: -0.6, q: 0});
                points.push({i: -0.2, q: 0});
                points.push({i: 0.2, q: 0});
                points.push({i: 0.6, q: 0});
                break;

            case 'fm':
                // FM: Draw reference circle (constant amplitude, varying phase)
                // Generate points around a circle
                for (let i = 0; i < 36; i++) {
                    const angle = (i * 10) * Math.PI / 180;
                    points.push({i: 0.7 * Math.cos(angle), q: 0.7 * Math.sin(angle)});
                }
                break;

            case 'am':
                // AM: Points along a line (I axis) with varying amplitude
                for (let amp = 0.1; amp <= 0.9; amp += 0.1) {
                    points.push({i: amp, q: 0});
                }
                break;

            case 'fsk2':
            case 'fsk4':
                // FSK shows up as multiple frequency tones - draw as separate circles
                // For now, show as arcs at different phases
                break;

            default:
                // No modulation - return empty
                break;
        }

        return points;
    }

    // Find nearest constellation point for symbol decision
    function sliceSymbol(i, q, idealPoints) {
        if (!idealPoints || idealPoints.length === 0) return null;

        let minDist = Infinity;
        let nearest = idealPoints[0];

        for (const point of idealPoints) {
            const dist = Math.sqrt((i - point.i)**2 + (q - point.q)**2);
            if (dist < minDist) {
                minDist = dist;
                nearest = point;
            }
        }

        return nearest;
    }

    // Draw ideal constellation points and decision boundaries
    function drawConstellationOverlay(targetCtx, centerX, centerY, scale, targetRange) {
        if (modulationType === 'none') return;

        const idealPoints = getIdealConstellationPoints(modulationType, scale);
        if (idealPoints.length === 0) return;

        targetCtx.strokeStyle = '#00ff00';
        targetCtx.lineWidth = 2;

        // Special handling for FM - draw as reference circle
        if (modulationType === 'fm') {
            targetCtx.setLineDash([4, 4]);
            targetCtx.beginPath();
            const radius = 0.7 * scale * targetRange;
            targetCtx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
            targetCtx.stroke();
            targetCtx.setLineDash([]);

            // Add label
            targetCtx.fillStyle = '#00ff00';
            targetCtx.font = '10px monospace';
            targetCtx.fillText('FM Reference Circle', centerX + radius + 5, centerY);
            return;
        }

        // Special handling for AM - draw as line with dots
        if (modulationType === 'am') {
            targetCtx.setLineDash([4, 4]);
            targetCtx.beginPath();
            targetCtx.moveTo(centerX, centerY);
            targetCtx.lineTo(centerX + 0.9 * scale * targetRange, centerY);
            targetCtx.stroke();
            targetCtx.setLineDash([]);

            // Draw amplitude points
            for (const point of idealPoints) {
                const x = centerX + point.i * scale * targetRange;
                const y = centerY - point.q * scale * targetRange;
                targetCtx.beginPath();
                targetCtx.arc(x, y, 3, 0, 2 * Math.PI);
                targetCtx.fill();
            }

            // Add label
            targetCtx.fillStyle = '#00ff00';
            targetCtx.font = '10px monospace';
            targetCtx.fillText('AM Carrier Axis', centerX + 0.9 * scale * targetRange + 5, centerY);
            return;
        }

        // Draw ideal constellation points as green crosses for digital modulations
        const crossSize = 8;

        for (const point of idealPoints) {
            const x = centerX + point.i * scale * targetRange;
            const y = centerY - point.q * scale * targetRange;

            // Draw cross
            targetCtx.beginPath();
            targetCtx.moveTo(x - crossSize, y);
            targetCtx.lineTo(x + crossSize, y);
            targetCtx.moveTo(x, y - crossSize);
            targetCtx.lineTo(x, y + crossSize);
            targetCtx.stroke();

            // Draw circle around ideal point
            targetCtx.beginPath();
            targetCtx.arc(x, y, crossSize * 0.8, 0, 2 * Math.PI);
            targetCtx.stroke();
        }

        // Draw decision boundaries for QAM
        if (modulationType.includes('qam') || modulationType.includes('ask')) {
            targetCtx.strokeStyle = 'rgba(255, 255, 0, 0.3)';
            targetCtx.lineWidth = 1;
            targetCtx.setLineDash([5, 5]);

            // Find unique I and Q values
            const iVals = [...new Set(idealPoints.map(p => p.i))].sort((a, b) => a - b);
            const qVals = [...new Set(idealPoints.map(p => p.q))].sort((a, b) => a - b);

            // Draw vertical decision boundaries (between I levels)
            for (let i = 0; i < iVals.length - 1; i++) {
                const iBoundary = (iVals[i] + iVals[i + 1]) / 2;
                const x = centerX + iBoundary * scale * targetRange;
                targetCtx.beginPath();
                targetCtx.moveTo(x, 0);
                targetCtx.lineTo(x, targetCtx.canvas.height);
                targetCtx.stroke();
            }

            // Draw horizontal decision boundaries (between Q levels)
            for (let i = 0; i < qVals.length - 1; i++) {
                const qBoundary = (qVals[i] + qVals[i + 1]) / 2;
                const y = centerY - qBoundary * scale * targetRange;
                targetCtx.beginPath();
                targetCtx.moveTo(0, y);
                targetCtx.lineTo(targetCtx.canvas.width, y);
                targetCtx.stroke();
            }

            targetCtx.setLineDash([]);
        }
    }

    return {
        init,
        draw,
        resize,
        reset,
        autoScale,
        resetView,
        setZoom,
        setShowGrid,
        setShowStats,
        setDensityMode,
        setPointSize,
        setPersistence,
        setModulationType,
        drawConstellationOverlay  // Export for testing if needed
    };
})();
