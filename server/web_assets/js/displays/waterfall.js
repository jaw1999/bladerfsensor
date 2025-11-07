/**
 * Waterfall Display Module
 * Handles dual-channel waterfall rendering with multiple color palettes
 */

const WaterfallDisplay = (function() {
    'use strict';

    // Module state
    let canvas, ctx, canvas2, ctx2;
    let zoomState = {
        zoomStartBin: 0,
        zoomEndBin: 4095,
        centerFreq: 915000000,
        fullBandwidth: 40000000
    };
    let waterfallIntensity = 1.0;
    let waterfallContrast = 1.0;
    let currentColorPalette = 'viridis';
    let waterfallSpeed = 1;

    // Persistence mode
    let persistenceEnabled = false;
    let persistenceBuffer = null;
    let persistenceDecay = 0.7;

    /**
     * Initialize the waterfall display
     * @param {HTMLCanvasElement} wfCanvas - Primary waterfall canvas
     * @param {HTMLCanvasElement} wfCanvas2 - Secondary waterfall canvas (optional)
     * @param {object} zoom - Zoom state object
     */
    function init(wfCanvas, wfCanvas2, zoom) {
        canvas = wfCanvas;
        ctx = canvas.getContext('2d');
        canvas2 = wfCanvas2;
        ctx2 = canvas2.getContext('2d');

        if (zoom) {
            zoomState = zoom;
        }
    }

    /**
     * Draw waterfall update for single or dual channel
     * @param {Uint8Array} ch1Data - Channel 1 FFT data
     * @param {Uint8Array} ch2Data - Channel 2 FFT data (optional)
     */
    function draw(ch1Data, ch2Data) {
        if (!ch1Data) return;

        // Check if dual-channel mode
        const isDualChannel = (ch2Data !== null && ch2Data !== undefined);

        if (isDualChannel) {
            drawChannel(ch1Data, canvas, ctx);
            drawChannel(ch2Data, canvas2, ctx2);
        } else {
            drawChannel(ch1Data, canvas, ctx);
        }
    }

    /**
     * Draw waterfall for a specific channel
     * @param {Uint8Array} data - FFT data
     * @param {HTMLCanvasElement} cnv - Canvas element
     * @param {CanvasRenderingContext2D} context - Canvas context
     */
    function drawChannel(data, cnv, context) {
        if (!data || !cnv || !context) return;

        // Scroll existing waterfall down
        if (cnv.width > 0 && cnv.height > 1) {
            try {
                context.drawImage(cnv, 0, 0, cnv.width, cnv.height - waterfallSpeed,
                                      0, waterfallSpeed, cnv.width, cnv.height - waterfallSpeed);
            } catch (e) {
                console.warn('Canvas scroll skipped:', e.message);
            }
        }

        // Apply persistence if enabled
        const processedData = persistenceEnabled ? applyPersistence(data) : data;

        // Draw new FFT line at top
        const lineData = context.createImageData(cnv.width, waterfallSpeed);

        for (let y = 0; y < waterfallSpeed; y++) {
            for (let x = 0; x < cnv.width; x++) {
                // Map canvas X to FFT bin, respecting zoom
                const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
                const fftIdx = zoomState.zoomStartBin + Math.floor((x / cnv.width) * zoomedBins);
                let value = processedData[fftIdx];

                // Apply intensity and contrast
                value = value * waterfallIntensity;
                value = 128 + (value - 128) * waterfallContrast;
                value = Math.max(0, Math.min(255, value));

                // Get color from palette
                const mag = value / 255.0;
                const rgb = getColorForValue(mag, currentColorPalette);
                const idx = (y * cnv.width + x) * 4;
                lineData.data[idx + 0] = rgb[0];
                lineData.data[idx + 1] = rgb[1];
                lineData.data[idx + 2] = rgb[2];
                lineData.data[idx + 3] = 255;
            }
        }

        context.putImageData(lineData, 0, 0);
    }

    /**
     * Get color for normalized value based on selected palette
     * @param {number} value - Normalized value [0-1]
     * @param {string} palette - Palette name
     * @returns {Array<number>} RGB array [r, g, b]
     */
    function getColorForValue(value, palette) {
        value = Math.max(0, Math.min(1, value));

        switch(palette) {
            case 'viridis':
                return viridisColor(value);
            case 'plasma':
                return plasmaColor(value);
            case 'inferno':
                return infernoColor(value);
            case 'turbo':
                return turboColor(value);
            case 'hot':
                return hotColor(value);
            case 'cool':
                return coolColor(value);
            case 'grayscale':
                return grayscaleColor(value);
            case 'rainbow':
                return rainbowColor(value);
            default:
                return viridisColor(value);
        }
    }

    /**
     * Viridis colormap (perceptually uniform)
     */
    function viridisColor(value) {
        value = Math.max(0, Math.min(1, value));
        let r, g, b;

        if (value < 0.25) {
            const t = value / 0.25;
            r = 68 + t * (59 - 68);
            g = 1 + t * (82 - 1);
            b = 84 + t * (139 - 84);
        } else if (value < 0.5) {
            const t = (value - 0.25) / 0.25;
            r = 59 + t * (33 - 59);
            g = 82 + t * (145 - 82);
            b = 139 + t * (140 - 139);
        } else if (value < 0.75) {
            const t = (value - 0.5) / 0.25;
            r = 33 + t * (94 - 33);
            g = 145 + t * (201 - 145);
            b = 140 + t * (98 - 140);
        } else {
            const t = (value - 0.75) / 0.25;
            r = 94 + t * (253 - 94);
            g = 201 + t * (231 - 201);
            b = 98 + t * (37 - 98);
        }

        return [Math.floor(r), Math.floor(g), Math.floor(b)];
    }

    /**
     * Plasma colormap
     */
    function plasmaColor(t) {
        const r = Math.floor(255 * (0.05 + 2.4*t - 5.1*t*t + 3.8*t*t*t));
        const g = Math.floor(255 * (0.02 + 1.7*t - 1.9*t*t));
        const b = Math.floor(255 * (0.6 + 1.3*t - 1.6*t*t));
        return [Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b))];
    }

    /**
     * Inferno colormap
     */
    function infernoColor(t) {
        const r = Math.floor(255 * Math.min(1, 1.8*t));
        const g = Math.floor(255 * Math.max(0, 1.5*t - 0.5));
        const b = Math.floor(255 * Math.max(0, 3*t - 2));
        return [r, g, b];
    }

    /**
     * Turbo colormap
     */
    function turboColor(t) {
        const r = Math.floor(255 * (0.13 + 1.5*t - 2.7*t*t + 2.2*t*t*t));
        const g = Math.floor(255 * (-0.01 + 2.6*t - 3.5*t*t + 1.9*t*t*t));
        const b = Math.floor(255 * (0.7 - 1.5*t + 2.3*t*t - 1.5*t*t*t));
        return [Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b))];
    }

    /**
     * Hot colormap
     */
    function hotColor(t) {
        let r, g, b;
        if (t < 0.4) {
            r = 255 * (t / 0.4);
            g = 0;
            b = 0;
        } else if (t < 0.7) {
            r = 255;
            g = 255 * ((t - 0.4) / 0.3);
            b = 0;
        } else {
            r = 255;
            g = 255;
            b = 255 * ((t - 0.7) / 0.3);
        }
        return [Math.floor(r), Math.floor(g), Math.floor(b)];
    }

    /**
     * Cool colormap
     */
    function coolColor(t) {
        const r = Math.floor(255 * t);
        const g = Math.floor(255 * (1 - t));
        const b = 255;
        return [r, g, b];
    }

    /**
     * Grayscale colormap
     */
    function grayscaleColor(t) {
        const v = Math.floor(255 * t);
        return [v, v, v];
    }

    /**
     * Rainbow colormap
     */
    function rainbowColor(t) {
        const h = t * 360;
        const s = 1;
        const v = 1;
        const c = v * s;
        const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
        const m = v - c;
        let r, g, b;
        if (h < 60) { r = c; g = x; b = 0; }
        else if (h < 120) { r = x; g = c; b = 0; }
        else if (h < 180) { r = 0; g = c; b = x; }
        else if (h < 240) { r = 0; g = x; b = c; }
        else if (h < 300) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }
        return [Math.floor((r + m) * 255), Math.floor((g + m) * 255), Math.floor((b + m) * 255)];
    }

    /**
     * Apply persistence effect to data
     * @param {Uint8Array} newData - New FFT data
     * @returns {Uint8Array} Processed data with persistence
     */
    function applyPersistence(newData) {
        if (!persistenceEnabled) {
            return newData;
        }

        // Initialize persistence buffer if needed
        if (!persistenceBuffer || persistenceBuffer.length !== newData.length) {
            persistenceBuffer = new Uint8Array(newData);
            return newData;
        }

        // Create output with persistence applied
        const output = new Uint8Array(newData.length);

        for (let i = 0; i < newData.length; i++) {
            // Decay old values
            persistenceBuffer[i] = Math.floor(persistenceBuffer[i] * persistenceDecay);

            // Take maximum of new data and decayed persistence
            output[i] = Math.max(newData[i], persistenceBuffer[i]);

            // Update persistence buffer with current max
            persistenceBuffer[i] = output[i];
        }

        return output;
    }

    /**
     * Update zoom state
     * @param {object} zoom - New zoom state
     */
    function updateZoomState(zoom) {
        zoomState = zoom;
    }

    /**
     * Set waterfall intensity
     * @param {number} intensity - Intensity multiplier
     */
    function setIntensity(intensity) {
        waterfallIntensity = intensity;
    }

    /**
     * Set waterfall contrast
     * @param {number} contrast - Contrast multiplier
     */
    function setContrast(contrast) {
        waterfallContrast = contrast;
    }

    /**
     * Set color palette
     * @param {string} palette - Palette name
     */
    function setColorPalette(palette) {
        currentColorPalette = palette;
    }

    /**
     * Set waterfall speed
     * @param {number} speed - Speed in pixels per update
     */
    function setSpeed(speed) {
        waterfallSpeed = speed;
    }

    /**
     * Toggle persistence mode
     * @param {boolean} enabled - Enable/disable persistence
     */
    function togglePersistence(enabled) {
        persistenceEnabled = enabled;
        if (!enabled) {
            persistenceBuffer = null;
        }
    }

    /**
     * Set persistence decay rate
     * @param {number} decay - Decay rate [0-1]
     */
    function setPersistenceDecay(decay) {
        persistenceDecay = decay;
    }

    /**
     * Clear persistence buffer
     */
    function clearPersistence() {
        persistenceBuffer = null;
    }

    /**
     * Resize canvases
     * @param {number} width1 - Width of canvas 1
     * @param {number} height1 - Height of canvas 1
     * @param {number} width2 - Width of canvas 2 (optional)
     * @param {number} height2 - Height of canvas 2 (optional)
     */
    function resize(width1, height1, width2, height2) {
        if (canvas && (canvas.width !== width1 || canvas.height !== height1)) {
            canvas.width = width1;
            canvas.height = height1;
        }
        if (canvas2 && width2 && height2 && (canvas2.width !== width2 || canvas2.height !== height2)) {
            canvas2.width = width2;
            canvas2.height = height2;
        }
    }

    // Public API
    return {
        init,
        draw,
        updateZoomState,
        setIntensity,
        setContrast,
        setColorPalette,
        setSpeed,
        togglePersistence,
        setPersistenceDecay,
        clearPersistence,
        resize
    };
})();
