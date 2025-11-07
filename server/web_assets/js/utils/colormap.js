/**
 * Colormap Utilities
 * Provides color mapping functions for visualization
 */

const Colormap = (function() {
    'use strict';

    /**
     * Viridis colormap - perceptually uniform
     * @param {number} value - Normalized value [0-1]
     * @returns {string} RGB color string
     */
    function viridis(value) {
        value = Math.max(0, Math.min(1, value));

        const r = Math.floor(255 * (0.267004 + value * (0.004874 + value * (0.329415 + value * (-0.026217)))));
        const g = Math.floor(255 * (0.004874 + value * (0.513480 + value * (0.390671 + value * (-0.154580)))));
        const b = Math.floor(255 * (0.329415 + value * (0.543699 + value * (-0.449730 + value * 0.131607))));

        return `rgb(${r},${g},${b})`;
    }

    /**
     * Heat colormap - blue to cyan to green to yellow to red
     * @param {number} value - Normalized value [0-1]
     * @returns {string} RGB color string
     */
    function heat(value) {
        value = Math.max(0, Math.min(1, value));

        let r, g, b;
        if (value < 0.25) {
            // Blue to Cyan
            const t = value / 0.25;
            r = 0;
            g = Math.floor(255 * t);
            b = 255;
        } else if (value < 0.5) {
            // Cyan to Green
            const t = (value - 0.25) / 0.25;
            r = 0;
            g = 255;
            b = Math.floor(255 * (1 - t));
        } else if (value < 0.75) {
            // Green to Yellow
            const t = (value - 0.5) / 0.25;
            r = Math.floor(255 * t);
            g = 255;
            b = 0;
        } else {
            // Yellow to Red
            const t = (value - 0.75) / 0.25;
            r = 255;
            g = Math.floor(255 * (1 - t));
            b = 0;
        }

        return `rgb(${r},${g},${b})`;
    }

    /**
     * Grayscale colormap
     * @param {number} value - Normalized value [0-1]
     * @returns {string} RGB color string
     */
    function grayscale(value) {
        value = Math.max(0, Math.min(1, value));
        const gray = Math.floor(255 * value);
        return `rgb(${gray},${gray},${gray})`;
    }

    /**
     * Apply intensity and contrast adjustments
     * @param {number} value - Input value [0-255]
     * @param {number} intensity - Intensity multiplier
     * @param {number} contrast - Contrast multiplier
     * @returns {number} Adjusted value [0-255]
     */
    function applyAdjustments(value, intensity, contrast) {
        value = value * intensity;
        value = 128 + (value - 128) * contrast;
        return Math.max(0, Math.min(255, value));
    }

    // Public API
    return {
        viridis,
        heat,
        grayscale,
        applyAdjustments
    };
})();
