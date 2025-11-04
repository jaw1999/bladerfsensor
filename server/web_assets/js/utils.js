/**
 * Utility Functions Module
 * Common helper functions used throughout the application
 */

const Utils = {
    /**
     * Format frequency for display
     */
    formatFrequency(freqMHz) {
        if (freqMHz >= 1000) {
            return (freqMHz / 1000).toFixed(3) + ' GHz';
        } else {
            return freqMHz.toFixed(3) + ' MHz';
        }
    },

    /**
     * Format file size
     */
    formatFileSize(bytes) {
        const units = ['B', 'KB', 'MB', 'GB', 'TB'];
        let size = bytes;
        let unitIndex = 0;

        while (size >= 1024 && unitIndex < units.length - 1) {
            size /= 1024;
            unitIndex++;
        }

        return size.toFixed(2) + ' ' + units[unitIndex];
    },

    /**
     * Format duration (milliseconds to human readable)
     */
    formatDuration(ms) {
        if (ms < 1000) {
            return ms + 'ms';
        } else if (ms < 60000) {
            return (ms / 1000).toFixed(1) + 's';
        } else if (ms < 3600000) {
            const mins = Math.floor(ms / 60000);
            const secs = Math.floor((ms % 60000) / 1000);
            return mins + 'm ' + secs + 's';
        } else {
            const hours = Math.floor(ms / 3600000);
            const mins = Math.floor((ms % 3600000) / 60000);
            return hours + 'h ' + mins + 'm';
        }
    },

    /**
     * Format timestamp
     */
    formatTimestamp(date = new Date()) {
        return date.toISOString().replace('T', ' ').substring(0, 19);
    },

    /**
     * Clamp value between min and max
     */
    clamp(value, min, max) {
        return Math.min(Math.max(value, min), max);
    },

    /**
     * Linear interpolation
     */
    lerp(a, b, t) {
        return a + (b - a) * t;
    },

    /**
     * Map value from one range to another
     */
    map(value, inMin, inMax, outMin, outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    },

    /**
     * Debounce function calls
     */
    debounce(func, wait) {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    /**
     * Throttle function calls
     */
    throttle(func, limit) {
        let inThrottle;
        return function(...args) {
            if (!inThrottle) {
                func.apply(this, args);
                inThrottle = true;
                setTimeout(() => inThrottle = false, limit);
            }
        };
    },

    /**
     * Generate UUID v4
     */
    generateUUID() {
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
            const r = Math.random() * 16 | 0;
            const v = c === 'x' ? r : (r & 0x3 | 0x8);
            return v.toString(16);
        });
    },

    /**
     * Download data as file
     */
    downloadFile(data, filename, mimeType = 'text/plain') {
        const blob = new Blob([data], { type: mimeType });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);

        URL.revokeObjectURL(url);
    },

    /**
     * Copy text to clipboard
     */
    async copyToClipboard(text) {
        try {
            await navigator.clipboard.writeText(text);
            return true;
        } catch (err) {
            console.error('Failed to copy to clipboard:', err);
            return false;
        }
    },

    /**
     * Parse frequency string (supports MHz, GHz, KHz)
     */
    parseFrequency(str) {
        const cleaned = str.trim().toLowerCase();
        const match = cleaned.match(/^([\d.]+)\s*(ghz|mhz|khz)?$/);

        if (!match) return null;

        const value = parseFloat(match[1]);
        const unit = match[2] || 'mhz';

        switch (unit) {
            case 'ghz':
                return value * 1000;
            case 'mhz':
                return value;
            case 'khz':
                return value / 1000;
            default:
                return value;
        }
    },

    /**
     * Convert degrees to radians
     */
    deg2rad(degrees) {
        return degrees * Math.PI / 180;
    },

    /**
     * Convert radians to degrees
     */
    rad2deg(radians) {
        return radians * 180 / Math.PI;
    },

    /**
     * Normalize angle to [0, 360)
     */
    normalizeAngle(degrees) {
        let angle = degrees % 360;
        if (angle < 0) angle += 360;
        return angle;
    },

    /**
     * Calculate angular difference (shortest path)
     */
    angleDifference(a, b) {
        let diff = (b - a + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    },

    /**
     * Calculate distance between two lat/lon points (Haversine formula)
     */
    haversineDistance(lat1, lon1, lat2, lon2) {
        const R = 6371; // Earth radius in km
        const dLat = this.deg2rad(lat2 - lat1);
        const dLon = this.deg2rad(lon2 - lon1);

        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(this.deg2rad(lat1)) * Math.cos(this.deg2rad(lat2)) *
                  Math.sin(dLon / 2) * Math.sin(dLon / 2);

        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    },

    /**
     * Calculate bearing between two lat/lon points
     */
    calculateBearing(lat1, lon1, lat2, lon2) {
        const dLon = this.deg2rad(lon2 - lon1);
        const y = Math.sin(dLon) * Math.cos(this.deg2rad(lat2));
        const x = Math.cos(this.deg2rad(lat1)) * Math.sin(this.deg2rad(lat2)) -
                  Math.sin(this.deg2rad(lat1)) * Math.cos(this.deg2rad(lat2)) * Math.cos(dLon);

        const bearing = this.rad2deg(Math.atan2(y, x));
        return this.normalizeAngle(bearing);
    },

    /**
     * Create color gradient
     */
    createGradient(ctx, x0, y0, x1, y1, colors) {
        const gradient = ctx.createLinearGradient(x0, y0, x1, y1);

        colors.forEach((color, index) => {
            const stop = index / (colors.length - 1);
            gradient.addColorStop(stop, color);
        });

        return gradient;
    },

    /**
     * RGB to Hex
     */
    rgbToHex(r, g, b) {
        return '#' + [r, g, b].map(x => {
            const hex = x.toString(16);
            return hex.length === 1 ? '0' + hex : hex;
        }).join('');
    },

    /**
     * Hex to RGB
     */
    hexToRgb(hex) {
        const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
        return result ? {
            r: parseInt(result[1], 16),
            g: parseInt(result[2], 16),
            b: parseInt(result[3], 16)
        } : null;
    },

    /**
     * Check if value is numeric and finite
     */
    isValidNumber(value) {
        return typeof value === 'number' && isFinite(value);
    },

    /**
     * Safe element getter (returns null if not found, prevents crashes)
     */
    getElement(id) {
        return document.getElementById(id);
    },

    /**
     * Safe element text setter
     */
    setElementText(id, text) {
        const el = this.getElement(id);
        if (el) el.textContent = text;
    },

    /**
     * Safe element value setter
     */
    setElementValue(id, value) {
        const el = this.getElement(id);
        if (el) el.value = value;
    },

    /**
     * Add tooltip to element
     */
    addTooltip(element, text, position = 'top') {
        element.setAttribute('data-tooltip', text);
        element.setAttribute('data-tooltip-position', position);
        element.classList.add('has-tooltip');
    }
};

// Make available globally
window.Utils = Utils;

console.log('✓ Utils module loaded');
