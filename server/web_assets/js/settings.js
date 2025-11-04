/**
 * Settings Persistence Module
 * Handles saving and loading all application settings to/from localStorage
 */

const Settings = {
    // Storage key prefix
    PREFIX: 'bladerf_',

    // Default settings
    defaults: {
        // LIVE workspace
        live_frequency: 100.0,
        live_samplerate: 40.0,
        live_gain: 30,
        live_bandwidth: 1.5,
        live_fft_size: 4096,
        live_fps_target: 30,
        live_waterfall_intensity: 1.0,
        live_waterfall_contrast: 1.0,
        live_color_palette: 'viridis',
        live_show_spectrum: true,
        live_peak_hold: false,

        // DIRECTION workspace
        direction_antenna_spacing: 0.5,
        direction_phase_offset: 0,
        direction_gain_imbalance: 1.0,
        direction_confidence_threshold: 50,
        direction_show_history: true,
        direction_history_length: 20,

        // SCANNER workspace
        scanner_start_freq: 400,
        scanner_stop_freq: 6000,
        scanner_step: 40,
        scanner_threshold: -80,
        scanner_dwell: 100,

        // Stream out
        stream_endpoint: '127.0.0.1',
        stream_port: 8089,
        stream_protocol: 'udp',
        stream_format: 'cot',
        stream_rate: 1000,
        sensor_lat: 37.7749,
        sensor_lon: -122.4194,
        sensor_alt: 10,

        // General
        notifications_enabled: true,
        notification_duration: 3000,
        auto_reconnect: true,
        keyboard_shortcuts_enabled: true,

        // Session state
        last_workspace: 'live',
        window_zoom_level: 1.0
    },

    /**
     * Save a setting to localStorage
     */
    set(key, value) {
        try {
            const fullKey = this.PREFIX + key;
            localStorage.setItem(fullKey, JSON.stringify(value));
            return true;
        } catch (err) {
            console.error('Failed to save setting:', key, err);
            return false;
        }
    },

    /**
     * Get a setting from localStorage
     */
    get(key, defaultValue = null) {
        try {
            const fullKey = this.PREFIX + key;
            const value = localStorage.getItem(fullKey);

            if (value === null) {
                // Return default if available
                return this.defaults[key] !== undefined ? this.defaults[key] : defaultValue;
            }

            return JSON.parse(value);
        } catch (err) {
            console.error('Failed to load setting:', key, err);
            return this.defaults[key] !== undefined ? this.defaults[key] : defaultValue;
        }
    },

    /**
     * Remove a setting
     */
    remove(key) {
        try {
            const fullKey = this.PREFIX + key;
            localStorage.removeItem(fullKey);
            return true;
        } catch (err) {
            console.error('Failed to remove setting:', key, err);
            return false;
        }
    },

    /**
     * Clear all settings
     */
    clear() {
        try {
            const keys = Object.keys(localStorage);
            keys.forEach(key => {
                if (key.startsWith(this.PREFIX)) {
                    localStorage.removeItem(key);
                }
            });
            return true;
        } catch (err) {
            console.error('Failed to clear settings:', err);
            return false;
        }
    },

    /**
     * Export all settings as JSON
     */
    export() {
        const settings = {};
        const keys = Object.keys(localStorage);

        keys.forEach(key => {
            if (key.startsWith(this.PREFIX)) {
                const shortKey = key.substring(this.PREFIX.length);
                settings[shortKey] = this.get(shortKey);
            }
        });

        return settings;
    },

    /**
     * Import settings from JSON
     */
    import(settings) {
        try {
            Object.keys(settings).forEach(key => {
                this.set(key, settings[key]);
            });
            return true;
        } catch (err) {
            console.error('Failed to import settings:', err);
            return false;
        }
    },

    /**
     * Download settings as JSON file
     */
    downloadConfig() {
        const settings = this.export();
        const json = JSON.stringify(settings, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = `bladerf_config_${new Date().toISOString().split('T')[0]}.json`;
        a.click();

        URL.revokeObjectURL(url);
    },

    /**
     * Load settings from JSON file
     */
    uploadConfig(fileInput) {
        return new Promise((resolve, reject) => {
            const file = fileInput.files[0];
            if (!file) {
                reject(new Error('No file selected'));
                return;
            }

            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const settings = JSON.parse(e.target.result);
                    this.import(settings);
                    resolve(settings);
                } catch (err) {
                    reject(err);
                }
            };
            reader.onerror = () => reject(new Error('Failed to read file'));
            reader.readAsText(file);
        });
    },

    /**
     * Get storage usage info
     */
    getStorageInfo() {
        let totalSize = 0;
        const keys = Object.keys(localStorage);

        keys.forEach(key => {
            if (key.startsWith(this.PREFIX)) {
                totalSize += key.length + localStorage.getItem(key).length;
            }
        });

        return {
            totalBytes: totalSize,
            totalKB: (totalSize / 1024).toFixed(2),
            itemCount: keys.filter(k => k.startsWith(this.PREFIX)).length,
            quotaMB: 5 // localStorage typically has 5MB quota
        };
    }
};

// Make available globally
window.Settings = Settings;

console.log('✓ Settings module loaded');
