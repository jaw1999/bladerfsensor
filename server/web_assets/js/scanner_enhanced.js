/**
 * Enhanced Scanner Workspace Module
 * Comprehensive frequency scanner with export, filtering, and scheduling
 */

const Scanner = {
    // State
    state: {
        scanning: false,
        startFreq: 0,
        stopFreq: 0,
        step: 0,
        dwellMs: 0,
        totalSteps: 0,
        startTime: 0,
        signals: [],
        scanHistory: []
    },

    // Update interval
    updateInterval: null,

    // Scheduled scan state
    scheduledScan: {
        enabled: false,
        interval: null,
        intervalMs: 3600000, // 1 hour default
        lastScan: 0
    },

    // Filter state
    filter: {
        minPower: -120,
        maxPower: 0,
        minFreq: 0,
        maxFreq: 10000,
        minBandwidth: 0,
        sortBy: 'power' // 'power', 'frequency', 'bandwidth', 'hits'
    },

    /**
     * Initialize scanner
     */
    init() {
        console.log('Initializing enhanced scanner...');

        // Load saved settings
        this.loadSettings();

        // Setup event listeners
        this.setupEventListeners();

        // Restore scan history from localStorage
        this.loadScanHistory();

        console.log('✓ Scanner initialized');
    },

    /**
     * Load settings from localStorage
     */
    loadSettings() {
        Utils.setElementValue('scan_start', Settings.get('scanner_start_freq', 400));
        Utils.setElementValue('scan_stop', Settings.get('scanner_stop_freq', 6000));
        Utils.setElementValue('scan_step', Settings.get('scanner_step', 40));
        Utils.setElementValue('scan_threshold', Settings.get('scanner_threshold', -80));
        Utils.setElementValue('scan_dwell', Settings.get('scanner_dwell', 100));
    },

    /**
     * Save settings to localStorage
     */
    saveSettings() {
        Settings.set('scanner_start_freq', parseInt(document.getElementById('scan_start').value));
        Settings.set('scanner_stop_freq', parseInt(document.getElementById('scan_stop').value));
        Settings.set('scanner_step', parseInt(document.getElementById('scan_step').value));
        Settings.set('scanner_threshold', parseInt(document.getElementById('scan_threshold').value));
        Settings.set('scanner_dwell', parseInt(document.getElementById('scan_dwell').value));
    },

    /**
     * Setup event listeners
     */
    setupEventListeners() {
        // Filter controls
        const filterBtn = Utils.getElement('scan_filter_btn');
        if (filterBtn) {
            filterBtn.addEventListener('click', () => this.showFilterDialog());
        }

        // Export button
        const exportBtn = Utils.getElement('scan_export_btn');
        if (exportBtn) {
            exportBtn.addEventListener('click', () => this.exportResults());
        }

        // Schedule button
        const scheduleBtn = Utils.getElement('scan_schedule_btn');
        if (scheduleBtn) {
            scheduleBtn.addEventListener('click', () => this.showScheduleDialog());
        }
    },

    /**
     * Start scanner with validation
     */
    async start() {
        const params = {
            start_freq: parseInt(document.getElementById('scan_start').value),
            stop_freq: parseInt(document.getElementById('scan_stop').value),
            step: parseInt(document.getElementById('scan_step').value),
            threshold: parseInt(document.getElementById('scan_threshold').value),
            dwell_ms: parseInt(document.getElementById('scan_dwell').value)
        };

        // Validate parameters
        if (!this.validateParams(params)) {
            return;
        }

        // Save settings
        this.saveSettings();

        // Store scanner parameters for progress calculation
        this.state.startFreq = params.start_freq;
        this.state.stopFreq = params.stop_freq;
        this.state.step = params.step;
        this.state.dwellMs = params.dwell_ms;
        this.state.totalSteps = Math.ceil((params.stop_freq - params.start_freq) / params.step);
        this.state.startTime = Date.now();
        this.state.scanning = true;

        try {
            const response = await fetchWithTimeout('/start_scanner', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(params)
            });
            const data = await response.json();

            if (data.status === 'ok') {
                Utils.getElement('scan_start_btn').style.display = 'none';
                Utils.getElement('scan_stop_btn').style.display = 'block';
                Utils.setElementText('scan_status', 'Scanning...');
                Utils.getElement('scan_status').style.color = '#0f0';

                // Start polling for updates
                this.updateInterval = setInterval(() => this.updateStatus(), 500);
                showNotification('Scanner started successfully', 'success', 2000);
            } else {
                showNotification(`Failed to start scanner: ${data.error || 'Unknown error'}`, 'error');
                this.state.scanning = false;
            }
        } catch (err) {
            console.error('Scanner start error:', err);
            showNotification(`Failed to start scanner: ${err.message}`, 'error');
            this.state.scanning = false;
        }
    },

    /**
     * Validate scan parameters
     */
    validateParams(params) {
        if (isNaN(params.start_freq) || isNaN(params.stop_freq) || isNaN(params.step) ||
            isNaN(params.threshold) || isNaN(params.dwell_ms)) {
            showNotification('Invalid scanner parameters. Please check all fields.', 'error');
            return false;
        }

        if (params.start_freq >= params.stop_freq) {
            showNotification('Start frequency must be less than stop frequency', 'error');
            return false;
        }

        if (params.step <= 0 || params.step > (params.stop_freq - params.start_freq)) {
            showNotification('Invalid step size. Must be positive and less than scan range.', 'error');
            return false;
        }

        if (params.dwell_ms < 10) {
            showNotification('Dwell time too short. Minimum is 10ms.', 'error');
            return false;
        }

        return true;
    },

    /**
     * Stop scanner
     */
    async stop() {
        try {
            const response = await fetchWithTimeout('/stop_scanner', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            await response.json();

            this.stopUI();
            showNotification('Scanner stopped', 'info', 2000);
        } catch (err) {
            console.error('Scanner stop error:', err);
            showNotification(`Failed to stop scanner: ${err.message}`, 'error');
            this.stopUI();
        }
    },

    /**
     * Stop scanner UI (called from stop or when server reports stopped)
     */
    stopUI() {
        Utils.getElement('scan_start_btn').style.display = 'block';
        Utils.getElement('scan_stop_btn').style.display = 'none';
        Utils.setElementText('scan_status', 'Stopped');
        Utils.getElement('scan_status').style.color = '#888';

        // Stop polling
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }

        this.state.scanning = false;
    },

    /**
     * Clear scanner results
     */
    async clear() {
        try {
            const response = await fetchWithTimeout('/clear_scanner', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            await response.json();

            Utils.getElement('scanner_signal_list').innerHTML = '<div style="color: #888; text-align: center; padding: 20px;">No signals detected</div>';
            Utils.setElementText('scan_count', '0');
            Utils.setElementText('scan_signal_count', '0');

            // Reset progress indicators
            Utils.getElement('scan_progress_bar').style.width = '0%';
            Utils.setElementText('scan_progress_pct', '0%');
            Utils.setElementText('scan_eta', '--');

            this.state.signals = [];

            showNotification('Scanner cleared', 'info', 2000);
        } catch (err) {
            console.error('Scanner clear error:', err);
            showNotification(`Failed to clear scanner: ${err.message}`, 'error');
        }
    },

    /**
     * Update scanner status (called periodically)
     */
    async updateStatus() {
        try {
            const response = await fetchWithTimeout('/scanner_status');
            const data = await response.json();

            // Update status displays
            Utils.setElementText('scan_current_freq', data.current_freq.toFixed(1));
            Utils.setElementText('scan_count', data.scan_count);
            Utils.setElementText('scan_signal_count', data.signals.length);

            // Store signals
            this.state.signals = data.signals;

            // Update progress
            this.updateProgress(data);

            // Update signal list with filtering
            this.updateSignalList(data.signals);

            // Check if stopped
            if (!data.scanning && this.updateInterval) {
                this.handleScanComplete(data);
            }
        } catch (err) {
            console.error('Scanner status update error:', err);
        }
    },

    /**
     * Update progress indicators
     */
    updateProgress(data) {
        if (data.scanning && this.state.totalSteps > 0) {
            const currentStep = Math.floor((data.current_freq - this.state.startFreq) / this.state.step);
            const progressPct = Math.min(100, Math.max(0, (currentStep / this.state.totalSteps) * 100));

            Utils.getElement('scan_progress_bar').style.width = progressPct.toFixed(1) + '%';
            Utils.setElementText('scan_progress_pct', progressPct.toFixed(0) + '%');

            // Calculate ETA
            const elapsedMs = Date.now() - this.state.startTime;
            if (progressPct > 5) {
                const totalEstimatedMs = (elapsedMs / progressPct) * 100;
                const remainingMs = totalEstimatedMs - elapsedMs;

                if (remainingMs > 0) {
                    Utils.setElementText('scan_eta', Utils.formatDuration(remainingMs));
                } else {
                    Utils.setElementText('scan_eta', 'Soon');
                }
            } else {
                Utils.setElementText('scan_eta', 'Calculating...');
            }
        } else {
            Utils.getElement('scan_progress_bar').style.width = '0%';
            Utils.setElementText('scan_progress_pct', '0%');
            Utils.setElementText('scan_eta', '--');
        }
    },

    /**
     * Update signal list with filtering and sorting
     */
    updateSignalList(signals) {
        const listDiv = Utils.getElement('scanner_signal_list');
        if (!listDiv) return;

        // Apply filters
        let filtered = this.applyFilters(signals);

        // Sort
        filtered = this.sortSignals(filtered);

        if (filtered.length === 0) {
            listDiv.innerHTML = '<div style="color: #888; text-align: center; padding: 20px;">No signals match filter criteria</div>';
            return;
        }

        // Build HTML
        let html = '<div style="display: grid; grid-template-columns: 1fr auto auto auto auto; gap: 8px; padding-bottom: 5px; border-bottom: 1px solid #333; font-weight: bold; color: #0ff;">';
        html += '<div>Frequency</div><div>Power</div><div>BW</div><div>Hits</div><div>Action</div></div>';

        filtered.forEach((sig, index) => {
            html += `<div style="display: grid; grid-template-columns: 1fr auto auto auto auto; gap: 8px; padding: 5px; border-bottom: 1px solid #222;" data-sig-index="${index}">`;
            html += `<div style="color: #0ff;">${sig.freq.toFixed(3)} MHz</div>`;
            html += `<div style="color: ${this.getPowerColor(sig.power)};">${sig.power.toFixed(1)} dBm</div>`;
            html += `<div style="color: #888;">${(sig.bw / 1000).toFixed(1)} kHz</div>`;
            html += `<div style="color: #888;">${sig.hits}</div>`;
            html += `<div><button onclick="Scanner.tuneToSignal(${sig.freq})" style="padding: 2px 6px; font-size: 10px; background: #0a0; border: none; color: #fff; border-radius: 2px; cursor: pointer;">TUNE</button></div>`;
            html += '</div>';
        });

        listDiv.innerHTML = html;
    },

    /**
     * Get color based on power level
     */
    getPowerColor(power) {
        if (power > -60) return '#0f0';
        if (power > -80) return '#ff0';
        return '#888';
    },

    /**
     * Apply filters to signal list
     */
    applyFilters(signals) {
        return signals.filter(sig => {
            return sig.power >= this.filter.minPower &&
                   sig.power <= this.filter.maxPower &&
                   sig.freq >= this.filter.minFreq &&
                   sig.freq <= this.filter.maxFreq &&
                   sig.bw >= this.filter.minBandwidth;
        });
    },

    /**
     * Sort signals
     */
    sortSignals(signals) {
        const sorted = [...signals];

        switch (this.filter.sortBy) {
            case 'power':
                sorted.sort((a, b) => b.power - a.power);
                break;
            case 'frequency':
                sorted.sort((a, b) => a.freq - b.freq);
                break;
            case 'bandwidth':
                sorted.sort((a, b) => b.bw - a.bw);
                break;
            case 'hits':
                sorted.sort((a, b) => b.hits - a.hits);
                break;
        }

        return sorted;
    },

    /**
     * Tune LIVE workspace to specific frequency
     */
    tuneToSignal(freqMHz) {
        // Switch to LIVE workspace
        if (typeof switchWorkspace === 'function') {
            switchWorkspace('live');
        }

        // Set frequency
        const freqInput = Utils.getElement('frequency');
        if (freqInput) {
            freqInput.value = freqMHz.toFixed(3);

            // Trigger frequency update
            if (typeof updateConfig === 'function') {
                setTimeout(() => updateConfig(), 100);
            }
        }

        showNotification(`Tuned to ${freqMHz.toFixed(3)} MHz`, 'success', 2000);
    },

    /**
     * Handle scan completion
     */
    handleScanComplete(data) {
        clearInterval(this.updateInterval);
        this.updateInterval = null;
        Utils.setElementText('scan_status', 'Complete');
        Utils.getElement('scan_status').style.color = '#0f0';

        // Save to history
        this.saveScanToHistory(data);

        // Show completion notification
        const currentFreq = data.current_freq;
        if (this.state.stopFreq > 0 && Math.abs(currentFreq - this.state.stopFreq) < this.state.step * 2) {
            showNotification(`Scan complete! Found ${data.signals.length} signals.`, 'success', 4000);
        }

        // Reset progress after 2 seconds
        setTimeout(() => {
            Utils.getElement('scan_progress_bar').style.width = '0%';
            Utils.setElementText('scan_progress_pct', '0%');
            Utils.setElementText('scan_eta', '--');
            Utils.setElementText('scan_status', 'Stopped');
            Utils.getElement('scan_status').style.color = '#888';
        }, 2000);

        this.state.scanning = false;
    },

    /**
     * Save scan to history
     */
    saveScanToHistory(data) {
        const scan = {
            timestamp: new Date().toISOString(),
            startFreq: this.state.startFreq,
            stopFreq: this.state.stopFreq,
            step: this.state.step,
            signalCount: data.signals.length,
            signals: data.signals,
            duration: Date.now() - this.state.startTime
        };

        this.state.scanHistory.unshift(scan);

        // Keep last 10 scans
        if (this.state.scanHistory.length > 10) {
            this.state.scanHistory = this.state.scanHistory.slice(0, 10);
        }

        // Save to localStorage
        try {
            Settings.set('scan_history', this.state.scanHistory);
        } catch (err) {
            console.warn('Failed to save scan history:', err);
        }
    },

    /**
     * Load scan history from localStorage
     */
    loadScanHistory() {
        this.state.scanHistory = Settings.get('scan_history', []);
    },

    /**
     * Export scan results
     */
    exportResults() {
        if (this.state.signals.length === 0) {
            showNotification('No signals to export', 'warning');
            return;
        }

        // Show export format dialog
        const format = prompt('Export format:\n1 = CSV\n2 = JSON\n3 = KML (for mapping)\nEnter 1, 2, or 3:');

        switch (format) {
            case '1':
                this.exportCSV();
                break;
            case '2':
                this.exportJSON();
                break;
            case '3':
                this.exportKML();
                break;
            default:
                showNotification('Export cancelled', 'info');
        }
    },

    /**
     * Export as CSV
     */
    exportCSV() {
        const headers = 'Frequency (MHz),Power (dBm),Bandwidth (Hz),Hits\n';
        const rows = this.state.signals.map(sig =>
            `${sig.freq},${sig.power},${sig.bw},${sig.hits}`
        ).join('\n');

        const csv = headers + rows;
        const filename = `scan_${new Date().toISOString().split('T')[0]}.csv`;

        Utils.downloadFile(csv, filename, 'text/csv');
        showNotification('Exported as CSV', 'success', 2000);
    },

    /**
     * Export as JSON
     */
    exportJSON() {
        const data = {
            timestamp: new Date().toISOString(),
            scanParams: {
                startFreq: this.state.startFreq,
                stopFreq: this.state.stopFreq,
                step: this.state.step
            },
            signals: this.state.signals
        };

        const json = JSON.stringify(data, null, 2);
        const filename = `scan_${new Date().toISOString().split('T')[0]}.json`;

        Utils.downloadFile(json, filename, 'application/json');
        showNotification('Exported as JSON', 'success', 2000);
    },

    /**
     * Export as KML (for Google Earth, etc.)
     */
    exportKML() {
        // Would need sensor location for this
        showNotification('KML export requires sensor position (TODO)', 'warning');
    },

    /**
     * Show filter dialog
     */
    showFilterDialog() {
        // TODO: Create modal dialog for filters
        showNotification('Filter dialog (TODO)', 'info');
    },

    /**
     * Show schedule dialog
     */
    showScheduleDialog() {
        // TODO: Create modal dialog for scheduling
        showNotification('Schedule dialog (TODO)', 'info');
    },

    /**
     * Enable scheduled scanning
     */
    enableScheduledScan(intervalMs) {
        this.scheduledScan.intervalMs = intervalMs;
        this.scheduledScan.enabled = true;

        if (this.scheduledScan.interval) {
            clearInterval(this.scheduledScan.interval);
        }

        this.scheduledScan.interval = setInterval(() => {
            if (!this.state.scanning) {
                console.log('Starting scheduled scan...');
                this.start();
            }
        }, intervalMs);

        showNotification(`Scheduled scan every ${Utils.formatDuration(intervalMs)}`, 'success');
    },

    /**
     * Disable scheduled scanning
     */
    disableScheduledScan() {
        if (this.scheduledScan.interval) {
            clearInterval(this.scheduledScan.interval);
            this.scheduledScan.interval = null;
        }

        this.scheduledScan.enabled = false;
        showNotification('Scheduled scan disabled', 'info');
    }
};

// Make available globally
window.Scanner = Scanner;

console.log('✓ Enhanced Scanner module loaded');
