/**
 * Spectrum Display Module
 * Handles FFT spectrum visualization with peak hold, min hold, and reference markers
 */

const SpectrumDisplay = (function() {
    'use strict';

    // Module state
    let spectrumCanvas, spectrumCtx, spectrumCanvas2, spectrumCtx2;
    let spectrumOffscreen, spectrumOffscreenCtx, spectrumOffscreen2, spectrumOffscreenCtx2;
    let peakHoldEnabled = false;
    let peakHoldData = null;
    let minHoldEnabled = false;
    let minHoldTrace = null;
    let refMarkersEnabled = false;
    let bwMeasureEnabled = false;
    let bwMeasurePoints = [];
    let spectrumMinDb = -100;
    let spectrumMaxDb = -10;
    let latestFFTData = null;
    let latestFFTData2 = null;
    let zoomState = {
        zoomStartBin: 0,
        zoomEndBin: 4095,
        centerFreq: 915000000,
        fullBandwidth: 40000000
    };

    // Helper: Convert raw magnitude to dB
    function rawToDb(raw) {
        return (raw / 255.0) * 120.0 - 100.0;
    }

    // Initialize the module
    function init(canvas1, canvas2, zoom) {
        spectrumCanvas = canvas1;
        spectrumCtx = canvas1.getContext('2d');
        spectrumCanvas2 = canvas2;
        spectrumCtx2 = canvas2.getContext('2d');

        // Create offscreen canvases for double buffering
        spectrumOffscreen = document.createElement('canvas');
        spectrumOffscreenCtx = spectrumOffscreen.getContext('2d');
        spectrumOffscreen.width = spectrumCanvas.width;
        spectrumOffscreen.height = spectrumCanvas.height;

        spectrumOffscreen2 = document.createElement('canvas');
        spectrumOffscreenCtx2 = spectrumOffscreen2.getContext('2d');
        spectrumOffscreen2.width = spectrumCanvas2.width;
        spectrumOffscreen2.height = spectrumCanvas2.height;

        if (zoom) {
            zoomState = zoom;
        }

        attachEventListeners();
    }

    // Attach event listeners for spectrum interactions
    function attachEventListeners() {
        // Wheel event for zoom/scroll
        [spectrumCanvas, spectrumCanvas2].forEach(canvas => {
            canvas.addEventListener('wheel', handleWheel);
            canvas.addEventListener('dblclick', handleDoubleClick);
            canvas.addEventListener('click', handleClick);
        });
    }

    function handleWheel(e) {
        e.preventDefault();

        if (e.ctrlKey) {
            // Zoom
            const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;
            const oldRange = spectrumMaxDb - spectrumMinDb;
            const newRange = Math.max(20, Math.min(400, oldRange * zoomFactor));
            const center = (spectrumMinDb + spectrumMaxDb) / 2;
            spectrumMinDb = Math.max(-50, center - newRange / 2);
            spectrumMaxDb = Math.min(350, center + newRange / 2);
        } else {
            // Scroll
            const scrollAmount = e.deltaY * 0.5;
            const range = spectrumMaxDb - spectrumMinDb;
            let newMin = spectrumMinDb + scrollAmount;
            let newMax = spectrumMaxDb + scrollAmount;
            const minLimit = -50;
            const maxLimit = 350;

            if (newMin < minLimit) {
                newMin = minLimit;
                newMax = minLimit + range;
            }
            if (newMax > maxLimit) {
                newMax = maxLimit;
                newMin = maxLimit - range;
            }

            spectrumMinDb = newMin;
            spectrumMaxDb = newMax;
        }

        draw(latestFFTData, latestFFTData2);
    }

    function handleDoubleClick() {
        spectrumMinDb = -100;
        spectrumMaxDb = -10;
        draw(latestFFTData, latestFFTData2);
    }

    function handleClick(e) {
        if (!bwMeasureEnabled) return;

        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;

        // Calculate frequency at click position
        const currentSR = zoomState.fullBandwidth || 40000000;
        const currentCF = zoomState.centerFreq || 915000000;
        const binWidth = currentSR / 4096;
        const fullStartFreq = currentCF - currentSR / 2;

        const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
        const displayedStartFreq = fullStartFreq + (zoomState.zoomStartBin * binWidth);
        const displayedBandwidth = zoomedBins * binWidth;

        const clickFreq = displayedStartFreq + (x / rect.width) * displayedBandwidth;
        const normalizedX = x / rect.width;

        if (bwMeasurePoints.length >= 2) {
            bwMeasurePoints = [];
        }

        bwMeasurePoints.push({ normalizedX, freq: clickFreq, db: 0 });
        draw(latestFFTData, latestFFTData2);
    }

    // Draw spectrum to a specific canvas context
    function drawToCanvas(data, offscreenCtx, offscreenCanvas, finalCtx) {
        if (!data) {
            return;
        }

        const width = offscreenCanvas.width;
        const height = offscreenCanvas.height;

        // Clear
        offscreenCtx.fillStyle = '#0a0a0a';
        offscreenCtx.fillRect(0, 0, width, height);

        // Grid
        offscreenCtx.strokeStyle = 'rgba(80, 80, 80, 0.3)';
        offscreenCtx.lineWidth = 1;
        for (let i = 0; i <= 10; i++) {
            const y = (height / 10) * i;
            offscreenCtx.beginPath();
            offscreenCtx.moveTo(0, y);
            offscreenCtx.lineTo(width, y);
            offscreenCtx.stroke();
        }

        offscreenCtx.strokeStyle = 'rgba(80, 80, 80, 0.2)';
        for (let i = 0; i <= 10; i++) {
            const x = (width / 10) * i;
            offscreenCtx.beginPath();
            offscreenCtx.moveTo(x, 0);
            offscreenCtx.lineTo(x, height);
            offscreenCtx.stroke();
        }

        // Draw spectrum line
        offscreenCtx.imageSmoothingEnabled = true;
        offscreenCtx.imageSmoothingQuality = 'high';

        const dbRange = spectrumMaxDb - spectrumMinDb;
        offscreenCtx.beginPath();
        offscreenCtx.moveTo(0, height);

        const points = [];
        for (let x = 0; x < width; x++) {
            const zoomedBins = zoomState.zoomEndBin - zoomState.zoomStartBin + 1;
            const fftIdx = zoomState.zoomStartBin + Math.floor((x / width) * zoomedBins);
            const raw = data[fftIdx];
            const magDb = rawToDb(raw);
            const normalizedMag = Math.max(0, Math.min(1, (magDb - spectrumMinDb) / dbRange));
            const y = height - (normalizedMag * height);
            points.push({ x, y, mag: normalizedMag });
            offscreenCtx.lineTo(x, y);
        }

        offscreenCtx.lineTo(width, height);
        offscreenCtx.closePath();

        // Gradient fill
        const gradient = offscreenCtx.createLinearGradient(0, 0, 0, height);
        gradient.addColorStop(0, 'rgba(255, 255, 0, 0.4)');
        gradient.addColorStop(0.3, 'rgba(0, 255, 100, 0.3)');
        gradient.addColorStop(0.7, 'rgba(0, 255, 200, 0.2)');
        gradient.addColorStop(1, 'rgba(0, 100, 255, 0.1)');
        offscreenCtx.fillStyle = gradient;
        offscreenCtx.fill();

        // Draw colored line
        offscreenCtx.lineJoin = 'round';
        offscreenCtx.lineCap = 'round';
        offscreenCtx.lineWidth = 1.5;

        for (let i = 0; i < points.length - 1; i++) {
            const p1 = points[i];
            const p2 = points[i + 1];
            const mag = (p1.mag + p2.mag) / 2;

            if (mag > 0.8) offscreenCtx.strokeStyle = '#ffff00';
            else if (mag > 0.5) offscreenCtx.strokeStyle = '#88ff00';
            else if (mag > 0.3) offscreenCtx.strokeStyle = '#00ff88';
            else offscreenCtx.strokeStyle = '#00ffff';

            offscreenCtx.beginPath();
            offscreenCtx.moveTo(p1.x, p1.y);
            offscreenCtx.lineTo(p2.x, p2.y);
            offscreenCtx.stroke();
        }

        // dB labels
        offscreenCtx.fillStyle = '#888';
        offscreenCtx.font = '10px monospace';
        offscreenCtx.textAlign = 'right';
        for (let i = 0; i <= 10; i++) {
            const y = (height / 10) * i;
            const dbValue = Math.floor(spectrumMaxDb - (i / 10) * dbRange);
            offscreenCtx.fillText(dbValue + ' dB', width - 5, y + 3);
        }

        // Copy to visible canvas
        finalCtx.drawImage(offscreenCanvas, 0, 0);
    }

    // Main draw function
    function draw(data, data2) {
        if (!data) {
            return;
        }

        latestFFTData = data;
        latestFFTData2 = data2;

        // Update peak hold
        if (peakHoldEnabled) {
            if (!peakHoldData || peakHoldData.length !== data.length) {
                peakHoldData = new Uint8Array(data);
            } else {
                for (let i = 0; i < data.length; i++) {
                    if (data[i] > peakHoldData[i]) {
                        peakHoldData[i] = data[i];
                    }
                }
            }
        }

        // Check dual-channel mode
        const chSelect = document.getElementById('channel_select');
        const isDualChannel = (chSelect && chSelect.value === 'both' && data2);

        if (isDualChannel) {
            drawToCanvas(data, spectrumOffscreenCtx, spectrumOffscreen, spectrumCtx);
            drawToCanvas(data2, spectrumOffscreenCtx2, spectrumOffscreen2, spectrumCtx2);
        } else {
            drawToCanvas(data, spectrumOffscreenCtx, spectrumOffscreen, spectrumCtx);
        }
    }

    // Public API
    return {
        init,
        draw,
        updateZoomState: (zoom) => { zoomState = zoom; },
        setRange: (min, max) => { spectrumMinDb = min; spectrumMaxDb = max; },
        getRange: () => ({ min: spectrumMinDb, max: spectrumMaxDb }),
        togglePeakHold: (enabled) => { peakHoldEnabled = enabled; },
        clearPeakHold: () => { peakHoldData = null; },
        toggleMinHold: (enabled) => { minHoldEnabled = enabled; },
        clearMinHold: () => { minHoldTrace = null; },
        toggleRefMarkers: (enabled) => { refMarkersEnabled = enabled; },
        toggleBandwidthMeasure: (enabled) => { bwMeasureEnabled = enabled; },
        resize: (width1, height1, width2, height2) => {
            if (spectrumOffscreen.width !== width1 || spectrumOffscreen.height !== height1) {
                spectrumOffscreen.width = width1;
                spectrumOffscreen.height = height1;
            }
            if (width2 && height2) {
                if (spectrumOffscreen2.width !== width2 || spectrumOffscreen2.height !== height2) {
                    spectrumOffscreen2.width = width2;
                    spectrumOffscreen2.height = height2;
                }
            }
        }
    };
})();
