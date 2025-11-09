/**
 * @file waveform_display.js
 * @brief Time-Domain IQ Waveform Visualization
 *
 * Features:
 * - Real-time I/Q waveform plotting
 * - Multiple view modes: I & Q, Magnitude, Phase, I only, Q only
 * - Auto-scaling and grid display
 * - Dual-channel averaging
 * - Time axis with sample numbers
 */

const WaveformDisplay = (function() {
    let canvas = null;
    let ctx = null;
    let viewMode = 'i_and_q'; // 'i_and_q', 'magnitude', 'phase', 'i_only', 'q_only'
    let sampleRate = 40e6; // Will be updated

    // Zoom and pan state
    let zoomLevel = 1.0; // 1.0 = no zoom, 2.0 = 2x zoom, etc.
    let panOffsetX = 0; // Horizontal pan in samples
    let panOffsetY = 0; // Vertical pan in normalized units

    // Mouse interaction state
    let isDragging = false;
    let lastMouseX = 0;
    let lastMouseY = 0;

    // Latest waveform data
    let currentData = {
        i: null,
        q: null,
        magnitude: null,
        phase: null
    };

    function init(canvasId) {
        canvas = document.getElementById(canvasId);
        if (!canvas) {
            console.error('[Waveform Display] Canvas not found:', canvasId);
            return false;
        }

        ctx = canvas.getContext('2d');
        if (!ctx) {
            console.error('[Waveform Display] Could not get 2D context');
            return false;
        }

        // Set canvas size to match display size
        resize();

        // Add mouse event listeners for pan and zoom
        canvas.addEventListener('wheel', handleWheel, { passive: false });
        canvas.addEventListener('mousedown', handleMouseDown);
        canvas.addEventListener('mousemove', handleMouseMove);
        canvas.addEventListener('mouseup', handleMouseUp);
        canvas.addEventListener('mouseleave', handleMouseUp);
        canvas.addEventListener('dblclick', handleDoubleClick);

        console.log('[Waveform Display] Initialized with zoom/pan controls');
        return true;
    }

    function resize() {
        if (!canvas) return;

        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;

        console.log(`[Waveform Display] Resized to ${canvas.width}x${canvas.height}`);

        // Redraw with current data if available
        if (currentData.i) {
            draw();
        }
    }

    function setViewMode(mode) {
        viewMode = mode;
        console.log(`[Waveform Display] View mode: ${mode}`);

        if (currentData.i) {
            draw();
        }
    }

    function setSampleRate(rate) {
        sampleRate = rate;
    }

    function update(ch1_i, ch1_q, ch2_i, ch2_q) {
        if (!ctx || !canvas) return;
        if (!ch1_i || ch1_i.length === 0) return;

        // Average both channels and normalize
        const numSamples = Math.min(ch1_i.length, ch1_q.length, ch2_i.length, ch2_q.length);
        const iSamples = new Float32Array(numSamples);
        const qSamples = new Float32Array(numSamples);
        const magnitudeSamples = new Float32Array(numSamples);
        const phaseSamples = new Float32Array(numSamples);

        for (let i = 0; i < numSamples; i++) {
            iSamples[i] = (ch1_i[i] + ch2_i[i]) / 2.0 / 2048.0; // Normalize int16 to [-1, 1]
            qSamples[i] = (ch1_q[i] + ch2_q[i]) / 2.0 / 2048.0;

            // Calculate magnitude and phase
            magnitudeSamples[i] = Math.sqrt(iSamples[i] * iSamples[i] + qSamples[i] * qSamples[i]);
            phaseSamples[i] = Math.atan2(qSamples[i], iSamples[i]) * (180.0 / Math.PI); // Convert to degrees
        }

        currentData.i = iSamples;
        currentData.q = qSamples;
        currentData.magnitude = magnitudeSamples;
        currentData.phase = phaseSamples;

        draw();
    }

    function draw() {
        if (!ctx || !canvas) return;
        if (!currentData.i) return;

        const width = canvas.width;
        const height = canvas.height;

        // Clear canvas
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, width, height);

        // Draw grid
        drawGrid();

        // Draw waveform based on view mode
        switch (viewMode) {
            case 'i_and_q':
                drawDualTrace(currentData.i, currentData.q, 'I', 'Q', '#0ff', '#ff0', -1, 1);
                break;
            case 'magnitude':
                drawSingleTrace(currentData.magnitude, 'Magnitude', '#0f0', 0, 1.5);
                break;
            case 'phase':
                drawSingleTrace(currentData.phase, 'Phase', '#f0f', -180, 180);
                break;
            case 'i_only':
                drawSingleTrace(currentData.i, 'I', '#0ff', -1, 1);
                break;
            case 'q_only':
                drawSingleTrace(currentData.q, 'Q', '#ff0', -1, 1);
                break;
        }

        // Draw labels
        drawLabels();
    }

    function drawGrid() {
        if (!ctx || !canvas) return;

        const width = canvas.width;
        const height = canvas.height;

        // Grid lines
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
        ctx.lineWidth = 1;

        // Horizontal lines
        for (let i = 0; i <= 4; i++) {
            const y = (height * i) / 4;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);

            // Center line is brighter
            if (i === 2) {
                ctx.strokeStyle = 'rgba(0, 255, 255, 0.3)';
            } else {
                ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
            }
            ctx.stroke();
        }

        // Vertical lines
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
        for (let i = 0; i <= 8; i++) {
            const x = (width * i) / 8;
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }
    }

    function drawSingleTrace(data, label, color, minVal, maxVal) {
        if (!data || data.length === 0) return;

        const width = canvas.width;
        const height = canvas.height;

        // Apply zoom and pan
        const samplesPerScreen = Math.floor(data.length / zoomLevel);
        const startIdx = Math.max(0, Math.min(panOffsetX, data.length - samplesPerScreen));
        const endIdx = Math.min(startIdx + samplesPerScreen, data.length);

        // Calculate vertical scale with zoom and pan
        const range = (maxVal - minVal) / zoomLevel;
        const centerVal = (minVal + maxVal) / 2 + panOffsetY;
        const scale = height / range;
        const offset = height / 2 - centerVal * scale;

        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        let firstPoint = true;
        for (let i = startIdx; i < endIdx; i++) {
            const x = ((i - startIdx) / (endIdx - startIdx - 1)) * width;
            const y = offset + height / 2 - data[i] * scale;

            if (firstPoint) {
                ctx.moveTo(x, y);
                firstPoint = false;
            } else {
                ctx.lineTo(x, y);
            }
        }

        ctx.stroke();

        // Draw label with zoom info
        ctx.fillStyle = color;
        ctx.font = '11px monospace';
        ctx.fillText(`${label} [${zoomLevel.toFixed(1)}x]`, 10, 20);
    }

    function drawDualTrace(data1, data2, label1, label2, color1, color2, minVal, maxVal) {
        if (!data1 || data1.length === 0) return;

        const width = canvas.width;
        const height = canvas.height;

        // Apply zoom and pan
        const samplesPerScreen = Math.floor(data1.length / zoomLevel);
        const startIdx = Math.max(0, Math.min(panOffsetX, data1.length - samplesPerScreen));
        const endIdx = Math.min(startIdx + samplesPerScreen, data1.length);

        // Calculate vertical scale with zoom and pan
        const range = (maxVal - minVal) / zoomLevel;
        const centerVal = (minVal + maxVal) / 2 + panOffsetY;
        const scale = (height * 0.4) / (range / 2); // Scale to ±40% of height
        const centerY = height / 2 - panOffsetY * height * 0.4;

        // Draw first trace
        ctx.strokeStyle = color1;
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        let firstPoint = true;
        for (let i = startIdx; i < endIdx; i++) {
            const x = ((i - startIdx) / (endIdx - startIdx - 1)) * width;
            const y = centerY - data1[i] * scale;

            if (firstPoint) {
                ctx.moveTo(x, y);
                firstPoint = false;
            } else {
                ctx.lineTo(x, y);
            }
        }

        ctx.stroke();

        // Draw second trace
        ctx.strokeStyle = color2;
        ctx.lineWidth = 1.5;
        ctx.beginPath();

        firstPoint = true;
        for (let i = startIdx; i < endIdx; i++) {
            const x = ((i - startIdx) / (endIdx - startIdx - 1)) * width;
            const y = centerY - data2[i] * scale;

            if (firstPoint) {
                ctx.moveTo(x, y);
                firstPoint = false;
            } else {
                ctx.lineTo(x, y);
            }
        }

        ctx.stroke();

        // Draw labels with zoom info
        ctx.fillStyle = color1;
        ctx.font = '11px monospace';
        ctx.fillText(`${label1} [${zoomLevel.toFixed(1)}x]`, 10, 20);

        ctx.fillStyle = color2;
        ctx.fillText(label2, 10, 35);
    }

    function drawLabels() {
        if (!ctx || !canvas || !currentData.i) return;

        const width = canvas.width;
        const height = canvas.height;

        // Draw sample count
        ctx.fillStyle = '#888';
        ctx.font = '10px monospace';
        ctx.fillText(`Samples: ${currentData.i.length}`, width - 150, height - 10);

        // Draw sample rate if available
        if (sampleRate > 0) {
            const timeSpan = (currentData.i.length / sampleRate) * 1e6; // microseconds
            ctx.fillText(`Time: ${timeSpan.toFixed(2)} µs`, width - 150, height - 25);
        }

        // Draw zoom/pan info
        if (zoomLevel > 1.0 || panOffsetX !== 0 || panOffsetY !== 0) {
            ctx.fillText(`Pan: ${panOffsetX} smp, ${panOffsetY.toFixed(2)}`, width - 150, height - 40);
        }
    }

    // Mouse wheel handler for zoom
    function handleWheel(e) {
        e.preventDefault();

        const zoomFactor = e.deltaY < 0 ? 1.2 : 0.8;
        const newZoom = Math.max(1.0, Math.min(50.0, zoomLevel * zoomFactor));

        zoomLevel = newZoom;

        // Constrain pan after zoom
        if (currentData.i) {
            const maxPan = Math.max(0, currentData.i.length * (1 - 1/zoomLevel));
            panOffsetX = Math.max(0, Math.min(panOffsetX, maxPan));
        }

        draw();
    }

    // Mouse down handler - start dragging
    function handleMouseDown(e) {
        isDragging = true;
        lastMouseX = e.offsetX;
        lastMouseY = e.offsetY;
        canvas.style.cursor = 'grabbing';
    }

    // Mouse move handler - pan while dragging
    function handleMouseMove(e) {
        if (!isDragging || !currentData.i) return;

        const deltaX = e.offsetX - lastMouseX;
        const deltaY = e.offsetY - lastMouseY;

        // Horizontal pan (in samples)
        const samplesPerPixel = (currentData.i.length / zoomLevel) / canvas.width;
        panOffsetX = Math.max(0, panOffsetX - deltaX * samplesPerPixel);

        const maxPan = Math.max(0, currentData.i.length * (1 - 1/zoomLevel));
        panOffsetX = Math.min(panOffsetX, maxPan);

        // Vertical pan (normalized units)
        const panSensitivity = 0.002;
        panOffsetY += deltaY * panSensitivity;
        panOffsetY = Math.max(-2.0, Math.min(2.0, panOffsetY)); // Limit vertical pan

        lastMouseX = e.offsetX;
        lastMouseY = e.offsetY;

        draw();
    }

    // Mouse up handler - stop dragging
    function handleMouseUp(e) {
        isDragging = false;
        canvas.style.cursor = 'grab';
    }

    // Double click handler - reset zoom and pan
    function handleDoubleClick(e) {
        zoomLevel = 1.0;
        panOffsetX = 0;
        panOffsetY = 0;
        draw();
    }

    // Public zoom/pan control functions
    function setZoom(zoom) {
        zoomLevel = Math.max(1.0, Math.min(50.0, zoom));
        draw();
    }

    function resetView() {
        zoomLevel = 1.0;
        panOffsetX = 0;
        panOffsetY = 0;
        draw();
    }

    return {
        init,
        resize,
        update,
        setViewMode,
        setSampleRate,
        setZoom,
        resetView
    };
})();
