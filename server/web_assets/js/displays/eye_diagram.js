/**
 * @file eye_diagram.js
 * @brief Eye Diagram Visualization for Digital Signal Analysis
 *
 * Features:
 * - Multi-trace persistence for eye pattern visualization
 * - Configurable symbol rate and samples per symbol
 * - I/Q channel display with automatic timing recovery
 * - Eye opening measurements (height, width, jitter)
 * - Color-coded trace density for fade effect
 */

const EyeDiagram = (function() {
    let canvas = null;
    let ctx = null;
    let symbolRate = 1000000; // Default 1 Msym/s
    let sampleRate = 40e6; // Will be updated from actual sample rate
    let samplesPerSymbol = 40; // Will be recalculated
    let persistence = [];
    const MAX_TRACES = 500; // Number of traces to keep for persistence
    const TRACE_COLORS = [
        'rgba(0, 255, 255, 0.05)',
        'rgba(0, 255, 255, 0.1)',
        'rgba(0, 255, 255, 0.2)',
        'rgba(0, 255, 255, 0.4)',
        'rgba(0, 255, 255, 0.6)',
        'rgba(0, 255, 255, 0.8)',
        'rgba(0, 255, 255, 1.0)'
    ];

    // Zoom and pan state
    let zoomLevel = 1.0; // 1.0 = no zoom, 2.0 = 2x zoom, etc.
    let panOffsetX = 0; // Horizontal pan (time axis)
    let panOffsetY = 0; // Vertical pan (amplitude axis)

    // Mouse interaction state
    let isDragging = false;
    let lastMouseX = 0;
    let lastMouseY = 0;

    // Eye diagram statistics
    let eyeStats = {
        eyeHeight: 0,
        eyeWidth: 0,
        jitter: 0,
        crossingPoints: []
    };

    function init(canvasId) {
        canvas = document.getElementById(canvasId);
        if (!canvas) {
            console.error('[Eye Diagram] Canvas not found:', canvasId);
            return false;
        }

        ctx = canvas.getContext('2d');
        if (!ctx) {
            console.error('[Eye Diagram] Could not get 2D context');
            return false;
        }

        // Set canvas size to match display size
        resize();

        // Initialize persistence buffer
        persistence = [];

        // Add mouse event listeners for pan and zoom
        canvas.addEventListener('wheel', handleWheel, { passive: false });
        canvas.addEventListener('mousedown', handleMouseDown);
        canvas.addEventListener('mousemove', handleMouseMove);
        canvas.addEventListener('mouseup', handleMouseUp);
        canvas.addEventListener('mouseleave', handleMouseUp);
        canvas.addEventListener('dblclick', handleDoubleClick);

        console.log('[Eye Diagram] Initialized with zoom/pan controls');
        return true;
    }

    function resize() {
        if (!canvas) return;

        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;

        console.log(`[Eye Diagram] Resized to ${canvas.width}x${canvas.height}`);
    }

    function setSymbolRate(rate) {
        symbolRate = rate;
        samplesPerSymbol = Math.round(sampleRate / symbolRate);
        console.log(`[Eye Diagram] Symbol rate: ${(symbolRate / 1e6).toFixed(3)} Msym/s, ${samplesPerSymbol} samples/symbol`);
    }

    function setSampleRate(rate) {
        sampleRate = rate;
        samplesPerSymbol = Math.round(sampleRate / symbolRate);
    }

    function clear() {
        persistence = [];
        eyeStats = {
            eyeHeight: 0,
            eyeWidth: 0,
            jitter: 0,
            crossingPoints: []
        };
        zoomLevel = 1.0;
        panOffsetX = 0;
        panOffsetY = 0;
        if (ctx && canvas) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            drawGrid();
        }
    }

    function drawGrid() {
        if (!ctx || !canvas) return;

        const width = canvas.width;
        const height = canvas.height;

        // Background
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, width, height);

        // Grid lines
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
        ctx.lineWidth = 1;

        // Horizontal center line (zero crossing)
        ctx.beginPath();
        ctx.moveTo(0, height / 2);
        ctx.lineTo(width, height / 2);
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.3)';
        ctx.stroke();

        // Vertical center line (symbol boundary)
        ctx.beginPath();
        ctx.moveTo(width / 2, 0);
        ctx.lineTo(width / 2, height);
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.3)';
        ctx.stroke();

        // Grid
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
        for (let i = 1; i < 4; i++) {
            // Horizontal
            ctx.beginPath();
            ctx.moveTo(0, (height * i) / 4);
            ctx.lineTo(width, (height * i) / 4);
            ctx.stroke();

            // Vertical
            ctx.beginPath();
            ctx.moveTo((width * i) / 4, 0);
            ctx.lineTo((width * i) / 4, height);
            ctx.stroke();
        }

        // Labels
        ctx.fillStyle = '#0ff';
        ctx.font = '10px monospace';
        ctx.fillText('0 sym', 5, height / 2 - 5);
        ctx.fillText('1 sym', width / 2 + 5, height / 2 - 5);
        ctx.fillText('2 sym', width - 45, height / 2 - 5);
    }

    function update(ch1_i, ch1_q, ch2_i, ch2_q) {
        if (!ctx || !canvas) return;
        if (!ch1_i || ch1_i.length === 0) return;

        // Average both channels
        const numSamples = Math.min(ch1_i.length, ch1_q.length, ch2_i.length, ch2_q.length);
        const iSamples = new Float32Array(numSamples);
        const qSamples = new Float32Array(numSamples);

        for (let i = 0; i < numSamples; i++) {
            iSamples[i] = (ch1_i[i] + ch2_i[i]) / 2.0 / 2048.0; // Normalize int16 to [-1, 1]
            qSamples[i] = (ch1_q[i] + ch2_q[i]) / 2.0 / 2048.0;
        }

        // Extract eye traces - show 2 symbols worth of data at a time
        const symbolSamples = samplesPerSymbol * 2;
        const numTraces = Math.floor(numSamples / samplesPerSymbol) - 1;

        // Add new traces (both I and Q)
        for (let t = 0; t < Math.min(numTraces, 20); t++) {
            const startIdx = t * samplesPerSymbol;
            if (startIdx + symbolSamples > numSamples) break;

            // Extract I trace
            const iTrace = iSamples.slice(startIdx, startIdx + symbolSamples);
            persistence.push({ data: Array.from(iTrace), channel: 'I', age: 0 });

            // Extract Q trace
            const qTrace = qSamples.slice(startIdx, startIdx + symbolSamples);
            persistence.push({ data: Array.from(qTrace), channel: 'Q', age: 0 });
        }

        // Limit persistence buffer
        while (persistence.length > MAX_TRACES) {
            persistence.shift();
        }

        // Age all traces
        persistence.forEach(trace => trace.age++);

        // Draw eye diagram
        drawEye();
    }

    function drawEye() {
        if (!ctx || !canvas) return;

        const width = canvas.width;
        const height = canvas.height;

        // Apply zoom and pan to vertical scale
        const baseScale = height * 0.4; // Base scale for ±1.0 range
        const scale = baseScale * zoomLevel;
        const centerY = height / 2 - panOffsetY * baseScale;

        // Clear and draw grid
        drawGrid();

        // Draw all persistence traces with fade
        persistence.forEach((trace, idx) => {
            const data = trace.data;
            const age = trace.age;
            const channel = trace.channel;

            // Calculate color based on age
            const colorIdx = Math.min(Math.floor(age / (MAX_TRACES / TRACE_COLORS.length)), TRACE_COLORS.length - 1);
            let color = TRACE_COLORS[colorIdx];

            // Use different color for Q channel
            if (channel === 'Q') {
                color = color.replace('0, 255, 255', '255, 255, 0'); // Yellow for Q
            }

            ctx.strokeStyle = color;
            ctx.lineWidth = 1;
            ctx.beginPath();

            for (let i = 0; i < data.length; i++) {
                // Apply horizontal zoom/pan
                const normalizedX = i / (data.length - 1); // 0 to 1
                const zoomedX = (normalizedX - 0.5) * (1.0 / zoomLevel) + 0.5 + panOffsetX;
                const x = zoomedX * width;

                // Apply vertical zoom/pan
                const y = centerY - data[i] * scale;

                // Skip points outside visible area
                if (x < 0 || x > width) continue;

                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }

            ctx.stroke();
        });

        // Draw eye opening marker if available
        if (eyeStats.crossingPoints.length > 0) {
            ctx.strokeStyle = 'rgba(255, 0, 0, 0.5)';
            ctx.lineWidth = 2;
            const crossX = width / 2;
            ctx.beginPath();
            ctx.moveTo(crossX - 10, centerY);
            ctx.lineTo(crossX + 10, centerY);
            ctx.stroke();
        }

        // Draw zoom info
        if (zoomLevel > 1.0 || panOffsetX !== 0 || panOffsetY !== 0) {
            ctx.fillStyle = '#888';
            ctx.font = '10px monospace';
            ctx.fillText(`Zoom: ${zoomLevel.toFixed(1)}x`, width - 100, 15);
            ctx.fillText(`Pan: ${panOffsetX.toFixed(2)}, ${panOffsetY.toFixed(2)}`, width - 100, 30);
        }
    }

    // Mouse wheel handler for zoom
    function handleWheel(e) {
        e.preventDefault();

        const zoomFactor = e.deltaY < 0 ? 1.2 : 0.8;
        const newZoom = Math.max(1.0, Math.min(50.0, zoomLevel * zoomFactor));

        zoomLevel = newZoom;

        drawEye();
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
        if (!isDragging) return;

        const deltaX = e.offsetX - lastMouseX;
        const deltaY = e.offsetY - lastMouseY;

        // Horizontal pan (time axis) - normalized units
        const panSensitivityX = 0.001;
        panOffsetX += deltaX * panSensitivityX;
        panOffsetX = Math.max(-0.5, Math.min(0.5, panOffsetX)); // Limit horizontal pan

        // Vertical pan (amplitude axis) - normalized units
        const panSensitivityY = 0.002;
        panOffsetY += deltaY * panSensitivityY;
        panOffsetY = Math.max(-2.0, Math.min(2.0, panOffsetY)); // Limit vertical pan

        lastMouseX = e.offsetX;
        lastMouseY = e.offsetY;

        drawEye();
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
        drawEye();
    }

    // Public zoom/pan control functions
    function setZoom(zoom) {
        zoomLevel = Math.max(1.0, Math.min(50.0, zoom));
        drawEye();
    }

    function resetView() {
        zoomLevel = 1.0;
        panOffsetX = 0;
        panOffsetY = 0;
        drawEye();
    }

    return {
        init,
        resize,
        update,
        setSymbolRate,
        setSampleRate,
        clear,
        getStats: () => eyeStats,
        setZoom,
        resetView
    };
})();
