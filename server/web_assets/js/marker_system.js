// ============================================================================
// PROFESSIONAL MARKER SYSTEM
// Multi-marker analysis with delta measurements and CSV export
// ============================================================================

const MarkerSystem = {
    markers: [],
    nextMarkerId: 1,
    referenceMarker: null,
    markerColors: ['#0ff', '#f0f', '#ff0', '#0f0', '#f80', '#80f', '#0ff', '#fff']
};

function toggleMarkerPanel() {
    const panel = document.getElementById('marker_panel');
    const isVisible = panel.style.display !== 'none';
    panel.style.display = isVisible ? 'none' : 'block';
    document.getElementById('marker_toggle').classList.toggle('active', !isVisible);
}

function addMarker() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;

    // Find peak
    let peakIdx = 0, peakVal = 0;
    for (let i = 0; i < data.length; i++) {
        if (data[i] > peakVal) {
            peakVal = data[i];
            peakIdx = i;
        }
    }

    const freq = cf - (sr / 2) + (peakIdx * sr / FFT_SIZE);
    const binBW = sr / FFT_SIZE;
    const power = rawToDbm(data[peakIdx], binBW);

    const marker = {
        id: MarkerSystem.nextMarkerId++,
        bin: peakIdx,
        freq: freq,
        power: power,
        type: 'normal'
    };

    MarkerSystem.markers.push(marker);
    updateMarkerTable();

    console.log(`✓ Marker M${marker.id} added at ${(freq / 1e6).toFixed(3)} MHz`);
}

function clearAllMarkers() {
    MarkerSystem.markers = [];
    MarkerSystem.referenceMarker = null;
    updateMarkerTable();
    updateDeltaMeasurements();
    console.log('✓ All markers cleared');
}

function deleteMarker(id) {
    MarkerSystem.markers = MarkerSystem.markers.filter(m => m.id !== id);
    if (MarkerSystem.referenceMarker === id) {
        MarkerSystem.referenceMarker = null;
    }
    updateMarkerTable();
    updateDeltaMeasurements();
}

function setReferenceMarker(id) {
    MarkerSystem.referenceMarker = id;
    updateMarkerTable();
    updateDeltaMeasurements();
}

function markerToPeak() {
    if (MarkerSystem.markers.length === 0) {
        alert('No markers to move');
        return;
    }

    if (!latestFFTData || latestFFTData.length === 0) return;

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;

    // Find peak
    let peakIdx = 0, peakVal = 0;
    for (let i = 0; i < data.length; i++) {
        if (data[i] > peakVal) {
            peakVal = data[i];
            peakIdx = i;
        }
    }

    // Move last marker to peak
    const marker = MarkerSystem.markers[MarkerSystem.markers.length - 1];
    marker.bin = peakIdx;
    marker.freq = cf - (sr / 2) + (peakIdx * sr / FFT_SIZE);
    marker.power = rawToDbm(data[peakIdx], sr / FFT_SIZE);

    updateMarkerTable();
    console.log(`✓ Marker M${marker.id} moved to peak`);
}

function markerToCenter() {
    if (MarkerSystem.markers.length === 0) {
        alert('No markers to move');
        return;
    }

    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const marker = MarkerSystem.markers[MarkerSystem.markers.length - 1];
    marker.freq = cf;

    // Update frequency input
    document.getElementById('freqInput').value = (marker.freq / 1e6).toFixed(6);
    applyFrequency();

    console.log(`✓ Tuned to marker M${marker.id} frequency`);
}

function markNextPeak() {
    if (!latestFFTData || latestFFTData.length === 0) return;

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;

    // Find all peaks
    const peaks = [];
    for (let i = 10; i < data.length - 10; i++) {
        if (data[i] > data[i - 1] && data[i] > data[i + 1] && data[i] > 100) {
            let isLocalMax = true;
            for (let j = i - 5; j <= i + 5; j++) {
                if (j !== i && data[j] > data[i]) {
                    isLocalMax = false;
                    break;
                }
            }
            if (isLocalMax) {
                const freq = cf - (sr / 2) + (i * sr / FFT_SIZE);
                const power = rawToDbm(data[i], binBW);
                peaks.push({ bin: i, freq: freq, power: power });
            }
        }
    }

    // Sort by power
    peaks.sort((a, b) => b.power - a.power);

    // Find next unmarked peak
    const markedBins = new Set(MarkerSystem.markers.map(m => m.bin));
    const nextPeak = peaks.find(p => !markedBins.has(p.bin));

    if (nextPeak) {
        const marker = {
            id: MarkerSystem.nextMarkerId++,
            bin: nextPeak.bin,
            freq: nextPeak.freq,
            power: nextPeak.power,
            type: 'normal'
        };
        MarkerSystem.markers.push(marker);
        updateMarkerTable();
        console.log(`✓ Marker M${marker.id} added to next peak`);
    } else {
        alert('No more peaks found');
    }
}

function updateMarkerTable() {
    const tbody = document.getElementById('marker_table_body');
    tbody.innerHTML = '';

    if (MarkerSystem.markers.length === 0) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align: center; color: #888; padding: 20px;">No markers</td></tr>';
        return;
    }

    MarkerSystem.markers.forEach((marker, idx) => {
        const isRef = MarkerSystem.referenceMarker === marker.id;
        const color = MarkerSystem.markerColors[idx % MarkerSystem.markerColors.length];

        // Calculate delta from reference
        let deltaFreq = '--', deltaPower = '--';
        if (MarkerSystem.referenceMarker !== null && !isRef) {
            const refMarker = MarkerSystem.markers.find(m => m.id === MarkerSystem.referenceMarker);
            if (refMarker) {
                deltaFreq = ((marker.freq - refMarker.freq) / 1000).toFixed(2) + ' kHz';
                deltaPower = (marker.power - refMarker.power).toFixed(2) + ' dB';
            }
        }

        const row = `<tr style="background: ${isRef ? 'rgba(255, 255, 0, 0.1)' : ''};">
            <td style="padding: 5px; color: ${color};">M${marker.id}${isRef ? ' ⭐' : ''}</td>
            <td style="padding: 5px; text-align: right; font-size: 10px;">${(marker.freq / 1e6).toFixed(6)}</td>
            <td style="padding: 5px; text-align: right; color: #0f0;">${marker.power.toFixed(2)}</td>
            <td style="padding: 5px; text-align: right; color: #888; font-size: 9px;">${isRef ? '--' : deltaFreq + '<br>' + deltaPower}</td>
            <td style="padding: 5px; text-align: center;">
                <button onclick="setReferenceMarker(${marker.id})" style="padding: 1px 4px; font-size: 8px; background: #1a1a1a; border: 1px solid #555; color: #ccc; cursor: pointer; border-radius: 2px; margin-right: 2px;" title="Set as reference">R</button>
                <button onclick="deleteMarker(${marker.id})" style="padding: 1px 4px; font-size: 8px; background: #3a1a1a; border: 1px solid #555; color: #f88; cursor: pointer; border-radius: 2px;" title="Delete">×</button>
            </td>
        </tr>`;
        tbody.innerHTML += row;
    });
}

function updateDeltaMeasurements() {
    if (MarkerSystem.referenceMarker === null || MarkerSystem.markers.length < 2) {
        document.getElementById('delta_ref_marker').textContent = 'None';
        document.getElementById('delta_freq').textContent = '-- kHz';
        document.getElementById('delta_power').textContent = '-- dB';
        return;
    }

    const refMarker = MarkerSystem.markers.find(m => m.id === MarkerSystem.referenceMarker);
    if (!refMarker) return;

    document.getElementById('delta_ref_marker').textContent = `M${refMarker.id}`;

    // Find largest delta
    let maxDeltaF = 0, maxDeltaP = 0;
    MarkerSystem.markers.forEach(m => {
        if (m.id !== refMarker.id) {
            const df = Math.abs(m.freq - refMarker.freq);
            const dp = Math.abs(m.power - refMarker.power);
            if (df > maxDeltaF) maxDeltaF = df;
            if (dp > maxDeltaP) maxDeltaP = dp;
        }
    });

    document.getElementById('delta_freq').textContent = (maxDeltaF / 1000).toFixed(3) + ' kHz';
    document.getElementById('delta_power').textContent = maxDeltaP.toFixed(2) + ' dB';
}

function exportMarkers() {
    if (MarkerSystem.markers.length === 0) {
        alert('No markers to export');
        return;
    }

    let csv = 'Marker,Frequency (Hz),Frequency (MHz),Power (dBm)\n';
    MarkerSystem.markers.forEach(m => {
        csv += `M${m.id},${m.freq.toFixed(0)},${(m.freq / 1e6).toFixed(6)},${m.power.toFixed(3)}\n`;
    });

    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `markers_${Date.now()}.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);

    console.log('✓ Markers exported to CSV');
}

// Draw markers on spectrum
function drawMarkersOnSpectrum(ctx, width, height) {
    if (MarkerSystem.markers.length === 0) return;

    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;

    MarkerSystem.markers.forEach((marker, idx) => {
        const relFreq = marker.freq - cf;
        const x = (width / 2) + (relFreq / sr) * width;

        if (x >= 0 && x <= width) {
            const color = MarkerSystem.markerColors[idx % MarkerSystem.markerColors.length];

            // Draw vertical line
            ctx.strokeStyle = color;
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
            ctx.setLineDash([]);

            // Draw marker label
            ctx.fillStyle = color;
            ctx.font = 'bold 11px monospace';
            ctx.fillText(`M${marker.id}`, x + 3, 15);
        }
    });
}

console.log('✓ Marker System module loaded');
