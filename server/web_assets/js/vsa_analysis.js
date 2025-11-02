// ============================================================================
// VECTOR SIGNAL ANALYZER (VSA)
// EVM, constellation analysis, and IQ impairment measurements
// ============================================================================

const VSA = {
    idealConstellation: null,
    measurementHistory: []
};

function toggleVSA() {
    const panel = document.getElementById('vsa_panel');
    const isVisible = panel.style.display !== 'none';
    panel.style.display = isVisible ? 'none' : 'block';
    document.getElementById('vsa_toggle').classList.toggle('active', !isVisible);
}

function updateVSA() {
    // Fetch current IQ data from server
    fetch('/iq_data?t=' + Date.now())
        .then(response => response.arrayBuffer())
        .then(buffer => {
            const data = new Int16Array(buffer);

            // Extract I and Q for both channels
            const numSamples = Math.floor(data.length / 4);
            const ch1_i = new Array(numSamples);
            const ch1_q = new Array(numSamples);

            for (let i = 0; i < numSamples; i++) {
                ch1_i[i] = data[i * 4];
                ch1_q[i] = data[i * 4 + 1];
            }

            // Run VSA analysis
            analyzeIQQuality(ch1_i, ch1_q);
            drawConstellation(ch1_i, ch1_q);
            drawEyeDiagram(ch1_i, ch1_q);

            console.log('✓ VSA analysis updated');
        })
        .catch(err => console.error('VSA update error:', err));
}

function analyzeIQQuality(I, Q) {
    // Calculate various IQ impairments

    // 1. EVM (Error Vector Magnitude)
    // Normalize constellation
    let maxMag = 0;
    for (let i = 0; i < I.length; i++) {
        const mag = Math.sqrt(I[i] * I[i] + Q[i] * Q[i]);
        if (mag > maxMag) maxMag = mag;
    }

    const normI = I.map(v => v / maxMag);
    const normQ = Q.map(v => v / maxMag);

    // Detect modulation order (simple clustering)
    const clusters = detectClusters(normI, normQ);

    // Calculate EVM assuming ideal constellation at cluster centers
    let sumErrorSq = 0;
    let sumIdealSq = 0;

    for (let i = 0; i < normI.length; i++) {
        // Find nearest cluster
        let minDist = Infinity;
        let nearestCluster = clusters[0];

        clusters.forEach(cluster => {
            const dist = Math.sqrt(
                Math.pow(normI[i] - cluster.i, 2) +
                Math.pow(normQ[i] - cluster.q, 2)
            );
            if (dist < minDist) {
                minDist = dist;
                nearestCluster = cluster;
            }
        });

        const errorI = normI[i] - nearestCluster.i;
        const errorQ = normQ[i] - nearestCluster.q;
        const errorMag = Math.sqrt(errorI * errorI + errorQ * errorQ);
        sumErrorSq += errorMag * errorMag;

        const idealMag = Math.sqrt(nearestCluster.i * nearestCluster.i + nearestCluster.q * nearestCluster.q);
        sumIdealSq += idealMag * idealMag;
    }

    const evmRMS = Math.sqrt(sumErrorSq / normI.length) * 100;
    const evmPeak = Math.sqrt(Math.max(...normI.map((val, idx) => {
        const nearestCluster = findNearestCluster(val, normQ[idx], clusters);
        const errI = val - nearestCluster.i;
        const errQ = normQ[idx] - nearestCluster.q;
        return errI * errI + errQ * errQ;
    }))) * 100;

    // 2. Magnitude Error
    const avgMag = normI.reduce((sum, val, idx) => {
        return sum + Math.sqrt(val * val + normQ[idx] * normQ[idx]);
    }, 0) / normI.length;
    const targetMag = Math.sqrt(clusters[0].i * clusters[0].i + clusters[0].q * clusters[0].q);
    const magError = 20 * Math.log10(avgMag / targetMag);

    // 3. Phase Error
    let sumPhaseError = 0;
    for (let i = 0; i < normI.length; i++) {
        const measured = Math.atan2(normQ[i], normI[i]);
        const nearest = findNearestCluster(normI[i], normQ[i], clusters);
        const ideal = Math.atan2(nearest.q, nearest.i);
        let phaseErr = measured - ideal;

        // Wrap to ±π
        while (phaseErr > Math.PI) phaseErr -= 2 * Math.PI;
        while (phaseErr < -Math.PI) phaseErr += 2 * Math.PI;

        sumPhaseError += Math.abs(phaseErr);
    }
    const phaseError = (sumPhaseError / normI.length) * (180 / Math.PI);

    // 4. IQ Offset (DC offset)
    const dcI = normI.reduce((a, b) => a + b, 0) / normI.length;
    const dcQ = normQ.reduce((a, b) => a + b, 0) / normQ.length;
    const iqOffset = 20 * Math.log10(Math.sqrt(dcI * dcI + dcQ * dcQ));

    // 5. Quadrature Error
    let sumIQ = 0;
    for (let i = 0; i < Math.min(normI.length, normQ.length); i++) {
        sumIQ += normI[i] * normQ[i];
    }
    const correlation = sumIQ / normI.length;
    const quadError = Math.asin(Math.abs(correlation)) * (180 / Math.PI);

    // 6. Gain Imbalance
    const rmsI = Math.sqrt(normI.reduce((sum, val) => sum + val * val, 0) / normI.length);
    const rmsQ = Math.sqrt(normQ.reduce((sum, val) => sum + val * val, 0) / normQ.length);
    const gainImbalance = 20 * Math.log10(rmsI / rmsQ);

    // Update display
    document.getElementById('vsa_evm_rms').textContent = evmRMS.toFixed(2) + '%';
    document.getElementById('vsa_evm_peak').textContent = evmPeak.toFixed(2) + '%';
    document.getElementById('vsa_mag_error').textContent = magError.toFixed(3) + ' dB';
    document.getElementById('vsa_phase_error').textContent = phaseError.toFixed(2) + ' deg';
    document.getElementById('vsa_iq_offset').textContent = iqOffset.toFixed(2) + ' dB';
    document.getElementById('vsa_quad_error').textContent = quadError.toFixed(2) + ' deg';
    document.getElementById('vsa_gain_imbal').textContent = gainImbalance.toFixed(3) + ' dB';

    console.log(`EVM: ${evmRMS.toFixed(2)}%, Phase Error: ${phaseError.toFixed(2)}°`);
}

// Simple k-means clustering for constellation detection
function detectClusters(I, Q, k = 4) {
    // Initialize clusters at extremes
    const clusters = [];
    const step = Math.PI * 2 / k;

    for (let i = 0; i < k; i++) {
        clusters.push({
            i: Math.cos(step * i) * 0.7,
            q: Math.sin(step * i) * 0.7,
            count: 0
        });
    }

    // Run k-means for a few iterations
    for (let iter = 0; iter < 10; iter++) {
        // Reset counts
        clusters.forEach(c => {
            c.count = 0;
            c.sumI = 0;
            c.sumQ = 0;
        });

        // Assign points to clusters
        for (let i = 0; i < I.length; i++) {
            const nearest = findNearestCluster(I[i], Q[i], clusters);
            nearest.count++;
            nearest.sumI += I[i];
            nearest.sumQ += Q[i];
        }

        // Update cluster centers
        clusters.forEach(c => {
            if (c.count > 0) {
                c.i = c.sumI / c.count;
                c.q = c.sumQ / c.count;
            }
        });
    }

    return clusters;
}

function findNearestCluster(i, q, clusters) {
    let minDist = Infinity;
    let nearest = clusters[0];

    clusters.forEach(cluster => {
        const dist = Math.sqrt(
            Math.pow(i - cluster.i, 2) +
            Math.pow(q - cluster.q, 2)
        );
        if (dist < minDist) {
            minDist = dist;
            nearest = cluster;
        }
    });

    return nearest;
}

function drawConstellation(I, Q) {
    const canvas = document.getElementById('constellation_canvas');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear
    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, width, height);

    // Draw grid
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;

    // Center lines
    ctx.beginPath();
    ctx.moveTo(width / 2, 0);
    ctx.lineTo(width / 2, height);
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.stroke();

    // Circles
    ctx.strokeStyle = '#1a1a1a';
    for (let r of [0.25, 0.5, 0.75, 1.0]) {
        ctx.beginPath();
        ctx.arc(width / 2, height / 2, r * (width / 2) * 0.9, 0, 2 * Math.PI);
        ctx.stroke();
    }

    // Normalize and find max
    let maxVal = 0;
    for (let i = 0; i < I.length; i++) {
        const mag = Math.sqrt(I[i] * I[i] + Q[i] * Q[i]);
        if (mag > maxVal) maxVal = mag;
    }

    // Draw points
    ctx.fillStyle = 'rgba(0, 255, 255, 0.3)';
    const scale = (width / 2) * 0.9 / maxVal;

    for (let i = 0; i < I.length; i++) {
        const x = width / 2 + I[i] * scale;
        const y = height / 2 - Q[i] * scale;

        ctx.beginPath();
        ctx.arc(x, y, 1.5, 0, 2 * Math.PI);
        ctx.fill();
    }

    // Labels
    ctx.fillStyle = '#666';
    ctx.font = '10px monospace';
    ctx.fillText('I', width - 15, height / 2 - 5);
    ctx.fillText('Q', width / 2 + 5, 12);
}

function drawEyeDiagram(I, Q) {
    const canvas = document.getElementById('eye_diagram_canvas');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear
    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, width, height);

    // Estimate symbol rate (assume oversampling)
    const samplesPerSymbol = 8; // Rough guess

    // Draw grid
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.stroke();

    // Find max for scaling
    const maxI = Math.max(...I.map(Math.abs));

    // Draw multiple symbol periods overlaid
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.2)';
    ctx.lineWidth = 1;

    for (let start = 0; start < I.length - samplesPerSymbol * 2; start += samplesPerSymbol) {
        ctx.beginPath();

        for (let i = 0; i < samplesPerSymbol * 2; i++) {
            const idx = start + i;
            if (idx >= I.length) break;

            const x = (i / (samplesPerSymbol * 2)) * width;
            const y = height / 2 - (I[idx] / maxI) * (height / 2) * 0.9;

            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }

        ctx.stroke();
    }

    // Labels
    ctx.fillStyle = '#666';
    ctx.font = '10px monospace';
    ctx.fillText('2 Symbols', 5, 12);
}

console.log('✓ VSA Analysis module loaded');
