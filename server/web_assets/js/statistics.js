// ============================================================================
// STATISTICS & CCDF ANALYSIS
// Power statistics, histograms, and CCDF plots
// ============================================================================

const Statistics = {
    powerHistory: [],
    maxHistorySize: 10000
};

function toggleStatsPanel() {
    const panel = document.getElementById('stats_panel');
    const isVisible = panel.style.display !== 'none';
    panel.style.display = isVisible ? 'none' : 'block';
    document.getElementById('stats_toggle').classList.toggle('active', !isVisible);
}

function updateStatistics() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;

    // Convert to power values (dBm)
    const powers = data.map(val => rawToDbm(val, binBW));

    // Add to history
    powers.forEach(p => {
        Statistics.powerHistory.push(p);
        if (Statistics.powerHistory.length > Statistics.maxHistorySize) {
            Statistics.powerHistory.shift();
        }
    });

    // Calculate statistics
    const sorted = [...Statistics.powerHistory].sort((a, b) => a - b);
    const n = sorted.length;

    const mean = sorted.reduce((a, b) => a + b, 0) / n;
    const median = sorted[Math.floor(n / 2)];

    // Variance and std dev
    const variance = sorted.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / n;
    const stdDev = Math.sqrt(variance);

    // Skewness
    const skewness = sorted.reduce((sum, val) => sum + Math.pow((val - mean) / stdDev, 3), 0) / n;

    // Kurtosis (excess)
    const kurtosis = (sorted.reduce((sum, val) => sum + Math.pow((val - mean) / stdDev, 4), 0) / n) - 3;

    // Update display
    document.getElementById('stats_mean').textContent = mean.toFixed(2) + ' dBm';
    document.getElementById('stats_median').textContent = median.toFixed(2) + ' dBm';
    document.getElementById('stats_stddev').textContent = stdDev.toFixed(2) + ' dB';
    document.getElementById('stats_variance').textContent = variance.toFixed(2);
    document.getElementById('stats_skew').textContent = skewness.toFixed(3);
    document.getElementById('stats_kurt').textContent = kurtosis.toFixed(3);

    // Draw plots
    drawCCDF(sorted);
    drawHistogram(sorted);

    console.log('✓ Statistics updated');
}

function resetStatistics() {
    Statistics.powerHistory = [];
    document.getElementById('stats_mean').textContent = '-- dBm';
    document.getElementById('stats_median').textContent = '-- dBm';
    document.getElementById('stats_stddev').textContent = '-- dB';
    document.getElementById('stats_variance').textContent = '--';
    document.getElementById('stats_skew').textContent = '--';
    document.getElementById('stats_kurt').textContent = '--';

    const ccdfCanvas = document.getElementById('ccdf_canvas');
    const ccdfCtx = ccdfCanvas.getContext('2d');
    ccdfCtx.fillStyle = '#0a0a0a';
    ccdfCtx.fillRect(0, 0, ccdfCanvas.width, ccdfCanvas.height);

    const histCanvas = document.getElementById('histogram_canvas');
    const histCtx = histCanvas.getContext('2d');
    histCtx.fillStyle = '#0a0a0a';
    histCtx.fillRect(0, 0, histCanvas.width, histCanvas.height);

    console.log('✓ Statistics reset');
}

function drawCCDF(sortedPowers) {
    const canvas = document.getElementById('ccdf_canvas');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear
    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, width, height);

    if (sortedPowers.length < 10) return;

    // Draw grid
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;

    // Horizontal lines (log scale 0.1%, 1%, 10%, 100%)
    const probabilities = [0.001, 0.01, 0.1, 1.0];
    probabilities.forEach(prob => {
        const y = height * (1 - Math.log10(prob * 100 + 0.1) / 2.1); // Log scale
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();

        // Label
        ctx.fillStyle = '#666';
        ctx.font = '9px monospace';
        ctx.fillText(`${(prob * 100).toFixed(1)}%`, 5, y - 2);
    });

    // Vertical grid
    for (let i = 0; i <= 10; i++) {
        const x = (i / 10) * width;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }

    // Calculate CCDF
    const ccdf = [];
    const minPower = sortedPowers[0];
    const maxPower = sortedPowers[sortedPowers.length - 1];
    const powerRange = maxPower - minPower;

    const numPoints = 200;
    for (let i = 0; i <= numPoints; i++) {
        const threshold = minPower + (i / numPoints) * powerRange;
        const count = sortedPowers.filter(p => p >= threshold).length;
        const probability = count / sortedPowers.length;
        ccdf.push({ threshold, probability });
    }

    // Draw CCDF curve
    ctx.strokeStyle = '#0ff';
    ctx.lineWidth = 2;
    ctx.beginPath();

    ccdf.forEach((point, idx) => {
        const x = ((point.threshold - minPower) / powerRange) * width;
        // Log scale for probability
        const logProb = Math.max(0.0001, point.probability);
        const y = height * (1 - Math.log10(logProb * 100 + 0.1) / 2.1);

        if (idx === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });

    ctx.stroke();

    // Draw axis labels
    ctx.fillStyle = '#888';
    ctx.font = '10px monospace';
    ctx.fillText('Power (dBm) →', width - 80, height - 5);
    ctx.save();
    ctx.translate(15, height / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('CCDF (%)', 0, 0);
    ctx.restore();

    // Draw power axis labels
    ctx.fillStyle = '#666';
    ctx.font = '9px monospace';
    for (let i = 0; i <= 4; i++) {
        const power = minPower + (i / 4) * powerRange;
        const x = (i / 4) * width;
        ctx.fillText(power.toFixed(1), x - 15, height - 2);
    }
}

function drawHistogram(sortedPowers) {
    const canvas = document.getElementById('histogram_canvas');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear
    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, width, height);

    if (sortedPowers.length < 10) return;

    // Create histogram bins
    const numBins = 50;
    const minPower = sortedPowers[0];
    const maxPower = sortedPowers[sortedPowers.length - 1];
    const binWidth = (maxPower - minPower) / numBins;

    const bins = new Array(numBins).fill(0);

    sortedPowers.forEach(power => {
        const binIdx = Math.min(Math.floor((power - minPower) / binWidth), numBins - 1);
        bins[binIdx]++;
    });

    // Find max count for scaling
    const maxCount = Math.max(...bins);

    // Draw grid
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
        const y = (i / 5) * height;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Draw bars
    const barWidth = width / numBins;

    bins.forEach((count, idx) => {
        const barHeight = (count / maxCount) * height * 0.9;
        const x = idx * barWidth;
        const y = height - barHeight;

        // Gradient fill
        const gradient = ctx.createLinearGradient(0, y, 0, height);
        gradient.addColorStop(0, '#0ff');
        gradient.addColorStop(1, '#008888');

        ctx.fillStyle = gradient;
        ctx.fillRect(x, y, barWidth - 1, barHeight);
    });

    // Draw labels
    ctx.fillStyle = '#888';
    ctx.font = '10px monospace';
    ctx.fillText('Power Distribution', 5, 12);

    // X-axis labels
    ctx.fillStyle = '#666';
    ctx.font = '9px monospace';
    for (let i = 0; i <= 4; i++) {
        const power = minPower + (i / 4) * (maxPower - minPower);
        const x = (i / 4) * width;
        ctx.fillText(power.toFixed(1), x - 15, height - 2);
    }
}

console.log('✓ Statistics module loaded');
