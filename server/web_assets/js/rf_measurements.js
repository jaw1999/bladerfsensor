// ============================================================================
// RF MEASUREMENTS MODULE
// Professional RF measurement functions for spectrum analysis
// ============================================================================

// Global state for RF measurements
const RFMeasurements = {
    referenceTrace: null,
    noiseFloorEstimate: -100,
    gainOffset: 0  // dBm calibration offset
};

// Tab switching
function switchMeasTab(tab) {
    // Hide all tabs
    document.querySelectorAll('.meas-content').forEach(el => el.style.display = 'none');
    document.querySelectorAll('.meas-tab').forEach(el => el.classList.remove('active'));

    // Show selected tab
    document.getElementById(`meas-content-${tab}`).style.display = 'block';
    document.getElementById(`tab-${tab}`).classList.add('active');
}

// Convert raw FFT magnitude to dBm (calibrated power measurement)
function rawToDbm(raw, binBandwidth, gainOffset = 0) {
    // Convert 8-bit magnitude to dBFS
    const dBFS = -100 + (raw / 255.0) * 100;

    // Reference: 0 dBFS = reference power level
    // Adjust for RBW (resolution bandwidth per bin)
    const rbwCorrection = 10 * Math.log10(binBandwidth);

    // Apply gain offset calibration
    return dBFS + rbwCorrection + gainOffset;
}

// Calculate effective resolution bandwidth
function calculateRBW() {
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const fftSize = FFT_SIZE;

    // Effective RBW depends on window function
    // Hamming window has ENBW factor of ~1.36
    const windowFactor = 1.36;
    const rbw = (sr / fftSize) * windowFactor;

    return rbw;
}

// ============================================================================
// BASIC RF MEASUREMENTS
// ============================================================================

function runBasicMeasurements() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;
    const rbw = calculateRBW();

    // Power measurements
    let peakRaw = 0, peakIdx = 0;
    let sumLinear = 0, sumSquare = 0;

    for (let i = 0; i < data.length; i++) {
        const linear = Math.pow(10, rawToDbm(data[i], binBW) / 10);
        sumLinear += linear;
        sumSquare += data[i] * data[i];

        if (data[i] > peakRaw) {
            peakRaw = data[i];
            peakIdx = i;
        }
    }

    const avgLinear = sumLinear / data.length;
    const peakPower = rawToDbm(peakRaw, binBW);
    const avgPower = 10 * Math.log10(avgLinear);
    const rmsPower = 10 * Math.log10(sumSquare / data.length);
    const crestFactor = peakPower - avgPower;

    // Occupied Bandwidth calculations
    const obw99 = calculateOBW(data, sr, 0.99);
    const obw3db = calculateOBW_3dB(data, sr);
    const obw20db = calculateOBW_NdB(data, sr, 20);

    // Measure center frequency (power-weighted)
    let weightedSum = 0, totalPower = 0;
    for (let i = 0; i < data.length; i++) {
        const freq = cf - (sr / 2) + (i * sr / FFT_SIZE);
        const power = Math.pow(10, rawToDbm(data[i], binBW) / 10);
        weightedSum += freq * power;
        totalPower += power;
    }
    const measuredCenter = weightedSum / totalPower;

    // Noise floor estimation (lowest 10th percentile)
    const sorted = Array.from(data).sort((a, b) => a - b);
    const noiseFloorRaw = sorted[Math.floor(sorted.length * 0.1)];
    const noiseFloor = rawToDbm(noiseFloorRaw, binBW);
    RFMeasurements.noiseFloorEstimate = noiseFloor;

    // SNR calculation
    const snr = peakPower - noiseFloor;

    // SINAD (Signal to Noise and Distortion)
    // Simplified: assuming harmonics are in noise floor
    const sinad = snr - 1; // Rough approximation

    // SFDR (Spurious Free Dynamic Range)
    const sfdr = calculateSFDR(data, peakIdx, binBW);

    // Update display
    document.getElementById('rf_peak_power').textContent = peakPower.toFixed(2) + ' dBm';
    document.getElementById('rf_avg_power').textContent = avgPower.toFixed(2) + ' dBm';
    document.getElementById('rf_rms_power').textContent = rmsPower.toFixed(2) + ' dBm';
    document.getElementById('rf_crest_factor').textContent = crestFactor.toFixed(2) + ' dB';

    document.getElementById('rf_obw_99').textContent = (obw99 / 1e6).toFixed(3) + ' MHz';
    document.getElementById('rf_obw_3db').textContent = (obw3db / 1e6).toFixed(3) + ' MHz';
    document.getElementById('rf_obw_20db').textContent = (obw20db / 1e6).toFixed(3) + ' MHz';
    document.getElementById('rf_center_measured').textContent = (measuredCenter / 1e6).toFixed(6) + ' MHz';

    document.getElementById('rf_noise_floor').textContent = noiseFloor.toFixed(2) + ' dBm';
    document.getElementById('rf_snr').textContent = snr.toFixed(2) + ' dB';
    document.getElementById('rf_sinad').textContent = sinad.toFixed(2) + ' dB';
    document.getElementById('rf_sfdr').textContent = sfdr.toFixed(2) + ' dBc';

    console.log('✓ Basic measurements complete');
}

// Calculate Occupied Bandwidth (99% power method)
function calculateOBW(data, sampleRate, percentage) {
    const binBW = sampleRate / FFT_SIZE;

    // Calculate total power
    let totalPower = 0;
    const powers = data.map(val => {
        const p = Math.pow(10, rawToDbm(val, binBW) / 10);
        totalPower += p;
        return p;
    });

    const targetPower = totalPower * percentage;

    // Find bandwidth containing target percentage of power
    let maxBW = 0;
    for (let start = 0; start < data.length; start++) {
        let accum = 0;
        for (let end = start; end < data.length; end++) {
            accum += powers[end];
            if (accum >= targetPower) {
                const bw = (end - start + 1) * binBW;
                if (bw < maxBW || maxBW === 0) {
                    maxBW = bw;
                }
                break;
            }
        }
    }

    return maxBW;
}

// Calculate OBW using -3dB points
function calculateOBW_3dB(data, sampleRate) {
    const peakVal = Math.max(...data);
    const threshold = peakVal * 0.707; // -3dB = 0.707

    let firstIdx = -1, lastIdx = -1;
    for (let i = 0; i < data.length; i++) {
        if (data[i] >= threshold) {
            if (firstIdx === -1) firstIdx = i;
            lastIdx = i;
        }
    }

    if (firstIdx === -1) return 0;
    return ((lastIdx - firstIdx + 1) * sampleRate) / FFT_SIZE;
}

// Calculate OBW using -NdB points
function calculateOBW_NdB(data, sampleRate, dBdown) {
    const peakVal = Math.max(...data);
    const threshold = peakVal * Math.pow(10, -dBdown / 20);

    let firstIdx = -1, lastIdx = -1;
    for (let i = 0; i < data.length; i++) {
        if (data[i] >= threshold) {
            if (firstIdx === -1) firstIdx = i;
            lastIdx = i;
        }
    }

    if (firstIdx === -1) return 0;
    return ((lastIdx - firstIdx + 1) * sampleRate) / FFT_SIZE;
}

// Calculate Spurious Free Dynamic Range
function calculateSFDR(data, fundamentalIdx, binBW) {
    const fundPower = rawToDbm(data[fundamentalIdx], binBW);

    // Find highest spur excluding fundamental (±5 bins)
    let maxSpurRaw = 0;
    for (let i = 0; i < data.length; i++) {
        if (Math.abs(i - fundamentalIdx) > 5) {
            maxSpurRaw = Math.max(maxSpurRaw, data[i]);
        }
    }

    const spurPower = rawToDbm(maxSpurRaw, binBW);
    return fundPower - spurPower; // SFDR in dBc
}

// ============================================================================
// CHANNEL POWER & ACPR MEASUREMENTS
// ============================================================================

function runChannelPowerMeasurements() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;

    // Get integration bandwidth
    const chanBW = parseFloat(document.getElementById('chan_bw').value) * 1e6;
    const acprOffset = parseFloat(document.getElementById('acpr_offset').value) * 1e6;

    // Calculate number of bins for channel
    const chanBins = Math.round(chanBW / binBW);
    const centerBin = Math.floor(FFT_SIZE / 2);

    // Channel power (integrate over bandwidth)
    let chanPowerLinear = 0;
    const startBin = Math.max(0, centerBin - Math.floor(chanBins / 2));
    const endBin = Math.min(FFT_SIZE - 1, centerBin + Math.floor(chanBins / 2));

    for (let i = startBin; i <= endBin; i++) {
        chanPowerLinear += Math.pow(10, rawToDbm(data[i], binBW) / 10);
    }

    const channelPower = 10 * Math.log10(chanPowerLinear);
    const powerDensity = channelPower - 10 * Math.log10(chanBW);

    // ACPR measurements
    const offsetBins = Math.round(acprOffset / binBW);

    // Lower adjacent channel
    let lowerACPLinear = 0;
    const lowerStart = Math.max(0, centerBin - offsetBins - Math.floor(chanBins / 2));
    const lowerEnd = Math.max(0, centerBin - offsetBins + Math.floor(chanBins / 2));
    for (let i = lowerStart; i <= lowerEnd; i++) {
        lowerACPLinear += Math.pow(10, rawToDbm(data[i], binBW) / 10);
    }
    const lowerACPR = 10 * Math.log10(lowerACPLinear) - channelPower;

    // Upper adjacent channel
    let upperACPLinear = 0;
    const upperStart = Math.min(FFT_SIZE - 1, centerBin + offsetBins - Math.floor(chanBins / 2));
    const upperEnd = Math.min(FFT_SIZE - 1, centerBin + offsetBins + Math.floor(chanBins / 2));
    for (let i = upperStart; i <= upperEnd; i++) {
        upperACPLinear += Math.pow(10, rawToDbm(data[i], binBW) / 10);
    }
    const upperACPR = 10 * Math.log10(upperACPLinear) - channelPower;

    // Alternate channels (2x offset)
    let altLowerLinear = 0;
    const altLowerStart = Math.max(0, centerBin - 2 * offsetBins - Math.floor(chanBins / 2));
    const altLowerEnd = Math.max(0, centerBin - 2 * offsetBins + Math.floor(chanBins / 2));
    for (let i = altLowerStart; i <= altLowerEnd; i++) {
        altLowerLinear += Math.pow(10, rawToDbm(data[i], binBW) / 10);
    }
    const altLowerACLR = 10 * Math.log10(altLowerLinear) - channelPower;

    let altUpperLinear = 0;
    const altUpperStart = Math.min(FFT_SIZE - 1, centerBin + 2 * offsetBins - Math.floor(chanBins / 2));
    const altUpperEnd = Math.min(FFT_SIZE - 1, centerBin + 2 * offsetBins + Math.floor(chanBins / 2));
    for (let i = altUpperStart; i <= altUpperEnd; i++) {
        altUpperLinear += Math.pow(10, rawToDbm(data[i], binBW) / 10);
    }
    const altUpperACLR = 10 * Math.log10(altUpperLinear) - channelPower;

    // Update display
    document.getElementById('rf_chan_power').textContent = channelPower.toFixed(2) + ' dBm';
    document.getElementById('rf_power_density').textContent = powerDensity.toFixed(2) + ' dBm/Hz';
    document.getElementById('rf_acpr_lower').textContent = lowerACPR.toFixed(2) + ' dBc';
    document.getElementById('rf_acpr_upper').textContent = upperACPR.toFixed(2) + ' dBc';
    document.getElementById('rf_aclr_lower').textContent = altLowerACLR.toFixed(2) + ' dBc';
    document.getElementById('rf_aclr_upper').textContent = altUpperACLR.toFixed(2) + ' dBc';

    console.log('✓ Channel power measurements complete');
}

// ============================================================================
// SPECTRAL ANALYSIS
// ============================================================================

function runSpectralAnalysis() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;

    // Spectral flatness (Wiener entropy)
    let arithmeticMean = 0;
    let geometricMean = 1;
    let sumSquares = 0;

    for (let i = 0; i < data.length; i++) {
        const val = data[i] + 1; // Avoid log(0)
        arithmeticMean += val;
        geometricMean *= Math.pow(val, 1.0 / data.length);
        sumSquares += val * val;
    }
    arithmeticMean /= data.length;

    const spectralFlatness = geometricMean / arithmeticMean;

    // Spectral entropy
    let entropy = 0;
    const sumForNorm = data.reduce((a, b) => a + b, 0);
    for (let i = 0; i < data.length; i++) {
        const p = data[i] / sumForNorm;
        if (p > 0) {
            entropy -= p * Math.log2(p);
        }
    }
    entropy /= Math.log2(data.length); // Normalize

    // Spectral kurtosis (measure of "peakiness")
    const mean = arithmeticMean;
    const variance = (sumSquares / data.length) - (mean * mean);
    const stdDev = Math.sqrt(variance);

    let fourthMoment = 0;
    for (let i = 0; i < data.length; i++) {
        fourthMoment += Math.pow((data[i] - mean) / stdDev, 4);
    }
    const kurtosis = (fourthMoment / data.length) - 3; // Excess kurtosis

    // Effective RBW
    const rbw = calculateRBW();

    // Update display
    document.getElementById('rf_spectral_flatness').textContent = spectralFlatness.toFixed(4);
    document.getElementById('rf_spectral_entropy').textContent = entropy.toFixed(4);
    document.getElementById('rf_spectral_kurtosis').textContent = kurtosis.toFixed(3);
    document.getElementById('rf_rbw').textContent = (rbw / 1000).toFixed(2) + ' kHz';

    // Peak analysis with excursion
    const threshold = parseFloat(document.getElementById('peak_threshold_pro').value);
    const excursion = parseFloat(document.getElementById('peak_excursion').value);

    const peaks = findPeaksWithExcursion(data, threshold, excursion, sr, cf, binBW);
    displayPeakTable(peaks);

    console.log('✓ Spectral analysis complete');
}

// Find peaks with minimum excursion
function findPeaksWithExcursion(data, thresholdDb, excursionDb, sampleRate, centerFreq, binBW) {
    const thresholdRaw = ((thresholdDb + 100) / 100) * 255;
    const excursionRaw = (excursionDb / 100) * 255;

    const peaks = [];

    for (let i = 10; i < data.length - 10; i++) {
        if (data[i] < thresholdRaw) continue;

        // Check if local maximum
        let isLocalMax = true;
        let minAround = data[i];

        for (let j = i - 10; j <= i + 10; j++) {
            if (j !== i && data[j] > data[i]) {
                isLocalMax = false;
                break;
            }
            minAround = Math.min(minAround, data[j]);
        }

        // Check excursion
        if (isLocalMax && (data[i] - minAround) >= excursionRaw) {
            const freq = centerFreq - (sampleRate / 2) + (i * sampleRate / FFT_SIZE);
            const power = rawToDbm(data[i], binBW);
            peaks.push({ bin: i, freq: freq, power: power });
        }
    }

    // Sort by power
    peaks.sort((a, b) => b.power - a.power);

    return peaks.slice(0, 20);
}

// Display peak table
function displayPeakTable(peaks) {
    const tbody = document.getElementById('peak_table_body');
    tbody.innerHTML = '';

    if (peaks.length === 0) {
        tbody.innerHTML = '<tr><td colspan="4" style="text-align: center; color: #888;">No peaks found</td></tr>';
        return;
    }

    peaks.forEach((peak, idx) => {
        const deltaF = idx > 0 ? (peak.freq - peaks[idx - 1].freq) / 1000 : 0;
        const row = `<tr style="cursor: pointer;" onmouseover="this.style.background='#1a1a1a'" onmouseout="this.style.background=''">
            <td style="padding: 3px; color: #0ff;">${idx + 1}</td>
            <td style="padding: 3px; text-align: right;">${(peak.freq / 1e6).toFixed(6)}</td>
            <td style="padding: 3px; text-align: right; color: #0f0;">${peak.power.toFixed(2)}</td>
            <td style="padding: 3px; text-align: right; color: #888;">${deltaF.toFixed(1)}</td>
        </tr>`;
        tbody.innerHTML += row;
    });
}

// ============================================================================
// HARMONIC & SPURIOUS ANALYSIS
// ============================================================================

function findFundamental() {
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

    const fundFreq = cf - (sr / 2) + (peakIdx * sr / FFT_SIZE);
    document.getElementById('fund_freq').value = (fundFreq / 1e6).toFixed(6);

    console.log('Found fundamental at', (fundFreq / 1e6).toFixed(3), 'MHz');
}

function runHarmonicAnalysis() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;
    const fundFreq = parseFloat(document.getElementById('fund_freq').value) * 1e6;

    // Find power at fundamental
    const fundBin = Math.round(((fundFreq - cf) + sr / 2) / binBW);
    const fundPower = rawToDbm(data[fundBin], binBW);

    // Find 2nd harmonic
    const h2Freq = fundFreq * 2;
    const h2Bin = Math.round(((h2Freq - cf) + sr / 2) / binBW);
    const h2Power = (h2Bin >= 0 && h2Bin < data.length) ?
        rawToDbm(data[h2Bin], binBW) - fundPower : -999;

    // Find 3rd harmonic
    const h3Freq = fundFreq * 3;
    const h3Bin = Math.round(((h3Freq - cf) + sr / 2) / binBW);
    const h3Power = (h3Bin >= 0 && h3Bin < data.length) ?
        rawToDbm(data[h3Bin], binBW) - fundPower : -999;

    // Calculate THD (Total Harmonic Distortion)
    let thdLinear = 0;
    if (h2Power > -900) thdLinear += Math.pow(10, h2Power / 10);
    if (h3Power > -900) thdLinear += Math.pow(10, h3Power / 10);
    const thd = 10 * Math.log10(thdLinear);

    // Update display
    document.getElementById('rf_h1_power').textContent = fundPower.toFixed(2) + ' dBm';
    document.getElementById('rf_h2_power').textContent = h2Power > -900 ? h2Power.toFixed(2) + ' dBc' : 'N/A';
    document.getElementById('rf_h3_power').textContent = h3Power > -900 ? h3Power.toFixed(2) + ' dBc' : 'N/A';
    document.getElementById('rf_thd').textContent = thd > -900 ? thd.toFixed(2) + ' dBc' : 'N/A';

    console.log('✓ Harmonic analysis complete');
}

function searchSpurious() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No spectrum data available');
        return;
    }

    const data = latestFFTData;
    const sr = parseFloat(document.getElementById('srInput').value) * 1e6;
    const cf = parseFloat(document.getElementById('freqInput').value) * 1e6;
    const binBW = sr / FFT_SIZE;
    const span = document.getElementById('spur_span').value;

    // Find fundamental
    let fundIdx = 0, fundVal = 0;
    for (let i = 0; i < data.length; i++) {
        if (data[i] > fundVal) {
            fundVal = data[i];
            fundIdx = i;
        }
    }
    const fundPower = rawToDbm(data[fundIdx], binBW);
    const fundFreq = cf - (sr / 2) + (fundIdx * sr / FFT_SIZE);

    // Define search range based on span
    let searchStart = 0, searchEnd = data.length - 1;
    const nearBandBins = Math.round(5e6 / binBW);

    if (span === 'nearband') {
        searchStart = Math.max(0, fundIdx - nearBandBins);
        searchEnd = Math.min(data.length - 1, fundIdx + nearBandBins);
    } else if (span === 'farband') {
        // Two ranges: below and above near-band
        // For simplicity, search full span excluding near-band
    }

    // Find spurs
    const spurs = [];
    const excludeBins = 10; // Exclude ±10 bins around fundamental

    for (let i = searchStart; i <= searchEnd; i++) {
        if (Math.abs(i - fundIdx) < excludeBins) continue;

        // Check if local maximum
        if (i > 0 && i < data.length - 1) {
            if (data[i] > data[i - 1] && data[i] > data[i + 1] && data[i] > 50) {
                const freq = cf - (sr / 2) + (i * sr / FFT_SIZE);
                const power = rawToDbm(data[i], binBW);
                const relative = power - fundPower;
                spurs.push({ freq: freq, power: power, relative: relative });
            }
        }
    }

    // Sort by power (highest first)
    spurs.sort((a, b) => b.power - a.power);

    // Display
    const listDiv = document.getElementById('spurious_list');
    if (spurs.length === 0) {
        listDiv.innerHTML = '<div style="color: #888;">No spurious emissions found</div>';
    } else {
        let html = '<div style="font-size: 9px;">';
        spurs.slice(0, 15).forEach((spur, idx) => {
            html += `<div style="padding: 2px 0; border-bottom: 1px solid #222;">
                <span style="color: #f90;">${idx + 1}.</span>
                <span style="color: #0ff;">${(spur.freq / 1e6).toFixed(3)} MHz</span>
                <span style="color: #0f0; margin-left: 5px;">${spur.power.toFixed(1)} dBm</span>
                <span style="color: #888; margin-left: 5px;">(${spur.relative.toFixed(1)} dBc)</span>
            </div>`;
        });
        html += '</div>';
        listDiv.innerHTML = html;
    }

    console.log(`✓ Found ${spurs.length} spurious emissions`);
}

// ============================================================================
// REFERENCE TRACE MANAGEMENT
// ============================================================================

function saveReferenceTrace() {
    if (!latestFFTData || latestFFTData.length === 0) {
        alert('No data to save');
        return;
    }

    RFMeasurements.referenceTrace = new Uint8Array(latestFFTData);
    console.log('✓ Reference trace saved');
    alert('Reference trace saved');
}

function clearReferenceTrace() {
    RFMeasurements.referenceTrace = null;
    console.log('✓ Reference trace cleared');
}

console.log('✓ RF Measurements module loaded');
