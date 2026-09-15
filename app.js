/**
 * Graficador ADC — 2 canales int16_t via CDC USB (puerto COM)
 *
 * Protocolo:
 *   Transmisión firmware:
 *     1) Header: "\xA5\xA5\xA5\xA5" (4 bytes) -> 0xA5A5A5A5
 *     2) samples_per_period: uint16_t (2 bytes, little-endian)
 *     3) clock_prescaler: uint16_t (2 bytes, little-endian)
 *     4) avg_0: int16_t[MAX_BUFFER_LEN] -> Canal 1
 *     5) avg_1: int16_t[MAX_BUFFER_LEN] -> Canal 2
 *   Tamaño paquete: HEADER_SIZE + CONFIG_SIZE + NUM_CH * (MAX_BUFFER_LEN * SAMPLE_SIZE)
 */

// ── Configuración del protocolo ─────────────────────────────────────────────
const BAUD_RATE = 115200;   // Irrelevante para CDC USB
const NUM_CH = 2;
const MAX_BUFFER_LEN = 256;// modifico para la diferencia entre semiciclos positivos y negativos 256; // Muestras por canal transmitidas en el paquete
const SAMPLES_PCH = MAX_BUFFER_LEN;   // Muestras por canal por paquete
const HEADER_SIZE = 4;
const CONFIG_SIZE = 4;      // 2 bytes samples_per_period + 2 bytes clock_prescaler
const SAMPLE_SIZE = 2;      // int16_t (2 bytes con signo)
const CH_BUFFER_SIZE = MAX_BUFFER_LEN * SAMPLE_SIZE; // Bytes por canal
const PACKET_SIZE = HEADER_SIZE + CONFIG_SIZE + NUM_CH * CH_BUFFER_SIZE;

const CH_COLORS = ['#4f9cf9', '#f97f4f'];
const CH_GRADIENTS = ['rgba(79,156,249,0.07)', 'rgba(249,127,79,0.07)'];

// ── Estado ───────────────────────────────────────────────────────────────────
let serialPort = null, reader = null, keepReading = false;
let byteBuffer = [];
let syncErrors = 0;
let currentSamplesPerPeriod = null;
let currentClockPrescaler = null;
let currentFreqExc = null;
let isSendingCommand = false;

let isRecording = false, recordedBuffer = [], recordStartTime = 0;
let recordTimer = null;   // Para grabación temporizada

let packetTimestamps = [];
let isAutoscale = false, yMin = 0, yMax = 5000;
let histLen = 500, decimFactor = 1;
let histories = [new Array(histLen).fill(0), new Array(histLen).fill(0)];
let writeIdx = 0;

// ── DOM ──────────────────────────────────────────────────────────────────────
const $ = id => document.getElementById(id);
const btnConnect = $('btn-connect');
const btnDisconnect = $('btn-disconnect');
const btnRecStart = $('btn-record-start');
const btnRecStop = $('btn-record-stop');
const btnRecTimed = $('btn-record-timed');
const inputRecDur = $('input-rec-duration');
const statusDot = $('status-dot');
const statusText = $('status-text');
const recDot = $('recording-dot');
const recText = $('recording-text');
const ratePkts = $('rate-pkts');
const rateSamp = $('rate-samp');
const statErrors = $('stat-errors');
const statRecorded = $('stat-recorded');
const statSamplesPerPeriod = $('stat-samples-per-period');
const statClockPrescaler = $('stat-clock-prescaler');
const statFreqExc = $('stat-freq-exc');
const freqButtons = document.querySelectorAll('.btn-freq');
const avgButtons = document.querySelectorAll('.btn-avg');
const chkAuto = $('chk-autoscale');
const inputYMin = $('input-ymin');
const inputYMax = $('input-ymax');
const inputHist = $('input-history');
const inputDecim = $('input-decim');

const canvases = [$('canvas-ch1'), $('canvas-ch2')];
const ctxs = canvases.map(c => c.getContext('2d'));
const valBadges = [$('val-ch1'), $('val-ch2')];
const ppBadges = [$('pp-ch1'), $('pp-ch2')];

// ── Init ─────────────────────────────────────────────────────────────────────
window.addEventListener('DOMContentLoaded', () => {
    initCanvases();
    setupEvents();
    requestAnimationFrame(drawLoop);
    setInterval(updateRateStats, 300);

    if (!('serial' in navigator)) {
        console.error('Web Serial API no disponible. Usar Chrome, Edge u Opera.');
        btnConnect.disabled = true;
    }
});

function initCanvases() {
    canvases.forEach(canvas => {
        new ResizeObserver(entries => {
            const { width, height } = entries[0].contentRect;
            const dpr = devicePixelRatio || 1;
            canvas.width = width * dpr;
            canvas.height = height * dpr;
            canvas.getContext('2d').scale(dpr, dpr);
        }).observe(canvas.parentElement);
    });
}

function setupEvents() {
    btnConnect.addEventListener('click', connectSerial);
    btnDisconnect.addEventListener('click', disconnectSerial);
    btnRecStart.addEventListener('click', () => startRecording());
    btnRecStop.addEventListener('click', stopRecording);
    btnRecTimed.addEventListener('click', startTimedRecording);

    chkAuto.addEventListener('change', e => {
        isAutoscale = e.target.checked;
        inputYMin.disabled = inputYMax.disabled = isAutoscale;
    });
    inputYMin.addEventListener('input', () => { yMin = +inputYMin.value || 0; });
    inputYMax.addEventListener('input', () => { yMax = +inputYMax.value || 0; });

    inputHist.addEventListener('change', () => {
        const n = parseInt(inputHist.value);
        if (n >= 64 && n <= 200000) resizeHistory(n);
        else inputHist.value = histLen;
    });
    inputDecim.addEventListener('change', () => {
        const n = parseInt(inputDecim.value);
        decimFactor = (n >= 1 && n <= 1000) ? n : decimFactor;
        inputDecim.value = decimFactor;
    });

    freqButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const freqHz = parseInt(btn.dataset.freq, 10);
            if (freqHz) setExcitationFrequency(freqHz);
        });
    });

    avgButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const avgVal = parseInt(btn.dataset.avg, 10);
            if (avgVal) setAveraging(avgVal);
        });
    });
}

// ── Conexión serie ────────────────────────────────────────────────────────────
async function sendSerialCommand(cmd) {
    if (!serialPort || !serialPort.writable) {
        console.warn('No se puede enviar comando: puerto serie no disponible para escritura.');
        return;
    }
    const encoder = new TextEncoder();
    const writer = serialPort.writable.getWriter();
    try {
        await writer.write(encoder.encode(cmd));
        console.log(`Comando serie enviado: ${JSON.stringify(cmd)}`);
    } catch (err) {
        console.error('Error al enviar comando serie:', err.message);
    } finally {
        writer.releaseLock();
    }
}

async function setExcitationFrequency(freqHz) {
    if (!serialPort || !serialPort.writable || !keepReading) {
        console.warn('No se puede cambiar frecuencia: puerto serie no conectado.');
        return;
    }
    if (isSendingCommand) {
        console.warn('Ya hay un comando en progreso.');
        return;
    }
    isSendingCommand = true;
    updateCommandButtonsState();

    try {
        console.log(`Configurando frecuencia de excitación: ${freqHz} Hz...`);
        await sendSerialCommand(`fexc=${freqHz}\r\n`);
        await new Promise(resolve => setTimeout(resolve, 1000));
        if (keepReading && serialPort?.writable) {
            await sendSerialCommand("start\r\n");
            console.log(`Frecuencia de excitación aplicada: ${freqHz} Hz.`);
        }
    } catch (err) {
        console.error('Error al configurar frecuencia de excitación:', err.message);
    } finally {
        isSendingCommand = false;
        updateCommandButtonsState();
    }
}

async function setAveraging(avgVal) {
    if (!serialPort || !serialPort.writable || !keepReading) {
        console.warn('No se puede cambiar promediado: puerto serie no conectado.');
        return;
    }
    if (isSendingCommand) {
        console.warn('Ya hay un comando en progreso.');
        return;
    }
    isSendingCommand = true;
    updateCommandButtonsState();

    try {
        console.log(`Configurando promediado: ${avgVal}...`);
        await sendSerialCommand(`avg=${avgVal}\r\n`);
        await new Promise(resolve => setTimeout(resolve, 1000));
        if (keepReading && serialPort?.writable) {
            await sendSerialCommand("start\r\n");
            console.log(`Promediado aplicado: ${avgVal}.`);
        }
        highlightActiveAvgButton(avgVal);
    } catch (err) {
        console.error('Error al configurar promediado:', err.message);
    } finally {
        isSendingCommand = false;
        updateCommandButtonsState();
    }
}

function updateCommandButtonsState() {
    const isConnected = !!(serialPort && keepReading);
    freqButtons.forEach(btn => {
        btn.disabled = !isConnected || isSendingCommand;
    });
    avgButtons.forEach(btn => {
        btn.disabled = !isConnected || isSendingCommand;
    });
}

function highlightActiveFreqButton(freqHz) {
    if (!freqButtons) return;
    freqButtons.forEach(btn => {
        if (!freqHz) {
            btn.classList.remove('active');
            return;
        }
        const targetHz = parseInt(btn.dataset.freq, 10);
        if (targetHz && Math.abs(freqHz - targetHz) / targetHz < 0.03) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
}

function highlightActiveAvgButton(avgVal) {
    if (!avgButtons) return;
    avgButtons.forEach(btn => {
        const targetAvg = parseInt(btn.dataset.avg, 10);
        if (avgVal === targetAvg) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
}

async function connectSerial() {
    try {
        serialPort = await navigator.serial.requestPort();
        await serialPort.open({ baudRate: BAUD_RATE });
        console.log('Puerto serie conectado.');
        setConnected(true);
        keepReading = true;
        readLoop();

        // Secuencia de inicialización
        await setAveraging(1);
    } catch (e) {
        console.error('Error al conectar:', e.message);
    }
}

async function disconnectSerial() {
    keepReading = false;
    if (reader) { try { await reader.cancel(); } catch { } }
    setTimeout(async () => {
        if (serialPort) {
            try { await serialPort.close(); } catch { }
            serialPort = null;
        }
        setConnected(false);
        console.log('Puerto serie desconectado.');
    }, 150);
}

async function readLoop() {
    while (serialPort?.readable && keepReading) {
        try {
            reader = serialPort.readable.getReader();
            while (keepReading) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) handleBytes(value);
            }
        } catch (e) {
            console.error('Error de lectura:', e.message);
            break;
        } finally {
            reader?.releaseLock();
            reader = null;
        }
    }
    if (keepReading) { console.warn('Conexión perdida.'); setConnected(false); }
}

// ── Parser de paquetes ────────────────────────────────────────────────────────
function handleBytes(chunk) {
    for (let i = 0; i < chunk.length; i++) byteBuffer.push(chunk[i]);

    while (byteBuffer.length >= PACKET_SIZE) {
        if (byteBuffer[0] === 0xA5 && byteBuffer[1] === 0xA5 &&
            byteBuffer[2] === 0xA5 && byteBuffer[3] === 0xA5) {

            const packetBytes = new Uint8Array(byteBuffer.slice(0, PACKET_SIZE));
            const dv = new DataView(packetBytes.buffer);

            // Leer configuración (uint16_t, little-endian)
            const samplesPerPeriod = dv.getUint16(HEADER_SIZE, true);
            const clockPrescaler = dv.getUint16(HEADER_SIZE + 2, true);

            // Leer buffers contiguos de int16_t (avg_0 y avg_1)
            const ch = [[], []];
            let offset = HEADER_SIZE + CONFIG_SIZE; // offset 8
            for (let i = 0; i < MAX_BUFFER_LEN; i++) {
                ch[0].push(dv.getInt16(offset, true));
                offset += SAMPLE_SIZE;
            }
            for (let i = 0; i < MAX_BUFFER_LEN; i++) {
                ch[1].push(dv.getInt16(offset, true));
                offset += SAMPLE_SIZE;
            }

            processPacket(ch, samplesPerPeriod, clockPrescaler);
            packetTimestamps.push({ t: performance.now(), n: SAMPLES_PCH });
            byteBuffer.splice(0, PACKET_SIZE);

        } else {
            // Buscar siguiente header
            let idx = -1;
            for (let i = 1; i <= byteBuffer.length - HEADER_SIZE; i++) {
                if (byteBuffer[i] === 0xA5 && byteBuffer[i + 1] === 0xA5 &&
                    byteBuffer[i + 2] === 0xA5 && byteBuffer[i + 3] === 0xA5) {
                    idx = i; break;
                }
            }
            if (idx !== -1) {
                byteBuffer.splice(0, idx);
                syncErrors++;
                statErrors.textContent = syncErrors;
            } else {
                byteBuffer = byteBuffer.slice(-3);
                break;
            }
        }
    }
}

function processPacket(ch, samplesPerPeriod, clockPrescaler) {
    // Ingresar las 256 muestras de cada canal al historial circular
    for (let s = 0; s < SAMPLES_PCH; s++) {
        for (let c = 0; c < NUM_CH; c++) histories[c][writeIdx] = ch[c][s];
        writeIdx = (writeIdx + 1) % histLen;
    }

    // Mostrar último valor de cada canal
    for (let c = 0; c < NUM_CH; c++) {
        valBadges[c].textContent = ch[c][SAMPLES_PCH - 1];
    }

    // Actualizar configuración en pantalla
    updateConfigDisplay(samplesPerPeriod, clockPrescaler);

    // Grabar si está activo
    if (isRecording) {
        for (let s = 0; s < SAMPLES_PCH; s++) {
            recordedBuffer.push([ch[0][s], ch[1][s]]);
        }
        statRecorded.textContent = recordedBuffer.length;
    }
}

function updateConfigDisplay(samplesPerPeriod, clockPrescaler) {
    currentSamplesPerPeriod = samplesPerPeriod;
    currentClockPrescaler = clockPrescaler;

    if (statSamplesPerPeriod) {
        statSamplesPerPeriod.textContent = samplesPerPeriod;
    }
    if (statClockPrescaler) {
        statClockPrescaler.textContent = clockPrescaler;
    }

    if (samplesPerPeriod > 0 && clockPrescaler > 0) {
        const freqHz = 72e6 / samplesPerPeriod / clockPrescaler;
        currentFreqExc = freqHz;
        if (statFreqExc) {
            if (freqHz >= 1000) {
                const khz = freqHz / 1000;
                statFreqExc.textContent = `${khz.toFixed(2)} kHz`;
            } else {
                statFreqExc.textContent = `${Math.round(freqHz)} Hz`;
            }
            statFreqExc.title = `${freqHz.toFixed(1)} Hz`;
        }
        highlightActiveFreqButton(freqHz);
    } else {
        currentFreqExc = null;
        if (statFreqExc) {
            statFreqExc.textContent = '—';
            statFreqExc.removeAttribute('title');
        }
        highlightActiveFreqButton(null);
    }
}

function resetConfigDisplay() {
    currentSamplesPerPeriod = null;
    currentClockPrescaler = null;
    currentFreqExc = null;
    if (statSamplesPerPeriod) {
        statSamplesPerPeriod.textContent = '—';
    }
    if (statClockPrescaler) {
        statClockPrescaler.textContent = '—';
    }
    if (statFreqExc) {
        statFreqExc.textContent = '—';
        statFreqExc.removeAttribute('title');
    }
    highlightActiveFreqButton(null);
    highlightActiveAvgButton(null);
}

// ── Grabación CSV ─────────────────────────────────────────────────────────────
function startRecording(durationMs = null) {
    if (isRecording) return;
    recordedBuffer = []; recordStartTime = Date.now(); isRecording = true;
    recDot.classList.remove('hidden'); recText.classList.remove('hidden');
    btnRecStart.classList.add('hidden');
    btnRecTimed.classList.add('hidden');
    btnRecStop.classList.remove('hidden');

    if (durationMs) {
        const secs = (durationMs / 1000).toFixed(1);
        recText.textContent = `Grabando ${secs}s`;
        recordTimer = setTimeout(() => stopRecording(), durationMs);
    } else {
        recText.textContent = 'Grabando';
    }
    console.log(`Grabación iniciada${durationMs ? ` (${durationMs / 1000}s)` : ' (manual)'}.`);
}

function startTimedRecording() {
    const secs = parseFloat(inputRecDur.value);
    if (!secs || secs <= 0) { console.warn('Duración inválida.'); return; }
    startRecording(secs * 1000);
}

function stopRecording() {
    if (!isRecording) return;
    isRecording = false;
    clearTimeout(recordTimer); recordTimer = null;
    recDot.classList.add('hidden'); recText.classList.add('hidden');
    recText.textContent = 'Grabando';
    btnRecStop.classList.add('hidden');
    btnRecStart.classList.remove('hidden');
    btnRecTimed.classList.remove('hidden');

    console.log(`Grabación detenida. ${recordedBuffer.length} filas.`);

    if (!recordedBuffer.length) { console.warn('Sin datos para guardar.'); return; }

    let csv = 'Ch1;Ch2\r\n';
    for (const row of recordedBuffer) csv += row.join(';') + '\r\n';

    const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const a = Object.assign(document.createElement('a'), {
        href: URL.createObjectURL(new Blob([csv], { type: 'text/csv' })),
        download: `adc_${ts}.csv`
    });
    document.body.appendChild(a); a.click(); document.body.removeChild(a);
    console.log('CSV descargado.');
    statRecorded.textContent = '0';
}

// ── Tasa de recepción ─────────────────────────────────────────────────────────
function updateRateStats() {
    const now = performance.now(), cutoff = now - 1000;
    while (packetTimestamps.length && packetTimestamps[0].t < cutoff) packetTimestamps.shift();

    if (!keepReading || !packetTimestamps.length) {
        ratePkts.textContent = rateSamp.textContent = '—';
        return;
    }
    const win = Math.max((now - packetTimestamps[0].t) / 1000, 0.1);
    const totalN = packetTimestamps.reduce((s, p) => s + p.n, 0);
    ratePkts.textContent = (packetTimestamps.length / win).toFixed(1);
    rateSamp.textContent = (totalN / win).toFixed(0);
}

// ── Render de gráficos con decimación ────────────────────────────────────────
function drawLoop() {
    drawCharts();
    requestAnimationFrame(drawLoop);
}

function drawCharts() {
    const step = Math.max(1, Math.round(decimFactor));

    for (let c = 0; c < NUM_CH; c++) {
        const canvas = canvases[c];
        const ctx = ctxs[c];
        const W = canvas.width / (devicePixelRatio || 1);
        const H = canvas.height / (devicePixelRatio || 1);

        // Fondo
        ctx.fillStyle = '#0d0d0f';
        ctx.fillRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = 'rgba(255,255,255,0.04)';
        ctx.lineWidth = 1;
        for (let i = 1; i < 10; i++) { const x = W / 10 * i; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke(); }
        for (let i = 1; i < 5; i++) { const y = H / 5 * i; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }

        // Datos en orden cronológico (buffer circular completo)
        const rawLen = histLen;
        const drawLen = Math.ceil(rawLen / step);

        // Calcular min/max sobre datos crudos para la escala
        let lo, hi;
        if (isAutoscale) {
            lo = Infinity; hi = -Infinity;
            for (let i = 0; i < rawLen; i++) {
                const v = histories[c][(writeIdx + i) % rawLen];
                if (v < lo) lo = v;
                if (v > hi) hi = v;
            }
            if (lo === hi) { lo -= 1; hi += 1; }
            else { const m = (hi - lo) * 0.07; lo -= m; hi += m; }
        } else { lo = yMin; hi = yMax || yMin + 1; }

        // Amplitud pico-pico (sobre datos crudos)
        let rawMin = Infinity, rawMax = -Infinity;
        for (let i = 0; i < rawLen; i++) {
            const v = histories[c][(writeIdx + i) % rawLen];
            if (v < rawMin) rawMin = v;
            if (v > rawMax) rawMax = v;
        }
        ppBadges[c].textContent = rawMax === -Infinity ? '—' : (rawMax - rawMin);

        // Etiquetas de escala
        ctx.fillStyle = 'rgba(255,255,255,0.3)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(Math.round(hi), 5, 12);
        ctx.fillText(Math.round(lo), 5, H - 4);

        // Trazar señal con decimación
        const getX = i => (W / (drawLen - 1 || 1)) * i;
        const getY = v => H - ((v - lo) / (hi - lo)) * H;

        ctx.beginPath();
        let started = false;
        for (let di = 0; di < drawLen; di++) {
            const rawIdx = (writeIdx + di * step) % rawLen;
            const v = histories[c][rawIdx];
            const x = getX(di), y = getY(v);
            if (!started) { ctx.moveTo(x, y); started = true; }
            else ctx.lineTo(x, y);
        }

        ctx.strokeStyle = CH_COLORS[c];
        ctx.lineWidth = 1.5;
        ctx.shadowBlur = 4;
        ctx.shadowColor = CH_COLORS[c];
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Relleno degradado
        ctx.lineTo(getX(drawLen - 1), H);
        ctx.lineTo(0, H);
        ctx.closePath();
        ctx.fillStyle = CH_GRADIENTS[c];
        ctx.fill();
    }
}

// ── Estado de conexión ────────────────────────────────────────────────────────
function setConnected(on) {
    statusDot.className = 'dot ' + (on ? 'dot-on' : 'dot-off');
    statusText.textContent = on ? 'Conectado' : 'Desconectado';
    btnConnect.classList.toggle('hidden', on);
    btnDisconnect.classList.toggle('hidden', !on);
    btnRecStart.disabled = !on;
    btnRecTimed.disabled = !on;
    if (!on && isRecording) {
        isRecording = false;
        clearTimeout(recordTimer); recordTimer = null;
        recDot.classList.add('hidden'); recText.classList.add('hidden');
        recText.textContent = 'Grabando';
        btnRecStop.classList.add('hidden');
        btnRecStart.classList.remove('hidden');
        btnRecTimed.classList.remove('hidden');
        console.warn('Grabación cancelada por desconexión.');
    }
    if (!on) {
        packetTimestamps = [];
        updateRateStats();
        resetConfigDisplay();
    }
    updateCommandButtonsState();
}

// ── Redimensionar historial ───────────────────────────────────────────────────
function resizeHistory(newLen) {
    const newH = [new Array(newLen).fill(0), new Array(newLen).fill(0)];
    for (let c = 0; c < NUM_CH; c++) {
        const ordered = [];
        for (let i = 0; i < histLen; i++) ordered.push(histories[c][(writeIdx + i) % histLen]);
        const start = Math.max(0, ordered.length - newLen);
        for (let i = start; i < ordered.length; i++) newH[c][i - start] = ordered[i];
    }
    histories = newH;
    histLen = newLen;
    writeIdx = 0;
    console.log(`Historial ajustado a ${newLen} muestras.`);
}
