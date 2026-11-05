/**
 * Graficador ADC — 2 canales int16_t por Web Serial o WebUSB CDC
 *
 * Protocolo:
 *   Transmisión firmware:
 *     1) Header: "\xA5\xA5" (2 bytes sync) + 2 bytes camera_flag (0/1) + amplitud_excitacion (15 bits superiores)
 *     2) samples_per_period: uint16_t (2 bytes, little-endian)
 *     3) clock_prescaler: uint16_t (2 bytes, little-endian)
 *     4) seq: uint32_t (4 bytes, little-endian) contador de paquete
 *     5) ch0: int16_t[MAX_BUFFER_LEN] -> Canal 1 (256 muestras de amplitud = 512 bytes)
 *     6) ch1: int16_t[MAX_BUFFER_LEN] -> Canal 2 (256 muestras de amplitud = 512 bytes)
 *   Tamaño paquete: HEADER_SIZE + CONFIG_SIZE + SEQ_SIZE + NUM_CH * (MAX_BUFFER_LEN * SAMPLE_SIZE) = 1036 B
 */

// ── Configuración del protocolo ─────────────────────────────────────────────
const BAUD_RATE = 115200;   // Irrelevante para CDC USB
const NUM_CH = 2;
const MAX_BUFFER_LEN = 256; // Muestras por canal transmitidas en el paquete
const SAMPLES_PCH = MAX_BUFFER_LEN;   // Muestras por canal por paquete
const HEADER_SIZE = 4;
const CONFIG_SIZE = 4;      // 2 bytes samples_per_period + 2 bytes clock_prescaler
const SEQ_SIZE = 4;         // 4 bytes uint32_t seq (contador de paquete)
const SAMPLE_SIZE = 2;      // int16_t (2 bytes con signo)
const CH_BUFFER_SIZE = MAX_BUFFER_LEN * SAMPLE_SIZE; // Bytes por canal (512 B)
const PACKET_SIZE = HEADER_SIZE + CONFIG_SIZE + SEQ_SIZE + NUM_CH * CH_BUFFER_SIZE; // 1036 B

const CH_COLORS = ['#4f9cf9', '#f97f4f'];
const CH_GRADIENTS = ['rgba(79,156,249,0.07)', 'rgba(249,127,79,0.07)'];

// ── Utilidades ───────────────────────────────────────────────────────────────
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
const USB_READ_BUFFER_SIZE = 16384;

// ── Estado ───────────────────────────────────────────────────────────────────
let serialPort = null, reader = null, serialWriter = null, keepReading = false;
let isClosingSerialPort = false;
let usbSerialApi = null;
let activeTransport = 'serie';
let rxBuffer = new Uint8Array(65536);
let rxLen = 0;
let syncErrors = 0;
let lostPackets = 0;
let lastPacketSeq = null;
let lastProcessedSample = null;
let currentSamplesPerPeriod = null;
let currentClockPrescaler = null;
let currentFreqExc = null;
let isSendingCommand = false;
let isCameraRecording = false;

let isRecording = false, recordedBuffer = [], recordStartTime = 0;
let recordTimer = null;   // Para grabación temporizada

let packetTimestamps = [];
let isAutoscale = false, yMin = 0, yMax = 4096;
let histLen = 500, decimFactor = 1;
let histories = [new Float32Array(histLen), new Float32Array(histLen)];
let writeIdx = 0;
let currentAveraging = 1;
let maHistory = [[], []];
let maSum = [0, 0];
let amplitudeExcitation = 0;
let lastAmplitudeExcitation = -1;

// ── DOM ──────────────────────────────────────────────────────────────────────
const $ = id => document.getElementById(id);
const btnConnect = $('btn-connect');
const btnUsbConnect = $('btn-usb-connect');
const btnDisconnect = $('btn-disconnect');
const btnRecStart = $('btn-record-start');
const btnRecStop = $('btn-record-stop');
const btnRecTimed = $('btn-record-timed');
const inputRecDur = $('input-rec-duration');
const statusDot = $('status-dot');
const statusText = $('status-text');
const connectionError = $('connection-error');
const recDot = $('recording-dot');
const recText = $('recording-text');
const ratePkts = $('rate-pkts');
const rateSamp = $('rate-samp');
const statErrors = $('stat-errors');
const statLostPkts = $('stat-lost-pkts');
const statSeq = $('stat-seq');
const statRecorded = $('stat-recorded');
const statSamplesPerPeriod = $('stat-samples-per-period');
const statClockPrescaler = $('stat-clock-prescaler');
const statFreqExc = $('stat-freq-exc');
const cameraDot = $('camera-dot');
const cameraText = $('camera-text');
const freqButtons = document.querySelectorAll('.btn-freq');
const avgButtons = document.querySelectorAll('.btn-avg');
const chkAuto = $('chk-autoscale');
const inputYMin = $('input-ymin');
const inputYMax = $('input-ymax');
const inputHist = $('input-history');
const inputDecim = $('input-decim');
const statAmpExc = $('stat-amp-exc');
const timeWindowHint = $('time-window-hint');

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

    const serialSupported = 'serial' in navigator;
    btnConnect.disabled = !serialSupported;
    btnConnect.classList.toggle('hidden', !serialSupported);
    btnUsbConnect.classList.remove('hidden');
    if (/Android/i.test(navigator.userAgent) || !serialSupported) {
        loadUsbSerialApi().catch(error => {
            console.error('No se pudo preparar Web Serial sobre WebUSB:', error.message);
        });
    }

    if (!serialSupported) {
        console.error('Web Serial API no disponible. Usar Chrome, Edge u Opera.');
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
    btnUsbConnect.addEventListener('click', connectUsb);
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
            currentAveraging = parseInt(btn.dataset.avg, 10);
            avgButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            maHistory = [[], []];
            maSum = [0, 0];

            histories = [new Float32Array(histLen), new Float32Array(histLen)];
            writeIdx = 0;
        });
    });
}

// ── Conexión serie ────────────────────────────────────────────────────────────
async function sendDeviceCommand(cmd) {
    if (serialWriter) {
        try {
            await serialWriter.write(new TextEncoder().encode(cmd));
            console.log(`Comando serie enviado: ${JSON.stringify(cmd)}`);
        } catch (err) {
            console.error('Error al enviar comando serie:', err.message);
        }
        return;
    }

    console.warn('No se puede enviar comando: dispositivo no disponible para escritura.');
}

async function setExcitationFrequency(freqHz) {
    if (!serialWriter || !keepReading) {
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
        await sendDeviceCommand(`fexc=${freqHz}\r\n`);
        await new Promise(resolve => setTimeout(resolve, 2000));
        if (keepReading && serialWriter) {
            await sendDeviceCommand("start\r\n");
            lastPacketSeq = null;
            console.log(`Frecuencia de excitación aplicada: ${freqHz} Hz.`);
        }
    } catch (err) {
        console.error('Error al configurar frecuencia de excitación:', err.message);
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

// ── Dispositivo objetivo ──────────────────────────────────────────────────────
const TARGET_DEVICE_NAME = 'Medidor de Burbujas';
const TARGET_USB_VID = 1155;    // 0x0483 (STMicroelectronics)
const TARGET_USB_PID = 22336;   // 0x5740 (Virtual COM Port)

function isMedidorDeBurbujas(port) {
    if (!port) return false;
    const info = (typeof port.getInfo === 'function') ? port.getInfo() : {};

    // Coincidencia con Vendor ID y Product ID del firmware STM32
    if (info.usbVendorId === TARGET_USB_VID && info.usbProductId === TARGET_USB_PID) {
        return true;
    }

    // Coincidencia con nombre textual si el navegador o sistema lo reporta
    const candidateStrings = [
        info.productName,
        info.usbProductName,
        info.name,
        info.friendlyName,
        info.description,
        port.name
    ];
    for (const text of candidateStrings) {
        if (typeof text === 'string' && text.toLowerCase().includes(TARGET_DEVICE_NAME.toLowerCase())) {
            return true;
        }
    }

    return false;
}

async function openSerialPort(port, { transport = 'serie', bufferSize = USB_READ_BUFFER_SIZE } = {}) {
    if (bufferSize) {
        try {
            await port.open({ baudRate: BAUD_RATE, bufferSize });
        } catch {
            await port.open({ baudRate: BAUD_RATE });
        }
    } else {
        await port.open({ baudRate: BAUD_RATE });
    }
    serialPort = port;
    activeTransport = transport;
    try {
        serialWriter = port.writable.getWriter();
        reader = port.readable.getReader();
    } catch (error) {
        try { serialWriter?.releaseLock(); } catch { }
        serialWriter = null;
        reader = null;
        serialPort = null;
        try { await port.close(); } catch (closeError) {
            console.error('Error al cerrar el puerto tras fallar la apertura de streams:', closeError.message);
        }
        throw error;
    }
    console.log(`Puerto conectado vía ${transport}.`);
    startReading();

    // Secuencia de inicialización
    try {
        await setExcitationFrequency(20000);
    } finally {
        resetLostPackets();
        syncErrors = 0;
        if (statErrors) statErrors.textContent = '0';
    }
}

function startReading() {
    rxLen = 0;
    syncErrors = 0;
    if (statErrors) statErrors.textContent = '0';
    resetLostPackets();
    setConnected(true);
    keepReading = true;
    readLoop(reader);
}

async function connectSerial() {
    try {
        // 1. Si ya se concedió permiso previamente, verificar si "Medidor de Burbujas" está disponible
        if ('serial' in navigator) {
            const availablePorts = await navigator.serial.getPorts();
            const matchedPort = availablePorts.find(isMedidorDeBurbujas);
            if (matchedPort) {
                try {
                    console.log(`Puerto "${TARGET_DEVICE_NAME}" detectado automáticamente. Conectando directamente...`);
                    await openSerialPort(matchedPort);
                    return;
                } catch (openErr) {
                    console.warn(`No se pudo conectar directamente al puerto "${TARGET_DEVICE_NAME}":`, openErr.message);
                }
            }
        }

        // 2. Si no se encontró directamente o no se pudo abrir, solicitar selección al usuario
        const requestedPort = await navigator.serial.requestPort();
        if (requestedPort) {
            await openSerialPort(requestedPort);
        }
    } catch (e) {
        if (e.name === 'NotFoundError') {
            console.log('Selección de puerto serie cancelada por el usuario.');
        } else {
            console.error('Error al conectar:', e.message);
        }
    }
}

function loadUsbSerialApi() {
    if (!usbSerialApi) {
        usbSerialApi = window.WebSerialPolyfill?.serial;
    }
    if (!usbSerialApi?.requestPort) {
        return Promise.reject(new Error('No se cargó el polyfill local de WebUSB.'));
    }
    return Promise.resolve(usbSerialApi);
}

async function connectUsb() {
    if (!window.isSecureContext) {
        showConnectionError('WebUSB requiere abrir la aplicación desde una dirección HTTPS.');
        return;
    }
    if (!('usb' in navigator)) {
        showConnectionError('Este navegador no ofrece WebUSB. Prueba con Chrome actualizado y una dirección HTTPS.');
        return;
    }

    try {
        const api = await loadUsbSerialApi();
        const port = await api.requestPort({
            filters: [{ usbVendorId: TARGET_USB_VID }]
        });
        await openSerialPort(port, { transport: 'WebUSB (polyfill)', bufferSize: USB_READ_BUFFER_SIZE });
    } catch (error) {
        if (error.name === 'NotFoundError') {
            console.log('Selección de dispositivo USB cancelada por el usuario.');
            return;
        }
        console.error('Error al conectar vía WebUSB (polyfill):', error.message);
        setConnected(false);
        showConnectionError(`USB: ${error.message}`);
    }
}

async function disconnectSerial() {
    if (isClosingSerialPort) return;
    isClosingSerialPort = true;
    keepReading = false;
    const port = serialPort;
    const activeReader = reader;
    const activeWriter = serialWriter;
    serialPort = null;
    reader = null;
    serialWriter = null;

    try { await activeReader?.cancel(); } catch { }
    try { activeReader?.releaseLock(); } catch { }
    try { activeWriter?.releaseLock(); } catch { }
    if (port) {
        try { await port.close(); } catch (error) {
            console.error('Error al cerrar el puerto serie:', error.message);
        }
    }
    isClosingSerialPort = false;
    setConnected(false);
    console.log('Dispositivo desconectado.');
}

async function readLoop(activeReader) {
    let readError = null;
    let stage = 'lectura';
    let receivedBytes = 0;
    try {
        while (keepReading) {
            stage = 'lectura';
            const { value, done } = await activeReader.read();
            if (done) {
                readError = `${activeTransport}: el flujo terminó (done=true).`;
                break;
            }
            if (value?.byteLength) {
                receivedBytes += value.byteLength;
                stage = 'parser';
                handleBytes(value);
            }
        }
    } catch (error) {
        if (keepReading) {
            readError = `${activeTransport}, etapa ${stage}: ${describeTransportError(error)} ` +
                `(bytes recibidos: ${receivedBytes}).`;
            console.error('Error en el flujo serie:', {
                stage,
                error,
                errorType: typeof error,
                errorName: error?.name,
                errorMessage: error?.message,
                errorStack: error?.stack,
                receivedBytes
            });
        }
    } finally {
        if (keepReading) {
            if (!readError) {
                readError = `${activeTransport}: el flujo se detuvo inesperadamente ` +
                    `(bytes recibidos: ${receivedBytes}).`;
            }
            await disconnectSerial();
            showConnectionError(readError);
        } else {
            try { activeReader.releaseLock(); } catch { }
            if (reader === activeReader) reader = null;
        }
    }
}

function describeTransportError(error) {
    if (error === undefined) return 'rechazo sin valor (undefined)';
    if (error === null) return 'rechazo sin valor (null)';
    if (typeof error === 'object') {
        const name = error.name || 'Error';
        const message = error.message || '';
        return message ? `${name}: ${message}` : `${name} sin mensaje`;
    }
    return `${typeof error}: ${String(error)}`;
}

function showConnectionError(message) {
    connectionError.textContent = message;
    connectionError.classList.remove('hidden');
}

// ── Parser de paquetes ────────────────────────────────────────────────────────
function handleBytes(chunk) {
    if (rxLen + chunk.length > rxBuffer.length) {
        const newBuf = new Uint8Array(Math.max(rxBuffer.length * 2, rxLen + chunk.length + 2048));
        newBuf.set(rxBuffer.subarray(0, rxLen));
        rxBuffer = newBuf;
    }
    rxBuffer.set(chunk, rxLen);
    rxLen += chunk.length;

    let offset = 0;
    while (rxLen - offset >= PACKET_SIZE) {
        if (rxBuffer[offset] === 0xA5 && rxBuffer[offset + 1] === 0xA5) {
            parsePacketAt(offset);
            offset += PACKET_SIZE;
        } else {
            // Buscar siguiente cabecera 0xA5 0xA5
            let nextSync = -1;
            for (let i = offset + 1; i <= rxLen - 2; i++) {
                if (rxBuffer[i] === 0xA5 && rxBuffer[i + 1] === 0xA5) {
                    nextSync = i;
                    break;
                }
            }
            if (nextSync !== -1) {
                syncErrors++;
                if (statErrors) statErrors.textContent = syncErrors;
                offset = nextSync;
            } else {
                offset = Math.max(offset, rxLen - 3);
                break;
            }
        }
    }

    if (offset > 0) {
        if (offset < rxLen) {
            rxBuffer.copyWithin(0, offset, rxLen);
            rxLen -= offset;
        } else {
            rxLen = 0;
        }
    }
}

function parsePacketAt(offset) {
    const cameraFlag = rxBuffer[offset + 2] & 0x01;
    updateCameraIndicator(cameraFlag === 1);

    amplitudeExcitation = ((rxBuffer[offset + 3] << 7) | (rxBuffer[offset + 2] >> 1)) & 0x7FFF;
    if (amplitudeExcitation !== lastAmplitudeExcitation) {
        lastAmplitudeExcitation = amplitudeExcitation;
        if (statAmpExc) statAmpExc.textContent = amplitudeExcitation;
    }

    const dv = new DataView(rxBuffer.buffer, rxBuffer.byteOffset + offset, PACKET_SIZE);

    // Leer configuración (uint16_t, little-endian)
    const samplesPerPeriod = dv.getUint16(HEADER_SIZE, true);
    const clockPrescaler = dv.getUint16(HEADER_SIZE + 2, true);
    // Leer número de secuencia (uint32_t, little-endian)
    const packetSeq = dv.getUint32(HEADER_SIZE + CONFIG_SIZE, true);

    // Detección de paquetes perdidos mediante seq
    let lostInThisGap = 0;
    if (lastPacketSeq !== null) {
        const diff = (packetSeq - lastPacketSeq) >>> 0;
        if (diff > 1 && diff <= 10000) {
            lostInThisGap = diff - 1;
            lostPackets += lostInThisGap;
            if (statLostPkts) statLostPkts.textContent = lostPackets;
        }
    }
    lastPacketSeq = packetSeq;

    // Use zero-copy views when aligned; sync recovery can find a header at an odd byte offset.
    const ch0Offset = rxBuffer.byteOffset + offset + HEADER_SIZE + CONFIG_SIZE + SEQ_SIZE;
    const ch1Offset = ch0Offset + CH_BUFFER_SIZE;
    let ch0, ch1;
    if (ch0Offset % SAMPLE_SIZE === 0 && ch1Offset % SAMPLE_SIZE === 0) {
        ch0 = new Int16Array(rxBuffer.buffer, ch0Offset, MAX_BUFFER_LEN);
        ch1 = new Int16Array(rxBuffer.buffer, ch1Offset, MAX_BUFFER_LEN);
    } else {
        ch0 = new Int16Array(MAX_BUFFER_LEN);
        ch1 = new Int16Array(MAX_BUFFER_LEN);
        const ch0DataOffset = HEADER_SIZE + CONFIG_SIZE + SEQ_SIZE;
        const ch1DataOffset = ch0DataOffset + CH_BUFFER_SIZE;
        for (let i = 0; i < MAX_BUFFER_LEN; i++) {
            ch0[i] = dv.getInt16(ch0DataOffset + i * SAMPLE_SIZE, true);
            ch1[i] = dv.getInt16(ch1DataOffset + i * SAMPLE_SIZE, true);
        }
    }
    const ch = [ch0, ch1];

    processPacket(ch, samplesPerPeriod, clockPrescaler, lostInThisGap);
    packetTimestamps.push({ t: performance.now(), n: SAMPLES_PCH });
}

function processPacket(ch, samplesPerPeriod, clockPrescaler, lostInThisGap = 0) {
    const outSamples = SAMPLES_PCH; // 256 muestras directas de amplitud

    let outCh = ch;
    if (currentAveraging > 1) {
        outCh = [new Float32Array(outSamples), new Float32Array(outSamples)];
        for (let c = 0; c < NUM_CH; c++) {
            const hist = maHistory[c];
            const avgLen = currentAveraging;
            for (let s = 0; s < outSamples; s++) {
                const val = ch[c][s];
                hist.push(val);
                maSum[c] += val;
                if (hist.length > avgLen) {
                    maSum[c] -= hist.shift();
                }
                outCh[c][s] = maSum[c] / hist.length;
            }
        }
    }

    // Ingresar las muestras de cada canal al historial circular (Float32Array)
    for (let s = 0; s < outSamples; s++) {
        for (let c = 0; c < NUM_CH; c++) {
            histories[c][writeIdx] = outCh[c][s];
        }
        writeIdx = (writeIdx + 1) % histLen;
    }

    // Mostrar último valor de cada canal
    for (let c = 0; c < NUM_CH; c++) {
        valBadges[c].textContent = outSamples > 0 ? Math.round(outCh[c][outSamples - 1]) : "—";
    }

    // Actualizar configuración en pantalla (solo si cambió)
    updateConfigDisplay(samplesPerPeriod, clockPrescaler);

    // Grabar si está activo
    if (isRecording) {
        const SAMPLES_PER_FRAME = SAMPLES_PCH;

        // Si se perdieron paquetes previos, replicar la última muestra (256 por paquete perdido)
        if (lostInThisGap > 0) {
            let fillCh0 = 0, fillCh1 = 0, fillCam = isCameraRecording ? 1 : 0;
            if (recordedBuffer.length > 0) {
                const lastRow = recordedBuffer[recordedBuffer.length - 1];
                fillCh0 = lastRow[0];
                fillCh1 = lastRow[1];
                fillCam = lastRow[2];
            } else if (lastProcessedSample !== null) {
                fillCh0 = lastProcessedSample[0];
                fillCh1 = lastProcessedSample[1];
                fillCam = lastProcessedSample[2];
            } else {
                fillCh0 = Math.abs(ch[0][0]);
                fillCh1 = Math.abs(ch[1][0]);
                fillCam = isCameraRecording ? 1 : 0;
            }

            for (let p = 0; p < lostInThisGap; p++) {
                for (let s = 0; s < SAMPLES_PER_FRAME; s++) {
                    recordedBuffer.push([fillCh0, fillCh1, fillCam]);
                }
            }
        }

        // Registrar las 256 medidas de amplitud del paquete actual (valor absoluto) + sincronismo de la cámara
        const currentCam = isCameraRecording ? 1 : 0;
        for (let j = 0; j < SAMPLES_PER_FRAME; j++) {
            recordedBuffer.push([
                Math.abs(ch[0][j]),
                Math.abs(ch[1][j]),
                currentCam
            ]);
        }
        statRecorded.textContent = recordedBuffer.length;
    }

    // Actualizar última muestra procesada para replicar en caso de pérdida futura
    const lastSampleIdx = SAMPLES_PCH - 1;
    lastProcessedSample = [
        Math.abs(ch[0][lastSampleIdx]),
        Math.abs(ch[1][lastSampleIdx]),
        isCameraRecording ? 1 : 0
    ];
}

function updateConfigDisplay(samplesPerPeriod, clockPrescaler) {
    if (samplesPerPeriod === currentSamplesPerPeriod && clockPrescaler === currentClockPrescaler) {
        return;
    }
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

    setExcitationFrequency(20000);
    delay(1000);

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

    let csv = '# freq_exc: ' + currentFreqExc + '; amplitude_exc: ' + amplitudeExcitation + ';\r\nCh1;Ch2;Cam\r\n';
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
        if (timeWindowHint) timeWindowHint.textContent = 'Ventana temporal: —';
        return;
    }
    const win = Math.max((now - packetTimestamps[0].t) / 1000, 0.1);
    const totalN = packetTimestamps.reduce((s, p) => s + p.n, 0);
    const sampPerSec = totalN / win;
    ratePkts.textContent = (packetTimestamps.length / win).toFixed(1);
    rateSamp.textContent = sampPerSec.toFixed(0);
    if (statSeq && lastPacketSeq !== null) statSeq.textContent = lastPacketSeq;

    if (timeWindowHint) {
        if (sampPerSec > 0) {
            const secs = histLen / sampPerSec;
            timeWindowHint.textContent = secs >= 1
                ? `Ventana temporal: ~${secs.toFixed(2)} s`
                : `Ventana temporal: ~${Math.round(secs * 1000)} ms`;
        } else {
            timeWindowHint.textContent = 'Ventana temporal: —';
        }
    }
}

// ── Render de gráficos con decimación Min-Max en tiempo real ─────────────────
const MAX_DRAW_POINTS = 8192;
const drawX = new Float32Array(MAX_DRAW_POINTS);
const drawY = new Float32Array(MAX_DRAW_POINTS);

function drawLoop() {
    drawCharts();
    requestAnimationFrame(drawLoop);
}

function drawCharts() {
    const userStep = Math.max(1, Math.round(decimFactor));

    for (let c = 0; c < NUM_CH; c++) {
        const canvas = canvases[c];
        const ctx = ctxs[c];
        const dpr = window.devicePixelRatio || 1;
        const W = canvas.width / dpr;
        const H = canvas.height / dpr;

        // Fondo
        ctx.fillStyle = '#0d0d0f';
        ctx.fillRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = 'rgba(255,255,255,0.04)';
        ctx.lineWidth = 1;
        for (let i = 1; i < 10; i++) {
            const x = (W / 10) * i;
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, H);
            ctx.stroke();
        }
        for (let i = 1; i < 5; i++) {
            const y = (H / 5) * i;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(W, y);
            ctx.stroke();
        }

        const rawLen = histLen;
        const hChannel = histories[c];

        // Límite de columnas horizontales adaptado a la resolución de pantalla
        const maxColumns = Math.max(64, Math.floor(W));
        const targetPoints = Math.min(maxColumns, Math.ceil(rawLen / userStep));

        let globalMin = Infinity;
        let globalMax = -Infinity;
        let pointCount = 0;

        if (targetPoints >= rawLen) {
            // Muestreo 1:1 directo (para buffers pequeños)
            for (let i = 0; i < rawLen; i++) {
                let rIdx = writeIdx + i;
                if (rIdx >= rawLen) rIdx -= rawLen;
                const v = hChannel[rIdx];
                if (v < globalMin) globalMin = v;
                if (v > globalMax) globalMax = v;
                drawX[i] = (W / (rawLen - 1 || 1)) * i;
                drawY[i] = v;
            }
            pointCount = rawLen;
        } else {
            // Min-Max (Envelope) Downsampling en una sola pasada:
            // Garantiza capturar el 100% de los picos/valles de burbujas sin retrasar el loop de lectura USB
            const numBuckets = Math.min(targetPoints, Math.floor(MAX_DRAW_POINTS / 2));
            const bucketSize = rawLen / numBuckets;

            let ptIdx = 0;
            for (let b = 0; b < numBuckets; b++) {
                const iStart = Math.floor(b * bucketSize);
                const iEnd = Math.min(rawLen, Math.floor((b + 1) * bucketSize));

                let bMin = Infinity, bMax = -Infinity;
                let bMinIdx = iStart, bMaxIdx = iStart;

                for (let i = iStart; i < iEnd; i++) {
                    let rIdx = writeIdx + i;
                    if (rIdx >= rawLen) rIdx -= rawLen;
                    const v = hChannel[rIdx];
                    if (v < bMin) { bMin = v; bMinIdx = i; }
                    if (v > bMax) { bMax = v; bMaxIdx = i; }
                }

                if (bMin < globalMin) globalMin = bMin;
                if (bMax > globalMax) globalMax = bMax;

                const x = (W / (numBuckets - 1 || 1)) * b;

                // Mantener el orden cronológico dentro del intervalo para preservar la forma de onda
                if (bMinIdx <= bMaxIdx) {
                    drawX[ptIdx] = x; drawY[ptIdx++] = bMin;
                    if (bMin !== bMax && ptIdx < MAX_DRAW_POINTS) {
                        drawX[ptIdx] = x; drawY[ptIdx++] = bMax;
                    }
                } else {
                    drawX[ptIdx] = x; drawY[ptIdx++] = bMax;
                    if (bMin !== bMax && ptIdx < MAX_DRAW_POINTS) {
                        drawX[ptIdx] = x; drawY[ptIdx++] = bMin;
                    }
                }
            }
            pointCount = ptIdx;
        }

        // Amplitud pico-pico
        ppBadges[c].textContent = (globalMax === -Infinity) ? '—' : Math.round(globalMax - globalMin);

        // Escala vertical
        let lo, hi;
        if (isAutoscale) {
            if (globalMin === Infinity) { globalMin = 0; globalMax = 1; }
            if (globalMin === globalMax) { lo = globalMin - 1; hi = globalMax + 1; }
            else {
                const margin = (globalMax - globalMin) * 0.07;
                lo = globalMin - margin;
                hi = globalMax + margin;
            }
        } else {
            lo = yMin;
            hi = yMax || yMin + 1;
        }

        // Etiquetas de escala
        ctx.fillStyle = 'rgba(255,255,255,0.3)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(Math.round(hi), 5, 12);
        ctx.fillText(Math.round(lo), 5, H - 4);

        if (pointCount < 2) continue;

        // Convertir drawY de amplitudes a coordenadas en píxeles
        const scaleY = H / (hi - lo || 1);
        for (let i = 0; i < pointCount; i++) {
            drawY[i] = H - (drawY[i] - lo) * scaleY;
        }

        // Trazado de señal
        ctx.beginPath();
        ctx.moveTo(drawX[0], drawY[0]);
        for (let i = 1; i < pointCount; i++) {
            ctx.lineTo(drawX[i], drawY[i]);
        }

        ctx.strokeStyle = CH_COLORS[c];
        ctx.lineWidth = 1.5;

        // Glow ligero solo con buffers pequeños para no impactar rendimiento
        if (pointCount <= 500) {
            ctx.shadowBlur = 4;
            ctx.shadowColor = CH_COLORS[c];
        } else {
            ctx.shadowBlur = 0;
        }

        ctx.stroke();
        ctx.shadowBlur = 0;

        // Relleno degradado
        ctx.lineTo(drawX[pointCount - 1], H);
        ctx.lineTo(drawX[0], H);
        ctx.closePath();
        ctx.fillStyle = CH_GRADIENTS[c];
        ctx.fill();
    }
}

// ── Indicador de cámara ────────────────────────────────────────────────────────────────
function updateCameraIndicator(isOn) {
    if (isOn === isCameraRecording) return;
    isCameraRecording = isOn;
    cameraDot.className = 'dot ' + (isOn ? 'dot-cam' : 'dot-off');
    cameraText.textContent = isOn ? 'Cámara: Grabando' : 'Cámara: Off';
}

// ── Estado de conexión ────────────────────────────────────────────────────────────────
function setConnected(on) {
    statusDot.className = 'dot ' + (on ? 'dot-on' : 'dot-off');
    statusText.textContent = on ? 'Conectado' : 'Desconectado';
    if (on) {
        connectionError.textContent = '';
        connectionError.classList.add('hidden');
    }
    btnConnect.classList.toggle('hidden', on || !('serial' in navigator));
    btnUsbConnect.classList.toggle('hidden', on);
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
        rxLen = 0;
        packetTimestamps = [];
        updateRateStats();
        resetConfigDisplay();
        updateCameraIndicator(false);
        resetLostPackets();
    }
    updateCommandButtonsState();
}

function resetLostPackets() {
    lostPackets = 0;
    lastPacketSeq = null;
    lastProcessedSample = null;
    if (statLostPkts) statLostPkts.textContent = '0';
    if (statSeq) statSeq.textContent = '—';
}

// ── Redimensionar historial ───────────────────────────────────────────────────
function resizeHistory(newLen) {
    const newH = [new Float32Array(newLen), new Float32Array(newLen)];
    const count = Math.min(histLen, newLen);
    for (let c = 0; c < NUM_CH; c++) {
        for (let i = 0; i < count; i++) {
            let oldIdx = writeIdx - count + i;
            if (oldIdx < 0) oldIdx += histLen;
            newH[c][i] = histories[c][oldIdx];
        }
    }
    histories = newH;
    histLen = newLen;
    writeIdx = count % newLen;
    updateRateStats();
    console.log(`Historial ajustado a ${newLen} muestras.`);
}
