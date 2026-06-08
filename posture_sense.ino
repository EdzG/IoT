// ============================================================
//  Posture Sense — Main Firmware
//  Board  : Heltec Wireless Tracker (ESP32-S3)
//  I2C    : SDA = GPIO 46 | SCL = GPIO 45
//  TCA    : 0x70  |  IMUs on channels 1, 2, 7
// ============================================================

#include <Wire.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <vector>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// ─────────────────────────────────────────
// Pin & bus constants
// ─────────────────────────────────────────
#define I2C_SDA       46
#define I2C_SCL       45
#define TCA_ADDR      0x70

#define CH_UPPER      1    // TCA channel → upper-back IMU
#define CH_MID        2    // TCA channel → mid-back IMU
#define CH_LOWER      7    // TCA channel → lower-back IMU

// ─────────────────────────────────────────
// Timing
// ─────────────────────────────────────────
#define SAMPLE_INTERVAL_US   250000UL   // 250 ms = 4 Hz
#define COUNTDOWN_MS         3000UL     // 3 s before calibration starts
#define CAL_SECONDS          5          // calibration collection window
#define CAL_SAMPLES          (CAL_SECONDS * 4)   // 20 samples @ 4 Hz
#define CAL_MIN_VALID        15         // accept cal if ≥15 of 20 samples valid
#define FROZEN_TIMEOUT_MS    300000UL   // 5 min — auto-resume if no ACK
#define ALERT_INTERVAL_MS    10000UL    // minimum gap between repeated alerts

// ─────────────────────────────────────────
// EMA & alert thresholds
// ─────────────────────────────────────────
#define EMA_ALPHA            0.05f
#define ALERT_THRESHOLD      0.65f      // EMA score that fires an alert
#define MAX_CONSECUTIVE_BAD  3          // invalid IMU readings before ERROR

// ─────────────────────────────────────────
// Posture detection thresholds (degrees)
// ─────────────────────────────────────────
// All values are research-informed starting points.
// Empirical calibration required after final IMU mounting.
//
//  Thoracic Slouch      : upper–mid pitch delta > 10°
//  Forward Flexion      : upper pitch alone     > 20°  (forward-head proxy)
//  Lateral Lean         : max roll spread       > 10°  (clinical scoliosis cutoff)
//  Lumbar Hyperlordosis : mid–lower pitch delta > 10°  (excessive inward arch)
//  Lumbar Flattening    : mid–lower pitch delta < −10° (posterior pelvic tilt)
#define THR_THORACIC_SLOUCH       10.0f
#define THR_FORWARD_FLEXION       20.0f
#define THR_LATERAL_LEAN          10.0f
#define THR_LUMBAR_HYPERLORDOSIS  10.0f
#define THR_LUMBAR_FLATTENING    -10.0f

// ─────────────────────────────────────────
// Posture flag bitmask (uint8_t)
// ─────────────────────────────────────────
#define POST_GOOD                 0x00
#define POST_THORACIC_SLOUCH      0x01
#define POST_FORWARD_FLEXION      0x02
#define POST_LATERAL_LEAN         0x04
#define POST_LUMBAR_HYPERLORDOSIS 0x08
#define POST_LUMBAR_FLATTENING    0x10

// ─────────────────────────────────────────
// Calibration stability
// ─────────────────────────────────────────
#define CAL_VARIANCE_LIMIT   2.0f   // degrees² — max acceptable variance

// ─────────────────────────────────────────
// WiFi / network
// ─────────────────────────────────────────
#define AP_SSID  "PostureSense"
#define AP_PASS  "12345678"

// ─────────────────────────────────────────
// Structs
// ─────────────────────────────────────────

// Raw orientation from one IMU after Madgwick fusion
struct IMUReading {
  float pitch, roll, yaw;
  bool  valid;  // false if IMU was not ready this tick
};

// All three IMU readings bundled together
struct SpineReading {
  IMUReading upper, mid, lower;
  bool allValid;  // true only when all three valid
};

// Calibrated baseline angles (recorded during CALIBRATING state)
struct Baseline {
  float pitchUpper, rollUpper;
  float pitchMid,   rollMid;
  float pitchLower, rollLower;
};

// ─────────────────────────────────────────
// State machine
// ─────────────────────────────────────────
enum SystemState {
  WAITING,           // idle — waiting for webapp to send START_CAL
  COUNTDOWN,         // 3-second countdown before collection begins
  CALIBRATING,       // collecting baseline samples
  WAITING_FOR_START, // calibration done — waiting for START_MON
  MONITORING,        // active posture detection
  FROZEN             // alert fired — waiting for ACK from webapp
};

// ─────────────────────────────────────────
// Dashboard HTML (PROGMEM — served at /)
// ─────────────────────────────────────────
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PostureSense</title>
  <style>
    /* ── reset & tokens ──────────────────────────────────────────────────── */
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    :root {
      --bg:        #0d1117;
      --surface:   #161b22;
      --surface-2: #21262d;
      --border:    #30363d;
      --text-1:    #e6edf3;
      --text-2:    #8b949e;
      --green:     #3fb950;
      --orange:    #f0883e;
      --red:       #f85149;
      --blue:      #58a6ff;
      --yellow:    #d29922;
      --radius:    10px;
    }

    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: var(--bg);
      color: var(--text-1);
      min-height: 100vh;
      padding: 1.5rem;
    }

    .page { max-width: 1400px; margin: 0 auto; }

    /* ── header ──────────────────────────────────────────────────────────── */
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 1.25rem;
      gap: 1rem;
    }

    .logo { display: flex; align-items: center; gap: 0.6rem; }

    .logo-spine {
      display: flex;
      flex-direction: column;
      gap: 2px;
      opacity: 0.85;
    }
    .logo-spine span {
      display: block;
      background: var(--blue);
      border-radius: 2px;
      transition: background 0.3s;
    }
    .logo-spine span:nth-child(1) { width: 18px; height: 4px; }
    .logo-spine span:nth-child(2) { width: 14px; height: 4px; margin-left: 2px; }
    .logo-spine span:nth-child(3) { width: 10px; height: 4px; margin-left: 4px; }

    h1 {
      font-size: 1.45rem;
      font-weight: 700;
      letter-spacing: -0.02em;
    }
    h1 em { font-style: normal; color: var(--blue); }

    .conn-badge {
      display: flex;
      align-items: center;
      gap: 0.45rem;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 999px;
      padding: 0.35rem 0.9rem;
      font-size: 0.78rem;
      color: var(--text-2);
      flex-shrink: 0;
    }
    .conn-dot {
      width: 8px; height: 8px;
      border-radius: 50%;
      background: var(--border);
      transition: background 0.3s;
      flex-shrink: 0;
    }
    .conn-dot.on  { background: var(--green); box-shadow: 0 0 6px var(--green); }
    .conn-dot.off { background: var(--red); }

    /* ── state banner ────────────────────────────────────────────────────── */
    .state-banner {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 0.9rem 1.1rem;
      margin-bottom: 1rem;
      display: flex;
      align-items: center;
      gap: 1rem;
      flex-wrap: wrap;
    }

    .state-chip {
      font-size: 0.68rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--text-2);
      border: 1px solid var(--border);
      border-radius: 999px;
      padding: 0.2rem 0.6rem;
      flex-shrink: 0;
    }

    #state-msg {
      font-size: 0.9rem;
      flex: 1;
      min-width: 0;
    }

    .cal-wrap {
      display: none;
      align-items: center;
      gap: 0.6rem;
      min-width: 160px;
    }
    .cal-wrap.show { display: flex; }

    .cal-bar {
      flex: 1;
      height: 6px;
      background: var(--surface-2);
      border-radius: 999px;
      overflow: hidden;
    }
    #cal-fill {
      height: 100%;
      width: 0%;
      background: var(--blue);
      border-radius: 999px;
      transition: width 0.3s ease;
    }
    #cal-pct { font-size: 0.75rem; color: var(--text-2); min-width: 2.2rem; text-align: right; }

    /* ── controls ────────────────────────────────────────────────────────── */
    .controls {
      display: flex;
      gap: 0.75rem;
      flex-wrap: wrap;
      margin-bottom: 1.25rem;
    }

    .btn {
      flex: 1;
      min-width: 110px;
      padding: 0.62rem 1rem;
      border: 1px solid var(--border);
      border-radius: var(--radius);
      background: var(--surface);
      color: var(--text-1);
      font-size: 0.85rem;
      font-weight: 500;
      cursor: pointer;
      transition: background 0.15s, opacity 0.15s, border-color 0.15s;
    }
    .btn:hover:not(:disabled) { background: var(--surface-2); }
    .btn:disabled { opacity: 0.3; cursor: not-allowed; }

    .btn-blue   { border-color: var(--blue); color: var(--blue); }
    .btn-blue:hover:not(:disabled)   { background: rgba(88,166,255,.08); }
    .btn-green  { border-color: var(--green); color: var(--green); }
    .btn-green:hover:not(:disabled)  { background: rgba(63,185,80,.08); }
    .btn-red    { border-color: var(--red); color: var(--red); }
    .btn-red:hover:not(:disabled)    { background: rgba(248,81,73,.08); }

    /* ── cards grid ──────────────────────────────────────────────────────── */
    .cards-grid {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 1rem;
      margin-bottom: 1.25rem;
    }

    /* ── posture card ────────────────────────────────────────────────────── */
    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 1.1rem;
      position: relative;
      overflow: hidden;
      transition: border-color 0.25s, box-shadow 0.25s;
    }
    .card::before {
      content: '';
      position: absolute;
      left: 0; top: 0; bottom: 0;
      width: 3px;
      border-radius: 0 2px 2px 0;
      background: var(--border);
      transition: background 0.25s;
    }

    /* idle — not yet monitoring */
    .card.s-idle   { }
    .card.s-idle::before { background: var(--border); }

    /* good — flag false */
    .card.s-good::before { background: var(--green); }
    .card.s-good { border-color: rgba(63,185,80,.2); }

    /* detected — flag currently true but no alert yet */
    .card.s-active::before { background: var(--orange); }
    .card.s-active { border-color: rgba(240,136,62,.35); }

    /* alert — EMA crossed threshold */
    .card.s-alert::before { background: var(--red); animation: pulse-side 1.1s ease-in-out infinite; }
    .card.s-alert {
      border-color: rgba(248,81,73,.55);
      box-shadow: 0 0 28px rgba(248,81,73,.18);
    }
    .card.s-alert::after {
      content: '';
      position: absolute;
      inset: 0;
      background: rgba(248,81,73,.04);
      pointer-events: none;
    }

    @keyframes pulse-side {
      0%, 100% { opacity: 1; }
      50%       { opacity: 0.3; }
    }

    .card-top {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 0.4rem;
      margin-bottom: 0.4rem;
    }

    .card-name {
      font-size: 0.82rem;
      font-weight: 600;
      line-height: 1.3;
    }

    .card-badge {
      font-size: 0.6rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      padding: 0.18rem 0.45rem;
      border-radius: 999px;
      border: 1px solid currentColor;
      flex-shrink: 0;
      white-space: nowrap;
      transition: color 0.2s, border-color 0.2s;
    }
    .badge-idle   { color: var(--text-2); }
    .badge-good   { color: var(--green); }
    .badge-active { color: var(--orange); }
    .badge-alert  { color: var(--red); animation: pulse-side 1.1s ease-in-out infinite; }

    .card-desc {
      font-size: 0.72rem;
      color: var(--text-2);
      line-height: 1.35;
      margin-bottom: 0.8rem;
    }

    /* EMA bar */
    .ema-track {
      position: relative;
      height: 5px;
      background: var(--surface-2);
      border-radius: 999px;
      overflow: visible;
      margin-bottom: 0.35rem;
    }
    .ema-fill {
      height: 100%;
      border-radius: 999px;
      width: 0%;
      background: var(--green);
      transition: width 0.25s ease, background 0.25s ease;
    }
    /* threshold marker at 65% */
    .ema-track::after {
      content: '';
      position: absolute;
      top: -3px; bottom: -3px;
      left: 65%;
      width: 2px;
      background: var(--text-2);
      opacity: 0.45;
      border-radius: 1px;
    }

    .ema-footer {
      display: flex;
      justify-content: space-between;
      font-size: 0.68rem;
      color: var(--text-2);
    }

    /* ── log ─────────────────────────────────────────────────────────────── */
    .log-section {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      overflow: hidden;
    }

    .log-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0.65rem 1rem;
      background: var(--surface-2);
      border-bottom: 1px solid var(--border);
    }
    .log-title {
      font-size: 0.72rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--text-2);
    }
    .btn-clear {
      font-size: 0.72rem;
      background: none;
      border: 1px solid var(--border);
      color: var(--text-2);
      padding: 0.18rem 0.55rem;
      border-radius: 4px;
      cursor: pointer;
      transition: color 0.15s, border-color 0.15s;
    }
    .btn-clear:hover { color: var(--text-1); border-color: var(--text-2); }

    #log {
      font-family: 'SF Mono', 'Fira Code', 'Cascadia Code', Consolas, monospace;
      font-size: 0.75rem;
      line-height: 1.55;
      max-height: 220px;
      overflow-y: auto;
      padding: 0.75rem 1rem;
      display: flex;
      flex-direction: column;
      gap: 0;
      scrollbar-width: thin;
      scrollbar-color: var(--border) transparent;
    }
    #log::-webkit-scrollbar { width: 6px; }
    #log::-webkit-scrollbar-track { background: transparent; }
    #log::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }

    .le { display: flex; gap: 0.75rem; }
    .lt { color: var(--text-2); flex-shrink: 0; user-select: none; }
    .lm { word-break: break-all; }
    .lm.t-ctrl  { color: var(--blue); }
    .lm.t-cal   { color: var(--yellow); }
    .lm.t-alert { color: var(--red); font-weight: 600; }
    .lm.t-json  { color: var(--text-2); }
    .lm.t-send  { color: var(--green); }
    .lm.t-sys   { color: var(--text-2); font-style: italic; }

    /* ── responsive ──────────────────────────────────────────────────────── */
    @media (max-width: 1100px) {
      .cards-grid { grid-template-columns: repeat(3, 1fr); }
    }
    @media (max-width: 700px) {
      body { padding: 1rem; }
      .cards-grid { grid-template-columns: repeat(2, 1fr); }
      .btn { min-width: 90px; font-size: 0.8rem; padding: 0.55rem 0.75rem; }
      h1 { font-size: 1.2rem; }
    }
    @media (max-width: 400px) {
      .cards-grid { grid-template-columns: 1fr; }
      .controls { flex-direction: column; }
      .btn { flex: unset; width: 100%; }
    }
  </style>
</head>
<body>
<div class="page">

  <header>
    <div class="logo">
      <div class="logo-spine">
        <span></span><span></span><span></span>
      </div>
      <h1>Posture<em>Sense</em></h1>
    </div>
    <div class="conn-badge">
      <div class="conn-dot off" id="conn-dot"></div>
      <span id="conn-text">Disconnected</span>
    </div>
  </header>

  <div class="state-banner">
    <span class="state-chip">Status</span>
    <span id="state-msg">Connecting to device…</span>
    <div class="cal-wrap" id="cal-wrap">
      <div class="cal-bar"><div id="cal-fill"></div></div>
      <span id="cal-pct">0%</span>
    </div>
  </div>

  <div class="controls">
    <button class="btn btn-blue"  id="btn-cal" disabled onclick="send('START_CAL')">Start Calibration</button>
    <button class="btn btn-green" id="btn-mon" disabled onclick="send('START_MON')">Start Monitoring</button>
    <button class="btn btn-red"   id="btn-ack" disabled onclick="send('ACK_ALERT')">Acknowledge Alert</button>
  </div>

  <div class="cards-grid" id="cards-grid"></div>

  <section class="log-section">
    <div class="log-header">
      <span class="log-title">Raw WebSocket Log</span>
      <button class="btn-clear" onclick="clearLog()">Clear</button>
    </div>
    <div id="log"></div>
  </section>

</div>
<script>
// ── configuration ─────────────────────────────────────────────────────────────
const WS_URL          = `ws://${window.location.hostname}/ws`;
const RECONNECT_MS    = 2000;
const MAX_LOG         = 300;
const EMA_THRESHOLD   = 0.65;

// ── posture condition definitions ─────────────────────────────────────────────
const CONDITIONS = [
  {
    key:   'thoracic_slouch',
    alert: 'THORACIC_SLOUCH',
    name:  'Thoracic Slouch',
    desc:  'Forward rounding of the upper back',
    zone:  'T1–T12',
  },
  {
    key:   'forward_flexion',
    alert: 'FORWARD_FLEXION',
    name:  'Forward Flexion',
    desc:  'Head-forward posture proxy',
    zone:  'C7–T1',
  },
  {
    key:   'lateral_lean',
    alert: 'LATERAL_LEAN',
    name:  'Lateral Lean',
    desc:  'Side-to-side spinal deviation',
    zone:  'Full spine',
  },
  {
    key:   'lumbar_hyperlordosis',
    alert: 'LUMBAR_HYPERLORDOSIS',
    name:  'Lumbar Hyperlordosis',
    desc:  'Excessive inward lumbar arch',
    zone:  'L1–L5',
  },
  {
    key:   'lumbar_flattening',
    alert: 'LUMBAR_FLATTENING',
    name:  'Lumbar Flattening',
    desc:  'Reduced natural lumbar curve',
    zone:  'L1–L5',
  },
];

// ── DOM refs ──────────────────────────────────────────────────────────────────
const connDot  = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');
const stateMsg = document.getElementById('state-msg');
const calWrap  = document.getElementById('cal-wrap');
const calFill  = document.getElementById('cal-fill');
const calPct   = document.getElementById('cal-pct');
const logEl    = document.getElementById('log');
const btnCal   = document.getElementById('btn-cal');
const btnMon   = document.getElementById('btn-mon');
const btnAck   = document.getElementById('btn-ack');

// ── build cards ───────────────────────────────────────────────────────────────
const cards = {};   // key → { el, fill, badge, emaVal, emaLabel }

(function buildCards() {
  const grid = document.getElementById('cards-grid');
  CONDITIONS.forEach(c => {
    const el = document.createElement('div');
    el.className = 'card s-idle';
    el.id = `card-${c.key}`;
    el.innerHTML = `
      <div class="card-top">
        <span class="card-name">${c.name}</span>
        <span class="card-badge badge-idle">—</span>
      </div>
      <p class="card-desc">${c.desc}</p>
      <div class="ema-track"><div class="ema-fill"></div></div>
      <div class="ema-footer">
        <span class="ema-val">EMA 0.000</span>
        <span>${c.zone}</span>
      </div>`;
    grid.appendChild(el);
    cards[c.key] = {
      el,
      fill:  el.querySelector('.ema-fill'),
      badge: el.querySelector('.card-badge'),
      val:   el.querySelector('.ema-val'),
    };
  });
})();

// ── state ─────────────────────────────────────────────────────────────────────
let ws          = null;
let uiState     = 'DISCONNECTED';

// ── WebSocket ─────────────────────────────────────────────────────────────────
function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    connDot.className  = 'conn-dot on';
    connText.textContent = 'Connected';
    addLog('WebSocket connected', 't-sys');
  };

  ws.onclose = () => {
    connDot.className  = 'conn-dot off';
    connText.textContent = 'Disconnected';
    addLog('WebSocket closed — reconnecting in 2 s…', 't-sys');
    setUIState('DISCONNECTED');
    setTimeout(connect, RECONNECT_MS);
  };

  ws.onerror = () => { /* onclose will also fire */ };

  ws.onmessage = ({ data }) => handleMessage(data);
}

function send(cmd) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(cmd);
  addLog(`→ ${cmd}`, 't-send');
}

// ── message routing ───────────────────────────────────────────────────────────
function handleMessage(raw) {
  // Per-tick monitoring JSON
  if (raw.charCodeAt(0) === 123 /* { */) {
    try {
      const data = JSON.parse(raw);
      onMonitorFrame(data);
      addLog(raw, 't-json');
      return;
    } catch (_) { /* fall through */ }
  }

  // Classify for log colouring before branching
  const logType = raw.startsWith('ALERT:')  ? 't-alert'
                : raw.startsWith('CAL_')    ? 't-cal'
                : raw === 'UNSTABLE_CAL'    ? 't-cal'
                : 't-ctrl';
  addLog(raw, logType);

  if      (raw === 'SYSTEM_READY')              { setUIState('WAITING');           resetCards(); }
  else if (raw === 'COUNTDOWN_START')           { setUIState('COUNTDOWN'); }
  else if (raw.startsWith('COUNTDOWN_TICK:'))   { onCountdownTick(+raw.split(':')[1]); }
  else if (raw === 'CAL_STARTED')               { setUIState('CALIBRATING'); }
  else if (raw.startsWith('CAL_PROGRESS:'))     { onCalProgress(+raw.split(':')[1]); }
  else if (raw === 'CAL_COMPLETE')              { setUIState('WAITING_FOR_START'); }
  else if (raw === 'UNSTABLE_CAL')              { onUnstableCal(); }
  else if (raw.startsWith('ALERT:'))            { onAlert(raw.slice(6)); }
  else if (raw === 'MONITORING_RESUMED')        { setUIState('MONITORING'); resetCards(); }
  else if (raw === 'FROZEN_TIMEOUT')            { /* MONITORING_RESUMED follows immediately */ }
}

// ── UI state machine ──────────────────────────────────────────────────────────
const STATE_MESSAGES = {
  DISCONNECTED:      'Not connected to PostureSense device',
  WAITING:           'Waiting — press Start Calibration to begin',
  COUNTDOWN:         'Hold still — calibration starting…',
  CALIBRATING:       'Calibrating…',
  WAITING_FOR_START: 'Calibration complete — press Start Monitoring',
  MONITORING:        'Monitoring active',
  FROZEN:            'Alert fired — correct your posture, then acknowledge',
};

function setUIState(state) {
  uiState = state;
  stateMsg.textContent = STATE_MESSAGES[state] ?? state;

  const showCal = state === 'CALIBRATING';
  calWrap.classList.toggle('show', showCal);
  if (showCal) { calFill.style.width = '0%'; calPct.textContent = '0%'; }

  btnCal.disabled = state !== 'WAITING';
  btnMon.disabled = state !== 'WAITING_FOR_START';
  btnAck.disabled = state !== 'FROZEN';
}

// ── event handlers ────────────────────────────────────────────────────────────
function onCountdownTick(n) {
  stateMsg.textContent = `Hold still — starting in ${n}…`;
}

function onCalProgress(pct) {
  calFill.style.width = `${pct}%`;
  calPct.textContent  = `${pct}%`;
  stateMsg.textContent = `Calibrating… ${pct}%`;
}

function onUnstableCal() {
  stateMsg.textContent = 'Calibration unstable — too much movement, retrying…';
  setUIState('WAITING');
}

function onAlert(alertKey) {
  setUIState('FROZEN');
  CONDITIONS.forEach(c => {
    setCard(c.key, c.alert === alertKey ? 's-alert' : 's-good', 0);
  });
  // Restore EMA on alerted card to 1.0 visually
  const cond = CONDITIONS.find(c => c.alert === alertKey);
  if (cond) setCard(cond.key, 's-alert', 1.0);
}

function onMonitorFrame(data) {
  if (uiState === 'FROZEN') return;   // hold the alert state until ACK
  if (uiState !== 'MONITORING') setUIState('MONITORING');

  CONDITIONS.forEach(c => {
    const flagOn = data[c.key] === true;
    const ema    = data.ema?.[c.alert] ?? 0;
    setCard(c.key, flagOn ? 's-active' : 's-good', ema);
  });
}

// ── card rendering ────────────────────────────────────────────────────────────
function setCard(key, stateClass, ema) {
  const r = cards[key];
  r.el.className = `card ${stateClass}`;

  const pct = Math.min(ema * 100, 100);
  r.fill.style.width = `${pct}%`;
  r.fill.style.background =
    stateClass === 's-alert' || ema >= EMA_THRESHOLD ? 'var(--red)'
    : ema > 0.35                                     ? 'var(--orange)'
    :                                                  'var(--green)';

  const map = {
    's-idle':   ['—',          'badge-idle'],
    's-good':   ['GOOD',       'badge-good'],
    's-active': ['DETECTED',   'badge-active'],
    's-alert':  ['ALERT',      'badge-alert'],
  };
  const [text, cls] = map[stateClass] ?? ['—', 'badge-idle'];
  r.badge.textContent = text;
  r.badge.className   = `card-badge ${cls}`;
  r.val.textContent   = `EMA ${ema.toFixed(3)}`;
}

function resetCards() {
  CONDITIONS.forEach(c => setCard(c.key, 's-idle', 0));
}

// ── log ───────────────────────────────────────────────────────────────────────
function addLog(msg, type) {
  const now = new Date();
  const ts  = `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}.${pad3(now.getMilliseconds())}`;

  const row = document.createElement('div');
  row.className = 'le';

  const t = document.createElement('span');
  t.className   = 'lt';
  t.textContent = ts;

  const m = document.createElement('span');
  m.className   = `lm ${type}`;
  m.textContent = msg;

  row.append(t, m);
  logEl.appendChild(row);

  while (logEl.children.length > MAX_LOG) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

function clearLog() { logEl.innerHTML = ''; }

function pad(n)  { return String(n).padStart(2, '0'); }
function pad3(n) { return String(n).padStart(3, '0'); }

// ── init ──────────────────────────────────────────────────────────────────────
resetCards();
setUIState('DISCONNECTED');
connect();
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────
// Globals
// ─────────────────────────────────────────
ICM_20948_I2C imuUpper, imuMid, imuLower;
Madgwick      filterUpper, filterMid, filterLower;

volatile SystemState currentState = WAITING;  // volatile: written by WS callback, read by loop()
Baseline      baseline;
bool          baselineSet     = false;

// Per-posture EMA scores (one float per condition)
float emaScores[5] = {0, 0, 0, 0, 0};

// Consecutive invalid reading counter (shared across MONITORING + CALIBRATING)
int consecutiveBadReadings = 0;

// Timestamp of the last alert sent (prevents alert spam)
unsigned long lastAlertTime = 0;

// Timestamp when FROZEN state was entered (for timeout)
unsigned long frozenEnteredAt = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─────────────────────────────────────────
// TCA helpers
// ─────────────────────────────────────────

// Select a single TCA channel; all others are disabled
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Deselect all TCA channels (good practice between reads)
void tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ─────────────────────────────────────────
// IMU initialisation helper
// ─────────────────────────────────────────

// Tries address 0x68 first, then 0x69.
// Returns true if the IMU responds on either address.
bool initIMU(ICM_20948_I2C &imu, uint8_t channel, const char *name) {
  tcaSelect(channel);
  delay(50);

  imu.begin(Wire, 0);  // AD0 LOW → 0x68
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x68 ✅");
    return true;
  }

  imu.begin(Wire, 1);  // AD0 HIGH → 0x69
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x69 ✅");
    return true;
  }

  Serial.print(name); Serial.println(" NOT FOUND ❌");
  return false;
}

// ─────────────────────────────────────────
// IMU reading
// ─────────────────────────────────────────

// Read one IMU, run it through its Madgwick filter, return pitch/roll/yaw.
// Returns valid=false if the IMU has no new data this tick.
IMUReading readOneIMU(ICM_20948_I2C &imu, Madgwick &filter, uint8_t channel) {
  IMUReading result = {0, 0, 0, false};
  tcaSelect(channel);

  if (!imu.dataReady()) return result;

  imu.getAGMT();
  filter.update(
    imu.gyrX(), imu.gyrY(), imu.gyrZ(),
    imu.accX(), imu.accY(), imu.accZ(),
    imu.magX(), imu.magY(), imu.magZ()
  );

  result.pitch = filter.getPitch();
  result.roll  = filter.getRoll();
  result.yaw   = filter.getYaw();
  result.valid = true;

  tcaDisableAll();  // deselect after each read to prevent bus conflicts
  return result;
}

// Read all three IMUs.
// NOTE: reads are sequential (not async). If any IMU is not ready,
// that sample is marked invalid. allValid is only true when all three succeed.
// The Madgwick filter for each IMU is only updated when that IMU has fresh data,
// so filters may accumulate at slightly different rates over time — acceptable at 4 Hz.
SpineReading readAllIMUs() {
  SpineReading tr;
  tr.upper = readOneIMU(imuUpper, filterUpper, CH_UPPER);
  tr.mid   = readOneIMU(imuMid,   filterMid,   CH_MID);
  tr.lower = readOneIMU(imuLower, filterLower,  CH_LOWER);
  tr.allValid = tr.upper.valid && tr.mid.valid && tr.lower.valid;
  return tr;
}

// ─────────────────────────────────────────
// Calibration helpers
// ─────────────────────────────────────────

// Returns true if the variance of samples is within CAL_VARIANCE_LIMIT.
// Called after collection is complete; the size guard is a safety fallback.
bool isStable(std::vector<float> &samples) {
  if ((int)samples.size() < CAL_MIN_VALID) return false;
  float mean = 0;
  for (float s : samples) mean += s;
  mean /= samples.size();
  float variance = 0;
  for (float s : samples) variance += (s - mean) * (s - mean);
  variance /= samples.size();
  return variance < CAL_VARIANCE_LIMIT;
}

// ─────────────────────────────────────────
// EMA helper
// ─────────────────────────────────────────

// Updates and returns the new EMA score.
// isBad should be true (1.0) when that posture condition is detected.
float updateEMA(float currentScore, bool isBad) {
  float sample = isBad ? 1.0f : 0.0f;
  return (EMA_ALPHA * sample) + (1.0f - EMA_ALPHA) * currentScore;
}

// ─────────────────────────────────────────
// Posture detection
// ─────────────────────────────────────────

// Compares delta angles against research-derived thresholds.
// Returns a bitmask of all active posture flags (POST_* constants).
// All deltas are relative to the calibrated baseline.
//
// Axis orientation note: pitch/roll polarity depends on how each IMU
// is physically mounted. Validate empirically after mounting on spine.
uint8_t detectPostures(SpineReading &r) {
  uint8_t flags = POST_GOOD;

  // Subtract baseline offsets so all comparisons are relative to "good posture"
  float dPitchUpper = r.upper.pitch - baseline.pitchUpper;
  float dPitchMid   = r.mid.pitch   - baseline.pitchMid;
  float dPitchLower = r.lower.pitch - baseline.pitchLower;
  float dRollUpper  = r.upper.roll  - baseline.rollUpper;
  float dRollMid    = r.mid.roll    - baseline.rollMid;
  float dRollLower  = r.lower.roll  - baseline.rollLower;

  // Thoracic Slouch: upper–mid pitch delta > 10°
  // Excessive forward rounding of the upper back relative to mid back
  if ((dPitchUpper - dPitchMid) > THR_THORACIC_SLOUCH)
    flags |= POST_THORACIC_SLOUCH;

  // Forward Flexion (forward-head proxy): upper pitch alone > 20°
  // Upper thoracic region bending forward, associated with head-forward posture
  if (dPitchUpper > THR_FORWARD_FLEXION)
    flags |= POST_FORWARD_FLEXION;

  // Lateral Lean: max roll spread across all three sensors > 10°
  // Person leaning left or right; uses spread rather than absolute value
  float maxRoll = max({dRollUpper, dRollMid, dRollLower});
  float minRoll = min({dRollUpper, dRollMid, dRollLower});
  if ((maxRoll - minRoll) > THR_LATERAL_LEAN)
    flags |= POST_LATERAL_LEAN;

  // Lumbar Hyperlordosis: mid–lower pitch delta > 10°
  // Excessive inward arching of the lower back
  if ((dPitchMid - dPitchLower) > THR_LUMBAR_HYPERLORDOSIS)
    flags |= POST_LUMBAR_HYPERLORDOSIS;

  // Lumbar Flattening: mid–lower pitch delta < −10°
  // Reduction of the natural lumbar curve (posterior pelvic tilt)
  // Uses the same sensor pair as Hyperlordosis but in the opposite direction
  if ((dPitchMid - dPitchLower) < THR_LUMBAR_FLATTENING)
    flags |= POST_LUMBAR_FLATTENING;

  return flags;
}

// ─────────────────────────────────────────
// Dual-channel broadcast helper
// ─────────────────────────────────────────

// Sends a protocol message to all WebSocket clients AND Serial.
// Use this for all state-machine protocol messages; Serial.println alone
// for boot-time diagnostics that pre-date any WS connection.
void broadcast(const String &msg) {
  Serial.println(msg);
  ws.textAll(msg);
}

// ─────────────────────────────────────────
// Invalid reading guard
// ─────────────────────────────────────────

// Call this whenever a SpineReading is invalid (not allValid).
// After MAX_CONSECUTIVE_BAD consecutive failures, sends an error signal
// and transitions to FROZEN so the webapp can alert the user.
void handleInvalidReading() {
  consecutiveBadReadings++;
  if (consecutiveBadReadings >= MAX_CONSECUTIVE_BAD) {
    broadcast("ERROR:IMU_MALFUNCTION");
    consecutiveBadReadings = 0;
    currentState = FROZEN;
    frozenEnteredAt = millis();
  }
}

// ─────────────────────────────────────────
// WebSocket event handler (async)
// ─────────────────────────────────────────

// Receives inbound control messages from the dashboard.
// Runs in the ESPAsyncWebServer FreeRTOS task — currentState is volatile
// to ensure the main loop() sees writes immediately.
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA) return;

  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  // Only handle complete, single-frame text messages (all control tokens fit in one frame)
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;

  String msg = String((char *)data, len);
  msg.trim();

  if (msg == "START_CAL") {
    currentState = COUNTDOWN;
  } else if (msg == "START_MON") {
    for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
    consecutiveBadReadings = 0;
    currentState = MONITORING;
  } else if (msg == "ACK_ALERT") {
    for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
    consecutiveBadReadings = 0;
    broadcast("MONITORING_RESUMED");
    currentState = MONITORING;
  }
}

// ─────────────────────────────────────────
// STATE HANDLERS
// ─────────────────────────────────────────

// WAITING ────────────────────────────────
// Idle. START_CAL is now received asynchronously via onWsEvent,
// which transitions directly to COUNTDOWN. Nothing to poll here.
void handleWaiting() {
}

// COUNTDOWN ──────────────────────────────
// Non-blocking 3-second countdown. Sends tick updates to webapp.
// Transitions to CALIBRATING when countdown expires.
void handleCountdown() {
  static unsigned long countdownStart = 0;
  static unsigned long lastTick       = 0;

  if (countdownStart == 0) {
    countdownStart = millis();
    lastTick       = millis();
    broadcast("COUNTDOWN_START");
  }

  unsigned long elapsed = millis() - countdownStart;

  // Send a tick every second so the webapp can update its UI timer
  if (elapsed - lastTick >= 1000) {
    lastTick = elapsed;
    broadcast("COUNTDOWN_TICK:" + String(3 - (int)(elapsed / 1000)));
  }

  if (elapsed >= COUNTDOWN_MS) {
    countdownStart = 0;  // reset for next calibration
    lastTick       = 0;
    currentState   = CALIBRATING;
  }
}

// CALIBRATING ────────────────────────────
// Collects CAL_SAMPLES readings at 4 Hz and computes baseline angles.
// Accepts the calibration if ≥ CAL_MIN_VALID samples are valid AND variance
// is within CAL_VARIANCE_LIMIT. Otherwise sends UNSTABLE_CAL and retries
// from COUNTDOWN. Consecutive invalid reading guard is active here too.
void handleCalibrating() {
  static std::vector<float> pU, rU, pM, rM, pL, rL;
  static int  collected = 0;
  static bool started   = false;

  if (!started) {
    pU.clear(); rU.clear();
    pM.clear(); rM.clear();
    pL.clear(); rL.clear();
    collected = 0;
    started   = true;
    broadcast("CAL_STARTED");
  }

  SpineReading reading = readAllIMUs();

  if (reading.allValid) {
    consecutiveBadReadings = 0;  // reset on any good reading

    pU.push_back(reading.upper.pitch); rU.push_back(reading.upper.roll);
    pM.push_back(reading.mid.pitch);   rM.push_back(reading.mid.roll);
    pL.push_back(reading.lower.pitch); rL.push_back(reading.lower.roll);
    collected++;

    // Report progress every 10% so webapp can show a progress bar
    int pct = (collected * 100) / CAL_SAMPLES;
    static int lastReported = -1;
    if (pct / 10 != lastReported / 10) {
      lastReported = pct;
      broadcast("CAL_PROGRESS:" + String(pct));
    }
  } else {
    // Invalid reading — check for IMU malfunction
    handleInvalidReading();
    // Do not increment collected; just skip this tick
  }

  // Check if we have enough samples to evaluate
  if (collected >= CAL_SAMPLES) {
    started = false;  // reset static state for next calibration

    // Require minimum valid samples AND stability on all six axes
    bool enoughSamples = (int)pU.size() >= CAL_MIN_VALID;
    bool stable = enoughSamples &&
                  isStable(pU) && isStable(rU) &&
                  isStable(pM) && isStable(rM) &&
                  isStable(pL) && isStable(rL);

    if (stable) {
      // Compute averages and store as baseline
      auto avg = [](std::vector<float> &v) {
        float s = 0; for (float x : v) s += x; return s / v.size();
      };
      baseline.pitchUpper = avg(pU); baseline.rollUpper = avg(rU);
      baseline.pitchMid   = avg(pM); baseline.rollMid   = avg(rM);
      baseline.pitchLower = avg(pL); baseline.rollLower = avg(rL);
      baselineSet = true;

      broadcast("CAL_COMPLETE");
      currentState = WAITING_FOR_START;
    } else {
      // Not enough valid samples or too much movement during calibration
      broadcast("UNSTABLE_CAL");
      currentState = COUNTDOWN;  // retry from countdown
    }
  }
}

// WAITING_FOR_START ──────────────────────
// START_MON is now received asynchronously via onWsEvent,
// which resets EMA scores and transitions to MONITORING. Nothing to poll here.
void handleWaitingForStart() {
}

// MONITORING ─────────────────────────────
// Core posture detection loop. Runs at 4 Hz via busy-compensation timing
// in the main loop(). Each tick: read IMUs → detect postures → update EMA
// → broadcast JSON telemetry → send alert if any EMA score crosses ALERT_THRESHOLD.
// Consecutive invalid reading guard is active here.
void handleMonitoring() {
  if (!baselineSet) {
    // Safety guard — should never reach here without a baseline
    broadcast("ERROR:NO_BASELINE");
    currentState = WAITING;
    return;
  }

  SpineReading reading = readAllIMUs();

  if (!reading.allValid) {
    handleInvalidReading();
    return;
  }

  consecutiveBadReadings = 0;  // reset on any good reading

  // Detect which posture conditions are currently active
  uint8_t flags = detectPostures(reading);

  // Update EMA for each condition independently.
  // Each EMA score represents a smoothed probability (0–1) that
  // the condition is currently active. A score > ALERT_THRESHOLD triggers alert.
  emaScores[0] = updateEMA(emaScores[0], (flags & POST_THORACIC_SLOUCH)      != 0);
  emaScores[1] = updateEMA(emaScores[1], (flags & POST_FORWARD_FLEXION)      != 0);
  emaScores[2] = updateEMA(emaScores[2], (flags & POST_LATERAL_LEAN)         != 0);
  emaScores[3] = updateEMA(emaScores[3], (flags & POST_LUMBAR_HYPERLORDOSIS) != 0);
  emaScores[4] = updateEMA(emaScores[4], (flags & POST_LUMBAR_FLATTENING)    != 0);

  // 4 Hz JSON telemetry — boolean flags + raw EMA scores for all connected clients
  {
    StaticJsonDocument<512> doc;
    doc["thoracic_slouch"]      = (flags & POST_THORACIC_SLOUCH)      != 0;
    doc["forward_flexion"]      = (flags & POST_FORWARD_FLEXION)      != 0;
    doc["lateral_lean"]         = (flags & POST_LATERAL_LEAN)         != 0;
    doc["lumbar_hyperlordosis"] = (flags & POST_LUMBAR_HYPERLORDOSIS) != 0;
    doc["lumbar_flattening"]    = (flags & POST_LUMBAR_FLATTENING)    != 0;
    JsonObject ema = doc.createNestedObject("ema");
    ema["THORACIC_SLOUCH"]      = emaScores[0];
    ema["FORWARD_FLEXION"]      = emaScores[1];
    ema["LATERAL_LEAN"]         = emaScores[2];
    ema["LUMBAR_HYPERLORDOSIS"] = emaScores[3];
    ema["LUMBAR_FLATTENING"]    = emaScores[4];
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
    Serial.println(json);
  }

  // Check if any condition has crossed the alert threshold.
  // ALERT_INTERVAL_MS prevents repeated alerts for the same ongoing issue.
  unsigned long now = millis();
  if (now - lastAlertTime >= ALERT_INTERVAL_MS) {
    if      (emaScores[0] > ALERT_THRESHOLD) { broadcast("ALERT:THORACIC_SLOUCH");      lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[1] > ALERT_THRESHOLD) { broadcast("ALERT:FORWARD_FLEXION");      lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[2] > ALERT_THRESHOLD) { broadcast("ALERT:LATERAL_LEAN");         lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[3] > ALERT_THRESHOLD) { broadcast("ALERT:LUMBAR_HYPERLORDOSIS"); lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[4] > ALERT_THRESHOLD) { broadcast("ALERT:LUMBAR_FLATTENING");    lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
  }
}

// FROZEN ─────────────────────────────────
// Waiting for ACK_ALERT from the webapp (handled asynchronously by onWsEvent).
// After FROZEN_TIMEOUT_MS (5 min) with no ACK, auto-resumes monitoring
// and notifies the webapp so it can also resume its UI state.
void handleFrozen() {
  // Timeout: auto-resume after 5 minutes with no ACK
  if (millis() - frozenEnteredAt >= FROZEN_TIMEOUT_MS) {
    for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
    consecutiveBadReadings = 0;
    broadcast("FROZEN_TIMEOUT");
    broadcast("MONITORING_RESUMED");
    currentState = MONITORING;
  }
}

// ─────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Start WiFi Access Point — dashboard connects to "PostureSense" network
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started — IP: ");
  Serial.println(WiFi.softAPIP());

  // mDNS: dashboard accessible at http://posture.local
  if (MDNS.begin("posture")) {
    Serial.println("mDNS started — http://posture.local");
  }

  Wire.begin(I2C_SDA, I2C_SCL);

  // Verify TCA is present before attempting IMU init
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR:TCA_NOT_FOUND");
    while (true) delay(1000);  // halt — nothing works without the mux
  }
  Serial.println("TCA9548A found ✅");

  // Initialise all three IMUs
  bool ok1 = initIMU(imuUpper, CH_UPPER, "IMU_UPPER (ch1)");
  bool ok2 = initIMU(imuMid,   CH_MID,   "IMU_MID   (ch2)");
  bool ok3 = initIMU(imuLower, CH_LOWER, "IMU_LOWER (ch7)");

  if (!ok1 || !ok2 || !ok3) {
    Serial.println("ERROR:IMU_INIT_FAILED");
    // Do not halt — partial operation may still be useful during development
  }

  // Initialise all three Madgwick filters at the chosen sample rate
  filterUpper.begin(4);  // 4 Hz — must match actual loop rate
  filterMid.begin(4);
  filterLower.begin(4);

  // Attach WebSocket handler and register with the HTTP server
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve the dashboard HTML at the root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", DASHBOARD_HTML);
  });

  server.begin();
  Serial.println("HTTP server started");

  broadcast("SYSTEM_READY");
  Serial.println("Waiting for START_CAL from webapp...");
}

// ─────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────
void loop() {
  ws.cleanupClients();  // reclaim memory from stale/closed WebSocket connections

  unsigned long start = micros();

  switch (currentState) {
    case WAITING:            handleWaiting();          break;
    case COUNTDOWN:          handleCountdown();         break;
    case CALIBRATING:        handleCalibrating();       break;
    case WAITING_FOR_START:  handleWaitingForStart();   break;
    case MONITORING:         handleMonitoring();        break;
    case FROZEN:             handleFrozen();            break;
  }

  // Busy-compensation timing — only applied during states that need
  // precise 4 Hz sampling (CALIBRATING and MONITORING).
  // Other states block on Serial or idle, so timing there doesn't matter.
  if (currentState == MONITORING || currentState == CALIBRATING) {
    unsigned long elapsed = micros() - start;
    if (elapsed < SAMPLE_INTERVAL_US) {
      delayMicroseconds(SAMPLE_INTERVAL_US - elapsed);
    }
  }
}
