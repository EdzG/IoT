#!/usr/bin/env python3
"""
mock_server.py — PostureSense ESP32 mock over WebSocket.

Mirrors the firmware state machine (posture_sense.ino):
  WAITING → COUNTDOWN → CALIBRATING → WAITING_FOR_START → MONITORING → FROZEN

Run:
    python mock_server.py

Connect your dashboard to ws://localhost:8765.

Plain-text commands (sent by the dashboard):
    START_CAL   — begin calibration sequence
    START_MON   — begin monitoring after calibration
    ACK_ALERT   — acknowledge a posture alert

Control messages broadcast by this server (identical to firmware Serial output):
    SYSTEM_READY
    COUNTDOWN_START
    COUNTDOWN_TICK:<n>          (n = 3, 2, 1)
    CAL_STARTED
    CAL_PROGRESS:<pct>          (pct = 0..100, reported at each 10% step)
    CAL_COMPLETE
    UNSTABLE_CAL
    ALERT:<CONDITION>           (see ALERT_NAMES below)
    MONITORING_RESUMED
    FROZEN_TIMEOUT

Per-tick JSON broadcast (only while MONITORING, every 250 ms):
    {
      "thoracic_slouch":      bool,
      "forward_flexion":      bool,
      "lateral_lean":         bool,
      "lumbar_hyperlordosis": bool,
      "lumbar_flattening":    bool,
      "ema": {
        "THORACIC_SLOUCH":      float,   # smoothed probability 0.0–1.0
        "FORWARD_FLEXION":      float,
        "LATERAL_LEAN":         float,
        "LUMBAR_HYPERLORDOSIS": float,
        "LUMBAR_FLATTENING":    float
      }
    }
"""

import asyncio
import json
import math
import time
import websockets

# ── constants matching posture_sense.ino ─────────────────────────────────────
EMA_ALPHA       = 0.05
ALERT_THRESHOLD = 0.65
FROZEN_TIMEOUT  = 300.0   # seconds — auto-resume if no ACK_ALERT
ALERT_INTERVAL  = 10.0    # minimum seconds between repeated alerts
SAMPLE_INTERVAL = 0.25    # 250 ms = 4 Hz

COUNTDOWN_SECS  = 3
CAL_SAMPLES     = 20      # 5 s × 4 Hz
CAL_MIN_VALID   = 15

FLAG_NAMES = [
    "thoracic_slouch",
    "forward_flexion",
    "lateral_lean",
    "lumbar_hyperlordosis",
    "lumbar_flattening",
]

ALERT_NAMES = [
    "THORACIC_SLOUCH",
    "FORWARD_FLEXION",
    "LATERAL_LEAN",
    "LUMBAR_HYPERLORDOSIS",
    "LUMBAR_FLATTENING",
]

# ── global client registry ────────────────────────────────────────────────────
clients: set = set()

# Wall-clock reference set in main() so mock posture data evolves continuously
# regardless of state transitions (prevents re-triggering immediately on resume).
_server_start: float = 0.0


# ── WebSocket broadcast ───────────────────────────────────────────────────────

async def broadcast(msg: str) -> None:
    if not clients:
        return
    snapshot = list(clients)
    results = await asyncio.gather(
        *[c.send(msg) for c in snapshot], return_exceptions=True
    )
    # Prune clients whose connections closed mid-send.
    dead = {c for c, r in zip(snapshot, results) if isinstance(r, Exception)}
    clients.difference_update(dead)


# ── mock posture data ─────────────────────────────────────────────────────────

def _mock_flags(t: float) -> list[bool]:
    """
    Simulate 5 posture conditions using square-wave duty cycles with staggered
    phases. Each "bad episode" is long enough (~14-18 s) for the EMA to cross
    ALERT_THRESHOLD (~5 s of continuous True ticks with alpha=0.05).

    Columns: (period_s, bad_fraction, phase_offset_s)
      bad_fraction × period_s = duration of each bad episode in seconds.
    """
    cycles = [
        ( 70, 0.20,  0.0),   # Thoracic Slouch      — bad 14 s / 70 s
        ( 90, 0.18, 15.0),   # Forward Flexion       — bad 16 s / 90 s
        ( 60, 0.22, 30.0),   # Lateral Lean          — bad 13 s / 60 s
        ( 80, 0.20, 45.0),   # Lumbar Hyperlordosis  — bad 16 s / 80 s
        (100, 0.18, 60.0),   # Lumbar Flattening     — bad 18 s / 100 s
    ]
    flags = []
    for period, frac, phase in cycles:
        pos = ((t + phase) % period) / period
        flags.append(pos < frac)
    return flags


def _update_ema(score: float, bad: bool) -> float:
    """Mirrors the firmware's updateEMA() — EMA_ALPHA * sample + (1-alpha) * score."""
    return EMA_ALPHA * (1.0 if bad else 0.0) + (1.0 - EMA_ALPHA) * score


# ── state machine ─────────────────────────────────────────────────────────────

async def state_machine(queue: asyncio.Queue) -> None:
    state           = "WAITING"
    ema             = [0.0] * 5
    last_alert_time = 0.0
    frozen_at       = 0.0

    print("[server] state machine started  —  waiting for START_CAL")

    while True:

        # ── WAITING ──────────────────────────────────────────────────────────
        if state == "WAITING":
            cmd = await queue.get()
            if cmd == "START_CAL":
                state = "COUNTDOWN"
                print("[server] → COUNTDOWN")

        # ── COUNTDOWN ────────────────────────────────────────────────────────
        elif state == "COUNTDOWN":
            await broadcast("COUNTDOWN_START")
            print("[server] COUNTDOWN_START")
            for tick in range(COUNTDOWN_SECS, 0, -1):
                await asyncio.sleep(1.0)
                msg = f"COUNTDOWN_TICK:{tick}"
                await broadcast(msg)
                print(f"[server] {msg}")
            state = "CALIBRATING"
            print("[server] → CALIBRATING")

        # ── CALIBRATING ──────────────────────────────────────────────────────
        elif state == "CALIBRATING":
            await broadcast("CAL_STARTED")
            print("[server] CAL_STARTED")
            last_reported_decade = 0   # C++ truncates -1/10 to 0, so first report is at 10%
            for sample in range(CAL_SAMPLES):
                await asyncio.sleep(SAMPLE_INTERVAL)
                pct     = ((sample + 1) * 100) // CAL_SAMPLES
                decade  = pct // 10
                if decade != last_reported_decade:
                    last_reported_decade = decade
                    msg = f"CAL_PROGRESS:{pct}"
                    await broadcast(msg)
                    print(f"[server] {msg}")
            # Mock always produces a stable calibration — no jitter to fail.
            await broadcast("CAL_COMPLETE")
            print("[server] CAL_COMPLETE")
            state = "WAITING_FOR_START"
            print("[server] → WAITING_FOR_START  (send START_MON to begin monitoring)")

        # ── WAITING_FOR_START ─────────────────────────────────────────────────
        elif state == "WAITING_FOR_START":
            cmd = await queue.get()
            if cmd == "START_MON":
                ema   = [0.0] * 5
                state = "MONITORING"
                print("[server] → MONITORING")

        # ── MONITORING ────────────────────────────────────────────────────────
        elif state == "MONITORING":
            await asyncio.sleep(SAMPLE_INTERVAL)

            t     = time.monotonic() - _server_start
            flags = _mock_flags(t)

            for i, bad in enumerate(flags):
                ema[i] = _update_ema(ema[i], bad)

            # Broadcast per-tick JSON posture packet
            payload: dict = {name: val for name, val in zip(FLAG_NAMES, flags)}
            payload["ema"] = {
                name: round(ema[i], 4) for i, name in enumerate(ALERT_NAMES)
            }
            payload["raw"] = {
                "upper": {"p": round(math.sin(t) * 5, 2),   "r": round(math.cos(t) * 2, 2),   "y": 0.0},
                "mid":   {"p": round(math.sin(t+1) * 5, 2), "r": round(math.cos(t+1) * 2, 2), "y": 0.0},
                "lower": {"p": round(math.sin(t+2) * 5, 2), "r": round(math.cos(t+2) * 2, 2), "y": 0.0},
            }
            await broadcast(json.dumps(payload))

            # Drain stale commands that arrived while monitoring (e.g. duplicate START_*)
            while not queue.empty():
                queue.get_nowait()

            # Alert check — same priority order as firmware
            now = time.monotonic()
            if now - last_alert_time >= ALERT_INTERVAL:
                for i, score in enumerate(ema):
                    if score > ALERT_THRESHOLD:
                        alert_msg = f"ALERT:{ALERT_NAMES[i]}"
                        await broadcast(alert_msg)
                        print(f"[server] {alert_msg}  (EMA={score:.3f})")
                        last_alert_time = now
                        frozen_at       = now
                        state           = "FROZEN"
                        break

        # ── FROZEN ────────────────────────────────────────────────────────────
        elif state == "FROZEN":
            try:
                cmd = await asyncio.wait_for(queue.get(), timeout=1.0)
                if cmd == "ACK_ALERT":
                    ema   = [0.0] * 5
                    await broadcast("MONITORING_RESUMED")
                    print("[server] ACK_ALERT received  →  MONITORING_RESUMED")
                    state = "MONITORING"
            except asyncio.TimeoutError:
                # Check 5-minute auto-resume timeout
                if time.monotonic() - frozen_at >= FROZEN_TIMEOUT:
                    ema   = [0.0] * 5
                    await broadcast("FROZEN_TIMEOUT")
                    await broadcast("MONITORING_RESUMED")
                    print("[server] FROZEN_TIMEOUT  →  MONITORING_RESUMED")
                    state = "MONITORING"


# ── per-connection handler ────────────────────────────────────────────────────

async def handler(websocket, queue: asyncio.Queue) -> None:
    clients.add(websocket)
    addr = websocket.remote_address
    print(f"[server] client connected     {addr}   (total={len(clients)})")
    try:
        # Mirror firmware setup(): greet each new client immediately on connect.
        await websocket.send("SYSTEM_READY")
        async for raw in websocket:
            cmd = raw.strip()
            print(f"[server] ← {cmd!r}")
            await queue.put(cmd)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        print(f"[server] client disconnected  {addr}   (total={len(clients)})")


# ── entry point ───────────────────────────────────────────────────────────────

async def main() -> None:
    global _server_start
    _server_start = time.monotonic()

    queue: asyncio.Queue = asyncio.Queue()

    async with websockets.serve(
        lambda ws: handler(ws, queue), "localhost", 8765
    ):
        print("[server] PostureSense mock listening on ws://localhost:8765")
        print("[server] Send START_CAL to begin the calibration sequence.\n")
        await state_machine(queue)


if __name__ == "__main__":
    asyncio.run(main())
