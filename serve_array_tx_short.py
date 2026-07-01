#!/usr/bin/env python3
"""Live browser viewer for the four-channel directed MUX scan."""

from __future__ import annotations

import argparse
import glob
import math
import struct
import threading
import time
from pathlib import Path

import numpy as np
import serial
from flask import Flask, Response, jsonify, request

from host.run_array_test import FOUR_CHANNEL_PATHS, TWO_CHANNEL_PATHS, Frame, read_frame, simulated_frames

SERIAL_BAUD = 921600
SAMPLE_RATE_MHZ = 60.0
SETTINGS_DEFAULTS = {
    "dac": 150,
    "pon": 167,
    "poff": 167,
    "damp": 10000,
    "samples": 2000,
    "settle_us": 10,
    "blank_us": 0,
    "short_delay_us": 3,
    "short_hold_us": 50,
}
DIAGNOSTIC_LABELS = ["All open", "TX1 only", "RX2 only", "TX1 + RX2"]
NOISE_REFERENCE_PATHS = [
    (1, 6),
    (1, 2),
    (1, 3),
    (1, 4),
    (2, 6),
    (2, 1),
    (2, 3),
    (2, 4),
    (3, 6),
    (3, 1),
    (3, 2),
    (3, 4),
    (4, 6),
    (4, 1),
    (4, 2),
    (4, 3),
]

app = Flask(__name__)
state_lock = threading.Condition()
stream_stop = threading.Event()
stream_thread: threading.Thread | None = None
stream_error: str | None = None
latest_payload: bytes | None = None
latest_cycle_id = 0
latest_cycle_time = 0.0
latest_cycle_ms = 0.0
latest_path_fps = 0.0
simulate = False


def validated_settings() -> dict[str, int]:
    settings = {}
    limits = {
        "dac": (0, 1023),
        "pon": (8, 800),
        "poff": (8, 800),
        "damp": (0, 30000),
        "samples": (1, 2000),
        "settle_us": (5, 1000),
        "blank_us": (0, 50),
        "short_delay_us": (0, 100),
        "short_hold_us": (0, 200),
    }
    for key, default in SETTINGS_DEFAULTS.items():
        value = request.args.get(key, default)
        try:
            value = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} must be an integer") from exc
        low, high = limits[key]
        if not low <= value <= high:
            raise ValueError(f"{key} must be between {low} and {high}")
        settings[key] = value
    scan_mode = request.args.get("scan_mode", "four_common")
    if scan_mode == "normal":
        scan_mode = "four"
    if scan_mode not in {"dual", "four", "four_common", "four_muxblank", "four_noise", "diagnostic"}:
        raise ValueError("scan_mode must be dual, four, four_common, four_muxblank, four_noise, or diagnostic")
    settings["scan_mode"] = scan_mode
    return settings


def paths_for_mode(scan_mode: str) -> list[tuple[int, int]]:
    return TWO_CHANNEL_PATHS if scan_mode == "dual" else FOUR_CHANNEL_PATHS


def frame_paths_for_mode(scan_mode: str) -> list[tuple[int, int]]:
    if scan_mode == "dual":
        return TWO_CHANNEL_PATHS
    if scan_mode == "four_noise":
        return NOISE_REFERENCE_PATHS
    return FOUR_CHANNEL_PATHS


def smooth_trace(values: np.ndarray, window: int = 301) -> np.ndarray:
    if values.size < 3:
        return values.astype(np.float64)
    window = min(window, values.size if values.size % 2 == 1 else values.size - 1)
    if window < 3:
        return values.astype(np.float64)
    kernel = np.ones(window, dtype=np.float64) / window
    pad = window // 2
    padded = np.pad(values.astype(np.float64), (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def subtract_noise_reference(frames: list[Frame]) -> list[Frame]:
    # TR6 is sampled sequentially, not simultaneously, so use it only as a
    # slow common trend reference. Point-by-point subtraction creates laggy
    # artifacts when the MUX path timing or loading is slightly different.
    references: dict[int, np.ndarray] = {}
    corrected: list[Frame] = []
    for frame in frames:
        receivers = frame.receivers
        if receivers == [6]:
            references[frame.tx] = smooth_trace(frame.samples)
    for frame in frames:
        receivers = frame.receivers
        if receivers == [6]:
            continue
        reference = references.get(frame.tx)
        if reference is None:
            corrected.append(frame)
            continue
        samples_float = frame.samples.astype(np.float64)
        baseline = float(np.median(samples_float[-max(20, samples_float.size // 10):]))
        ref_baseline = float(np.median(reference[-max(20, reference.size // 10):]))
        ref_trend = reference - ref_baseline
        signal_trend = smooth_trace(samples_float) - baseline
        fit_start = min(samples_float.size, int(6 * SAMPLE_RATE_MHZ))
        fit_end = min(samples_float.size, int(35 * SAMPLE_RATE_MHZ))
        if fit_end > fit_start + 20:
            ref_fit = ref_trend[fit_start:fit_end]
            sig_fit = signal_trend[fit_start:fit_end]
            denom = float(np.dot(ref_fit, ref_fit))
            scale = float(np.dot(sig_fit, ref_fit) / denom) if denom > 1e-6 else 0.0
            scale = max(0.0, min(1.5, scale))
        else:
            scale = 1.0
        samples = np.clip(samples_float - scale * ref_trend, 0, 1023).astype(np.uint16)
        corrected.append(
            Frame(
                frame.mode,
                frame.tx,
                frame.rx_mask,
                frame.sample_count,
                frame.sequence,
                frame.mux_mask,
                samples,
                frame.received_at,
            )
        )
    return corrected


def find_port() -> str:
    ports = (
        glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/tty.usbmodem*")
        + glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/tty.usbserial*")
    )
    if not ports:
        raise RuntimeError("No Pico USB serial device found.")
    return ports[0]


def pack_10bit(samples: np.ndarray) -> bytes:
    values = np.asarray(samples, dtype=np.uint16) & 0x03FF
    out = bytearray(math.ceil(values.size * 10 / 8))
    bitbuf = 0
    bits = 0
    index = 0
    for sample in values:
        bitbuf |= int(sample) << bits
        bits += 10
        while bits >= 8:
            out[index] = bitbuf & 0xFF
            index += 1
            bitbuf >>= 8
            bits -= 8
    if bits:
        out[index] = bitbuf & 0xFF
    return bytes(out)


def publish_cycle(frames: list[Frame], elapsed: float, scan_mode: str) -> None:
    global latest_payload, latest_cycle_id, latest_cycle_time, latest_cycle_ms, latest_path_fps
    expected_count = 4 if scan_mode == "diagnostic" else len(frame_paths_for_mode(scan_mode))
    if len(frames) != expected_count:
        return
    if scan_mode == "diagnostic":
        masks = [frame.mux_mask for frame in frames]
        if masks != [0x0000, 0x0002, 0x0004, 0x0006]:
            raise RuntimeError(f"Unexpected diagnostic MUX masks: {masks}")
    else:
        actual = [(frame.tx, frame.receivers[0]) for frame in frames]
        expected = frame_paths_for_mode(scan_mode)
        if actual != expected:
            raise RuntimeError(f"Unexpected scan order: {actual}")
    output_frames = subtract_noise_reference(frames) if scan_mode == "four_noise" else frames
    count = output_frames[0].sample_count
    if any(frame.sample_count != count for frame in output_frames):
        raise RuntimeError("Sample count changed inside a scan.")

    payload = struct.pack("<IH", latest_cycle_id + 1, count)
    payload += b"".join(pack_10bit(frame.samples) for frame in output_frames)
    with state_lock:
        latest_cycle_id += 1
        latest_payload = payload
        latest_cycle_time = time.perf_counter()
        latest_cycle_ms = elapsed * 1000.0
        latest_path_fps = len(output_frames) / elapsed if elapsed > 0 else 0.0
        state_lock.notify_all()


def hardware_worker(settings: dict[str, int]) -> None:
    port = find_port()
    if settings["scan_mode"] == "diagnostic":
        command_name = "mux diagnostic"
    elif settings["scan_mode"] == "dual":
        command_name = "dual short"
    elif settings["scan_mode"] == "four_common":
        command_name = "four common"
    elif settings["scan_mode"] == "four_muxblank":
        command_name = "four muxblank"
    elif settings["scan_mode"] == "four_noise":
        command_name = "four noise"
    else:
        command_name = "four short"
    command = (
        f"{command_name} {settings['samples']} {settings['pon']} "
        f"{settings['poff']} {settings['damp']} {settings['settle_us']} "
        f"{settings['blank_us']} {settings['short_delay_us']} "
        f"{settings['short_hold_us']}\n"
    )
    with serial.Serial(port, SERIAL_BAUD, timeout=2, write_timeout=2) as ser:
        ser.write(b"q\n")
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(f"write dac {settings['dac']}\n".encode("ascii"))
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(command.encode("ascii"))

        cycle: list[Frame] = []
        frame_count = 4 if settings["scan_mode"] == "diagnostic" else len(frame_paths_for_mode(settings["scan_mode"]))
        cycle_started = time.perf_counter()
        while not stream_stop.is_set():
            frame = read_frame(ser)
            if frame.sequence % frame_count == 0:
                cycle = []
                cycle_started = time.perf_counter()
            cycle.append(frame)
            if len(cycle) == frame_count:
                publish_cycle(cycle, time.perf_counter() - cycle_started, settings["scan_mode"])
                cycle = []
        ser.write(b"q\n")
        ser.flush()


def simulation_worker(settings: dict[str, int]) -> None:
    while not stream_stop.is_set():
        started = time.perf_counter()
        if settings["scan_mode"] == "diagnostic":
            base = simulated_frames(2, settings["samples"], SAMPLE_RATE_MHZ)
            frames = []
            for test, mask in enumerate([0x0000, 0x0002, 0x0004, 0x0006]):
                source = base[test]
                scale = [0.08, 0.35, 0.12, 1.0][test]
                samples = 512 + (source.samples.astype(float) - 512) * scale
                frames.append(Frame(2, test + 1, 0, source.sample_count, test, mask, samples.astype(np.uint16), time.time()))
        else:
            frames = simulated_frames(1, settings["samples"], SAMPLE_RATE_MHZ, frame_paths_for_mode(settings["scan_mode"]))
        time.sleep(0.055)
        publish_cycle(frames, time.perf_counter() - started, settings["scan_mode"])


def worker(settings: dict[str, int]) -> None:
    global stream_error
    try:
        if simulate:
            simulation_worker(settings)
        else:
            hardware_worker(settings)
    except Exception as exc:
        with state_lock:
            stream_error = str(exc)
            state_lock.notify_all()


def stop_worker() -> None:
    global stream_thread
    stream_stop.set()
    thread = stream_thread
    if thread is not None and thread.is_alive():
        thread.join(timeout=2.5)
    stream_thread = None
    with state_lock:
        state_lock.notify_all()


@app.post("/stream/start")
def start_stream():
    global stream_thread, stream_error, latest_payload, latest_cycle_id
    try:
        settings = validated_settings()
    except ValueError as exc:
        return str(exc), 400
    stop_worker()
    with state_lock:
        stream_error = None
        latest_payload = None
        latest_cycle_id = 0
    stream_stop.clear()
    stream_thread = threading.Thread(target=worker, args=(settings,), daemon=True)
    stream_thread.start()
    return jsonify({"ok": True, "simulate": simulate})


@app.post("/stream/stop")
def stop_stream():
    stop_worker()
    return jsonify({"ok": True})


@app.get("/stream/latest.bin")
def latest():
    after = request.args.get("after", 0, type=int)
    timeout_ms = min(max(request.args.get("timeout_ms", 1000, type=int), 1), 3000)
    deadline = time.monotonic() + timeout_ms / 1000.0
    with state_lock:
        while latest_cycle_id <= after and stream_error is None and time.monotonic() < deadline:
            state_lock.wait(timeout=max(0.0, deadline - time.monotonic()))
        if stream_error is not None:
            return stream_error, 500
        if latest_payload is None or latest_cycle_id <= after:
            return Response(status=204)
        payload = latest_payload
        cycle_id = latest_cycle_id
        age_ms = (time.perf_counter() - latest_cycle_time) * 1000.0
        cycle_ms = latest_cycle_ms
        path_fps = latest_path_fps
    response = Response(payload, mimetype="application/octet-stream")
    response.headers["X-Cycle-Id"] = str(cycle_id)
    response.headers["X-Cycle-Ms"] = f"{cycle_ms:.3f}"
    response.headers["X-Path-FPS"] = f"{path_fps:.2f}"
    response.headers["X-Cycle-Age-Ms"] = f"{age_ms:.3f}"
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>pic0rick 4-channel MUX scan</title>
<style>
:root { color-scheme: dark; }
* { box-sizing: border-box; }
html, body { min-height: 100%; }
body {
  margin: 0;
  background: #101318;
  color: #e8ebf0;
  font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Helvetica Neue", sans-serif;
}
main { width: min(1760px, 100%); margin: 0 auto; padding: 12px 16px 16px; }
header { display: flex; align-items: center; gap: 18px; margin-bottom: 8px; }
.header-copy { min-width: 0; flex: 1; }
h1 { margin: 0; font-size: 20px; font-weight: 650; letter-spacing: 0; }
.sub { margin-left: auto; color: #98a2b3; font-size: 12px; text-align: right; }
.scan-order { color: #62d9cc; font-variant-numeric: tabular-nums; white-space: nowrap; }
.presets {
  display: flex;
  align-self: end;
  justify-content: flex-start;
  gap: 6px;
  min-width: 0;
  white-space: nowrap;
}
button {
  height: 34px;
  border: 1px solid #334155;
  border-radius: 6px;
  background: #1b2430;
  color: #f8fafc;
  padding: 0 13px;
  font: inherit;
  cursor: pointer;
}
.preset { width: 84px; padding: 0; color: #aeb7c5; white-space: nowrap; }
.preset.active { background: #0f766e; border-color: #14b8a6; color: #fff; }
.toolbar {
  --field-w: 132px;
  display: grid;
  grid-template-columns: repeat(6, var(--field-w)) 444px;
  justify-content: start;
  gap: 8px;
  margin-bottom: 10px;
}
label {
  display: grid;
  gap: 5px;
  width: var(--field-w);
  color: #aeb7c5;
  font-size: 12px;
}
input, select {
  min-width: 0;
  width: 100%;
  height: 34px;
  border: 1px solid #2c3340;
  border-radius: 6px;
  background: #171c24;
  color: #f4f7fb;
  padding: 0 9px;
  font: inherit;
}
input:disabled, select:disabled, button:disabled { opacity: .48; cursor: not-allowed; }
.status {
  min-height: 50px;
  margin-bottom: 10px;
  padding: 7px 10px 7px 12px;
  border-left: 4px solid #2dd4bf;
  border-radius: 6px;
  background: #151b23;
}
.status.warn { border-color: #f59e0b; }
.status-grid {
  display: grid;
  grid-template-columns: repeat(6, 112px) 112px;
  justify-content: space-between;
  gap: 10px;
  align-items: center;
}
.metric { width: 112px; min-width: 0; font-variant-numeric: tabular-nums; }
.metric span { display: block; color: #95a0ae; font-size: 10px; white-space: nowrap; }
.metric b {
  display: block;
  overflow: hidden;
  color: #edf4fb;
  font-size: 13px;
  font-weight: 650;
  text-overflow: ellipsis;
  white-space: nowrap;
}
#liveBtn { width: 112px; justify-self: end; }
#liveBtn.recording {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 7px;
  background: #dc2626;
  border-color: #dc2626;
  color: white;
}
.rec-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background: white;
  animation: pulse 1.1s ease-out infinite;
}
@keyframes pulse {
  0% { box-shadow: 0 0 0 0 rgba(255,255,255,.7); }
  70% { box-shadow: 0 0 0 8px rgba(255,255,255,0); }
  100% { box-shadow: 0 0 0 0 rgba(255,255,255,0); }
}
.matrix {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 8px;
}
.plot {
  position: relative;
  height: clamp(170px, 19vh, 230px);
  min-width: 0;
  overflow: hidden;
  border-radius: 6px;
  background: #151b23;
}
.plot-title {
  position: absolute;
  z-index: 2;
  top: 7px;
  left: 0;
  right: 0;
  color: #dce6f2;
  font-size: 12px;
  font-weight: 600;
  text-align: center;
  pointer-events: none;
}
canvas { display: block; width: 100%; height: 100%; }
@media (max-width: 1050px) {
  header { flex-wrap: wrap; }
  .sub { text-align: left; }
  label { width: 100%; }
  .presets { grid-column: 1 / -1; justify-content: flex-start; }
  .toolbar { grid-template-columns: repeat(4, minmax(110px, 1fr)); }
  .status-grid { grid-template-columns: repeat(4, minmax(90px, 1fr)); }
  .metric { width: auto; }
  .matrix { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
@media (max-width: 620px) {
  main { padding: 12px 10px; }
  .toolbar { grid-template-columns: repeat(2, minmax(100px, 1fr)); }
  .matrix { grid-template-columns: 1fr; }
  .plot { height: 280px; }
  .scan-order { white-space: normal; }
}
</style>
</head>
<body>
<main>
  <header>
    <div class="header-copy">
      <h1>pic0rick 4-channel scan</h1>
    </div>
    <div class="sub">Sequence: <span class="scan-order">TX1→RX2/3/4 · TX2→RX1/3/4 · TX3→RX1/2/4 · TX4→RX1/2/3 · repeat · TX short after pulse</span></div>
  </header>

  <section class="toolbar">
    <input id="scan_mode" type="hidden" value="four_common">
    <label>DAC gain<input id="dac" type="number" min="0" max="1023" value="150"></label>
    <label>pon ns<input id="pon" type="number" min="8" max="800" value="167"></label>
    <label>poff ns<input id="poff" type="number" min="8" max="800" value="167"></label>
    <label>damp ns<input id="damp" type="number" min="0" max="30000" value="10000"></label>
    <label>MUX settle us<input id="settle_us" type="number" min="5" max="1000" value="10"></label>
    <label>TX short delay us<input id="short_delay_us" type="number" min="0" max="100" value="3"></label>
    <div class="presets" aria-label="Piezo frequency">
      <button class="preset" data-frequency="1">1 MHz</button>
      <button class="preset" data-frequency="2">2 MHz</button>
      <button class="preset active" data-frequency="3">3 MHz</button>
      <button class="preset" data-frequency="3.8">3.8 MHz</button>
      <button class="preset" data-frequency="5">5 MHz</button>
    </div>
    <input id="samples" type="hidden" value="2000">
    <input id="blank_us" type="hidden" value="0">
    <input id="short_hold_us" type="hidden" value="50">
    <input id="display_filter" type="hidden" value="raw">
    <input id="start_us" type="hidden" value="0">
    <input id="end_us" type="hidden" value="50">
  </section>

  <section class="status" id="status">
    <div class="status-grid">
      <div class="metric"><span>Scan</span><b id="scan">--</b></div>
      <div class="metric"><span>Scan rate</span><b id="scanRate">-- Hz</b></div>
      <div class="metric"><span>Path rate</span><b id="pathRate">-- fps</b></div>
      <div class="metric"><span>Scan time</span><b id="scanMs">-- ms</b></div>
      <div class="metric"><span>Samples / path</span><b id="sampleInfo">--</b></div>
      <div class="metric"><span>Global min / max</span><b id="minMax">--</b></div>
      <button id="liveBtn">Start live</button>
    </div>
  </section>

  <section class="matrix" id="matrix"></section>
</main>
<script>
const NORMAL_PATHS = [
  {id:"12",label:"TX1 → RX2"},
  {id:"13",label:"TX1 → RX3"},
  {id:"14",label:"TX1 → RX4"},
  {id:"21",label:"TX2 → RX1"},
  {id:"23",label:"TX2 → RX3"},
  {id:"24",label:"TX2 → RX4"},
  {id:"31",label:"TX3 → RX1"},
  {id:"32",label:"TX3 → RX2"},
  {id:"34",label:"TX3 → RX4"},
  {id:"41",label:"TX4 → RX1"},
  {id:"42",label:"TX4 → RX2"},
  {id:"43",label:"TX4 → RX3"},
];
const TWO_CHANNEL_PATHS = [{id:"12",label:"TX1 → RX2"},{id:"21",label:"TX2 → RX1"}];
const DIAGNOSTIC_PATHS = [
  {id:"d0",label:"All switches open · 0x0000"},
  {id:"d1",label:"TX1 only · SW1 · 0x0002"},
  {id:"d2",label:"RX2 only · SW2 · 0x0004"},
  {id:"d3",label:"TX1 + RX2 · SW1 + SW2 · 0x0006"},
];
let PATHS = NORMAL_PATHS;
const SETTINGS = ["scan_mode","dac","pon","poff","damp","samples","settle_us","blank_us","short_delay_us","short_hold_us","display_filter","start_us","end_us"];
const STORAGE_KEY = "pic0rick.txShort.settings.v7";
const PRESETS = {"1":500,"2":250,"3":167,"3.8":133,"5":100};
const SAMPLE_RATE_MHZ = 60;
let running = false;
let inFlight = false;
let cycleId = 0;
let framesSeen = 0;
let rateHistory = [];
let displayCache = new Map();

const el = id => document.getElementById(id);
const number = id => Number(el(id).value);

function buildMatrix(){
  const matrix = el("matrix");
  matrix.innerHTML = "";
  displayCache = new Map();
  const mode = el("scan_mode").value;
  PATHS = mode === "diagnostic" ? DIAGNOSTIC_PATHS : (mode === "dual" ? TWO_CHANNEL_PATHS : NORMAL_PATHS);
  for(const path of PATHS){
    const cell = document.createElement("div");
    cell.className = "plot";
    cell.innerHTML = `<div class="plot-title">${path.label}</div><canvas id="p${path.id}"></canvas>`;
    matrix.appendChild(cell);
  }
}

function saveSettings(){
  const data = {};
  for(const id of SETTINGS) data[id] = el(id).value;
  localStorage.setItem(STORAGE_KEY, JSON.stringify(data));
}

function loadSettings(){
  try{
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || "{}");
    saved.scan_mode = "four_common";
    saved.samples = "2000";
    saved.blank_us = "0";
    saved.short_hold_us = "50";
    saved.display_filter = "raw";
    saved.start_us = "0";
    saved.end_us = "50";
    for(const id of SETTINGS){
      if(saved[id] === undefined) continue;
      el(id).value = saved[id];
      if(id === "scan_mode" && !el(id).value) el(id).value = "four_common";
    }
  }catch(_){}
}

function setControls(disabled){
  for(const id of ["dac","pon","poff","damp","settle_us","short_delay_us"]) el(id).disabled = disabled;
  for(const button of document.querySelectorAll(".preset")) button.disabled = disabled;
}

function setLiveUi(){
  const button = el("liveBtn");
  button.className = running ? "recording" : "";
  button.innerHTML = running ? `<i class="rec-dot"></i><span>Stop live</span>` : "Start live";
  setControls(running);
}

function params(){
  const value = new URLSearchParams();
  el("scan_mode").value = "four_common";
  el("samples").value = "2000";
  el("blank_us").value = "0";
  el("short_hold_us").value = "50";
  for(const id of ["scan_mode","dac","pon","poff","damp","samples","settle_us","blank_us","short_delay_us","short_hold_us"]) value.set(id, el(id).value);
  return value;
}

function despikeSamples(samples){
  const out = new Float32Array(samples.length);
  if(samples.length === 0) return out;
  out[0] = samples[0];
  out[samples.length - 1] = samples[samples.length - 1];
  for(let i=1;i<samples.length-1;i++){
    const prev = samples[i-1], value = samples[i], next = samples[i+1];
    const isolated = Math.abs(value-prev) > 90 && Math.abs(value-next) > 90 && Math.abs(prev-next) < 70;
    out[i] = isolated ? (prev + next) / 2 : value;
  }
  return out;
}

function displaySamples(samples, pathId){
  const mode = el("display_filter").value;
  if(mode === "raw") return samples;
  const cleaned = despikeSamples(samples);
  if(mode !== "stable") return cleaned;
  const previous = displayCache.get(pathId);
  if(!previous || previous.length !== cleaned.length){
    displayCache.set(pathId, cleaned);
    return cleaned;
  }
  const out = new Float32Array(cleaned.length);
  const alpha = 0.35;
  for(let i=0;i<cleaned.length;i++){
    out[i] = previous[i] * (1 - alpha) + cleaned[i] * alpha;
  }
  displayCache.set(pathId, out);
  return out;
}

function unpack10bit(bytes, count){
  const output = new Uint16Array(count);
  let bitbuf = 0;
  let bits = 0;
  let byteIndex = 0;
  for(let i=0;i<count;i++){
    while(bits < 10){
      bitbuf |= bytes[byteIndex++] << bits;
      bits += 8;
    }
    output[i] = bitbuf & 0x03ff;
    bitbuf >>= 10;
    bits -= 10;
  }
  return output;
}

function draw(canvas, samples){
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if(canvas.width !== width || canvas.height !== height){
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio,0,0,ratio,0,0);
  const w = rect.width, h = rect.height;
  const left = 40, right = 9, top = 28, bottom = 25;
  const pw = Math.max(1, w-left-right), ph = Math.max(1, h-top-bottom);
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle = "#151b23";
  ctx.fillRect(0,0,w,h);

  const totalUs = samples.length / SAMPLE_RATE_MHZ;
  let startUs = Math.max(0, number("start_us"));
  let endUs = Math.min(totalUs, Math.max(startUs + 0.1, number("end_us")));
  const i0 = Math.max(0, Math.floor(startUs * SAMPLE_RATE_MHZ));
  const i1 = Math.min(samples.length, Math.ceil(endUs * SAMPLE_RATE_MHZ));
  const y0 = 0, y1 = 1023;

  ctx.strokeStyle = "#3b4655";
  ctx.lineWidth = 1;
  ctx.setLineDash([]);
  ctx.strokeRect(left + .5, top + .5, pw, ph);

  ctx.fillStyle = "#aab5c4";
  ctx.font = "10px -apple-system, sans-serif";
  ctx.textAlign = "right";
  for(const tick of [0, 256, 512, 768, 1023]){
    const y = top + (y1-tick)/Math.max(1,y1-y0)*ph;
    ctx.fillText(String(tick), left-5, y+3);
  }

  ctx.save();
  ctx.strokeStyle = "#3b4655";
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 4]);
  ctx.textAlign = "center";
  const firstTick = Math.ceil(startUs / 5) * 5;
  for(let tick=firstTick; tick<=endUs + 0.001; tick+=5){
    const x = left + (tick-startUs)/Math.max(0.1,endUs-startUs)*pw;
    ctx.beginPath(); ctx.moveTo(x+.5,top); ctx.lineTo(x+.5,top+ph); ctx.stroke();
    ctx.fillText(String(Math.round(tick)), x, h-8);
  }
  ctx.restore();
  ctx.strokeStyle = "#2dd4bf";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  const visible = Math.max(1, i1-i0-1);
  const step = Math.max(1, Math.floor(visible / Math.max(1, Math.floor(pw*1.5))));
  let first = true;
  for(let i=i0;i<i1;i+=step){
    const x = left + (i-i0)/visible*pw;
    const y = top + (y1-samples[i])/Math.max(1,y1-y0)*ph;
    if(first){ ctx.moveTo(x,y); first=false; } else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

function renderCycle(buffer, response){
  const view = new DataView(buffer);
  const embeddedId = view.getUint32(0, true);
  const count = view.getUint16(4, true);
  const payload = new Uint8Array(buffer, 6);
  const packedBytes = Math.ceil(count * 10 / 8);
  let globalMin = 1023, globalMax = 0;
  for(let pathIndex=0; pathIndex<PATHS.length; pathIndex++){
    const path = PATHS[pathIndex];
    const packed = payload.subarray(pathIndex*packedBytes, (pathIndex+1)*packedBytes);
    const samples = unpack10bit(packed, count);
    let min=1023,max=0;
    for(const value of samples){ if(value<min)min=value; if(value>max)max=value; }
    if(min<globalMin)globalMin=min;
    if(max>globalMax)globalMax=max;
    draw(el(`p${path.id}`), displaySamples(samples, path.id));
  }
  cycleId = embeddedId;
  const cycleMs = Number(response.headers.get("X-Cycle-Ms")) || 0;
  const pathFps = Number(response.headers.get("X-Path-FPS")) || 0;
  rateHistory.push(cycleMs);
  if(rateHistory.length > 30) rateHistory.shift();
  const avgMs = rateHistory.reduce((a,b)=>a+b,0)/rateHistory.length;
  el("scan").textContent = cycleId;
  el("scanRate").textContent = `${(1000/avgMs).toFixed(1)} Hz`;
  el("pathRate").textContent = `${pathFps.toFixed(1)} fps`;
  el("scanMs").textContent = `${avgMs.toFixed(1)} ms`;
  el("sampleInfo").textContent = `${count} / ${(count/SAMPLE_RATE_MHZ).toFixed(2)} us`;
  el("minMax").textContent = `${globalMin} / ${globalMax}`;
}

async function nextCycle(){
  if(!running || inFlight) return;
  inFlight = true;
  try{
    const response = await fetch(`/stream/latest.bin?after=${cycleId}&timeout_ms=1000`);
    if(response.status === 204) return;
    if(!response.ok) throw new Error(await response.text() || "Live scan failed");
    renderCycle(await response.arrayBuffer(), response);
    el("status").classList.remove("warn");
  }catch(error){
    el("status").classList.add("warn");
    el("minMax").textContent = error.message;
  }finally{
    inFlight = false;
    if(running) setTimeout(nextCycle, 0);
  }
}

async function toggleLive(){
  if(!running){
    saveSettings();
    running = true;
    cycleId = 0;
    rateHistory = [];
    displayCache = new Map();
    setLiveUi();
    try{
      const response = await fetch(`/stream/start?${params()}`, {method:"POST"});
      if(!response.ok) throw new Error(await response.text() || "Could not start scan");
      nextCycle();
    }catch(error){
      running = false;
      setLiveUi();
      el("status").classList.add("warn");
      el("minMax").textContent = error.message;
    }
  }else{
    running = false;
    setLiveUi();
    try{ await fetch("/stream/stop", {method:"POST"}); }catch(_){}
  }
}

buildMatrix();
loadSettings();
buildMatrix();
for(const id of SETTINGS){
  el(id).addEventListener("input", () => {
    saveSettings();
    if(id === "scan_mode") buildMatrix();
    if(id === "start_us" || id === "end_us" || id === "display_filter") window.dispatchEvent(new Event("resize"));
  });
}
for(const button of document.querySelectorAll(".preset")){
  button.addEventListener("click", () => {
    document.querySelectorAll(".preset").forEach(item => item.classList.remove("active"));
    button.classList.add("active");
    el("pon").value = PRESETS[button.dataset.frequency];
    el("poff").value = PRESETS[button.dataset.frequency];
    saveSettings();
  });
}
el("liveBtn").addEventListener("click", toggleLive);
window.addEventListener("resize", () => {});
</script>
</body>
</html>
"""


def main() -> None:
    global simulate
    parser = argparse.ArgumentParser(description="Serve the two-channel alternating live viewer.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5175)
    parser.add_argument("--simulate", action="store_true", help="Run the UI without connected hardware")
    args = parser.parse_args()
    simulate = args.simulate
    app.run(host=args.host, port=args.port, threaded=True, debug=False)


if __name__ == "__main__":
    main()
