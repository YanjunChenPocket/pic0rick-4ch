#!/usr/bin/env python3
"""Capture the pic0rick four-channel directed MUX scan."""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

MAGIC = b"MUX10A1"
HEADER = struct.Struct("<BBBHIH")
TWO_CHANNEL_PATHS = [(1, 2), (2, 1)]
FOUR_CHANNEL_PATHS = [
    (1, 2),
    (1, 3),
    (1, 4),
    (2, 1),
    (2, 3),
    (2, 4),
    (3, 1),
    (3, 2),
    (3, 4),
    (4, 1),
    (4, 2),
    (4, 3),
]
PAIRWISE_PATHS = FOUR_CHANNEL_PATHS


@dataclass
class Frame:
    mode: int
    tx: int
    rx_mask: int
    sample_count: int
    sequence: int
    mux_mask: int
    samples: np.ndarray
    received_at: float

    @property
    def receivers(self) -> list[int]:
        return [channel for channel in range(1, 9) if self.rx_mask & (1 << (channel - 1))]


def unpack_10bit(payload: bytes, sample_count: int) -> np.ndarray:
    """Unpack the firmware's little-endian groups of four 10-bit samples."""
    raw = np.frombuffer(payload, dtype=np.uint8)
    complete_groups = sample_count // 4
    output = np.empty(sample_count, dtype=np.uint16)

    if complete_groups:
        groups = raw[: complete_groups * 5].reshape(-1, 5).astype(np.uint16)
        output[0 : complete_groups * 4 : 4] = groups[:, 0] | ((groups[:, 1] & 0x03) << 8)
        output[1 : complete_groups * 4 : 4] = (groups[:, 1] >> 2) | ((groups[:, 2] & 0x0F) << 6)
        output[2 : complete_groups * 4 : 4] = (groups[:, 2] >> 4) | ((groups[:, 3] & 0x3F) << 4)
        output[3 : complete_groups * 4 : 4] = (groups[:, 3] >> 6) | (groups[:, 4] << 2)

    remainder = sample_count - complete_groups * 4
    if remainder:
        start_byte = complete_groups * 5
        bit_buffer = int.from_bytes(payload[start_byte:], "little")
        for index in range(remainder):
            output[complete_groups * 4 + index] = (bit_buffer >> (index * 10)) & 0x3FF

    return output


def read_exact(serial_port, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = serial_port.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"Timed out after {len(data)} of {size} bytes")
        data.extend(chunk)
    return bytes(data)


def find_magic(serial_port) -> None:
    matched = 0
    while matched < len(MAGIC):
        byte = read_exact(serial_port, 1)[0]
        if byte == MAGIC[matched]:
            matched += 1
        else:
            matched = 1 if byte == MAGIC[0] else 0


def read_frame(serial_port) -> Frame:
    find_magic(serial_port)
    mode, tx, rx_mask, count, sequence, mux_mask = HEADER.unpack(read_exact(serial_port, HEADER.size))
    payload_size = math.ceil(count * 10 / 8)
    samples = unpack_10bit(read_exact(serial_port, payload_size), count)
    frame = Frame(mode, tx, rx_mask, count, sequence, mux_mask, samples, time.time())
    validate_frame(frame)
    return frame


def validate_frame(frame: Frame) -> None:
    if frame.mode == 2:
        if frame.tx not in range(1, 5) or frame.rx_mask != 0:
            raise ValueError("Invalid MUX diagnostic frame")
        return
    if frame.tx not in range(1, 5):
        raise ValueError(f"Invalid TX channel {frame.tx}")
    if frame.rx_mask & (1 << (frame.tx - 1)):
        raise ValueError(f"TX{frame.tx} is also enabled as a receiver")
    if frame.mode != 0 or frame.rx_mask.bit_count() != 1:
        raise ValueError(f"Expected one receiver, got mode {frame.mode}, mask 0x{frame.rx_mask:02X}")


def find_serial_port() -> str:
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    likely = [
        port.device
        for port in ports
        if any(name in (port.description or "").lower() for name in ("pico", "rp2", "usb serial"))
    ]
    if len(likely) == 1:
        return likely[0]
    if len(ports) == 1:
        return ports[0].device
    choices = ", ".join(port.device for port in ports) or "none"
    raise RuntimeError(f"Could not choose the Pico serial port. Available ports: {choices}. Use --port.")


def frame_summary(frame: Frame, sample_rate_mhz: float) -> dict[str, object]:
    samples = frame.samples.astype(np.float64)
    centered = samples - np.median(samples)
    peak_sample = int(np.argmax(np.abs(centered)))
    receivers = "+".join(map(str, frame.receivers))
    frames_per_cycle = len(PAIRWISE_PATHS)
    return {
        "received_iso": datetime.fromtimestamp(frame.received_at).isoformat(timespec="milliseconds"),
        "sequence": frame.sequence,
        "cycle": frame.sequence // frames_per_cycle,
        "mode": "pairwise" if frame.mode == 0 else "summed",
        "tx": frame.tx,
        "rx": receivers,
        "rx_mask_hex": f"0x{frame.rx_mask:02X}",
        "mux_mask_hex": f"0x{frame.mux_mask:04X}",
        "samples": frame.sample_count,
        "min_adc": int(frame.samples.min()),
        "max_adc": int(frame.samples.max()),
        "mean_adc": float(samples.mean()),
        "std_adc": float(samples.std()),
        "peak_sample": peak_sample,
        "peak_us": peak_sample / sample_rate_mhz,
    }


def save_results(frames: list[Frame], output_dir: Path, sample_rate_mhz: float) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    run_dir = output_dir / datetime.now().strftime("array_%Y%m%d_%H%M%S")
    run_dir.mkdir()

    summaries = [frame_summary(frame, sample_rate_mhz) for frame in frames]
    with (run_dir / "frame_statistics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    np.savez_compressed(
        run_dir / "raw_frames.npz",
        adc=np.stack([frame.samples for frame in frames]),
        sequence=np.array([frame.sequence for frame in frames], dtype=np.uint32),
        tx=np.array([frame.tx for frame in frames], dtype=np.uint8),
        rx_mask=np.array([frame.rx_mask for frame in frames], dtype=np.uint8),
        mux_mask=np.array([frame.mux_mask for frame in frames], dtype=np.uint16),
        sample_rate_mhz=np.array(sample_rate_mhz),
    )
    save_overview_plot(frames, run_dir / "directed_paths.png", sample_rate_mhz)
    return run_dir


def save_overview_plot(frames: list[Frame], filename: Path, sample_rate_mhz: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pairwise = [frame for frame in frames if frame.mode == 0]
    if not pairwise:
        return

    fig, axes = plt.subplots(4, 3, figsize=(13, 9), sharex=True, sharey=True)
    time_us = np.arange(pairwise[0].sample_count) / sample_rate_mhz
    for axis, (tx, rx) in zip(axes.ravel(), PAIRWISE_PATHS):
        selected = [frame.samples for frame in pairwise if frame.tx == tx and frame.receivers == [rx]]
        if selected:
            data = np.mean(np.stack(selected).astype(np.float64), axis=0)
            axis.plot(time_us, data, color="#00a99d", linewidth=0.8)
        axis.set_title(f"TX{tx} -> RX{rx}")
        axis.grid(alpha=0.2)
    fig.supxlabel("Time (us)")
    fig.supylabel("ADC raw")
    fig.suptitle(f"Four-channel directed scan: mean of {len(pairwise) // len(PAIRWISE_PATHS)} cycles")
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def simulated_frames(cycles: int, samples: int, sample_rate_mhz: float, paths: list[tuple[int, int]] | None = None) -> list[Frame]:
    rng = np.random.default_rng(42)
    frames: list[Frame] = []
    sequence = 0
    x = np.arange(samples)
    paths = paths or PAIRWISE_PATHS
    for _ in range(cycles):
        for tx, rx in paths:
            delay = int(samples * (0.18 + 0.04 * abs(tx - rx)))
            envelope = np.exp(-((x - delay) / 55.0) ** 2)
            signal = 512 + 150 * envelope * np.sin(2 * np.pi * 3.8 * x / sample_rate_mhz)
            signal += rng.normal(0, 4, samples)
            rx_mask = 1 << (rx - 1)
            mux_mask = (1 << (2 * (tx - 1) + 1)) | (1 << (2 * (rx - 1)))
            frames.append(
                Frame(0, tx, rx_mask, samples, sequence, mux_mask, np.clip(signal, 0, 1023).astype(np.uint16), time.time())
            )
            sequence += 1
    return frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the separate pic0rick four-channel MUX experiment.")
    parser.add_argument("--port", help="Pico serial port; auto-detected when omitted")
    parser.add_argument("--cycles", type=int, default=100, help="Complete scans to capture")
    parser.add_argument("--samples", type=int, default=2000, choices=range(1, 2001), metavar="1..2000")
    parser.add_argument("--dac", type=int, default=500)
    parser.add_argument("--pon", type=int, default=100, help="Pulse on time in ns")
    parser.add_argument("--poff", type=int, default=100, help="Pulse off time in ns")
    parser.add_argument("--damp", type=int, default=10, help="Damping time in ns")
    parser.add_argument("--settle-us", type=int, default=8, help="MUX settling time; firmware enforces at least 5 us")
    parser.add_argument("--sample-rate-mhz", type=float, default=60.0)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "output")
    parser.add_argument("--simulate", action="store_true", help="Generate a short hardware-free test run")
    parser.add_argument("--scan-mode", choices=("dual", "four"), default="four", help="Path sequence to capture")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.cycles < 1:
        raise SystemExit("--cycles must be at least 1")

    paths = TWO_CHANNEL_PATHS if args.scan_mode == "dual" else FOUR_CHANNEL_PATHS

    if args.simulate:
        frames = simulated_frames(args.cycles, args.samples, args.sample_rate_mhz, paths)
    else:
        try:
            import serial
        except ImportError as exc:
            raise SystemExit("pyserial is required. Run: python -m pip install -r requirements.txt") from exc

        port = args.port or find_serial_port()
        frame_target = args.cycles * len(paths)
        frames = []
        command_name = "dual stream" if args.scan_mode == "dual" else "four stream"
        command = (
            f"{command_name} {args.samples} {args.pon} "
            f"{args.poff} {args.damp} {args.settle_us}\n"
        )
        print(f"Opening {port}")
        with serial.Serial(port, 921600, timeout=2, write_timeout=2) as serial_port:
            serial_port.write(b"q\n")
            time.sleep(0.05)
            serial_port.reset_input_buffer()
            serial_port.write(f"write dac {args.dac}\n".encode("ascii"))
            time.sleep(0.05)
            serial_port.reset_input_buffer()
            serial_port.write(command.encode("ascii"))
            started = time.perf_counter()
            try:
                while len(frames) < frame_target:
                    frame = read_frame(serial_port)
                    frames.append(frame)
                    if len(frames) % len(paths) == 0:
                        elapsed = time.perf_counter() - started
                        completed = len(frames) // len(paths)
                        print(
                            f"\rCycles {completed}/{args.cycles} | "
                            f"{len(frames) / elapsed:6.1f} frames/s | "
                            f"last TX{frame.tx}->RX{'+'.join(map(str, frame.receivers))}",
                            end="",
                            flush=True,
                        )
            finally:
                serial_port.write(b"q\n")
        print()

    run_dir = save_results(frames, args.output, args.sample_rate_mhz)
    print(f"Saved {len(frames)} frames to {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
