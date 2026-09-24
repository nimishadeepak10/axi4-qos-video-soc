#!/usr/bin/env python3
"""bandwidth_analysis.py

Computes sustained memory-subsystem bandwidth from a real RTL simulation
run, not from a claimed number. Reads bandwidth_log.csv (written by
tb/axi4_qos_video_tb.sv during Test 10 - the DMA-frame-transfer-under-
concurrent-CPU-and-codec-load scenario) and reports:

  1. The measured bytes/cycle the DMA actually achieved while contending
     with CPU and codec traffic in cycle-accurate RTL simulation.
  2. That ratio projected to a real clock frequency (MB/s), since
     simulating a full real-time 1080p frame beat-by-beat in RTL sim
     (millions of beats) is not practical - the steady-state throughput
     measured over a representative contended transfer is what "sustained
     bandwidth" means, and is a standard verification/perf-engineering
     technique (measure steady-state ratio, scale by the real clock).

Usage:
    python bandwidth_analysis.py [--csv bandwidth_log.csv] [--freq-mhz 200]
                                  [--frame-bytes 4147200]

--frame-bytes defaults to a 1920x1080 YUV422 frame (1920*1080*2).
"""

import argparse
import csv
import sys


def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="bandwidth_log.csv", help="Transaction log written by the testbench")
    ap.add_argument("--freq-mhz", type=float, default=200.0, help="Clock frequency to project sustained bandwidth at")
    ap.add_argument("--frame-bytes", type=int, default=1920 * 1080 * 2, help="Bytes in one full frame (default: 1080p YUV422)")
    args = ap.parse_args()

    try:
        rows = load_rows(args.csv)
    except FileNotFoundError:
        print(f"error: {args.csv} not found - run the testbench first (it writes this file).", file=sys.stderr)
        sys.exit(1)

    meta = {r["dir"]: int(r["bytes"]) for r in rows if r["master"] == "meta"}
    if "bw_cycles" not in meta or "bw_bytes" not in meta:
        print("error: bandwidth_log.csv has no bw_cycles/bw_bytes meta rows - "
              "did Test 10 in the testbench run and complete?", file=sys.stderr)
        sys.exit(1)

    measured_cycles = meta["bw_cycles"]
    measured_bytes = meta["bw_bytes"]
    bytes_per_cycle = measured_bytes / measured_cycles

    freq_hz = args.freq_mhz * 1e6
    bytes_per_sec = bytes_per_cycle * freq_hz
    mb_per_sec = bytes_per_sec / (1024 * 1024)

    # Per-master traffic mix during the whole run, for context.
    per_master_bytes = {}
    for r in rows:
        if r["master"] == "meta":
            continue
        key = (r["master"], r["dir"])
        per_master_bytes[key] = per_master_bytes.get(key, 0) + int(r["bytes"])

    frame_time_s = args.frame_bytes / bytes_per_sec if bytes_per_sec > 0 else float("inf")

    print("=" * 70)
    print(" Bandwidth analysis (from real RTL simulation, not a target number)")
    print("=" * 70)
    print(f" Source log                  : {args.csv}")
    print(f" DMA transfer under contention: {measured_bytes} bytes in {measured_cycles} cycles")
    print(f" Measured throughput          : {bytes_per_cycle:.3f} bytes/cycle")
    print(f" Projected clock              : {args.freq_mhz:.1f} MHz")
    print(f" Projected sustained bandwidth: {mb_per_sec:.1f} MB/s")
    print(f" Time to move one {args.frame_bytes}-byte frame at this rate: {frame_time_s * 1000:.2f} ms "
          f"({1.0 / frame_time_s:.1f} fps if this were the only transfer)")
    print("-" * 70)
    print(" Methodology: measured over a representative contended DMA burst")
    print(" (255 words/row x 16 rows, concurrent CPU writes + codec read/write")
    print(" bursts), not a full frame simulated beat-by-beat in RTL - that would")
    print(" be ~1M+ beats and impractical to simulate. This is the standard")
    print(" steady-state-throughput-then-scale technique, not a full-system")
    print(" real-time replay.")
    print("-" * 70)
    print(" Per-master traffic during the full test run:")
    for (master, direction), b in sorted(per_master_bytes.items()):
        print(f"   {master:8s} {direction:3s} : {b:8d} bytes")
    print("=" * 70)


if __name__ == "__main__":
    main()
