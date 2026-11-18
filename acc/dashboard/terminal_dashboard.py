#!/usr/bin/env python3
"""Simple terminal dashboard for the ACC project.

Reads the latest row from the simulator trace CSV and renders a live dashboard.
This is intended to run on the QNX target or Pi over SSH.
"""

import csv
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = PROJECT_ROOT / "logs"


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def safest_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def read_latest_csv_row(trace_path: Path):
    if not trace_path.exists():
        return None

    with trace_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows:
            return None
        return rows[-1]


def read_safety_log(log_path: Path):
    if not log_path.exists():
        return []
    with log_path.open("r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]
    return lines[-5:]


def fmt_mode(m):
    modes = {
        "0": "OFF",
        "1": "SPEED",
        "2": "FOLLOW",
        "3": "EMERGENCY",
        "4": "FAULT",
    }
    return modes.get(str(int(float(m) if m not in (None, "") else 0)), "UNKNOWN")


def build_dashboard(scenario: int):
    trace_path = LOG_ROOT / f"s{scenario}" / "trace.csv"
    safety_path = LOG_ROOT / f"s{scenario}" / "safety_monitor.log"
    row = read_latest_csv_row(trace_path)
    safety_lines = read_safety_log(safety_path)

    if row is None:
        return (
            "No live trace available yet.\n"
            "Start the ACC run first: ./scripts/run_acc.sh 1 60\n"
        )

    t = safest_float(row.get("t"), 0.0)
    v_ego = safest_float(row.get("v_ego"), 0.0)
    v_lead = safest_float(row.get("v_lead"), 0.0)
    gap = safest_float(row.get("gap"), 0.0)
    d_des = safest_float(row.get("d_des"), 0.0)
    tau = safest_float(row.get("tau"), 0.0)
    throttle = safest_float(row.get("throttle"), 0.0)
    brake = safest_float(row.get("brake"), 0.0)
    mode = fmt_mode(row.get("mode", "0"))
    fault = int(float(row.get("fault", 0) or 0))
    warn = int(float(row.get("warn", 0) or 0))
    ttc = safest_float(row.get("ttc"), 0.0)
    if ttc < 0:
        ttc = -1.0

    throttle_pct = min(max(throttle, 0.0), 1.0) * 100.0
    brake_pct = min(max(brake, 0.0), 1.0) * 100.0

    safety_text = safety_lines[-2:] if safety_lines else ["No safety events yet"]
    bars = {
        "throttle": min(max(int(throttle_pct), 0), 100),
        "brake": min(max(int(brake_pct), 0), 100),
    }

    throttle_bar = "#" * bars["throttle"] + " " * (100 - bars["throttle"])
    brake_bar = "#" * bars["brake"] + " " * (100 - bars["brake"])

    lines = []
    lines.append("===========================================================")
    lines.append("                 ACC SYSTEM LIVE DASHBOARD")
    lines.append("===========================================================")
    lines.append(f"Time: {t:6.1f}s    Scenario: {scenario}    Mode: {mode:<10}    Faults: 0x{fault:02x}")
    lines.append(f"Ego Speed: {v_ego:6.1f} m/s   Lead Speed: {v_lead:6.1f} m/s   Gap: {gap:6.1f} m")
    lines.append(f"Desired Gap: {d_des:6.1f} m   TTC: {ttc:5.1f}s   Tau: {tau:4.2f}s")
    lines.append(f"Throttle: {throttle_pct:5.1f}%   Brake: {brake_pct:5.1f}%   Warn Level: {warn}")
    lines.append("")
    lines.append("Pedal demand")
    lines.append(f"THR  [{throttle_bar[:100]}] {throttle_pct:5.1f}%")
    lines.append(f"BRK  [{brake_bar[:100]}] {brake_pct:5.1f}%")
    lines.append("")
    lines.append("Safety")
    for entry in safety_text:
        lines.append(f"  {entry}")
    lines.append("===========================================================")
    lines.append("Commands: q=quit   r=refresh   SSH-friendly dashboard")
    lines.append("===========================================================")
    return "\n".join(lines)


def main():
    scenario = 1
    if len(sys.argv) > 1:
        try:
            scenario = int(sys.argv[1])
        except ValueError:
            print("Usage: python3 terminal_dashboard.py [scenario]")
            sys.exit(1)

    try:
        while True:
            clear_screen()
            print(build_dashboard(scenario))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
