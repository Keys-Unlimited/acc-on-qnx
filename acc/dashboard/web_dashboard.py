#!/usr/bin/env python3
"""Simple web dashboard for the ACC project.

Serves a browser dashboard and exposes a JSON API that reads the current trace CSV.
This is intended for Pi deployment or a laptop connected over SSH/browser.
"""

import csv
from datetime import datetime
from pathlib import Path

from flask import Flask, jsonify, render_template

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = PROJECT_ROOT / "logs"

app = Flask(__name__, template_folder="templates")


def safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def mode_name(value):
    modes = {
        0: "OFF",
        1: "SPEED",
        2: "FOLLOW",
        3: "EMERGENCY",
        4: "FAULT",
    }
    try:
        return modes.get(int(float(value)), "UNKNOWN")
    except (TypeError, ValueError):
        return "UNKNOWN"


def read_trace_rows(trace_path: Path):
    if not trace_path.exists():
        return []
    with trace_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def latest_snapshot(scenario: int):
    trace_path = LOG_ROOT / f"s{scenario}" / "trace.csv"
    rows = read_trace_rows(trace_path)
    if not rows:
        return {
            "time_s": 0.0,
            "scenario": scenario,
            "mode": "OFF",
            "enabled": False,
            "fault_flags": 0,
            "warn_level": 0,
            "ego_speed_mps": 0.0,
            "lead_speed_mps": 0.0,
            "gap_m": 0.0,
            "desired_gap_m": 0.0,
            "ttc_s": 0.0,
            "throttle_pct": 0.0,
            "brake_pct": 0.0,
            "set_speed_mps": 0.0,
            "tau_s": 0.0,
            "gain_scale": 1.0,
            "status": "waiting",
            "timestamp": datetime.utcnow().isoformat() + "Z",
        }

    row = rows[-1]
    ttc = safe_float(row.get("ttc"), 0.0)
    if ttc < 0:
        ttc = 0.0

    return {
        "time_s": safe_float(row.get("t"), 0.0),
        "scenario": scenario,
        "mode": mode_name(row.get("mode")),
        "enabled": True,
        "fault_flags": int(float(row.get("fault", 0) or 0)),
        "warn_level": int(float(row.get("warn", 0) or 0)),
        "ego_speed_mps": safe_float(row.get("v_ego"), 0.0),
        "lead_speed_mps": safe_float(row.get("v_lead"), 0.0),
        "gap_m": safe_float(row.get("gap"), 0.0),
        "desired_gap_m": safe_float(row.get("d_des"), 0.0),
        "ttc_s": ttc,
        "throttle_pct": safe_float(row.get("throttle"), 0.0),
        "brake_pct": safe_float(row.get("brake"), 0.0),
        "set_speed_mps": 30.0,
        "tau_s": safe_float(row.get("tau"), 0.0),
        "gain_scale": 1.0,
        "status": "running",
        "timestamp": datetime.utcnow().isoformat() + "Z",
    }


def read_recent_events(scenario: int):
    log_path = LOG_ROOT / f"s{scenario}" / "safety_monitor.log"
    if not log_path.exists():
        return []
    events = []
    with log_path.open("r", encoding="utf-8") as f:
        for line in f:
            text = line.strip()
            if not text:
                continue
            if any(token in text.lower() for token in ["fault", "warning", "aeb", "reset", "healthy"]):
                events.append({"text": text})
    return events[-10:]


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    scenario = 1
    try:
        scenario = int(app.config.get("SCENARIO", 1))
    except Exception:
        scenario = 1
    return jsonify(latest_snapshot(scenario))


@app.route("/api/history")
def api_history():
    scenario = 1
    trace_path = LOG_ROOT / f"s{scenario}" / "trace.csv"
    rows = read_trace_rows(trace_path)
    history = []
    for row in rows[-200:]:
        history.append({
            "t": safe_float(row.get("t"), 0.0),
            "ego_speed_mps": safe_float(row.get("v_ego"), 0.0),
            "gap_m": safe_float(row.get("gap"), 0.0),
            "ttc_s": safe_float(row.get("ttc"), 0.0),
            "throttle_pct": safe_float(row.get("throttle"), 0.0),
            "brake_pct": safe_float(row.get("brake"), 0.0),
            "mode": mode_name(row.get("mode")),
        })
    return jsonify({"points": history})


@app.route("/api/events")
def api_events():
    scenario = 1
    return jsonify({"events": read_recent_events(scenario)})


if __name__ == "__main__":
    app.config["SCENARIO"] = 1
    app.run(host="0.0.0.0", port=8000, debug=False)
