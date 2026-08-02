#!/usr/bin/env python3
"""SuperCAP UPS health check (one-shot).

Runs the LTC3350 CAP/ESR measurement, reads ESR, capacitance, VCAP, SOC,
temperature and alarms, updates a small persistent state file (baseline +
tracked max_vcap) and emits a 0-100 health score as JSON.

Usage:
    monitor-ups-health.py [--out FILE]

State file (ups_health_state.json, in the working directory):
    baseline_cap_f, baseline_esr_mohm, tracked_max_vcap
The first successful measurement seeds the baseline; the health score then
measures drift from it, so per-box supercap tolerance needs no calibration.
"""

import json
import os
import sys

from datetime import datetime

try:
    from ltc3350_driver import LTC3350
except ImportError:
    LTC3350 = None

STATE_FILE = "ups_health_state.json"
DEFAULT_FULL_V = 10.571          # schematic design 100% charge
MIN_INTERVAL_HOURS = float(os.getenv("UPS_HEALTH_MIN_INTERVAL", "6"))
ESR_HIGH_ALARM = "ESR_HIGH"
CAP_LOW_ALARM = "CAP_LOW"
ALARM_PENALTY = 15               # per minor alarm
MAJOR_ALARM_PENALTY = 40         # ESR_HIGH / CAP_LOW

WEIGHTS = {"esr": 0.35, "cap": 0.35, "vcap": 0.15, "temp": 0.10, "alarm": 0.05}


def compute_health(esr_mohm, cap_f, max_vcap, temp_c, alarms,
                   baseline_esr, baseline_cap, full_v=DEFAULT_FULL_V):
    """Pure 0-100 health score. baselines may be None before first seed."""

    def ratio(value, baseline):
        if baseline is None or baseline <= 0 or value is None:
            return 1.0
        return min(1.0, value / baseline)

    esr_score = ratio(baseline_esr, esr_mohm) * 100 if esr_mohm else 100
    cap_score = ratio(cap_f, baseline_cap) * 100
    vcap_score = min(1.0, max_vcap / full_v) * 100
    temp_score = 100.0 if temp_c <= 60 else max(0.0, 100 - (temp_c - 60) * 2)

    penalty = 0
    for alarm in alarms:
        penalty += MAJOR_ALARM_PENALTY if alarm in (ESR_HIGH_ALARM, CAP_LOW_ALARM) \
            else ALARM_PENALTY
    alarm_score = max(0.0, 100.0 - penalty)

    score = (
        WEIGHTS["esr"] * esr_score
        + WEIGHTS["cap"] * cap_score
        + WEIGHTS["vcap"] * vcap_score
        + WEIGHTS["temp"] * temp_score
        + WEIGHTS["alarm"] * alarm_score
    )

    return {
        "score": round(score),
        "components": {
            "esr": round(esr_score, 1),
            "capacitance": round(cap_score, 1),
            "max_vcap": round(vcap_score, 1),
            "temperature": round(temp_score, 1),
            "alarms": round(alarm_score, 1),
        },
    }


def _load_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def main(argv):
    out_file = None
    if "--out" in argv:
        out_file = argv[argv.index("--out") + 1]

    state = _load_state()
    now = datetime.now()

    last_check = state.get("last_check")
    if last_check and not _interval_elapsed(last_check, MIN_INTERVAL_HOURS):
        out = {
            "timestamp": now.isoformat(),
            "skipped": "within min interval",
            "health": state.get("last_health"),
            "measurements": {
                "tracked_max_vcap": state.get("tracked_max_vcap"),
                "full_v_reference": round(max(DEFAULT_FULL_V, state.get("tracked_max_vcap", 0.0)), 4),
            },
        }
        _write_out(out, out_file)
        return

    if LTC3350 is None:
        print(json.dumps({"error": "ltc3350_driver not importable"}))
        sys.exit(1)

    ltc = LTC3350()

    try:
        ltc.start_cap_esr_measurement()
        result = ltc.wait_cap_esr_measurement(timeout=12)
    except Exception as exc:
        print(json.dumps({"error": f"measurement failed: {exc}"}))
        sys.exit(1)

    snap = ltc.snapshot()
    m = snap["measurements"]
    raw = snap["raw"]

    tracked_max = state.get("tracked_max_vcap", 0.0)
    full_v = max(DEFAULT_FULL_V, tracked_max)

    cap_f = m["stack_capacitance_f"]
    esr_mohm = m["esr_milliohms"]
    vcap = m["vcap"]

    if vcap > tracked_max:
        tracked_max = vcap

    baseline_cap = state.get("baseline_cap_f")
    baseline_esr = state.get("baseline_esr_mohm")

    if result == "done":
        if baseline_cap is None and cap_f > 0:
            baseline_cap = cap_f
        if baseline_esr is None and esr_mohm > 0:
            baseline_esr = esr_mohm

    health = compute_health(
        esr_mohm, cap_f, tracked_max, m["temperature_c"],
        snap["alarms"]["active"], baseline_esr, baseline_cap, full_v,
    )

    _save_state({
        "last_check": now.isoformat(),
        "measurement": result,
        "baseline_cap_f": baseline_cap,
        "baseline_esr_mohm": baseline_esr,
        "tracked_max_vcap": round(tracked_max, 4),
        "last_health": health,
    })

    out = {
        "timestamp": now.isoformat(),
        "measurement": result,
        "health": health,
        "measurements": {
            "esr_milliohms": esr_mohm,
            "stack_capacitance_f": cap_f,
            "vcap": vcap,
            "state_of_charge": m["state_of_charge"],
            "temperature_c": m["temperature_c"],
            "tracked_max_vcap": round(tracked_max, 4),
            "full_v_reference": round(full_v, 4),
        },
        "alarms": snap["alarms"]["active"],
        "num_caps": ltc.num_capacitors(raw["num_caps"]),
    }

    _write_out(out, out_file)


def _write_out(out, out_file):
    text = json.dumps(out, indent=2)
    if out_file:
        with open(out_file, "w") as f:
            f.write(text + "\n")
    else:
        print(text)


def _interval_elapsed(iso, hours):
    from datetime import timedelta
    try:
        last = datetime.fromisoformat(iso)
    except ValueError:
        return True
    return datetime.now() - last >= timedelta(hours=hours)


def _self_check():
    # healthy box, matches baseline -> near 100
    h = compute_health(41.72, 1.75, 10.55, 50.0, [], 41.72, 1.75)
    assert h["score"] >= 95, h
    # ESR drift up halves the esr component -> score drops
    h_bad = compute_health(90.0, 1.75, 10.55, 50.0, [], 41.72, 1.75)
    assert h_bad["score"] < h["score"]
    # major alarm caps the alarm component
    h_alarm = compute_health(41.72, 1.75, 10.55, 50.0, ["ESR_HIGH"], 41.72, 1.75)
    assert h_alarm["components"]["alarms"] == 60.0
    # no baseline yet -> only cap-limited components, no crash
    h_first = compute_health(41.72, 1.75, 10.55, 50.0, [], None, None)
    assert 0 <= h_first["score"] <= 100
    # interval guard
    assert _interval_elapsed("now", 1) is True            # bad iso -> run
    assert _interval_elapsed("1970-01-01T00:00:00", 1) is True
    assert _interval_elapsed(datetime.now().isoformat(), 1) is False
    print("monitor-ups-health: self-check OK")


if __name__ == "__main__":
    if "--check" in sys.argv:
        _self_check()
    else:
        main(sys.argv[1:])
