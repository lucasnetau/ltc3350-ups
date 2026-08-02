#!/usr/bin/env python3
"""SuperCAP UPS monitor.

Shuts the system down just before the capacitor stack runs out, using the
LTC3350's VCAP decay curve (V^2 decays linearly at constant load power).

Dual path:
  - GPIO 25 (power-fail, active low) is the primary failure signal.
  - The I2C VCAP reading is used to estimate remaining runtime from the
    slope of V^2 over a sliding window.

Shutdown when any of:
  - estimated seconds-to-cutoff <= UPS_LEAD_S  (default 7s)
  - VCAP <= UPS_VCAP_FLOOR                      (default 7.0 V)
  - time since power loss >= UPS_MAX_HOLD       (default 30s, fail-safe)

If I2C is unavailable the GPIO + max-hold path still shuts down.
"""

import csv
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    from ltc3350_driver import LTC3350
except ImportError:
    LTC3350 = None

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

POWER_FAIL_GPIO = 25
POLL_INTERVAL = float(os.getenv("UPS_POLL_INTERVAL", "0.25"))
GRACE_PERIOD = float(os.getenv("UPS_GRACE_PERIOD", "3"))
LEAD_S = float(os.getenv("UPS_LEAD_S", "7"))
VCAP_FLOOR = float(os.getenv("UPS_VCAP_FLOOR", "7.0"))
MAX_HOLD_S = float(os.getenv("UPS_MAX_HOLD", "30"))
CUTOFF_V = float(os.getenv("UPS_CUTOFF_V", "6.5"))

WINDOW_SAMPLES = 12          # ~3s at 0.25s poll
MIN_SAMPLES = 6
MIN_SLOPE_V2_PER_S = 0.05    # ignore near-flat curves (self-discharge noise)

STATE_PRESENT = "PRESENT"
STATE_GRACE = "GRACE"
STATE_BACKUP = "BACKUP"


def log(message):
    print(f"{datetime.now():%Y-%m-%d %H:%M:%S}  {message}", flush=True)


def shutdown():
    log("Synchronising filesystems...")
    subprocess.run(["sync"], check=False)
    time.sleep(1)
    subprocess.run(["sync"], check=False)
    log("Issuing shutdown command...")
    subprocess.run(["shutdown", "-h", "now"], check=False)


# ---------------------------------------------------------------------
# Pure decision logic (testable off-device)
# ---------------------------------------------------------------------

def time_to_cutoff(samples):
    # Least-squares slope of V^2 vs time -> seconds until cutoff.
    if len(samples) < MIN_SAMPLES:
        return None

    n = len(samples)
    sum_t = sum(s[0] for s in samples)
    sum_v2 = sum(s[1] for s in samples)
    sum_tt = sum(s[0] * s[0] for s in samples)
    sum_tv2 = sum(s[0] * s[1] for s in samples)

    denom = n * sum_tt - sum_t * sum_t
    if abs(denom) < 1e-9:
        return None

    slope = (n * sum_tv2 - sum_t * sum_v2) / denom
    if slope > -MIN_SLOPE_V2_PER_S:
        return None

    last_v2 = samples[-1][1]
    if last_v2 <= CUTOFF_V ** 2:
        return 0.0

    return (last_v2 - CUTOFF_V ** 2) / abs(slope)


def shutdown_due(elapsed, vcap, estimate):
    # Return (bool, reason). estimate is None when no usable curve.
    if vcap is not None and vcap <= VCAP_FLOOR:
        return True, f"vcap {vcap:.2f}V <= floor {VCAP_FLOOR}V"
    if estimate is not None and estimate <= LEAD_S:
        return True, f"est {estimate:.1f}s <= lead {LEAD_S}s"
    if elapsed >= MAX_HOLD_S:
        return True, f"hold {elapsed:.1f}s >= max {MAX_HOLD_S}s"
    return False, None


# ---------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------

def main():
    if GPIO is None:
        log("RPi.GPIO unavailable - cannot monitor power fail signal.")
        sys.exit(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(POWER_FAIL_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    ltc = None
    if LTC3350 is not None:
        try:
            ltc = LTC3350()
            ltc.read_measurements()
        except Exception as exc:
            log(f"I2C unavailable, GPIO-only mode: {exc}")
            ltc = None

    def cleanup(signum=None, frame=None):
        log("Stopping UPS monitor.")
        GPIO.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    log("UPS monitor started.")

    state = STATE_PRESENT
    state_since = time.monotonic()
    samples = []          # (t, v2) during backup
    capture = None        # outage capture file
    writer = None         # csv writer for capture
    start = time.monotonic()

    try:
        while True:
            power_present = GPIO.input(POWER_FAIL_GPIO) == GPIO.HIGH
            now = time.monotonic()

            if power_present:
                if state != STATE_PRESENT:
                    log("Power restored. Shutdown cancelled.")
                    if capture is not None:
                        capture.close()
                        capture = None
                    writer = None
                    samples = []
                    state = STATE_PRESENT
            else:
                if state == STATE_PRESENT:
                    state = STATE_GRACE
                    state_since = now
                    start = now
                    samples = []
                    capture = open(
                        os.path.join(
                            os.getcwd(),
                            f"outage-{datetime.now():%Y%m%d-%H%M%S}.csv",
                        ),
                        "w",
                        newline="",
                    )
                    writer = csv.writer(capture)
                    writer.writerow(["elapsed_s", "vcap", "v2", "est_s"])
                    # Flush the bulk of dirty pages now while the stack is full;
                    # shutdown()'s sync then only has the capture file to write.
                    subprocess.Popen(["sync"])
                    log(f"External power lost. Sampling decay, grace {GRACE_PERIOD:.1f}s.")
                elif state == STATE_GRACE:
                    if now - state_since >= GRACE_PERIOD:
                        state = STATE_BACKUP
                        log("Backup in progress.")

            if state in (STATE_GRACE, STATE_BACKUP):
                vcap = None
                est = None
                if ltc is not None:
                    try:
                        vcap = ltc.capacitor_voltage(ltc.read16(0x26))
                    except Exception:
                        ltc = None
                        log("I2C lost - GPIO + hold fallback.")

                if vcap is not None:
                    samples.append((now - start, vcap ** 2))
                    if len(samples) > WINDOW_SAMPLES:
                        samples.pop(0)
                    est = time_to_cutoff(samples)
                    if capture is not None and writer is not None:
                        writer.writerow([
                            round(now - start, 2),
                            round(vcap, 3),
                            round(vcap ** 2, 3),
                            "NA" if est is None else round(est, 1),
                        ])
                        capture.flush()

                if state == STATE_BACKUP:
                    elapsed = now - start
                    due, reason = shutdown_due(elapsed, vcap, est)
                    if due:
                        log(f"Shutdown: {reason}")
                        shutdown()
                        break

            time.sleep(POLL_INTERVAL)
    finally:
        GPIO.cleanup()
        if capture is not None:
            capture.close()


def _self_check():
    # perfect linear V^2 decay, slope -10 V^2/s -> (50 - 42.25)/10 = 0.775s
    est = time_to_cutoff([(0, 100), (1, 90), (2, 80), (3, 70), (4, 60), (5, 50)])
    assert est is not None and abs(est - 0.775) < 1e-9
    assert shutdown_due(10, 7.1, 3.0)[0] is True               # est lead
    assert shutdown_due(10, 6.9, None)[0] is True              # floor
    assert shutdown_due(31, None, None)[0] is True             # max hold
    assert shutdown_due(10, 8.5, 20.0)[0] is False             # all clear
    assert time_to_cutoff([(0, 100), (1, 99.5), (2, 100)]) is None  # flat
    print("monitor-ups: self-check OK")


if __name__ == "__main__":
    if "--check" in sys.argv:
        _self_check()
    else:
        main()
