# LTC3350 SuperCap UPS Monitor

Shuts down a Raspberry Pi cleanly before its supercapacitor backup stack is
exhausted, using the LTC3350's V² decay curve, and reports stack health over time.

## Hardware
- Seeed SuperCAP UPS (LTC3350) module, I2C bus 6 addr 0x09
- RPi power-fail signal: GPIO 25, active low
- Stack: 4× cell, ~1.75 F, 6.5–10.57 V design window (per R17/R16 divider)

## Components
| File | Role |
|---|---|
| `ltc3350_driver.py` | Flat I2C driver: one snapshot per read, all conversions |
| `monitor-ups.py` | Long-running monitor: detects power loss, estimates runtime, shuts down |
| `monitor-ups-health.py` | One-shot health check: CAP/ESR measurement → 0-100 score |
| `ups.service` / `ups-health.service` / `ups-health.timer` | systemd integration |

## How the shutdown decision works
- Sampling starts the moment GPIO25 goes low (during the grace debounce)
- Least-squares slope of V² over a 12-sample window → seconds to 6.5 V cutoff
  (constant-power discharge ⇒ V² decays linearly)
- Shut down when any of:
    - est ≤ UPS_LEAD_S (4–7 s)
    - VCAP ≤ UPS_VCAP_FLOOR (7.0 V)
    - time since power loss ≥ UPS_MAX_HOLD (30 s fail-safe)
- Power restored → cancel; I2C lost → GPIO + hold fallback still shuts down

## Configuration (environment variables)
| Var | Default | Meaning |
|---|---|---|
| UPS_GRACE_PERIOD | 3 | debounce after power loss, s |
| UPS_LEAD_S | 7 | shutdown lead, s |
| UPS_VCAP_FLOOR | 7.0 | floor voltage, V |
| UPS_MAX_HOLD | 30 | max time on backup, s |
| UPS_CUTOFF_V | 6.5 | cutoff for estimate, V |
| UPS_POLL_INTERVAL | 0.25 | poll period, s |
| UPS_HEALTH_MIN_INTERVAL | 6 | min hours between real health measurements |

## Health scoring
- Real CAP/ESR measurement via CTL register; first good read seeds baseline
- Score = esr 35% + capacitance 35% + max_vcap 15% + temp 10% + alarms 5%
- ESR_HIGH / CAP_LOW alarms → −40; other alarms → −15
- State in `ups_health_state.json`; output JSON via `--out`

## Installation
- Dependencies: `python3-smbus i2c-tools` (and `RPi.GPIO`, pip-only)
- `apt install ./ups-monitor_*.deb` (build via `build-deb.sh`), or run from
  this directory with the units copied to `/etc/systemd/system`
- Ensure the i2c-dev module is loaded `echo i2c-dev | sudo tee /etc/modules-load.d/i2c.conf`

## Validation
- `python3 ltc3350_driver.py && python3 monitor-ups.py --check &&
  python3 monitor-ups-health.py --check`
- Field-validated: estimator tracks real decay to within a second
  (12-sample window; ~3.2 W load → ~19 s full-stack runtime)

## Outage capture
- Each outage → `outage-YYYYMMDD-HHMMSS.csv` (elapsed_s, vcap, v2, est_s)
  in the working directory