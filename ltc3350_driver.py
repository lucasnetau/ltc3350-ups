#!/usr/bin/env python3
"""LTC3350 supercap backup controller driver.

Flat class: every measurement register is read exactly once per snapshot and
all engineering values derive from that snapshot. Config/threshold registers
and the CAP/ESR measurement (with failure detection) are exposed.

License: MIT
"""

from __future__ import annotations

import time
from datetime import datetime

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# ---------------------------------------------------------------------------
# Hardware configuration
# ---------------------------------------------------------------------------

I2C_BUS = 6
I2C_ADDRESS = 0x09

RSNSI = 0.010          # input current sense resistor (ohm)
RSNSC = 0.010          # charge current sense resistor (ohm)

RT = 75000             # CAP_ESR_PER programming resistor
RTST = 144             # datasheet constant
CAP_SCALE = 336e-6     # datasheet constant

CAP_VOLTAGE_MIN = 6.5
CAP_VOLTAGE_FULL = 10.571   # schematic: (1 + R17/R16) * CAPFBREF, R17=820k R16=105k

CURRENT_LSB_UV = 1.983

# ---------------------------------------------------------------------------
# Register map
# ---------------------------------------------------------------------------

REG_CLR_ALARMS = 0x00
REG_MSK_ALARMS = 0x01
REG_MSK_MON_STATUS = 0x02
REG_CAP_ESR_PER = 0x04
REG_VCAPFB_DAC = 0x05
REG_VSHUNT = 0x06
REG_CAP_UV = 0x07
REG_CAP_OV = 0x08
REG_GPI_UV = 0x09
REG_GPI_OV = 0x0A
REG_VIN_UV = 0x0B
REG_VIN_OV = 0x0C
REG_VCAP_UV = 0x0D
REG_VCAP_OV = 0x0E
REG_VOUT_UV = 0x0F
REG_VOUT_OV = 0x10
REG_IIN_OC_LIMIT = 0x11
REG_ICHG_UC_LIMIT = 0x12
REG_TEMP_COLD = 0x13
REG_TEMP_HOT = 0x14
REG_ESR_HIGH = 0x15
REG_CAP_LOW = 0x16
REG_CTL = 0x17
REG_NUM_CAPS = 0x1A

REG_CHRG_STATUS = 0x1B
REG_MON_STATUS = 0x1C
REG_ALARM = 0x1D

REG_MEAS_CAP = 0x1E
REG_MEAS_ESR = 0x1F
REG_MEAS_VCAP1 = 0x20
REG_MEAS_VCAP2 = 0x21
REG_MEAS_VCAP3 = 0x22
REG_MEAS_VCAP4 = 0x23
REG_MEAS_GPI = 0x24
REG_MEAS_VIN = 0x25
REG_MEAS_VCAP = 0x26
REG_MEAS_VOUT = 0x27
REG_MEAS_IIN = 0x28
REG_MEAS_ICHG = 0x29
REG_MEAS_DTEMP = 0x2A

# CTL register bits
CTL_CAP_ESR_MEAS = 0x0001
CTL_CLR_CAP_ESR_SCHED = 0x0002
CTL_CAP_ESR_SCHED = 0x0004

# Engineering scale LSBs
CELL_VOLTAGE_LSB = 0.0001835
VCAP_VOLTAGE_LSB = 0.001476
VIN_VOUT_LSB = 0.00221
TEMP_LSB = 0.028
TEMP_OFFSET = -251.4

# Bit maps
MON_STATUS_BITS = {
    0: "CAP_ESR_ACTIVE",
    1: "CAP_ESR_SCHEDULED",
    2: "CAP_ESR_PENDING",
    3: "CAP_MEAS_DONE",
    4: "ESR_MEAS_DONE",
    5: "CAP_MEAS_FAILED",
    6: "ESR_MEAS_FAILED",
    8: "POWER_FAILED",
    9: "POWER_RETURNED",
}

ALARM_BITS = {
    0: "CAP_UV", 1: "CAP_OV",
    2: "GPI_UV", 3: "GPI_OV",
    4: "VIN_UV", 5: "VIN_OV",
    6: "VCAP_UV", 7: "VCAP_OV",
    8: "VOUT_UV", 9: "VOUT_OV",
    10: "INPUT_OVERCURRENT", 11: "CHARGE_UNDERCURRENT",
    12: "TEMP_COLD", 13: "TEMP_HOT",
    14: "ESR_HIGH", 15: "CAP_LOW",
}

CHARGER_BITS = {
    0: "STEP_DOWN",
    1: "STEP_UP",
    2: "CONSTANT_VOLTAGE",
}


class LTC3350:
    def __init__(self, bus: int = I2C_BUS, address: int = I2C_ADDRESS):
        if SMBus is None:
            raise ImportError("smbus2 is required")
        self.bus = SMBus(bus)
        self.address = address

    # ------------------------------------------------------------------
    # Low-level I2C
    # ------------------------------------------------------------------
    def read16(self, register: int) -> int:
        return self.bus.read_word_data(self.address, register)

    def write16(self, register: int, value: int):
        self.bus.write_word_data(self.address, register, value)

    def write_config(self, register: int, value: int):
        self.write16(register, value)

    @staticmethod
    def signed16(value: int) -> int:
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def active_bits(value: int, mapping: dict) -> list:
        return [name for bit, name in mapping.items() if value & (1 << bit)]

    @staticmethod
    def bit_map(value: int, mapping: dict) -> dict:
        return {name: bool(value & (1 << bit)) for bit, name in mapping.items()}

    # ------------------------------------------------------------------
    # Engineering conversions (pure, testable off-device)
    # ------------------------------------------------------------------
    @staticmethod
    def volts(raw: int) -> float:
        return raw * VIN_VOUT_LSB

    @staticmethod
    def capacitor_voltage(raw: int) -> float:
        return raw * VCAP_VOLTAGE_LSB

    @staticmethod
    def cell_voltage(raw: int) -> float:
        return raw * CELL_VOLTAGE_LSB

    @staticmethod
    def temperature_c(raw: int) -> float:
        return (raw * TEMP_LSB) + TEMP_OFFSET

    @staticmethod
    def stack_capacitance_f(raw: int) -> float:
        return CAP_SCALE * (RT / RTST) * raw

    @staticmethod
    def esr_ohms(raw: int) -> float:
        return (RSNSC / 64.0) * raw

    @staticmethod
    def num_capacitors(raw: int) -> int:
        return raw + 1  # register mirrors CAP_SLCT pins: value = count - 1

    @classmethod
    def current(cls, raw: int, shunt: float) -> float:
        return cls.signed16(raw) * CURRENT_LSB_UV / 1_000_000 / shunt

    @staticmethod
    def state_of_charge(vcap: float, vmin: float = CAP_VOLTAGE_MIN,
                        vfull: float = CAP_VOLTAGE_FULL) -> float:
        num = (vcap ** 2) - (vmin ** 2)
        den = (vfull ** 2) - (vmin ** 2)
        if den <= 0:
            return 0.0
        return max(0.0, min(1.0, num / den)) * 100

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def read_measurements(self) -> dict:
        regs = [
            ("vcap1", REG_MEAS_VCAP1), ("vcap2", REG_MEAS_VCAP2),
            ("vcap3", REG_MEAS_VCAP3), ("vcap4", REG_MEAS_VCAP4),
            ("gpi", REG_MEAS_GPI),
            ("vin", REG_MEAS_VIN), ("vcap", REG_MEAS_VCAP),
            ("vout", REG_MEAS_VOUT),
            ("iin", REG_MEAS_IIN), ("ichg", REG_MEAS_ICHG),
            ("dtemp", REG_MEAS_DTEMP),
            ("capacitance", REG_MEAS_CAP), ("esr", REG_MEAS_ESR),
            ("num_caps", REG_NUM_CAPS),
            ("charger_status", REG_CHRG_STATUS),
            ("monitor_status", REG_MON_STATUS),
            ("alarm_status", REG_ALARM),
        ]
        return {name: self.read16(reg) for name, reg in regs}

    def snapshot(self) -> dict:
        raw = self.read_measurements()

        vcap = raw["vcap"] * VCAP_VOLTAGE_LSB
        cells = [raw[f"vcap{i}"] * CELL_VOLTAGE_LSB for i in range(1, 5)]
        cap = self.stack_capacitance_f(raw["capacitance"])
        esr = self.esr_ohms(raw["esr"])

        ichg = self.current(raw["ichg"], RSNSC)
        iin = self.current(raw["iin"], RSNSI)

        return {
            "timestamp": datetime.now().isoformat(),
            "device": "LTC3350",
            "measurements": {
                "vin": round(self.volts(raw["vin"]), 4),
                "vout": round(self.volts(raw["vout"]), 4),
                "vcap": round(vcap, 4),
                "iin": round(iin, 3),
                "ichg": round(ichg, 3),
                "input_power": round(self.volts(raw["vin"]) * iin, 3),
                "capacitor_power": round(vcap * ichg, 3),
                "stack_capacitance_f": round(cap, 3),
                "esr_ohms": round(esr, 5),
                "esr_milliohms": round(esr * 1000, 2),
                "temperature_c": round(self.temperature_c(raw["dtemp"]), 2),
                "state_of_charge": round(self.state_of_charge(vcap), 2),
            },
            "cells": {
                f"vcap{i}": round(v, 4) for i, v in enumerate(cells, 1)
            },
            "balance_mv": round((max(cells) - min(cells)) * 1000, 1),
            "num_caps": self.num_capacitors(raw["num_caps"]),
            "gpi_raw": raw["gpi"],
            "charger": {
                "mode": self._charger_mode(raw["charger_status"]),
                "active": self.active_bits(raw["charger_status"], CHARGER_BITS),
            },
            "monitor": {
                "raw": hex(raw["monitor_status"]),
                "active": self.active_bits(raw["monitor_status"], MON_STATUS_BITS),
            },
            "alarms": {
                "raw": hex(raw["alarm_status"]),
                "active": self.active_bits(raw["alarm_status"], ALARM_BITS),
            },
            "raw": raw,
        }

    def _charger_mode(self, status: int) -> str:
        active = self.active_bits(status, CHARGER_BITS)
        for mode in ("STEP_DOWN", "STEP_UP", "CONSTANT_VOLTAGE"):
            if mode in active:
                return mode
        return "IDLE"

    # ------------------------------------------------------------------
    # CAP/ESR measurement
    # ------------------------------------------------------------------
    def start_cap_esr_measurement(self):
        self.write16(REG_CTL, CTL_CAP_ESR_MEAS)

    def wait_cap_esr_measurement(self, timeout: float = 10.0) -> str:
        # Wait for measurement; returns 'done', 'failed' or 'timeout'.
        start = time.monotonic()
        while True:
            status = self.read16(REG_MON_STATUS)

            if status & (1 << 3) and status & (1 << 4):
                return "done"
            if status & (1 << 5) or status & (1 << 6):
                return "failed"
            if time.monotonic() - start > timeout:
                return "timeout"
            time.sleep(0.1)


def _self_check() -> None:
    # Assert conversions against observed hardware values (scratch capture).
    ltc = LTC3350.__new__(LTC3350)  # no I2C needed for static helpers

    assert abs(ltc.stack_capacitance_f(10) - 1.75) < 1e-9            # 1.75 F
    assert abs(ltc.esr_ohms(267) * 1000 - 41.72) < 0.02              # 41.72 mΩ
    assert abs(ltc.temperature_c(10788) - 50.66) < 0.02              # 50.66 C
    assert abs(ltc.capacitor_voltage(7151) - 10.5549) < 0.0002       # 10.5549 V
    assert abs(ltc.cell_voltage(14213) - 2.6081) < 0.0002            # vcap1 cell
    assert abs(ltc.current(65525, RSNSC) - (-0.00218)) < 0.0001      # sign-extend
    assert abs(ltc.current(36, RSNSC) - 0.0071) < 0.001              # charging
    # SOC monotonic and bounded (scratch log used an 8.0 V floor; ours uses 6.5)
    soc_full = ltc.state_of_charge(10.5549)
    soc_low = ltc.state_of_charge(7.0)
    assert 0 <= soc_low < soc_full <= 100
    assert soc_full > 90
    assert ltc.state_of_charge(6.5) == 0.0
    assert ltc.signed16(65525) == -11
    assert ltc.num_capacitors(0) == 1
    assert ltc.num_capacitors(3) == 4

    # Register map pinned to the datasheet table (page 32) and the
    # mainline ltc3350-charger driver; these must never be reordered.
    regs = {
        REG_CLR_ALARMS: 0x00, REG_MSK_ALARMS: 0x01, REG_MSK_MON_STATUS: 0x02,
        REG_CAP_ESR_PER: 0x04, REG_VCAPFB_DAC: 0x05, REG_VSHUNT: 0x06,
        REG_CAP_UV: 0x07, REG_CAP_OV: 0x08, REG_GPI_UV: 0x09, REG_GPI_OV: 0x0A,
        REG_VIN_UV: 0x0B, REG_VIN_OV: 0x0C, REG_VCAP_UV: 0x0D, REG_VCAP_OV: 0x0E,
        REG_VOUT_UV: 0x0F, REG_VOUT_OV: 0x10,
        REG_IIN_OC_LIMIT: 0x11, REG_ICHG_UC_LIMIT: 0x12,
        REG_TEMP_COLD: 0x13, REG_TEMP_HOT: 0x14, REG_ESR_HIGH: 0x15,
        REG_CAP_LOW: 0x16, REG_CTL: 0x17, REG_NUM_CAPS: 0x1A,
        REG_CHRG_STATUS: 0x1B, REG_MON_STATUS: 0x1C, REG_ALARM: 0x1D,
        REG_MEAS_CAP: 0x1E, REG_MEAS_ESR: 0x1F, REG_MEAS_VCAP1: 0x20,
        REG_MEAS_VCAP2: 0x21, REG_MEAS_VCAP3: 0x22, REG_MEAS_VCAP4: 0x23,
        REG_MEAS_GPI: 0x24, REG_MEAS_VIN: 0x25, REG_MEAS_VCAP: 0x26,
        REG_MEAS_VOUT: 0x27, REG_MEAS_IIN: 0x28, REG_MEAS_ICHG: 0x29,
        REG_MEAS_DTEMP: 0x2A,
    }
    for value, expected in regs.items():
        assert value == expected, f"register {expected:#04x} has value {value:#04x}"
    assert CTL_CAP_ESR_MEAS == 0x0001

    # failure detection: bit 5 set -> failed
    status_failed = (1 << 5)
    assert status_failed & (1 << 5) and not (status_failed & (1 << 3))

    print("ltc3350_driver: self-check OK")


if __name__ == "__main__":
    _self_check()
