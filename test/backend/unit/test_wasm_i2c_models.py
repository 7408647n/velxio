"""
The compiled DS3231 in the WORKER host (project i2c-model-fidelity-2026-09,
P5, decision O4).

frontend/src/simulation/buses/models/ds3231.c is one model of the chip that
the tab and the worker can both run, as the microSD card already is. This
suite is the worker's half of the evidence: the same .wasm the tab loads,
hosted by WasmChipRuntime and WasmChipI2CSlave (wasm_i2c_models.py), replays
test/fixtures/i2c-vectors/ds3231.json in both bus flavours, exactly as
DS3231Slave does in test_i2c_slaves.py. The tab's half is
frontend/src/__tests__/rtc-vectors-wasm.test.ts.

The model is only reached behind a flag: rtc_slave builds it when the part's
record carries its bytes, and the twin otherwise.
"""
from __future__ import annotations

import base64
import sys
import unittest
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))

pytest.importorskip('wasmtime', reason='chip runtime needs wasmtime')

from app.services.esp32_i2c_slaves import DS1307Slave, DS3231Slave, rtc_slave  # noqa: E402
from app.services.wasm_i2c_models import WasmDS3231Slave  # noqa: E402

# The vector runner and its clock are test_i2c_slaves.py's, so the two
# models are replayed by one runner.
sys.path.insert(0, str(Path(__file__).parent))
from test_i2c_slaves import (  # noqa: E402
    BUS_FLAVOURS,
    DS3231_VECTORS,
    VectorClock,
    build_time,
    replay_vector,
)

WASM = (Path(__file__).resolve().parents[3] / 'frontend' / 'public' / 'bus-chips'
        / 'ds3231.wasm').read_bytes()


def power_on(vector: dict):
    clock = VectorClock(vector.get('clock', DS3231_VECTORS['clock']))
    built = [build_time(pair)
             for pair in vector.get('build_times', DS3231_VECTORS['build_times'])]
    slave = WasmDS3231Slave(WASM, dict(DS3231_VECTORS['inputs']), clock=clock,
                            build_times=lambda: built)
    return slave, clock


class TestWasmDS3231Vectors(unittest.TestCase):
    """One test per shared vector and bus flavour, as TestDS3231Slave."""

    def test_the_vectors_are_the_ones_the_twin_replays(self):
        self.assertGreaterEqual(len(DS3231_VECTORS['vectors']), 27)


def _case(vector: dict, flavour: str):
    def case(self):
        slave, clock = power_on(vector)
        replay_vector(self, slave, vector, flavour, clock=clock)
    case.__doc__ = f'{vector["name"]} ({flavour})'
    return case


for _flavour in BUS_FLAVOURS:
    for _n, _vector in enumerate(DS3231_VECTORS['vectors'], start=1):
        setattr(TestWasmDS3231Vectors,
                f'test_vector_{_n:02d}_{_flavour.replace("-", "_")}',
                _case(_vector, _flavour))


class TestRtcSlaveFlag(unittest.TestCase):
    """The worker builds the compiled model only when the record carries it."""

    RECORD = {'temperature': 21.5, 'addr': 0x68}

    def test_without_the_model_the_worker_keeps_its_twins(self):
        self.assertIs(type(rtc_slave('ds3231', dict(self.RECORD))), DS3231Slave)
        self.assertIs(type(rtc_slave('ds1307', dict(self.RECORD))), DS1307Slave)

    def test_a_record_with_the_model_runs_it(self):
        record = dict(self.RECORD, wasmB64=base64.b64encode(WASM).decode())
        slave = rtc_slave('ds3231', record)
        self.assertIsInstance(slave, WasmDS3231Slave)
        self.assertEqual(slave.temperatureC, 21.5)
        # 21.5 C is 86 quarters: 0x15, 0x80.
        self.assertEqual(bytes(slave.dump_registers()[0x11:0x13]), b'\x15\x80')

    def test_a_model_that_cannot_run_leaves_the_twin(self):
        record = dict(self.RECORD, wasmB64=base64.b64encode(b'not wasm').decode())
        self.assertIs(type(rtc_slave('ds3231', record)), DS3231Slave)
