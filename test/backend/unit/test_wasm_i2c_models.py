"""
The compiled clocks in the WORKER host (project i2c-model-fidelity-2026-09,
P5, decision O4).

frontend/src/simulation/buses/models/ds1307.c and ds3231.c (on rtc.h) are one
model of each chip that the tab and the worker both run, as the microSD card
already is. This suite is the worker's half of the proof: the same .wasm the
tab loads, hosted by WasmChipRuntime and WasmChipI2CSlave (wasm_i2c_models.py),
replays test/fixtures/i2c-vectors/ds1307.json and ds3231.json in both bus
flavours, exactly as DS1307Slave and DS3231Slave do in test_i2c_slaves.py. The
tab's half is frontend/src/__tests__/rtc-vectors-wasm.test.ts.

rtc_slave builds the model when the part's record carries its bytes, which the
tab does by default, and the twin otherwise or when the bytes cannot run.
"""
from __future__ import annotations

import base64
import sys
import time
import unittest
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))

pytest.importorskip('wasmtime', reason='chip runtime needs wasmtime')

from app.services import wasm_chip_runtime  # noqa: E402
from app.services.esp32_i2c_slaves import (  # noqa: E402
    I2C_FINISH, I2C_READ, I2C_START_RECV, I2C_START_SEND, I2C_WRITE,
    DS1307Slave, DS3231Slave, rtc_slave,
)
from app.services.wasm_i2c_models import (  # noqa: E402
    SLAVES, WasmDS1307Slave, WasmDS3231Slave,
)

# The vector runner and its clock are test_i2c_slaves.py's, so the two
# models are replayed by one runner.
sys.path.insert(0, str(Path(__file__).parent))
from test_i2c_slaves import (  # noqa: E402
    BUS_FLAVOURS,
    DS1307_VECTORS,
    DS3231_VECTORS,
    VectorClock,
    build_time,
    replay_vector,
)

BUS_CHIPS = Path(__file__).resolve().parents[3] / 'frontend' / 'public' / 'bus-chips'
WASM = {name: (BUS_CHIPS / f'{name}.wasm').read_bytes() for name in ('ds1307', 'ds3231')}
CHIPS = {
    'ds1307': (WasmDS1307Slave, DS1307_VECTORS, 22),
    'ds3231': (WasmDS3231Slave, DS3231_VECTORS, 27),
}


def power_on(name: str, vector: dict):
    cls, file, _ = CHIPS[name]
    clock = VectorClock(vector.get('clock', file['clock']))
    built = [build_time(pair) for pair in vector.get('build_times', file['build_times'])]
    slave = cls(WASM[name], dict(file['inputs']), clock=clock, build_times=lambda: built)
    return slave, clock


class TestWasmRtcVectors(unittest.TestCase):
    """One test per shared vector, chip and bus flavour, as TestDS1307Slave
    and TestDS3231Slave."""

    def test_the_vectors_are_the_ones_the_twins_replay(self):
        for _cls, file, count in CHIPS.values():
            self.assertGreaterEqual(len(file['vectors']), count)


def _case(name: str, vector: dict, flavour: str):
    def case(self):
        slave, clock = power_on(name, vector)
        replay_vector(self, slave, vector, flavour, clock=clock)
    case.__doc__ = f'{name}: {vector["name"]} ({flavour})'
    return case


for _name, (_cls, _file, _count) in CHIPS.items():
    for _flavour in BUS_FLAVOURS:
        for _n, _vector in enumerate(_file['vectors'], start=1):
            setattr(TestWasmRtcVectors,
                    f'test_{_name}_vector_{_n:02d}_{_flavour.replace("-", "_")}',
                    _case(_name, _vector, _flavour))


class TestRtcSlave(unittest.TestCase):
    """The worker runs the compiled model when the record carries it, which
    the tab does unless its flag is off, and the twin otherwise."""

    RECORD = {'temperature': 21.5, 'addr': 0x68}

    def test_without_the_model_the_worker_keeps_its_twins(self):
        self.assertIs(type(rtc_slave('ds3231', dict(self.RECORD))), DS3231Slave)
        self.assertIs(type(rtc_slave('ds1307', dict(self.RECORD))), DS1307Slave)

    def test_a_record_with_the_model_runs_it(self):
        for name, cls in SLAVES.items():
            record = dict(self.RECORD, wasmB64=base64.b64encode(WASM[name]).decode())
            self.assertIsInstance(rtc_slave(name, record), cls)
        slave = rtc_slave('ds3231', dict(self.RECORD, wasmB64=base64.b64encode(WASM['ds3231']).decode()))
        self.assertEqual(slave.temperatureC, 21.5)
        # 21.5 C is 86 quarters: 0x15, 0x80.
        self.assertEqual(bytes(slave.dump_registers()[0x11:0x13]), b'\x15\x80')

    def test_the_panel_moves_the_temperature_the_model_reads(self):
        slave = WasmDS3231Slave(WASM['ds3231'], dict(self.RECORD))
        slave.update(temperature=-10.25)
        # -41 quarters: 0xF5, 0xC0.
        self.assertEqual(bytes(slave.dump_registers()[0x11:0x13]), b'\xf5\xc0')
        slave.update(temperature=float('nan'))
        self.assertEqual(slave.temperatureC, -10.25)

    def test_a_model_that_cannot_run_leaves_the_twin(self):
        for name, twin in (('ds3231', DS3231Slave), ('ds1307', DS1307Slave)):
            record = dict(self.RECORD, wasmB64=base64.b64encode(b'not wasm').decode())
            self.assertIs(type(rtc_slave(name, record)), twin)

    def test_the_pointer_wraps_where_the_tab_says(self):
        self.assertEqual(WasmDS1307Slave.LAST_REGISTER, 0x3F)
        self.assertEqual(WasmDS3231Slave.LAST_REGISTER, 0x12)


class TestWasmRtcCost(unittest.TestCase):
    """What the worker pays for a compiled clock (P5 measured 26 to 30 us an
    event and 14 to 20 ms an attach): the inputs are pushed, not called for,
    and the module is compiled once per process."""

    def test_every_attach_after_the_first_reuses_the_compiled_module(self):
        a = WasmDS3231Slave(WASM['ds3231'])
        b = WasmDS3231Slave(WASM['ds3231'])
        self.assertIs(a.runtime._module, b.runtime._module)
        c = WasmDS1307Slave(WASM['ds1307'])
        self.assertIsNot(c.runtime._module, a.runtime._module)
        # A runtime that did not ask for it still compiles its own.
        own = wasm_chip_runtime.WasmChipRuntime(WASM['ds3231'])
        self.assertIsNot(own._module, a.runtime._module)

    def test_a_start_calls_no_host_function(self):
        slave = WasmDS3231Slave(WASM['ds3231'])
        asked = []
        slave.runtime._live_attrs = lambda name: asked.append(name)
        for event in (I2C_START_SEND, (0 << 8) | I2C_WRITE, I2C_FINISH, I2C_START_RECV,
                      *[I2C_READ] * 7, I2C_FINISH):
            slave.handle_event(event)
        self.assertEqual(asked, [])

    def test_the_clock_reaches_the_model_without_a_call(self):
        now = [time.mktime((2026, 9, 30, 12, 0, 0, 0, 0, -1)) * 1000.0]
        slave = WasmDS1307Slave(WASM['ds1307'], clock=lambda: now[0])
        seconds = lambda: (slave.handle_event(I2C_START_SEND), slave.handle_event(I2C_WRITE),
                           slave.handle_event(I2C_START_RECV), slave.handle_event(I2C_READ),
                           )[-1]
        first = seconds()
        slave.handle_event(I2C_FINISH)
        now[0] += 5000
        self.assertEqual((seconds() - first) & 0xFF, 0x05)
