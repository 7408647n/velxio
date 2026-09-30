"""
The compiled I2C models in the WORKER host (project i2c-model-fidelity-2026-09,
P5, decision O4).

frontend/src/simulation/buses/models/ds1307.c, ds3231.c (on rtc.h) and
bmp280.c are one model of each chip that the tab and the worker both run, as
the microSD card already is. This suite is the worker's half of the proof: the
same .wasm the tab loads, hosted by WasmChipRuntime through the entries of
buses/models/i2c_host.h (wasm_i2c_models.py), replays
test/fixtures/i2c-vectors/ds1307.json, ds3231.json and bmp280.json in both bus
flavours, exactly as DS1307Slave, DS3231Slave and BMP280Slave do in
test_i2c_slaves.py, and on each of the three ways the worker can reach a model
(events held back and delivered with a peek, one call per event with reads
from a peek, and byte by byte). The tab's half is
frontend/src/__tests__/rtc-vectors-wasm.test.ts and bmp280-vectors-wasm.test.ts.

rtc_slave and bmp280_slave build the model when the part's record carries its
bytes, which the tab does by default, and the twin otherwise or when the bytes
cannot run.
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
    BMP280Slave, DS1307Slave, DS3231Slave, bmp280_slave, rtc_slave,
)
from app.services.wasm_i2c_models import (  # noqa: E402
    SLAVES, WasmBMP280Slave, WasmDS1307Slave, WasmDS3231Slave,
)

# The vector runner and its clock are test_i2c_slaves.py's, so the two
# models are replayed by one runner.
sys.path.insert(0, str(Path(__file__).parent))
from test_i2c_slaves import (  # noqa: E402
    BMP_VECTORS,
    BUS_FLAVOURS,
    DS1307_VECTORS,
    DS3231_VECTORS,
    VectorClock,
    build_time,
    replay_vector,
)

BUS_CHIPS = Path(__file__).resolve().parents[3] / 'frontend' / 'public' / 'bus-chips'
WASM = {name: (BUS_CHIPS / f'{name}.wasm').read_bytes()
        for name in ('ds1307', 'ds3231', 'bmp280')}
CHIPS = {
    'ds1307': (WasmDS1307Slave, DS1307_VECTORS, 22),
    'ds3231': (WasmDS3231Slave, DS3231_VECTORS, 27),
    'bmp280': (WasmBMP280Slave, BMP_VECTORS, 22),
}
RTCS = ('ds1307', 'ds3231')

# The ways the worker reaches a model (_WasmI2cModel): the events held back
# and delivered with the peek of the next read (chip_i2c_run), one call per
# event with the reads from a peek, and one call per event and byte.
PATHS = ('held', 'per-event', 'byte-by-byte')


def on_path(slave, path: str):
    if path in ('per-event', 'byte-by-byte'):
        slave._run = None
    if path == 'byte-by-byte':
        slave._peek = None
    return slave


def power_on(name: str, vector: dict, path: str = 'held'):
    cls, file, _ = CHIPS[name]
    if name == 'bmp280':
        slave = cls(WASM[name], int(file['address'], 16))
        slave.update(**file['inputs'])
        return on_path(slave, path), None
    clock = VectorClock(vector.get('clock', file['clock']))
    built = [build_time(pair) for pair in vector.get('build_times', file['build_times'])]
    slave = cls(WASM[name], dict(file['inputs']), clock=clock, build_times=lambda: built)
    return on_path(slave, path), clock


class TestWasmModelVectors(unittest.TestCase):
    """One test per shared vector, chip, bus flavour and path, as
    TestDS1307Slave, TestDS3231Slave and TestBMP280Slave."""

    def test_the_vectors_are_the_ones_the_twins_replay(self):
        for _cls, file, count in CHIPS.values():
            self.assertGreaterEqual(len(file['vectors']), count)


def _case(name: str, vector: dict, flavour: str, path: str):
    def case(self):
        slave, clock = power_on(name, vector, path)
        replay_vector(self, slave, vector, flavour, clock=clock)
    case.__doc__ = f'{name}: {vector["name"]} ({flavour}, {path})'
    return case


for _name, (_cls, _file, _count) in CHIPS.items():
    for _flavour in BUS_FLAVOURS:
        for _path in PATHS:
            for _n, _vector in enumerate(_file['vectors'], start=1):
                setattr(TestWasmModelVectors,
                        f'test_{_name}_vector_{_n:02d}_{_flavour.replace("-", "_")}'
                        f'_{_path.replace("-", "_")}',
                        _case(_name, _vector, _flavour, _path))


class TestRtcSlave(unittest.TestCase):
    """The worker runs the compiled model when the record carries it, which
    the tab does unless its flag is off, and the twin otherwise."""

    RECORD = {'temperature': 21.5, 'addr': 0x68}

    def test_without_the_model_the_worker_keeps_its_twins(self):
        self.assertIs(type(rtc_slave('ds3231', dict(self.RECORD))), DS3231Slave)
        self.assertIs(type(rtc_slave('ds1307', dict(self.RECORD))), DS1307Slave)

    def test_a_record_with_the_model_runs_it(self):
        for name in RTCS:
            record = dict(self.RECORD, wasmB64=base64.b64encode(WASM[name]).decode())
            self.assertIsInstance(rtc_slave(name, record), SLAVES[name])
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


class TestBmp280Slave(unittest.TestCase):
    """bmp280_slave: the compiled model when the record carries it, the twin
    otherwise, both at the panel's values."""

    RECORD = {'temperature': 31.5, 'pressure': 990.0, 'addr': 0x76}

    @staticmethod
    def sample(slave) -> bytes:
        for event in (I2C_START_SEND, (0xF4 << 8) | I2C_WRITE, (0x27 << 8) | I2C_WRITE,
                      I2C_FINISH, I2C_START_SEND, (0xF7 << 8) | I2C_WRITE, I2C_START_RECV):
            slave.handle_event(event)
        out = bytes(slave.handle_event(I2C_READ) for _ in range(6))
        slave.handle_event(I2C_FINISH)
        return out

    def test_without_the_model_the_worker_keeps_its_twin(self):
        slave = bmp280_slave(dict(self.RECORD))
        self.assertIs(type(slave), BMP280Slave)
        self.assertEqual(slave.inputs(), {'temperature': 31.5, 'pressure': 990.0})

    def test_a_record_with_the_model_runs_it_at_the_panels_values(self):
        record = dict(self.RECORD, wasmB64=base64.b64encode(WASM['bmp280']).decode())
        slave = bmp280_slave(record)
        self.assertIsInstance(slave, WasmBMP280Slave)
        self.assertEqual(self.sample(slave), self.sample(bmp280_slave(dict(self.RECORD))))

    def test_bytes_that_cannot_run_leave_the_twin(self):
        record = dict(self.RECORD, wasmB64=base64.b64encode(b'not wasm').decode())
        self.assertIs(type(bmp280_slave(record)), BMP280Slave)

    def test_the_address_is_the_records(self):
        for addr in (0x76, 0x77):
            record = dict(self.RECORD, addr=addr,
                          wasmB64=base64.b64encode(WASM['bmp280']).decode())
            slave = bmp280_slave(record)
            self.assertEqual(slave.addr, addr)
            self.assertEqual(slave.runtime.i2c_address, addr)

    def test_the_panel_takes_what_the_twin_takes(self):
        wasm, twin = WasmBMP280Slave(WASM['bmp280']), BMP280Slave()
        for values in ({'pressure': 1005.0}, {'temperature': 'nan'}, {'temperature': True},
                       {'temperature_c': '12.5'}, {'pressure_hpa': float('inf')}):
            wasm.update(**values)
            twin.update(**values)
            self.assertEqual(wasm.inputs(), twin.inputs(), values)
            self.assertEqual(self.sample(wasm), self.sample(twin), values)


def _events(slave, events) -> list[int]:
    return [slave.handle_event(event) for event in events]


def _now_events(n: int = 7) -> list[int]:
    """RTClib now() as QEMU's ESP32 delivers it: pointer, FINISH, START, n bytes."""
    return [I2C_START_SEND, I2C_WRITE, I2C_FINISH, I2C_START_RECV, *[I2C_READ] * n, I2C_FINISH]


class TestTheWorkersCallPath(unittest.TestCase):
    """What a bus event costs the worker is the number of calls into the
    model (P5: 26 to 30 us an event through the generic chip path, 14 us
    after step 1). A model that holds back its events (I2C_HOST_DEFERRABLE)
    is called once per read transaction, whatever its length."""

    def counted(self, slave):
        calls = []
        for name in ('_event', '_run', '_peek', '_commit'):
            fn = getattr(slave, name, None)
            if fn is not None:
                setattr(slave, name, (lambda f, n: lambda *a: (calls.append(n), f(*a))[1])(fn, name))
        return calls

    def test_a_register_read_is_one_call_into_the_model(self):
        for name in RTCS:
            # A clock that stands still: one that moves is pushed, and the
            # events held before it are delivered first.
            slave = SLAVES[name](WASM[name], clock=lambda: 1_000_000_000_000)
            _events(slave, _now_events())
            calls = self.counted(slave)
            for _ in range(3):
                _events(slave, _now_events())
            # The STOP of the last one is held until something asks.
            self.assertEqual(calls, ['_run'] * 3, name)

    def test_a_fourteen_byte_burst_is_one_call_too(self):
        slave = WasmDS1307Slave(WASM['ds1307'], clock=lambda: 1_000_000_000_000)
        want = _events(on_path(WasmDS1307Slave(WASM['ds1307'], clock=lambda: 1_000_000_000_000),
                               'byte-by-byte'), _now_events(14))
        calls = self.counted(slave)
        self.assertEqual(_events(slave, _now_events(14)), want)
        # The first peek asks for 16, and every one after it for 14.
        self.assertEqual(calls, ['_run'])
        calls.clear()
        _events(slave, _now_events(14))
        self.assertEqual(calls, ['_run'])

    def test_a_burst_longer_than_the_peek_peeks_again(self):
        slave = on_path(WasmBMP280Slave(WASM['bmp280']), 'held')
        twin = on_path(WasmBMP280Slave(WASM['bmp280']), 'byte-by-byte')
        # The calibration and on, 40 bytes: past the first peek of 16.
        events = [I2C_START_SEND, (0x88 << 8) | I2C_WRITE, I2C_START_RECV,
                  *[I2C_READ] * 40, I2C_FINISH]
        self.assertEqual(_events(slave, events), _events(twin, events))
        self.assertEqual(slave.dump_registers(), twin.dump_registers())

    def test_a_clock_that_moves_inside_a_burst_is_seen_as_it_was_byte_by_byte(self):
        """The host delivers what it held and pushes the new clock before
        the next byte, as it pushed before every event byte by byte."""
        results = []
        for path in ('held', 'byte-by-byte'):
            now = [time.mktime((2026, 9, 30, 23, 59, 59, 0, 0, -1)) * 1000.0]
            slave = on_path(WasmDS1307Slave(WASM['ds1307'], clock=lambda: now[0]), path)
            got = _events(slave, [I2C_START_SEND, I2C_WRITE, I2C_FINISH, I2C_START_RECV,
                                  I2C_READ, I2C_READ])
            now[0] += 1000
            got += _events(slave, [I2C_READ] * 62 + [I2C_FINISH])
            got += list(slave.dump_registers()[:8])
            results.append(got)
        self.assertEqual(results[0], results[1])

    def test_a_peek_that_calls_the_host_is_thrown_away(self):
        """A DS1307 read with no START after a time was written runs the
        write's commit in on_read, which asks the host for the build times:
        the peek is not a read-ahead then. The host is asked once, for real,
        and the bytes are the ones byte by byte gives."""
        results, asked = [], []
        events = [I2C_START_SEND, I2C_WRITE, (0x30 << 8) | I2C_WRITE, I2C_READ, I2C_READ,
                  I2C_FINISH]
        for path in ('held', 'byte-by-byte'):
            names: list = []
            slave = on_path(WasmDS1307Slave(WASM['ds1307'], clock=lambda: 1_000_000_000_000),
                            path)
            live = slave.runtime._live_attrs
            slave.runtime._live_attrs = lambda name, live=live: (names.append(name), live(name))[1]
            results.append(_events(slave, events) + list(slave.dump_registers()[:8]))
            asked.append(names)
        self.assertEqual(results[0], results[1])
        self.assertEqual(asked[0], ['build_times'])
        self.assertEqual(asked[0], asked[1])

    def test_a_model_from_an_older_tab_goes_the_generic_way(self):
        """Bytes with none of the i2c_host.h entries: the chip's callbacks."""
        slave = WasmDS1307Slave(WASM['ds1307'])
        exports = dict(slave.runtime._exports)
        for name in ('chip_i2c_event', 'chip_i2c_run', 'chip_i2c_peek'):
            exports.pop(name, None)
        slave.runtime._exports = exports
        slave._start = WasmDS1307Slave._start.__get__(slave)
        slave.runtime.run_chip_setup = lambda: None
        slave._start()
        self.assertIsNone(slave._run)
        self.assertIsNone(slave._peek)
        want = _events(on_path(WasmDS1307Slave(WASM['ds1307']), 'byte-by-byte'), _now_events())
        self.assertEqual(_events(slave, _now_events())[4:11], want[4:11])
