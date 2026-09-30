"""The worker's copy of an I2C part, run from the part's compiled model.

Project i2c-model-fidelity-2026-09, P5 (decision O4): the one C model the tab
runs too, in place of the hand-written twins of esp32_i2c_slaves.py, as the
microSD card already is (frontend/src/simulation/buses/models/): the two
real-time clocks (ds1307.c and ds3231.c, on the shared rtc.h), the BMP280
(bmp280.c) and the MPU-6050 (mpu6050.c). The tab puts the model's bytes in
the part's record (`wasmB64`) unless its `i2cwasm` flag is off, and the worker
builds the slave below from them (esp32_i2c_slaves.rtc_slave, bmp280_slave,
mpu6050_slave); a record without them, or bytes that cannot run, keep the
Python twin exactly as before.

A slave here is plug-compatible with the twin it stands in for: the same
constructor, the same `handle_event`, `update` and `dump_registers`. The bus
events reach the model through the entries of buses/models/i2c_host.h, on the
cheapest call wasmtime-py offers (_WasmI2cModel): a register read, pointer and
bytes, is one call into the model, where it was one call per event through
WasmChipI2CSlave, the adapter every custom chip uses.

What the model needs from the host that the chip ABI has no call for is
pushed into its memory, not asked for (rtc.h, bmp280.c and mpu6050.c, "What
the chip needs from a host"): the wall clock before every bus event, the dump
and the setup, the panel's values when they move, and the MPU-6050's guest
clock before every call and with every event held back. A call out of the
model into Python cost about 35 us here. Only the firmware's build times are still asked for
(`live_attrs`), at the moment a clock needs them: when a sketch has written a
time, the moment DS1307Slave calls its `build_times`.

The compiled module is shared by every slave built from the same bytes
(WasmChipRuntime `share_module`), so a part that attaches again on every Run
pays the compile once per worker process.
"""
from __future__ import annotations

import base64
import importlib
import importlib.util
import math
import pathlib
import struct
import sys
import threading


def _sibling(name: str):
    """A module of this package, also when the worker runs as a script from
    a directory with no `app.services` on the path (the workers' fallback)."""
    try:
        return importlib.import_module(f'app.services.{name}')
    except ImportError:
        pass
    mod = sys.modules.get(name)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(
        name, pathlib.Path(__file__).parent / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    # wasm_chip_slave imports the runtime by its package name.
    sys.modules[f'app.services.{name}'] = mod
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


def _build_times_text(build_times) -> str:
    """The build times as the model reads them: "YYYYMMDDhhmmss" each."""
    return ' '.join('%04d%02d%02d%02d%02d%02d' % tuple(int(n) for n in built[:6])
                    for built in build_times())


_F64 = struct.Struct('<d')
_U32 = struct.Struct('<I')
# rtc_inputs in rtc.h: host_ms at offset 0, temperature at offset 8.
_HOST_MS = 0
_TEMPERATURE = 8

_I2C_READ = 0x06
_I2C_START_RECV = 0x00
_I2C_START_SEND = 0x01
# How many bytes a read-ahead asks for before the model has shown how long
# its reads are: an RTClib now() is 7, an MPU-6050 sample 14.
_FIRST_PEEK = 16
_MAX_PEEK = 64
# i2c_host.h, I2C_HOST_EVENTS_MAX.
_MAX_HELD = 64
_NACKED = 0x10000


class _WasmI2cModel:
    """An I2C part answered by a compiled model of buses/models/, through the
    entries of buses/models/i2c_host.h.

    Each entry is called through the runtime's cheapest call
    (WasmChipRuntime.export_caller), one foreign call where the generic chip
    path (WasmChipI2CSlave, call_i2c_callback, the indirect table) cost
    several. A model that promises it (I2C_HOST_DEFERRABLE) has its START,
    writes and STOP answered here and held back, and a read delivers them,
    commits the bytes taken from the last peek and peeks the next burst in
    one call (chip_i2c_run): a register read, pointer and bytes, is one call
    into the model. The held events go in before an input that moved is
    pushed, so the model sees every event with the inputs it had when it
    happened. A model without the promise gets one call per event and its
    reads from a peek; bytes from an older build of the tab, with none of
    these entries, go the generic path.

    A peek that called the host is thrown away by the model (_PeekGuard in
    wasm_chip_runtime.py) and the bytes of that transaction are asked one by
    one.

    A subclass pushes its inputs (_push_inputs, before every event) and says
    when one moved (_inputs_moved).
    """

    ADDRESS = 0x68

    def __init__(self, wasm_bytes: bytes, *, live_attrs=None) -> None:
        runtime_mod = _sibling('wasm_chip_runtime')
        self.runtime = runtime_mod.WasmChipRuntime(
            wasm_bytes, live_attrs=live_attrs, share_module=True)
        self._inputs = self.runtime.call_export('chip_inputs')
        self._lock = threading.RLock()
        self._ahead = b''
        self._taken = 0
        # Events answered here and not yet delivered, as chip_i2c_run takes them.
        self._held: list[int] = []
        # Bytes read since the last START, and how many the next peek asks for.
        self._burst = 0
        self._peek_n = _FIRST_PEEK
        # A peek of this transaction called the host: bytes one at a time.
        self._no_peek = False
        self._nack_said = False
        # A model that keeps the guest's time (I2C_HOST_TIMED in i2c_host.h):
        # the time of each read served from the peek and of each event held,
        # handed to the model with them, and until when the peek holds.
        self._timed = False
        self._read_times: list[float] = []
        self._held_times: list[float] = []
        self._until = 0.0

    def _start(self) -> None:
        """chip_setup, then the entries the bus goes through."""
        rt = self.runtime
        rt.run_chip_setup()
        exports = rt._exports
        self._peek = self._run = None
        if exports.get('chip_i2c_event') is None:
            slave_mod = _sibling('wasm_chip_slave')
            generic = slave_mod.WasmChipI2CSlave(self.ADDRESS, rt)
            self._event = lambda event, addr: generic.handle_event(event, addr)
            return
        self._event = rt.export_caller('chip_i2c_event')
        if not rt.watch_peeks(rt.call_export('chip_i2c_guard')):
            return
        self._peek = rt.export_caller('chip_i2c_peek')
        self._commit = rt.export_caller('chip_i2c_commit')
        self._buffer = rt.call_export('chip_i2c_buffer')
        if exports.get('chip_i2c_run') is not None:
            self._run = rt.export_caller('chip_i2c_run')
            self._events_at = rt.call_export('chip_i2c_events')
        if exports.get('chip_i2c_times') is not None:
            self._timed = True
            self._times_at = rt.call_export('chip_i2c_times')
            self._until_at = rt.call_export('chip_i2c_until')

    # Inputs: what a subclass pushes into the model's memory.
    def _push_inputs(self) -> None:
        pass

    def _inputs_moved(self) -> bool:
        return False

    # Time: what a model that keeps the guest's time is at (NaN: no clock).
    def _now(self) -> float:
        return math.nan

    def _deliver(self, peek: int = 0) -> int:
        """Everything the model has not heard yet: the bytes taken from the
        last peek, then the held events. With `peek`, the next burst is
        peeked in the same call; returns how many bytes it holds."""
        taken, held = self._taken, self._held
        self._ahead = b''
        self._taken = 0
        if self._timed:
            # The reads the commit replays and the held events, each at the
            # instant it happened, and the time of now for the peek.
            times = self._read_times[:taken] + self._held_times
            self._read_times = []
            self._held_times = []
            if times:
                self.runtime.write_memory(
                    self._times_at, struct.pack(f'<{len(times)}d', *times))
            self._push_inputs()
        if self._run is None:
            if taken:
                self._commit(taken)
            return self._peek(peek) if peek else 0
        if held:
            self._held = []
            self.runtime.write_memory(
                self._events_at, struct.pack(f'<{len(held)}I', *held))
        elif not taken and not peek:
            return 0
        answer = self._run(taken, len(held), peek)
        if answer & _NACKED and not self._nack_said:
            self._nack_said = True
            print(f'{type(self).__name__}: the model did not ACK a byte the worker '
                  'had ACKed for it', file=sys.stderr)
        return answer & 0xFFFF

    def handle_event(self, event: int, addr: int | None = None) -> int:
        with self._lock:
            op = event & 0xFF
            if op == _I2C_READ and self._peek is not None:
                i = self._taken
                if i < len(self._ahead) and not self._inputs_moved():
                    if not self._timed:
                        self._taken = i + 1
                        self._burst += 1
                        return self._ahead[i]
                    # A byte of the peek holds until the next sample is due.
                    now = self._now()
                    if now < self._until or (now != now and self._until == math.inf):
                        self._read_times.append(now)
                        self._taken = i + 1
                        self._burst += 1
                        return self._ahead[i]
                return self._read(event, self.addr if addr is None else int(addr))
            at = self.addr if addr is None else int(addr)
            if op == _I2C_START_SEND or op == _I2C_START_RECV:
                if self._burst:
                    self._peek_n = min(self._burst, _MAX_PEEK)
                    self._burst = 0
                self._no_peek = False
            if self._run is not None:
                # Answered here: a START and a STOP are ACKed by the chip
                # being there, and the model promised to ACK every byte. The
                # bytes taken from the peek are committed ahead of the held
                # events (chip_i2c_run); what is left of it is stale.
                if self._inputs_moved() or len(self._held) >= _MAX_HELD:
                    self._deliver()
                    self._push_inputs()
                elif self._taken < len(self._ahead):
                    self._ahead = self._ahead[:self._taken]
                self._held.append((event & 0xFFFF) | (at << 16))
                if self._timed:
                    self._held_times.append(self._now())
                return 0
            if self._peek is not None:
                self._deliver()
            self._push_inputs()
            return self._event(event, at)

    def _read(self, event: int, at: int) -> int:
        """A read the last peek does not answer."""
        self._burst += 1
        if self._inputs_moved():
            self._deliver()
            self._push_inputs()
        # A burst that ran past its peek goes on: the rest of it in one peek.
        want = self._peek_n if self._burst == 1 else max(self._peek_n, _FIRST_PEEK)
        n = 0 if self._no_peek else self._deliver(want)
        if n:
            self._ahead = self.runtime.read_memory(self._buffer, n)
            self._taken = 1
            if self._timed:
                # The peek was taken at the time _deliver pushed.
                self._read_times = [self._pushed_ns]
                self._until = _F64.unpack(self.runtime.read_memory(self._until_at, 8))[0]
            return self._ahead[0]
        # A peek that called the host: this transaction byte by byte.
        self._no_peek = True
        self._deliver()
        return self._event(event, at)

    def _export_now(self, name: str) -> int:
        """An export called with everything delivered and the inputs pushed."""
        with self._lock:
            self._deliver()
            self._push_inputs()
            return self.runtime.call_export(name)

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now."""
        with self._lock:
            ptr = self._export_now('chip_dump_registers')
            return bytearray(self.runtime.read_memory(ptr, 256))


class _WasmRtcSlave(_WasmI2cModel):
    """A clock chip at 0x68, answered by a model on buses/models/rtc.h."""

    ADDRESS = 0x68
    # Where the model's pointer wraps (the tab's pointerWrapsAfter).
    LAST_REGISTER = 0xFF

    def __init__(self, wasm_bytes: bytes, record=None, *, clock=None,
                 build_times=None) -> None:
        slaves = _sibling('esp32_i2c_slaves')
        self.addr = self.ADDRESS
        # The record says what the tab's clock is; a test hands its own.
        self.tab_clock = clock if isinstance(clock, slaves.TabClock) else slaves.TabClock()
        if isinstance(record, dict):
            self.tab_clock.set(record)
        self._clock = clock or self.tab_clock
        self._build_times = build_times or (lambda: ())
        super().__init__(wasm_bytes, live_attrs=self._live_attr)
        self._pushed_ms: float | None = None
        self._set_inputs(record)
        # Power-on reads the clock.
        self._push_inputs()
        self._start()

    @classmethod
    def from_b64(cls, wasm_b64: str, record=None, *, build_times=None):
        return cls(base64.b64decode(wasm_b64), record, build_times=build_times)

    def _live_attr(self, name: str):
        if name == 'build_times':
            return _build_times_text(self._build_times)
        return None

    def _push_inputs(self) -> None:
        # The tab's clock moves in whole milliseconds: most events of a
        # transaction find it where the last one left it.
        now = float(self._clock())
        if now != self._pushed_ms:
            self._pushed_ms = now
            self.runtime.write_memory(self._inputs + _HOST_MS, _F64.pack(now))

    def _inputs_moved(self) -> bool:
        return float(self._clock()) != self._pushed_ms

    def _set_inputs(self, record) -> None:
        """What a record says about the chip's inputs, besides the clock."""

    def update(self, /, **record) -> None:
        """The tab sent the part's record again, or a part of it."""
        with self._lock:
            self._deliver()
            self.tab_clock.set(record)
            self._set_inputs(record)


class WasmDS1307Slave(_WasmRtcSlave):
    """DS1307 at 0x68, answered by buses/models/ds1307.c. Replays
    test/fixtures/i2c-vectors/ds1307.json as DS1307Slave does."""

    LAST_REGISTER = 0x3F


class WasmDS3231Slave(_WasmRtcSlave):
    """DS3231 at 0x68, answered by buses/models/ds3231.c. Replays
    test/fixtures/i2c-vectors/ds3231.json as DS3231Slave does."""

    LAST_REGISTER = 0x12

    def __init__(self, wasm_bytes: bytes, record=None, *, clock=None,
                 build_times=None) -> None:
        self.temperatureC = 25.0
        super().__init__(wasm_bytes, record, clock=clock, build_times=build_times)

    def _set_inputs(self, record) -> None:
        if isinstance(record, dict):
            self._set_temperature(record.get('temperature'))

    def _set_temperature(self, value) -> None:
        # As DS3231Slave: a value that is not a finite number changes nothing.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return
        try:
            celsius = float(value)
        except OverflowError:
            return
        if not math.isfinite(celsius):
            return
        self.temperatureC = celsius
        self.runtime.write_memory(self._inputs + _TEMPERATURE, _F64.pack(celsius))

    def update(self, temperature=None, /, **record) -> None:
        """The panel moved, or the tab sent the record again (DS3231Slave.update)."""
        with self._lock:
            self._deliver()
            self.tab_clock.set(record)
            self._set_temperature(record.get('temperature', temperature))


# bmp280_io in bmp280.c.
_BMP_TEMPERATURE = 0
_BMP_PRESSURE = 8
_BMP_ADDRESS = 16


class WasmBMP280Slave(_WasmI2cModel):
    """BMP280 at 0x76 or 0x77, answered by buses/models/bmp280.c. Replays
    test/fixtures/i2c-vectors/bmp280.json as BMP280Slave does, and takes the
    panel the way it does: update() with the record's names, a value that is
    not a finite number left out (esp32_i2c_slaves._bmp_number)."""

    def __init__(self, wasm_bytes: bytes, addr: int = 0x76) -> None:
        slaves = _sibling('esp32_i2c_slaves')
        self._number = slaves._bmp_number
        self.addr = 0x77 if int(addr) == 0x77 else 0x76
        self.ADDRESS = self.addr
        super().__init__(wasm_bytes)
        self._temp_c = float(slaves.BMP280_INPUTS['temperature'])
        self._press_hpa = float(slaves.BMP280_INPUTS['pressure'])
        self.runtime.write_memory(self._inputs + _BMP_ADDRESS, _U32.pack(self.addr))
        self._write_inputs()
        self._start()

    @classmethod
    def from_b64(cls, wasm_b64: str, record=None):
        record = record if isinstance(record, dict) else {}
        slave = cls(base64.b64decode(wasm_b64), int(record.get('addr', 0x76)))
        slave.update(**record)
        return slave

    def _write_inputs(self) -> None:
        self.runtime.write_memory(self._inputs + _BMP_TEMPERATURE,
                                  _F64.pack(self._temp_c) + _F64.pack(self._press_hpa))

    def update(self, temperature_c=None, pressure_hpa=None, /, **inputs) -> None:
        """The panel moved (BMP280Slave.update): only the values it names."""
        named = {'temperature': temperature_c, 'pressure': pressure_hpa}
        for name, legacy in (('temperature', 'temperature_c'), ('pressure', 'pressure_hpa')):
            for key in (legacy, name):
                if key in inputs:
                    named[name] = inputs[key]
        temp_c = self._number(named['temperature'])
        press = self._number(named['pressure'])
        with self._lock:
            self._deliver()
            if temp_c is not None:
                self._temp_c = temp_c
            if press is not None:
                self._press_hpa = press
            self._write_inputs()

    def inputs(self) -> dict:
        return {'temperature': self._temp_c, 'pressure': self._press_hpa}


# mpu_io in mpu6050.c.
_MPU_NOW = 0
_MPU_INPUTS = 8
_MPU_NOT_BEFORE = 64
_MPU_WAKE = 72
_MPU_ADDRESS = 80
_MPU_VARIANT = 84
_MPU_PULSES = 100
_MPU_INT_STATE = 104
_U32X2 = struct.Struct('<2I')
_MPU_INPUT_ORDER = ('accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'temp')
_MPU_PADS = ('low', 'high', 'z')
_NAN = _F64.pack(math.nan)


class WasmMPU6050Slave(_WasmI2cModel):
    """MPU-6050 (or the MPU-9250 die) at 0x68 or 0x69, answered by
    buses/models/mpu6050.c. Replays test/fixtures/i2c-vectors/mpu6050.json as
    MPU6050Slave does, and stands in for it in the worker: the same
    constructor, handle_event, update, inputs, dump_registers, board_reset,
    and the INT pad the ESP32 worker drives (int_pad, int_pad_levels,
    int_pulses, int_wake_ns, on_int_change).

    The guest's clock (`now_ns`, QEMU_CLOCK_VIRTUAL; None on a libqemu that
    does not export it) is pushed into the model before every call, and the
    time of every event held back goes with it (I2C_HOST_TIMED), so the model
    takes each sample at its own instant. A burst is served from one peek
    until the next sample is due.

    With a pad to drive (`on_int_change` set, the INT pin wired), every event
    goes to the model at once, as the twin handles it, and the pad is looked
    at after each: holding back a read of a latched INT_STATUS would hold the
    pad active past it."""

    def __init__(self, wasm_bytes: bytes, addr: int = 0x68, now_ns=None,
                 variant: str = 'mpu6050') -> None:
        slaves = _sibling('esp32_i2c_slaves')
        self._slaves = slaves
        self.addr = int(addr) & 0x7F
        self.ADDRESS = self.addr
        self._now_ns = now_ns
        self._variant = slaves.parse_variant(variant)
        self._pushed_ns = math.nan
        self._on_int_change = None
        # What the pad does, as the model last said, and when it said it: the
        # events heard until then and the guest's time (_int_state).
        self._events_heard = 0
        self._int_seen = None
        self._fast = None
        super().__init__(wasm_bytes)
        self._inputs_values = dict(slaves.MPU6050_INPUTS)
        rt = self.runtime
        rt.write_memory(self._inputs + _MPU_ADDRESS, _U32.pack(self.addr))
        rt.write_memory(self._inputs + _MPU_VARIANT,
                        _U32.pack(1 if self._variant == 'mpu9250' else 0))
        self._write_inputs()
        # The clock is not asked while the chip powers on asleep: a worker
        # builds its copy before QEMU is initialised (MPU6050Slave does the
        # same). The MPU-9250 powers on awake and reads it, as its twin does.
        if self._variant == 'mpu9250':
            self._push_inputs()
        else:
            rt.write_memory(self._inputs + _MPU_NOW, _NAN)
        self._start()
        self._int_state_call = rt.export_caller('chip_int_state')
        self._event_int_call = rt.export_caller('chip_i2c_event_int')
        self._int_wake_call = rt.export_caller('chip_int_wake')
        self._sync_call = rt.export_caller('chip_sync')
        self._restart_call = rt.export_caller('chip_restart_sampling')
        self._pointer_call = rt.export_caller('chip_pointer')

    @classmethod
    def from_b64(cls, wasm_b64: str, record=None, *, now_ns=None):
        """The part's model from its worker record, at the address the tab
        resolved and already at the panel's values (as the twin is after
        update(**record))."""
        record = record if isinstance(record, dict) else {}
        slaves = _sibling('esp32_i2c_slaves')
        slave = cls(base64.b64decode(wasm_b64), slaves.mpu6050_address(record),
                    now_ns=now_ns, variant=record.get('variant'))
        slave.update(**record)
        return slave

    @staticmethod
    def address_of(record: dict) -> int:
        return _sibling('esp32_i2c_slaves').mpu6050_address(record)

    # ── Time ────────────────────────────────────────────────────────────────
    def _now(self) -> float:
        clock = self._now_ns
        return math.nan if clock is None else float(clock())

    def _push_inputs(self) -> None:
        now = self._now()
        # Most events of a transaction find the guest where the last left it.
        if now != self._pushed_ns:
            self._pushed_ns = now
            self.runtime.write_memory(self._inputs + _MPU_NOW, _F64.pack(now))

    def _write_inputs(self) -> None:
        values = self._inputs_values
        self.runtime.write_memory(
            self._inputs + _MPU_INPUTS,
            struct.pack('<7d', *(float(values[k]) for k in _MPU_INPUT_ORDER)))

    # ── The INT pad ─────────────────────────────────────────────────────────
    @property
    def on_int_change(self):
        return self._on_int_change

    @on_int_change.setter
    def on_int_change(self, cb) -> None:
        with self._lock:
            self._deliver()
            if cb is not None and self._fast is None:
                # Every event now, in order, and the pad looked at after it:
                # one call does both (chip_i2c_event_int).
                self._fast = (self._run, self._peek, self._event)
                self._run = self._peek = None
                self._event = self._event_then_pad
            elif cb is None and self._fast is not None:
                self._run, self._peek, self._event = self._fast
                self._fast = None
            self._on_int_change = cb

    def _event_then_pad(self, event: int, addr: int) -> int:
        # A STOP takes no sample, as the twin never looks at the chip on one:
        # samples due at a STOP are taken at the next START or read.
        if (event & 0xFF) == 0x03:
            return self._fast[2](event, addr)
        answer = self._event_int_call(event, addr)
        pulses, state = _U32X2.unpack(self.runtime.read_memory(self._inputs + _MPU_PULSES, 8))
        self._int_seen = (self._events_heard, self._pushed_ns, self._int_answer(state, pulses))
        return answer

    @staticmethod
    def _int_answer(state: int, pulses: int) -> tuple:
        return (_MPU_PADS[state & 0x0F], _MPU_PADS[(state >> 4) & 0x0F],
                _MPU_PADS[state >> 8], pulses)

    def handle_event(self, event: int, addr: int | None = None) -> int:
        self._events_heard += 1
        result = super().handle_event(event, addr)
        cb = self._on_int_change
        if cb is not None and (event & 0xFF) != 0x03:
            cb()
        return result

    def _call_now(self, call, *args) -> int:
        """An entry called with everything delivered and the time pushed."""
        with self._lock:
            self._deliver()
            self._push_inputs()
            return call(*args)

    def _int_state(self) -> tuple:
        """(pad, active, idle, pulses) at this instant. The pad driver asks
        all three after every event (esp32_worker._MpuIntPin.refresh): one
        call into the model answers them, and the next two questions, at the
        same guest time with no event in between, are answered from it."""
        with self._lock:
            now = self._now()
            seen = self._int_seen
            if (seen is not None and seen[0] == self._events_heard and seen[1] == now):
                return seen[2]
            state = self._call_now(self._int_state_call)
            pulses = _U32.unpack(self.runtime.read_memory(self._inputs + _MPU_PULSES, 4))[0]
            answer = self._int_answer(state, pulses)
            self._int_seen = (self._events_heard, now, answer)
            return answer

    def int_pad(self) -> str:
        """What the INT pad does at this instant (MPU6050Slave.int_pad)."""
        return self._int_state()[0]

    def int_pad_levels(self) -> tuple:
        """The pad active and idle under the INT_PIN_CFG of now."""
        return self._int_state()[1:3]

    def int_pulses(self) -> int:
        """How many 50 us INT pulses the chip has started, ever."""
        return self._int_state()[3]

    def int_wake_ns(self, not_before_ns=None):
        """The guest time the pad moves next, or None (MPU6050Slave.int_wake_ns)."""
        with self._lock:
            self.runtime.write_memory(
                self._inputs + _MPU_NOT_BEFORE,
                _NAN if not_before_ns is None else _F64.pack(float(not_before_ns)))
            if not self._call_now(self._int_wake_call):
                return None
            return _F64.unpack(self.runtime.read_memory(self._inputs + _MPU_WAKE, 8))[0]

    # ── The rest of the twin's interface ────────────────────────────────────
    @property
    def reg_ptr(self) -> int:
        """Where the register pointer is (the worker's I2C trace)."""
        return self._call_now(self._pointer_call)

    def board_reset(self) -> None:
        """The MCU was reset: the periods are counted from where the guest's
        clock stands now (MPU6050Slave.board_reset)."""
        with self._lock:
            self._events_heard += 1
            self._call_now(self._restart_call)

    def update(self, /, **inputs) -> None:
        """The panel moved: only the values it names, and what is not a
        finite number is left out (MPU6050Slave.update). The samples due
        until now are taken of the world as it was."""
        with self._lock:
            self._events_heard += 1
            self._call_now(self._sync_call)
            changed = dict(self._inputs_values)
            names = self._slaves._MPU_INPUT_NAMES
            for name, value in inputs.items():
                key = names.get(name)
                if key is None or isinstance(value, bool) or not isinstance(value, (int, float)):
                    continue
                try:
                    number = float(value)
                except OverflowError:
                    continue
                if math.isfinite(number):
                    changed[key] = number
            self._inputs_values = changed
            self._write_inputs()

    def inputs(self) -> dict:
        return dict(self._inputs_values)


# The worker's copy of each part that has a compiled model, by sensor type.
SLAVES = {'ds1307': WasmDS1307Slave, 'ds3231': WasmDS3231Slave, 'bmp280': WasmBMP280Slave,
          'mpu6050': WasmMPU6050Slave}
