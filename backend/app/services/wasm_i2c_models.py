"""The worker's copy of an I2C part, run from the part's compiled model.

Project i2c-model-fidelity-2026-09, P5 (decision O4): the one C model the tab
runs too, in place of the hand-written twins of esp32_i2c_slaves.py, as the
microSD card already is (frontend/src/simulation/buses/models/). The two
real-time clocks are the first: buses/models/ds1307.c and ds3231.c, on the
shared rtc.h. The tab puts the model's bytes in the part's record (`wasmB64`)
unless its `i2cwasm` flag is off, and the worker builds the slave below from
them (esp32_i2c_slaves.rtc_slave); a record without them, or bytes that cannot
run, keep DS1307Slave or DS3231Slave exactly as before.

A slave here is plug-compatible with the twin it stands in for: the same
constructor (a record, and for a test a clock and the build times), the same
`handle_event`, `update` and `dump_registers`. The bus events go through
WasmChipI2CSlave, the adapter every custom chip in this worker uses, so the
model hears what a custom chip would.

What the model needs from the host that the chip ABI has no call for is
pushed into its memory, not asked for (rtc.h, "What the chip needs from a
host"): the wall clock before every bus event, the dump and the setup, the
panel's temperature when it moves. A call out of the model into Python cost
about 35 us here, and a START made two. Only the firmware's build times are
still asked for (`live_attrs`), at the moment the model needs them: when a
sketch has written a time, the moment DS1307Slave calls its `build_times`.

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
# rtc_inputs in rtc.h: host_ms at offset 0, temperature at offset 8.
_HOST_MS = 0
_TEMPERATURE = 8


class _WasmRtcSlave:
    """A clock chip at 0x68, answered by a model on buses/models/rtc.h."""

    ADDRESS = 0x68
    # Where the model's pointer wraps (the tab's pointerWrapsAfter).
    LAST_REGISTER = 0xFF

    def __init__(self, wasm_bytes: bytes, record=None, *, clock=None,
                 build_times=None) -> None:
        slaves = _sibling('esp32_i2c_slaves')
        runtime_mod = _sibling('wasm_chip_runtime')
        slave_mod = _sibling('wasm_chip_slave')
        self.addr = self.ADDRESS
        # The record says what the tab's clock is; a test hands its own.
        self.tab_clock = clock if isinstance(clock, slaves.TabClock) else slaves.TabClock()
        if isinstance(record, dict):
            self.tab_clock.set(record)
        self._clock = clock or self.tab_clock
        self._build_times = build_times or (lambda: ())
        self.runtime = runtime_mod.WasmChipRuntime(
            wasm_bytes, live_attrs=self._live_attr, share_module=True)
        self._inputs = self.runtime.call_export('chip_inputs')
        self._pushed_ms: float | None = None
        self._set_inputs(record)
        # Power-on reads the clock.
        self._push_clock()
        self.runtime.run_chip_setup()
        self._slave = slave_mod.WasmChipI2CSlave(self.ADDRESS, self.runtime)

    @classmethod
    def from_b64(cls, wasm_b64: str, record=None, *, build_times=None):
        return cls(base64.b64decode(wasm_b64), record, build_times=build_times)

    def _live_attr(self, name: str):
        if name == 'build_times':
            return _build_times_text(self._build_times)
        return None

    def _push_clock(self) -> None:
        # The tab's clock moves in whole milliseconds: most events of a
        # transaction find it where the last one left it.
        now = float(self._clock())
        if now != self._pushed_ms:
            self._pushed_ms = now
            self.runtime.write_memory(self._inputs + _HOST_MS, _F64.pack(now))

    def _set_inputs(self, record) -> None:
        """What a record says about the chip's inputs, besides the clock."""

    def handle_event(self, event: int, addr: int | None = None) -> int:
        self._push_clock()
        return self._slave.handle_event(event, addr)

    def update(self, /, **record) -> None:
        """The tab sent the part's record again, or a part of it."""
        self.tab_clock.set(record)
        self._set_inputs(record)

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now."""
        self._push_clock()
        ptr = self.runtime.call_export('chip_dump_registers')
        return bytearray(self.runtime.read_memory(ptr, 256))


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
        self.tab_clock.set(record)
        self._set_temperature(record.get('temperature', temperature))


# The worker's copy of each part that has a compiled model, by sensor type.
SLAVES = {'ds1307': WasmDS1307Slave, 'ds3231': WasmDS3231Slave}
