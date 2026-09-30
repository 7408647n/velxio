"""The worker's copy of an I2C part, run from the part's compiled model.

Project i2c-model-fidelity-2026-09, P5 (decision O4, open with the owner):
the evidence for replacing the hand-written twins of esp32_i2c_slaves.py with
the one C model the tab runs too, as the microSD card already is
(frontend/src/simulation/buses/models/). Only the DS3231 exists so far
(buses/models/ds3231.c), and only behind a flag: the tab puts the model's
bytes in the part's record (`wasmB64`) when `?i2cwasm=ds3231` is set, and
without them the worker builds DS3231Slave exactly as before.

A slave here is plug-compatible with the twin it stands in for: the same
constructor (a record, and for a test a clock and the build times), the same
`handle_event`, `update` and `dump_registers`. The bus events go through
WasmChipI2CSlave, the adapter every custom chip in this worker uses, so the
model hears what a custom chip would.

What the model needs from the host that the chip ABI has no call for (the
wall clock it counts, the panel's temperature, the firmware's build times)
it reads as attributes the runtime answers live (`live_attrs`), at the
moment the model asks: the same instants at which DS3231Slave calls its
clock and its `build_times`.
"""
from __future__ import annotations

import base64
import importlib
import importlib.util
import math
import pathlib
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


class WasmDS3231Slave:
    """DS3231 at 0x68, answered by buses/models/ds3231.c. Replays
    test/fixtures/i2c-vectors/ds3231.json as DS3231Slave does."""

    ADDRESS = 0x68

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
        self.temperatureC = 25.0
        if isinstance(record, dict):
            self._set_temperature(record.get('temperature'))
        self.runtime = runtime_mod.WasmChipRuntime(wasm_bytes, live_attrs=self._live_attr)
        self.runtime.run_chip_setup()
        self._slave = slave_mod.WasmChipI2CSlave(self.ADDRESS, self.runtime)

    @classmethod
    def from_b64(cls, wasm_b64: str, record=None, *, build_times=None) -> 'WasmDS3231Slave':
        return cls(base64.b64decode(wasm_b64), record, build_times=build_times)

    def _live_attr(self, name: str):
        if name == 'host_ms':
            return float(self._clock())
        if name == 'temperature':
            return self.temperatureC
        if name == 'build_times':
            return _build_times_text(self._build_times)
        return None

    def _set_temperature(self, value) -> None:
        # As DS3231Slave: a value that is not a finite number changes nothing.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return
        if math.isfinite(float(value)):
            self.temperatureC = float(value)

    def handle_event(self, event: int, addr: int | None = None) -> int:
        return self._slave.handle_event(event, addr)

    def update(self, temperature=None, /, **record) -> None:
        """The panel moved, or the tab sent the record again (DS3231Slave.update)."""
        self.tab_clock.set(record)
        self._set_temperature(record.get('temperature', temperature))

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now."""
        ptr = self.runtime.call_export('chip_dump_registers')
        return bytearray(self.runtime.read_memory(ptr, 256))
