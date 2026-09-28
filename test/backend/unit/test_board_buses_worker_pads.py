"""
test_board_buses_worker_pads.py: a module's pull resistor reaches the guest of
the ESP32 QEMU worker (app/services/esp32_worker.py + pad_model.py).

The tab resolves a module's resistor on its board-pin nets (busNets
setBoardPinPull, board-buses.md "Pull resistors on a line") and sends the
pulls of every board pin in the bus map (`pulls`). Before the pad model the
worker had no use for them: QEMU's injection keeps the last level written, so
a line the guest released with pinMode(INPUT) read the LOW it had driven, a
chip hosted here that released its line (vx_pin_set_mode(VX_INPUT)) left its
own LOW behind, and a chip watching the line never saw it rise.

The worker runs unmodified on the repro suite's rig (libqemu replaced at the
ctypes boundary); the test plays the guest through the worker's own callbacks
(picsimlab_write_pin for the output latch, picsimlab_dir_pin for pinMode) and
reads what the worker puts into QEMU (qemu_picsimlab_set_pin, slot = GPIO + 1).
The chip is fixtures/board_buses_worker/pull-probe.c: an open-drain LINE it
watches and pulls low while the guest holds CTL high.
"""
from __future__ import annotations

import base64

import pytest

pytest.importorskip('wasmtime', reason='the custom-chip runtime needs wasmtime')

from .test_board_buses_repro_worker import FIXTURES, worker  # noqa: E402,F401

DIO, CLK, CTL, FREE = 18, 19, 21, 22


def pulls(**by_pin) -> list[dict]:
    """pulls(p18='up') -> the `pulls` half the tab sends for GPIO 18."""
    return [{'pin': int(k[1:]), 'pull': v, 'owner': f'grove-4digit::{k}~pull'}
            for k, v in by_pin.items()]


def probe_chip() -> dict:
    return {
        'sensor_type': 'custom-chip', 'pin': 400,
        'wasm_b64': base64.b64encode((FIXTURES / 'pull-probe.wasm').read_bytes()).decode(),
        'attrs': {}, 'component_id': 'probe',
        'pin_map': {'LINE': DIO, 'CTL': CTL}, 'nets': [],
    }


def set_pins(w, gpio: int) -> list[int]:
    """Every level the worker put on `gpio`'s pad, in order."""
    return [c[2] for c in w.guest('calls')['calls'] if c[0] == 'set_pin' and c[1] == gpio + 1]


def release(w, gpio: int) -> None:
    w.guest('dir', slot=gpio + 1, value=0)


def drive(w, gpio: int, level: int) -> None:
    """pinMode(OUTPUT) + digitalWrite, as esp32_gpio.c reports it: the level
    on the output line, then the direction."""
    w.pin(gpio, level)
    w.guest('dir', slot=gpio + 1, value=1)


# ── the guest lets go ─────────────────────────────────────────────────────────

def test_a_released_line_reads_the_module_pull_up(worker):
    """avishorp's TM1637Display sends a 1 by pinMode(INPUT): the Grove
    module's 10k makes it HIGH. The worker used to leave the LOW."""
    w = worker(bus_map={'pulls': pulls(p18='up', p19='up')})
    drive(w, DIO, 0)
    before = len(set_pins(w, DIO))
    release(w, DIO)
    assert set_pins(w, DIO)[before:] == [1]


def test_a_released_line_reads_the_module_pull_down(worker):
    w = worker(bus_map={'pulls': pulls(p18='down')})
    drive(w, DIO, 1)
    before = len(set_pins(w, DIO))
    release(w, DIO)
    assert set_pins(w, DIO)[before:] == [0]


def test_a_pad_never_driven_reads_the_pull_from_the_start(worker):
    """A sketch that reads a line before it ever drives it (the pad is an
    input out of reset) reads the resistor."""
    w = worker(bus_map={'pulls': pulls(p18='up')})
    assert set_pins(w, DIO) == [1]


def test_a_guest_output_low_stays_low(worker):
    w = worker(bus_map={'pulls': pulls(p18='up')})
    before = len(set_pins(w, DIO))
    drive(w, DIO, 0)
    w.pin(DIO, 0)
    assert set_pins(w, DIO)[before:] == []


def test_setup_without_a_pull_the_release_keeps_the_level(worker):
    """setup: a pad no module pulls is not the model's; the worker writes
    nothing there, as before."""
    w = worker(bus_map={'pulls': pulls(p18='up')})
    drive(w, FREE, 0)
    release(w, FREE)
    assert set_pins(w, FREE) == []


def test_the_map_can_arrive_while_the_guest_runs_and_go_away(worker):
    w = worker()
    drive(w, DIO, 0)
    release(w, DIO)
    assert set_pins(w, DIO) == [], 'no map, no pull'
    w.send({'cmd': 'bus_map', 'pulls': pulls(p18='up')})
    w.sync()
    assert set_pins(w, DIO) == [1], 'the part mounted: the released line rises'
    w.send({'cmd': 'bus_map', 'pulls': []})
    w.sync()
    drive(w, DIO, 0)
    release(w, DIO)
    assert set_pins(w, DIO) == [1], 'the part went away: nothing more'


def test_an_injection_beats_the_pull_until_the_guest_drives_the_pad(worker):
    """A button to GND on the pulled line reads LOW (the tab's rule)."""
    w = worker(bus_map={'pulls': pulls(p18='up')})
    w.send({'cmd': 'set_pin', 'pin': DIO, 'value': 0})
    w.sync()
    n = len(set_pins(w, DIO))
    release(w, DIO)
    assert set_pins(w, DIO)[n:] == []
    drive(w, DIO, 0)
    release(w, DIO)
    assert set_pins(w, DIO)[n:] == [1]


def test_a_map_with_another_half_only_keeps_the_pulls(worker):
    w = worker(bus_map={'pulls': pulls(p18='up')})
    w.send({'cmd': 'bus_map', 'i2c': []})
    w.sync()
    drive(w, DIO, 0)
    before = len(set_pins(w, DIO))
    release(w, DIO)
    assert set_pins(w, DIO)[before:] == [1]


# ── a chip hosted in the worker ──────────────────────────────────────────────

def _chip_lines(w) -> list[str]:
    return [t.strip() for t in w.chip_log() if t.startswith(('line=', 'ctl='))]


def test_setup_the_chip_pulls_its_line_low(worker):
    w = worker(sensors=[probe_chip()], bus_map={'pulls': pulls(p18='up')})
    assert 'pull probe ready' in w.chip_log()
    w.pin(CTL, 1)
    assert w.wait_for(lambda: 'ctl=1 read=0' in _chip_lines(w)), w.chip_log()
    assert set_pins(w, DIO)[-1] == 0


def test_a_chip_releasing_its_line_hands_it_to_the_pull(worker):
    """An ACK, a presence pulse: the chip pulls low, lets go, and the line
    rises through the module's resistor, in the guest and in the chip's read."""
    w = worker(sensors=[probe_chip()], bus_map={'pulls': pulls(p18='up')})
    w.pin(CTL, 1)
    w.pin(CTL, 0)
    assert w.wait_for(lambda: 'ctl=0 read=1' in _chip_lines(w)), w.chip_log()
    assert set_pins(w, DIO)[-1] == 1


def test_a_chip_watching_the_line_sees_the_guest_release_rise(worker):
    """What a TM1637 model in the worker needs: avishorp raises CLK by
    releasing it, and the watch has to fire on that rise."""
    w = worker(sensors=[probe_chip()], bus_map={'pulls': pulls(p18='up')})
    drive(w, DIO, 0)
    assert w.wait_for(lambda: 'line=0' in _chip_lines(w)), w.chip_log()
    release(w, DIO)
    assert w.wait_for(lambda: _chip_lines(w)[-1:] == ['line=1']), w.chip_log()


def test_a_chip_that_leaves_lets_go_of_its_line(worker):
    w = worker(sensors=[probe_chip()], bus_map={'pulls': pulls(p18='up')})
    w.pin(CTL, 1)
    assert set_pins(w, DIO)[-1] == 0
    w.send({'cmd': 'sensor_detach', 'pin': 400})
    w.sync()
    assert set_pins(w, DIO)[-1] == 1
