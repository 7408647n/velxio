"""
test_esp32_qemu_pads.py: a module's pull resistor on the classic ESP32 under
the REAL libqemu-xtensa, with the `pulls` half of the bus map the tab sends.

QEMU's pin injection writes GPIO_IN and nothing else, and the guest's own
output writes it too, so a pad the sketch releases with pinMode(INPUT) kept
the last level it drove: a line with a module's pull-up on it read LOW after
every release. avishorp's TM1637Display sends each 1 that way, and so do
OneWire and SoftwareWire with pull-ups off. The worker's pad model
(app/services/pad_model.py) now puts the resistor on a pad nothing strong
drives, inside the GPIO_ENABLE write that released it, so the sketch's next
digitalRead sees it.

Inputs (fixtures/esp32-qemu-pads/):
  pad-probe.ino      releases GPIO 18 (module pull-up) and GPIO 19 (module
                     pull-down) and prints what it reads
  pad-probe.ino.bin  built by the production compile service (esp32:esp32:esp32,
                     project/board-buses-2026-09/harness/compile-fixture.mjs)
  pad-probe-s3.ino.bin  the same sketch for esp32:esp32:esp32s3, run on the
                     esp32s3-picsimlab machine

Needs libqemu-xtensa.so with its ROMs beside it: VELXIO_LIBQEMU_XTENSA, else
backend/app/services/, else /app/lib. Skipped when none is there. A candidate
worker can be measured with BOARD_BUSES_ESP32_WORKER=/path/to/esp32_worker.py.
"""
from __future__ import annotations

import base64
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
BACKEND = ROOT / 'backend'
FIXTURES = HERE / 'fixtures' / 'esp32-qemu-pads'
WORKER = Path(os.environ.get('BOARD_BUSES_ESP32_WORKER')
              or BACKEND / 'app' / 'services' / 'esp32_worker.py')


def _find_lib() -> Path | None:
    for cand in (os.environ.get('VELXIO_LIBQEMU_XTENSA'),
                 str(BACKEND / 'app' / 'services' / 'libqemu-xtensa.so'),
                 '/app/lib/libqemu-xtensa.so'):
        if cand and Path(cand).is_file() and (Path(cand).parent / 'esp32-v3-rom.bin').is_file():
            return Path(cand)
    return None


LIB = _find_lib()
pytestmark = pytest.mark.skipif(LIB is None, reason='libqemu-xtensa.so with its ROMs not found')

# The Grove 4-Digit Display's 10k pull-ups, as busNets.boardPinPulls lists them.
PULL_UP_18 = {'pin': 18, 'pull': 'up', 'owner': 'grove::DIO~pull'}
PULL_DOWN_19 = {'pin': 19, 'pull': 'down', 'owner': 'mod::OUT~pull'}
BOARDS = {
    'esp32': ('pad-probe.ino.bin', 'esp32-picsimlab'),
    'esp32-s3': ('pad-probe-s3.ino.bin', 'esp32s3-picsimlab'),
}
LINE = re.compile(r'PADS out_low=(\d) released_up=(\d) out_high=(\d) released_down=(\d) '
                  r'still_low=(\d) module_beats_pad=(\d)')


class Run:
    """esp32_worker.py on the real libqemu until the probe printed `lines`
    lines or `secs` pass."""

    def __init__(self, board: str, pulls: list | None, lines: int = 3, secs: float = 90) -> None:
        firmware, machine = BOARDS[board]
        bus_map: dict = {'spi': [], 'i2c': [], 'uart': []}
        if pulls is not None:
            bus_map['pulls'] = pulls
        cfg = {
            'lib_path': str(LIB),
            'firmware_b64': base64.b64encode((FIXTURES / firmware).read_bytes()).decode(),
            'machine': machine, 'sensors': [], 'wifi_enabled': False,
            'wifi_hostfwd_port': 0, 'bus_map': bus_map,
        }
        self.serial = bytearray()
        self.stderr: list[str] = []
        env = {**os.environ, 'PYTHONPATH': str(BACKEND), 'PYTHONDONTWRITEBYTECODE': '1'}
        proc = subprocess.Popen([sys.executable, str(WORKER)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        threading.Thread(target=self._out, args=(proc,), daemon=True).start()
        threading.Thread(target=self._err, args=(proc,), daemon=True).start()
        proc.stdin.write((json.dumps(cfg) + '\n').encode())
        proc.stdin.flush()
        deadline = time.monotonic() + secs
        while time.monotonic() < deadline and len(self.reads) < lines:
            time.sleep(0.2)
        try:
            proc.stdin.write(b'{"cmd": "stop"}\n')
            proc.stdin.flush()
            proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            proc.kill()
            proc.wait(timeout=5)

    def _out(self, proc) -> None:
        for raw in proc.stdout:
            i = raw.find(b'{"type":')
            if i < 0:
                continue
            try:
                ev = json.loads(raw[i:])
            except ValueError:
                continue
            if ev.get('type') == 'uart_tx' and ev.get('uart', 0) == 0:
                self.serial.append(ev['byte'] & 0xFF)

    def _err(self, proc) -> None:
        for raw in proc.stderr:
            self.stderr.append(raw.decode(errors='replace'))

    @property
    def reads(self) -> list[dict]:
        keys = ('out_low', 'released_up', 'out_high', 'released_down', 'still_low', 'module_beats_pad')
        return [dict(zip(keys, map(int, m.groups())))
                for m in LINE.finditer(self.serial.decode(errors='replace'))]

    def why(self) -> str:
        return (f'serial:\n{self.serial.decode(errors="replace")[-1200:]}\n'
                f'stderr:\n{"".join(self.stderr[-20:])}')


@pytest.fixture(scope='module', params=sorted(BOARDS))
def pulled(request) -> Run:
    return Run(request.param, [PULL_UP_18, PULL_DOWN_19])


def test_a_released_line_reads_the_module_pull_up(pulled):
    reads = pulled.reads
    assert len(reads) >= 3, pulled.why()
    assert all(r['released_up'] == 1 for r in reads), reads


def test_a_released_line_reads_the_module_pull_down(pulled):
    reads = pulled.reads
    assert len(reads) >= 3, pulled.why()
    assert all(r['released_down'] == 0 for r in reads), reads


def test_the_guest_output_low_still_reads_low_under_the_pull_up(pulled):
    reads = pulled.reads
    assert len(reads) >= 3, pulled.why()
    assert all(r['out_low'] == 0 and r['still_low'] == 0 and r['out_high'] == 1 for r in reads), reads


def test_the_module_pull_up_beats_the_pads_own_pull_down(pulled):
    reads = pulled.reads
    assert len(reads) >= 3, pulled.why()
    assert all(r['module_beats_pad'] == 1 for r in reads), reads


@pytest.mark.parametrize('board', sorted(BOARDS))
def test_setup_without_the_pulls_a_released_line_keeps_what_the_guest_drove(board):
    """setup: the same sketch with no `pulls` in the map reads what QEMU
    alone gives it, the level the guest last drove. This is the behaviour
    every run had before the pad model, and it is what makes the four tests
    above measure the model and not the firmware."""
    run = Run(board, None)
    reads = run.reads
    assert len(reads) >= 3, run.why()
    assert all(r['released_up'] == 0 and r['released_down'] == 1 for r in reads), reads
