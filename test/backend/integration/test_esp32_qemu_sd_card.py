"""
test_esp32_qemu_sd_card.py: a microSD card on the classic ESP32's VSPI, under
the REAL libqemu-xtensa, with the bus map the tab really sends.

The 2026-09-27 velxio.dev regression (project/board-buses-2026-09, evidence
matrix-2026-09-27-postdeploy.md): after board-buses shipped, SD.begin() failed
on the QEMU engine with `sdCommand(): Card Failed! cmd: 0x00`. The unit rig
(test_board_buses_*_worker.py) replaces libqemu at ctypes and could not see
any of the three causes, because each one lives in what the real machine
reports:

  1. QEMU names the controller of an SPI event by its host shim's attach
     order (HSPI 0, VSPI 1); the tab's map names it by the SoC's unit
     (VSPI 3). Nothing ever matched.
  2. Every write-only batch arrives with a literal id 0, whichever controller
     clocked it.
  3. GPIO 5 is VSPI's CS0 in the pin table, so the map says `hw`, but SD.begin
     drives it with digitalWrite; the peripheral's own CS0-CS2 toggle around
     every byte on no pad at all. Following them cut every SD command into
     one-byte frames, and relaying them sent the tab six frames a byte.

So this test drives esp32_worker.py exactly as esp32_lib_manager does (config
on stdin, events on stdout) against the shared library the container loads,
with a real arduino-esp32 SD sketch and the card image the tab builds.

Inputs (fixtures/esp32-qemu-sd/):
  sd-only.ino        the bus-matrix sketch (harness/scenarios/sd-only.ino)
  sd-only.ino.bin    built by the production compile service (esp32:esp32:esp32,
                     project/board-buses-2026-09/harness/compile-fixture.mjs)
  card.img.gz        buildFat16Image([{name: 'data.txt', ...}]) from
                     frontend/src/utils/fatImage.ts, the tab's own builder

Needs libqemu-xtensa.so with its ROMs beside it (esp32-v3-rom*.bin): the path
in VELXIO_LIBQEMU_XTENSA, else backend/app/services/, else /app/lib (inside
the image). Skipped when none is there. A candidate worker can be measured
with BOARD_BUSES_ESP32_WORKER=/path/to/esp32_worker.py.
"""
from __future__ import annotations

import base64
import gzip
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

pytest.importorskip('wasmtime', reason='the portable SD model runs in wasmtime')

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
BACKEND = ROOT / 'backend'
FIXTURES = HERE / 'fixtures' / 'esp32-qemu-sd'
WORKER = Path(os.environ.get('BOARD_BUSES_ESP32_WORKER')
              or BACKEND / 'app' / 'services' / 'esp32_worker.py')
SD_WASM = ROOT / 'frontend' / 'public' / 'bus-chips' / 'microsd.wasm'


def _find_lib() -> Path | None:
    for cand in (os.environ.get('VELXIO_LIBQEMU_XTENSA'),
                 str(BACKEND / 'app' / 'services' / 'libqemu-xtensa.so'),
                 '/app/lib/libqemu-xtensa.so'):
        if cand and Path(cand).is_file() and (Path(cand).parent / 'esp32-v3-rom.bin').is_file():
            return Path(cand)
    return None


LIB = _find_lib()
pytestmark = pytest.mark.skipif(LIB is None, reason='libqemu-xtensa.so with its ROMs not found')

# What the tab sent in start_esp32 on velxio.dev (probe-ws.json), card and
# model aside: VSPI by its SoC unit, the select as VSPI's hardware CS0 on GPIO 5.
VSPI, HSPI = 3, 2
TAB_CS = {'kind': 'hw', 'index': 0, 'gpio': 5, 'active_low': True}


def _sd_entry(bus_id: int, cs: dict) -> dict:
    card = gzip.decompress((FIXTURES / 'card.img.gz').read_bytes())
    return {
        'owner': 'sd', 'bus_id': bus_id, 'cs': cs,
        'model': {
            'wasm_b64': base64.b64encode(SD_WASM.read_bytes()).decode('ascii'),
            'pin_map': {'SCK': 18, 'DI': 23, 'DO': 19, 'CS': 5},
            'attrs': {},
            'blobs': {'card': base64.b64encode(card).decode('ascii')},
            'blob_ids': {'card': 'fixture-1'},
        },
    }


class Run:
    """esp32_worker.py on the real libqemu until `until` shows up on UART0 or
    `secs` pass. Keeps UART0 and a count of every event type."""

    def __init__(self, entry: dict, until: bytes, secs: float) -> None:
        cfg = {
            'lib_path': str(LIB),
            'firmware_b64': base64.b64encode((FIXTURES / 'sd-only.ino.bin').read_bytes()).decode(),
            'machine': 'esp32-picsimlab', 'sensors': [], 'wifi_enabled': False,
            'wifi_hostfwd_port': 0,
            'bus_map': {'spi': [entry, {'sinks': {'all': False, 'cs': []}}], 'i2c': [], 'uart': []},
        }
        self.serial = bytearray()
        self.counts: dict[str, int] = {}
        self.stderr: list[str] = []
        env = {**os.environ, 'PYTHONPATH': str(BACKEND), 'PYTHONDONTWRITEBYTECODE': '1'}
        proc = subprocess.Popen([sys.executable, str(WORKER)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        threading.Thread(target=self._out, args=(proc,), daemon=True).start()
        threading.Thread(target=self._err, args=(proc,), daemon=True).start()
        proc.stdin.write((json.dumps(cfg) + '\n').encode())
        proc.stdin.flush()
        deadline = time.monotonic() + secs
        while time.monotonic() < deadline and until not in self.serial:
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
            t = ev.get('type', '')
            self.counts[t] = self.counts.get(t, 0) + 1
            if t == 'uart_tx' and ev.get('uart', 0) == 0:
                self.serial.append(ev['byte'] & 0xFF)

    def _err(self, proc) -> None:
        for raw in proc.stderr:
            self.stderr.append(raw.decode(errors='replace'))

    @property
    def text(self) -> str:
        return self.serial.decode(errors='replace')

    def why(self) -> str:
        return (f'serial:\n{self.text[-1500:]}\nevents: {self.counts}\n'
                f'stderr:\n{"".join(self.stderr[-20:])}')


def test_the_card_mounts_with_the_map_the_tab_sends():
    """The velxio.dev cell e-esp32-sd/qemu/L1: the loop reads the card back."""
    run = Run(_sd_entry(VSPI, TAB_CS), until=b'SD PASS', secs=90)
    assert 'SD PASS begin=1' in run.text, run.why()
    assert 'Card Failed' not in run.text, run.why()


def test_the_peripherals_own_selects_stay_in_the_worker():
    """SD.begin drives GPIO 5 as a GPIO, so no pad carries VSPI's CS0-CS2 and
    the tab must not hear them: they were six WebSocket frames per byte, which
    the tab read as its CS pad moving (it re-selected its copy of the card and
    dumped the whole image for each one) until it stopped reading the socket."""
    run = Run(_sd_entry(VSPI, TAB_CS), until=b'SD PASS', secs=90)
    assert 'SD PASS begin=1' in run.text, run.why()
    assert run.counts.get('spi_event', 0) == 0, run.why()


def test_a_card_on_hspi_does_not_answer_vspi():
    """The negative control that keeps the translation honest: the same card
    placed on the other controller must NOT answer what the sketch clocks on
    VSPI. A worker that ignored the controller would pass the test above and
    this one would catch it."""
    run = Run(_sd_entry(HSPI, TAB_CS), until=b'SD FAIL', secs=90)
    assert 'SD FAIL begin=0' in run.text, run.why()
