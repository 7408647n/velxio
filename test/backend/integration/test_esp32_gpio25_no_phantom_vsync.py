#!/usr/bin/env python3
"""GPIO25 on a plain ESP32 must read what the host holds it at (issue #333).

The esp32-picsimlab machine carries an I2S0 camera model (OV2640 over DVP,
for the ESP32-CAM) on every ESP32 board, and its VSYNC timer used to pulse
GPIO25 LOW for 8 ms every 100 ms of guest time from power-on, forever, then
force it back HIGH. `qemu_picsimlab_set_pin` writes the same GPIO_IN bit the
host's own `set_pin` injection writes, so on a DevKit with a button, switch or
sensor on GPIO25:

  - a pad the circuit holds HIGH read a phantom LOW about one read in twelve
    (the reporter's "Conflicting inputs. Forcing ALL OFF." with no input), and
  - a pad the circuit holds LOW was forced HIGH at the end of every pulse.

The camera model now only drives VSYNC once the firmware has brought up a
camera (esp32-camera's ll_cam_config sets I2S0.conf2.camera_en).

Each case boots a probe sketch that samples GPIO25 every ~0.5 ms and prints,
once a second, how many samples read LOW; the host sets the pad once after
boot and never touches it again. A library without the fix fails both cases
(lows > 0 while held HIGH, lows < reads while held LOW).

Skipped when libqemu-xtensa or a firmware source is missing. The firmware is
compiled with the production ESP-IDF compiler, or taken from
ESP32_GPIO25_PROBE_BIN when that is set (a prebuilt image of PROBE_SKETCH).
QEMU_ESP32_LIB overrides the library path (the ROM files must sit next to it).

    python -m unittest test/backend/integration/test_esp32_gpio25_no_phantom_vsync.py -v
"""
from __future__ import annotations

import asyncio
import base64
import io
import json
import os
import re
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path
from typing import cast

BACKEND_DIR = Path(__file__).resolve().parents[3] / 'backend'
SERVICES_DIR = BACKEND_DIR / 'app' / 'services'
WORKER = SERVICES_DIR / 'esp32_worker.py'
LIB_XTENSA = Path(os.environ.get('QEMU_ESP32_LIB') or (SERVICES_DIR / 'libqemu-xtensa.so'))
PROBE_BIN = os.environ.get('ESP32_GPIO25_PROBE_BIN')

if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

PROBE_SKETCH = r"""
void setup() {
  Serial.begin(115200);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, OUTPUT);
  delay(1500);
  Serial.println("PROBE READY");
}
unsigned long n = 0, lows = 0, t0 = 0;
void loop() {
  if (!t0) t0 = millis();
  if (digitalRead(25) == LOW) lows++;
  n++;
  delayMicroseconds(500);
  if (millis() - t0 >= 1000) {
    Serial.printf("GPIO25 reads=%lu lows=%lu\n", n, lows);
    n = 0; lows = 0; t0 = millis();
  }
}
"""

REPORT_RE = re.compile(rb'GPIO25 reads=(\d+) lows=(\d+)')


def _compiler():
    try:
        from app.services.espidf_compiler import espidf_compiler  # type: ignore[import-not-found]
    except Exception:
        return None
    return espidf_compiler if espidf_compiler.available else None


def _firmware() -> bytes:
    if PROBE_BIN:
        return Path(PROBE_BIN).read_bytes()
    comp = _compiler()
    if comp is None:
        raise unittest.SkipTest('no ESP-IDF compiler and no ESP32_GPIO25_PROBE_BIN')
    result = asyncio.run(comp.compile([{'name': 'sketch.ino', 'content': PROBE_SKETCH}],
                                      'esp32:esp32:esp32'))
    if not result.get('success') or not result.get('binary_content'):
        raise AssertionError(f"probe compile failed: {result.get('error')}\n"
                             f"{(result.get('stderr') or '')[-2000:]}")
    return base64.b64decode(result['binary_content'])


def _run_probe(firmware: bytes, level: int, seconds: float = 14.0) -> list[tuple[int, int]]:
    """Boot the probe, hold GPIO25 at `level` from 'booted' on, and return
    the (reads, lows) of every report printed after PROBE READY."""
    cfg = {
        'lib_path': str(LIB_XTENSA),
        'firmware_b64': base64.b64encode(firmware).decode('ascii'),
        'machine': 'esp32-picsimlab',
    }
    proc = subprocess.Popen([sys.executable, str(WORKER)], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    p_in = cast(io.BufferedWriter, proc.stdin)
    p_out = cast(io.BufferedReader, proc.stdout)
    serial: list[bytes] = []
    booted = threading.Event()

    def send(obj: dict) -> None:
        p_in.write((json.dumps(obj) + '\n').encode())
        p_in.flush()

    def reader() -> None:
        for raw in p_out:
            try:
                evt = json.loads(raw)
            except Exception:
                continue
            t = evt.get('type')
            if t == 'uart_tx' and isinstance(evt.get('byte'), int):
                serial.append(bytes([evt['byte']]))
            elif t == 'serial_output' and isinstance(evt.get('data'), str):
                serial.append(evt['data'].encode('utf-8', errors='replace'))
            elif t == 'system' and evt.get('event') == 'booted':
                booted.set()

    send(cfg)
    threading.Thread(target=reader, daemon=True).start()
    try:
        if not booted.wait(60):
            raise AssertionError('worker never reported booted')
        # Exactly what the tab does for a wired input: one injection, then
        # nothing until the circuit changes.
        send({'cmd': 'set_pin', 'pin': 25, 'value': level})
        time.sleep(seconds)
    finally:
        try:
            send({'cmd': 'stop'})
        except Exception:
            pass
        time.sleep(0.5)
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        for stream in (p_in, p_out):
            try:
                stream.close()
            except Exception:
                pass
    text = b''.join(serial)
    ready = text.find(b'PROBE READY')
    if ready < 0:
        raise AssertionError(f'probe never started; serial tail: {text[-600:]!r}')
    return [(int(a), int(b)) for a, b in REPORT_RE.findall(text[ready:])]


@unittest.skipUnless(WORKER.exists() and LIB_XTENSA.exists(),
                     'Requires esp32_worker.py and libqemu-xtensa')
class TestGpio25NoPhantomVsync(unittest.TestCase):
    firmware: bytes

    @classmethod
    def setUpClass(cls) -> None:
        cls.firmware = _firmware()

    def test_pad_held_high_never_reads_low(self) -> None:
        reports = _run_probe(self.firmware, 1)
        self.assertGreaterEqual(len(reports), 5, f'too few reports: {reports}')
        lows = sum(l for _, l in reports)
        self.assertEqual(lows, 0, f'GPIO25 held HIGH read LOW {lows} times: {reports}')

    def test_pad_held_low_never_reads_high(self) -> None:
        reports = _run_probe(self.firmware, 0)
        self.assertGreaterEqual(len(reports), 5, f'too few reports: {reports}')
        highs = sum(n - l for n, l in reports)
        self.assertEqual(highs, 0, f'GPIO25 held LOW read HIGH {highs} times: {reports}')


if __name__ == '__main__':
    unittest.main()
