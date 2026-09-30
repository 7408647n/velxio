"""
test_i2c_write_sink.py: the worker's copy of the display and expander parts
(project i2c-model-fidelity-2026-09, item "displays").

An ESP32 or STM32 on QEMU answers a display or an expander from a write sink
beside the guest (app/services/esp32_i2c_slaves.py I2CWriteSink), and the tab
draws from the write phases the sink echoes. Two things were wrong there:

  - the PCF8574 of an expander part or an LCD backpack read back 0xFF whatever
    was written, while the chip reads its pins: a bit written 0 reads 0, a bit
    written 1 reads what the outside drives. hd44780_I2Cexp tells the chip from
    an MCP23008 by that read (IdentifyIOexp: write 0x00, read, expect 0x00) and
    reads the backpack's wiring and backlight polarity the same way
    (autocfg8574), so on QEMU it took the backpack for an MCP23008;
  - the echo named the address only, and the tab kept one listener per
    address, so two panels at 0x3C on Wire and Wire1 drew one stream.

Two layers, as in test_board_buses_f5_worker_i2c.py: the sink driven directly,
then the ESP32 worker running unmodified on the repro suite's rig.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))

from app.services.esp32_i2c_slaves import (  # noqa: E402
    I2C_FINISH,
    I2C_READ,
    I2C_START_RECV,
    I2C_START_SEND,
    I2C_WRITE,
    I2CWriteSink,
)


def w(byte: int) -> int:
    return ((byte & 0xFF) << 8) | I2C_WRITE


def write(sink: I2CWriteSink, *data: int) -> None:
    """Wire.beginTransmission(); write(data...); endTransmission()."""
    sink.handle_event(I2C_START_SEND)
    for b in data:
        assert sink.handle_event(w(b)) == 0
    sink.handle_event(I2C_FINISH)


def read(sink: I2CWriteSink) -> int:
    """Wire.requestFrom(addr, 1); Wire.read()."""
    sink.handle_event(I2C_START_RECV)
    v = sink.handle_event(I2C_READ)
    sink.handle_event(I2C_FINISH)
    return v


def backpack(emitted: list) -> I2CWriteSink:
    """The record the tab sends for an lcd1602-i2c / lcd2004-i2c part."""
    return I2CWriteSink.from_record(
        'pcf8574', {'sensor_type': 'pcf8574', 'addr': 0x27, 'owner': 'lcd1', 'portState': 0xF7},
        emitted.append)


class TestExpanderLatch:
    def test_a_read_returns_what_was_written_where_nothing_pulls_a_pin_down(self):
        sink = I2CWriteSink.from_record('pcf8574', {'addr': 0x20}, lambda _m: None)
        assert read(sink) == 0xFF, 'power-on: every pin released and pulled up'
        write(sink, 0x5A)
        assert read(sink) == 0x5A

    def test_the_last_byte_of_a_write_phase_is_what_the_pins_hold(self):
        sink = I2CWriteSink.from_record('pcf8574', {'addr': 0x20}, lambda _m: None)
        write(sink, 0x00, 0xFF, 0x3C)
        assert read(sink) == 0x3C

    def test_a_released_pin_reads_what_the_outside_drives(self):
        sink = I2CWriteSink.from_record('pcf8574', {'addr': 0x20, 'portState': 0x0F},
                                        lambda _m: None)
        write(sink, 0xFF)
        assert read(sink) == 0x0F
        write(sink, 0xF3)
        assert read(sink) == 0x03

    def test_hd44780_identifies_the_backpack_as_a_pcf8574(self):
        """IdentifyIOexp(): write [0x00, 0xFF], write [0x00], read. An MCP23008
        reads back the IODIR it was given (0xFF), a PCF8574 its port (0x00)."""
        sink = backpack([])
        write(sink, 0x00, 0xFF)
        write(sink, 0x00)
        assert read(sink) == 0x00

    def test_hd44780_autoconfig_finds_the_common_wiring_with_an_active_high_backlight(self):
        """autocfg8574(): write 0xFF and read, then clear E (bit 2) and read. The
        control bits float high, P3 is held low by the transistor's base, and
        the data nibble reads high once E is low: rs=0 rw=1 en=2 d4-d7=4-7,
        bl=3 active HIGH (hd44780_I2Cexp.h)."""
        sink = backpack([])
        write(sink, 0xFF)
        data = read(sink)
        write(sink, 0xFB)
        data2 = read(sink)
        assert (data & 0x07) == 0x07 and (data2 & 0xF0) == 0xF0
        assert not data & 0x08, 'bit 3 reads high: the library picks an active-low backlight'

    def test_the_echo_carries_every_byte_and_names_the_part(self):
        emitted: list = []
        sink = backpack(emitted)
        write(sink, 0x08, 0x0C, 0x08)
        assert emitted == [{'type': 'i2c_transaction', 'addr': 0x27, 'owner': 'lcd1',
                            'data': [0x08, 0x0C, 0x08]}]

    def test_a_read_is_not_echoed(self):
        emitted: list = []
        sink = backpack(emitted)
        read(sink)
        assert emitted == []


class TestPanelSink:
    def test_a_display_still_reads_0xff_whatever_was_written(self):
        sink = I2CWriteSink.from_record('ssd1306', {'addr': 0x3C, 'owner': 'oled'},
                                        lambda _m: None)
        write(sink, 0x00, 0xAE)
        assert read(sink) == 0xFF

    def test_the_default_addresses_follow_the_type(self):
        assert I2CWriteSink.from_record('ssd1306', {}, lambda _m: None).addr == 0x3C
        assert I2CWriteSink.from_record('pcf8574', {}, lambda _m: None).addr == 0x27
        assert I2CWriteSink.from_record('i2c-write-sink', {'addr': 0x3E}, lambda _m: None).addr == 0x3E

    def test_a_record_without_an_owner_echoes_as_before(self):
        emitted: list = []
        sink = I2CWriteSink(0x3C, emitted.append)
        write(sink, 0x40, 0xFF)
        assert emitted == [{'type': 'i2c_transaction', 'addr': 0x3C, 'data': [0x40, 0xFF]}]


# ── The ESP32 worker ─────────────────────────────────────────────────────────

# Same rig and the same skip as test_board_buses_f5_worker_i2c.py: only the
# tests that boot the worker are skipped where wasmtime is missing.
try:
    from .test_board_buses_repro_worker import worker  # noqa: E402,F401  (fixture)
except pytest.skip.Exception as _no_rig:
    _RIG_MISSING = str(_no_rig)

    @pytest.fixture
    def worker():
        pytest.skip(_RIG_MISSING)


def oled(pin: int, bus: int, owner: str) -> dict:
    return {'sensor_type': 'ssd1306', 'pin': pin, 'addr': 0x3C, 'owner': owner, 'bus': bus}


class TestWorker:
    def test_two_panels_at_one_address_echo_under_their_own_names(self, worker):
        wk = worker(sensors=[oled(400, 0, 'oledA'), oled(401, 1, 'oledB')])
        wk.i2c(1, 0x3C, [I2C_START_SEND, w(0x40), w(0xB1), I2C_FINISH])
        wk.i2c(0, 0x3C, [I2C_START_SEND, w(0x40), w(0xA0), I2C_FINISH])
        assert wk.wait_for(lambda: len(wk.events('i2c_transaction')) == 2)
        got = [(e['owner'], e['data']) for e in wk.events('i2c_transaction')]
        assert got == [('oledB', [0x40, 0xB1]), ('oledA', [0x40, 0xA0])]

    def test_an_lcd_backpack_reads_back_its_port(self, worker):
        rec = {'sensor_type': 'pcf8574', 'pin': 402, 'addr': 0x27, 'owner': 'lcd1',
               'portState': 0xF7}
        wk = worker(sensors=[rec])
        ret = wk.i2c(0, 0x27, [I2C_START_SEND, w(0x00), I2C_FINISH,
                               I2C_START_RECV, I2C_READ, I2C_FINISH,
                               I2C_START_SEND, w(0xFF), I2C_FINISH,
                               I2C_START_RECV, I2C_READ, I2C_FINISH])
        assert (ret[4], ret[10]) == (0x00, 0xF7)

    def test_an_expander_attached_while_running_is_a_latch_too(self, worker):
        wk = worker()
        wk.send({'cmd': 'sensor_attach', 'sensor_type': 'pcf8574', 'pin': 403,
                 'addr': 0x20, 'owner': 'io1'})
        wk.sync()
        ret = wk.i2c(0, 0x20, [I2C_START_SEND, w(0xA5), I2C_FINISH,
                               I2C_START_RECV, I2C_READ, I2C_FINISH])
        assert ret[4] == 0xA5
