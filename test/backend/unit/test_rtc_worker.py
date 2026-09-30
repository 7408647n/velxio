"""
test_rtc_worker.py: the DS1307 and the DS3231 a QEMU worker hosts (project
i2c-model-fidelity-2026-09).

The models themselves are held to the shared bus vectors in
test_i2c_slaves.py. This file is about what only the worker does with them,
on the rig of the board-buses suite (app/services/esp32_worker.py running
unmodified, libqemu replaced at the ctypes boundary, the guest played by
calling the worker's own callbacks):

  - the worker's copy shows the tab's time and not the container's. Its
    record carries the tab's epoch and the offset of its time zone; the copy
    used to read the clock of the container, in UTC;
  - it knows when the firmware it runs was built, from the image the worker
    was handed, so a sketch that sets the clock to __DATE__ and __TIME__
    stays on the tab's time and one that sets a date of its own keeps it;
  - the DS3231 starts from the temperature of its record, and a slider that
    moves changes it.

Every case runs twice: on the record the tab files by default since P5, which
carries the part's compiled model (`wasmB64`, buses/models/ds1307.c and
ds3231.c) that the worker then runs, and on a record without it (the tab's
`?i2cwasm=off`), for which the worker keeps its Python twin.

The worker's clock cannot be replaced from here. Every record below puts the
tab's clock years away from the machine's, at the start of a minute, so what
a test reads in the minute after the worker started is known: minutes, hours,
day of week, date, month and year. The seconds are never compared.
"""
from __future__ import annotations

import base64
import calendar
import time
from pathlib import Path

import pytest

# The rig loads custom chips, so its module skips itself where wasmtime is
# missing; that skip is handed to the fixture, as the F5 suite does.
try:
    from .test_board_buses_repro_worker import (  # noqa: F401  (worker fixture)
        I2C_FINISH,
        I2C_READ,
        I2C_START_RECV,
        I2C_START_SEND,
        I2C_WRITE,
        worker,
    )
except pytest.skip.Exception as _no_rig:
    _RIG_MISSING = str(_no_rig)
    I2C_START_RECV, I2C_START_SEND, I2C_FINISH, I2C_WRITE, I2C_READ = 0x00, 0x01, 0x03, 0x05, 0x06

    @pytest.fixture
    def worker():
        pytest.skip(_RIG_MISSING)

ADDR = 0x68
PIN = 300


def epoch_ms(stamp: str) -> int:
    return calendar.timegm(time.strptime(stamp, '%Y-%m-%dT%H:%M:%S')) * 1000


# The tab's clock: 04:05:00 UTC of Saturday 3 February 2001, in a zone five
# and a half hours east. On its wall it is 09:35.
TAB = {'epochMs': epoch_ms('2001-02-03T04:05:00'), 'utcOffsetMin': 330}
TAB_WALL = [0x35, 0x09, 0x06, 0x03, 0x02, 0x01]        # minutes to year

# A firmware image with the two strings of a sketch built on 29 September
# 2026 at 23:39:41, as the linker leaves them.
BUILT = b'\xe9' * 64 + b'23:39:41\x00Sep 29 2026\x00' + b'\xff' * 64
BUILD_TIME = [0x41, 0x39, 0x23, 0x02, 0x29, 0x09, 0x26]
OWN_TIME = [0x00, 0x30, 0x12, 0x06, 0x19, 0x01, 0x13]   # 19 January 2013, 12:30:00


BUS_CHIPS = Path(__file__).resolve().parents[3] / 'frontend' / 'public' / 'bus-chips'
_MODEL = {'now': 'compiled'}


@pytest.fixture(autouse=True, params=['compiled', 'twin'])
def model(request):
    """What the record carries: the compiled model, or nothing (the twin)."""
    _MODEL['now'] = request.param
    yield request.param
    _MODEL['now'] = 'compiled'


def record(kind: str, **values) -> dict:
    """The record the tab's part files for the worker (parts/i2cPart.ts)."""
    rec = {'sensor_type': kind, 'pin': PIN, 'addr': ADDR, 'owner': 'rtc1', **values}
    if _MODEL['now'] == 'compiled':
        rec['wasmB64'] = base64.b64encode((BUS_CHIPS / f'{kind}.wasm').read_bytes()).decode()
    return rec


def write_reg(w, reg: int, *values: int) -> None:
    ret = w.i2c(0, ADDR, [I2C_START_SEND, I2C_WRITE | (reg << 8),
                          *[I2C_WRITE | (v << 8) for v in values], I2C_FINISH])
    assert ret[:-1] == [0] * (len(ret) - 1), 'the chip acknowledges its address and every byte'


def read_regs(w, reg: int, n: int) -> list[int]:
    ret = w.i2c(0, ADDR, [I2C_START_SEND, I2C_WRITE | (reg << 8), I2C_FINISH,
                          I2C_START_RECV, *[I2C_READ] * n, I2C_FINISH])
    return ret[4:4 + n]


@pytest.mark.parametrize('kind', ['ds1307', 'ds3231'])
class TestTheTabsClock:
    def test_the_start_config_record_sets_the_wall(self, worker, kind):
        w = worker(sensors=[record(kind, **TAB)])
        assert read_regs(w, 0x01, 6) == TAB_WALL

    def test_a_record_attached_while_running_sets_the_wall(self, worker, kind):
        w = worker()
        w.send({'cmd': 'sensor_attach', **record(kind, **TAB)})
        w.sync()
        assert read_regs(w, 0x01, 6) == TAB_WALL

    def test_an_update_moves_the_wall(self, worker, kind):
        """The tab sends its clock again: the user crossed a time zone."""
        w = worker(sensors=[record(kind, **TAB)])
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'utcOffsetMin': -120})
        w.sync()
        assert read_regs(w, 0x01, 6) == [0x05, 0x02, 0x06, 0x03, 0x02, 0x01]


@pytest.mark.parametrize('kind', ['ds1307', 'ds3231'])
class TestTheBuildTimeOfTheFirmware:
    def test_the_build_time_leaves_the_clock_on_the_tabs_time(self, worker, kind):
        """rtc.adjust(DateTime(F(__DATE__), F(__TIME__)))"""
        w = worker(sensors=[record(kind, **TAB)], firmware=BUILT)
        write_reg(w, 0x00, *BUILD_TIME)
        # The day of week is the one written, Tuesday as 2, moved by the days
        # from the build to the tab's date, 9369 of them back: a Saturday, 6.
        assert read_regs(w, 0x01, 6) == TAB_WALL

    def test_a_date_of_the_sketchs_own_is_kept(self, worker, kind):
        w = worker(sensors=[record(kind, **TAB)], firmware=BUILT)
        write_reg(w, 0x00, *OWN_TIME)
        assert read_regs(w, 0x01, 6) == OWN_TIME[1:]

    def test_an_image_with_no_build_time_keeps_what_is_written(self, worker, kind):
        w = worker(sensors=[record(kind, **TAB)])
        write_reg(w, 0x00, *BUILD_TIME)
        assert read_regs(w, 0x01, 6) == BUILD_TIME[1:]

    def test_a_part_attached_while_running_knows_the_build_time_too(self, worker, kind):
        w = worker(firmware=BUILT)
        w.send({'cmd': 'sensor_attach', **record(kind, **TAB)})
        w.sync()
        write_reg(w, 0x00, *BUILD_TIME)
        assert read_regs(w, 0x01, 6) == TAB_WALL


class TestTheTemperatureOfTheDs3231:
    def test_the_start_config_record_sets_the_first_read(self, worker):
        """The copy used to start at 25 C whatever the record said."""
        w = worker(sensors=[record('ds3231', temperature=31.75, **TAB)])
        assert read_regs(w, 0x11, 2) == [31, 0xC0]

    def test_a_record_attached_while_running_sets_the_first_read(self, worker):
        w = worker()
        w.send({'cmd': 'sensor_attach', **record('ds3231', temperature=-5.25, **TAB)})
        w.sync()
        assert read_regs(w, 0x11, 2) == [0xFA, 0xC0]

    def test_a_record_with_no_temperature_reads_the_panels_default(self, worker):
        w = worker(sensors=[record('ds3231', **TAB)])
        assert read_regs(w, 0x11, 2) == [25, 0x00]

    def test_the_slider_moves_it_and_leaves_the_clock(self, worker):
        w = worker(sensors=[record('ds3231', temperature=31.75, **TAB)])
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'temperature': 18.5})
        w.sync()
        assert read_regs(w, 0x11, 2) == [18, 0x80]
        assert read_regs(w, 0x01, 6) == TAB_WALL


class TestWhatTheChipsKeep:
    def test_ds1307_clock_halt_and_ram(self, worker):
        w = worker(sensors=[record('ds1307', **TAB)])
        # 12:30:00 with CH set: stopped, so the seconds are known as well.
        write_reg(w, 0x00, 0x80, 0x30, 0x12, 0x06, 0x19, 0x01, 0x13)
        write_reg(w, 0x08, 0xDE, 0xAD)
        assert read_regs(w, 0x00, 10) == [0x80, 0x30, 0x12, 0x06, 0x19, 0x01, 0x13, 0x03, 0xDE, 0xAD]

    def test_ds3231_control_and_status(self, worker):
        w = worker(sensors=[record('ds3231', **TAB)])
        assert read_regs(w, 0x0E, 2) == [0x1C, 0x08]
        write_reg(w, 0x0F, 0x80)
        assert read_regs(w, 0x0E, 2) == [0x1C, 0x00]


@pytest.mark.parametrize('kind', ['ds1307', 'ds3231'])
def test_the_worker_runs_the_model_the_record_carries(worker, kind, model):
    """And says so on stderr when it cannot, keeping the twin."""
    w = worker(sensors=[record(kind, **TAB)])
    assert read_regs(w, 0x01, 6) == TAB_WALL
    assert not any('could not be run' in line for line in w.stderr)
    bad = dict(record(kind, **TAB), wasmB64=base64.b64encode(b'not wasm').decode())
    w = worker(sensors=[bad])
    assert read_regs(w, 0x01, 6) == TAB_WALL
    assert w.wait_for(lambda: any(f'{kind}: the compiled model could not be run' in line
                                  for line in w.stderr), 5.0)
