"""
test_bmp280_worker.py: the BMP280 a QEMU worker hosts (project
i2c-model-fidelity-2026-09).

The model itself is held to the shared bus vectors in test_i2c_slaves.py. This
file is about what only the worker does with it, on the rig of the board-buses
suite (app/services/esp32_worker.py running unmodified, libqemu replaced at the
ctypes boundary, the guest played by calling the worker's own callbacks):

  - the worker's copy starts from the values its sensor record carries, which
    are where the panel's sliders are, whether the record came with the start
    of the board or while it runs. A part attached to a running board used to
    start from the twin's own values and to take the record's at the first
    slider move;
  - a later update changes only what it names. One that named the pressure
    alone used to take the temperature back to a default.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))
from app.services.esp32_i2c_slaves import BMP280Slave  # noqa: E402

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

ADDR = 0x76
PIN = 200 + ADDR
CTRL_MEAS, PRESS_MSB = 0xF4, 0xF7
NORMAL_X1 = 0x27

# Where a project left the sliders: nothing here is the value the panel, the
# tab model or the twin starts from.
PANEL = {'temperature': 31.5, 'pressure': 990.0}
# Where the panel starts (sensorControlConfig, bmp280).
REST = {'temperature': 24.0, 'pressure': 1013.25}
RESET_VALUE = [0x80, 0x00, 0x00, 0x80, 0x00, 0x00]

# The compensation formulas of the datasheet, which a driver runs on what it
# reads; they take no state of the instance.
_FORMULAS = BMP280Slave()


def record(addr: int = ADDR, **values) -> dict:
    """The record the tab's part files for the worker (parts/i2cPart.ts)."""
    return {'sensor_type': 'bmp280', 'pin': 200 + addr, 'addr': addr, 'owner': 'bmp1', **values}


def write_reg(w, reg: int, value: int, addr: int = ADDR) -> None:
    ret = w.i2c(0, addr, [I2C_START_SEND, I2C_WRITE | (reg << 8), I2C_WRITE | (value << 8),
                          I2C_FINISH])
    assert ret[:-1] == [0] * (len(ret) - 1), 'the chip acknowledges its address and every byte'


def read_data(w, addr: int = ADDR) -> list[int]:
    """The six data registers in one burst."""
    ret = w.i2c(0, addr, [I2C_START_SEND, I2C_WRITE | (PRESS_MSB << 8), I2C_FINISH,
                          I2C_START_RECV, *[I2C_READ] * 6, I2C_FINISH])
    return ret[4:10]


def measured(w, addr: int = ADDR) -> dict:
    """What a driver works out from the data registers, to the hundredth of a
    degree and of a hectopascal."""
    data = read_data(w, addr)
    adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    return {
        'temperature': _FORMULAS._compensate_t(adc_t) / 100,
        'pressure': round(_FORMULAS._compensate_p(adc_p, adc_t) / 100, 2),
    }


class TestSeededFromTheRecord:
    def test_the_start_config_record_sets_the_first_read(self, worker):
        w = worker(sensors=[record(**PANEL)])
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        assert measured(w) == PANEL

    def test_a_record_attached_while_running_sets_the_first_read(self, worker):
        w = worker()
        w.send({'cmd': 'sensor_attach', **record(**PANEL)})
        w.sync()
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        assert measured(w) == PANEL

    def test_a_record_attached_at_the_other_address(self, worker):
        """The Grove BMP280 is at 0x77."""
        w = worker()
        w.send({'cmd': 'sensor_attach', **record(0x77, **PANEL)})
        w.sync()
        write_reg(w, CTRL_MEAS, NORMAL_X1, addr=0x77)
        assert measured(w, addr=0x77) == PANEL

    def test_a_record_with_no_values_reads_the_panels_rest(self, worker):
        w = worker(sensors=[record()])
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        assert measured(w) == REST

    def test_a_record_attached_with_no_values_reads_the_panels_rest(self, worker):
        w = worker()
        w.send({'cmd': 'sensor_attach', **record()})
        w.sync()
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        assert measured(w) == REST

    def test_an_update_changes_what_it_names_and_nothing_else(self, worker):
        w = worker(sensors=[record(**PANEL)])
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'pressure': 1000.25})
        w.sync()
        assert measured(w) == {'temperature': 31.5, 'pressure': 1000.25}
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'temperature': -5})
        w.sync()
        assert measured(w) == {'temperature': -5.0, 'pressure': 1000.25}

    def test_an_update_of_a_part_that_carried_no_values(self, worker):
        """The temperature used to fall to 25 C with the first move of the
        pressure slider, beside a panel that says 24."""
        w = worker(sensors=[record()])
        write_reg(w, CTRL_MEAS, NORMAL_X1)
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'pressure': 990})
        w.sync()
        assert measured(w) == {'temperature': 24.0, 'pressure': 990.0}

    def test_the_chip_measures_nothing_until_the_sketch_selects_a_mode(self, worker):
        w = worker(sensors=[record(**PANEL)])
        assert w.read_reg(0, ADDR, CTRL_MEAS) == (0, 0x00)
        assert read_data(w) == RESET_VALUE
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'temperature': 40})
        w.sync()
        assert read_data(w) == RESET_VALUE
