"""
Tests for ESP32 I2C slave state machines.

Covers BMP280Slave, DS1307Slave, DS3231Slave, I2CWriteSink, and MPU6050Slave
from app/services/esp32_i2c_slaves.py.

Correct picsimlab I2C event encoding (from hw/i2c/picsimlab_i2c.c + QEMU i2c.h):
  event & 0xFF  = operation type:
    0x00 = I2C_START_RECV  — firmware called requestFrom  (read  direction START)
    0x01 = I2C_START_SEND  — firmware called beginTransmission (write direction START)
    0x03 = I2C_FINISH      — end of transaction (STOP or RSTART between write+read)
    0x05 = WRITE byte      — (event >> 8) & 0xFF is the data byte
    0x06 = READ  byte      — return value is the data byte to deliver to firmware

  ACK convention: return 0 = ACK (success), non-zero = NACK.
  For READ events: return value is the data byte sent to the firmware.

Run from the backend/ directory:
    python test_esp32_i2c_slaves.py
"""

import calendar
import json
import sys
import time
import unittest
from pathlib import Path
from unittest import mock

# Ensure backend/ is importable (for direct execution; pytest uses conftest.py)
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))

from app.services.esp32_i2c_slaves import (
    BMP280_RULES,
    BMP280Slave,
    DS1307Slave,
    DS1307_RULES,
    DS3231Slave,
    DS3231_RULES,
    I2CWriteSink,
    MPU6050Slave,
    MPU6050_RULES,
    TabClock,
    find_build_times,
    I2C_START_RECV,
    I2C_START_SEND,
    I2C_FINISH,
    I2C_WRITE,
    I2C_READ,
)


# ── I2C protocol helpers ──────────────────────────────────────────────────────

def i2c_write(byte: int) -> int:
    """WRITE event: data in high byte, op 0x05 in low byte."""
    return ((byte & 0xFF) << 8) | I2C_WRITE


def i2c_read_seq(slave, reg: int, n: int) -> list[int]:
    """Simulate write-then-read: write register address, then read n bytes.

    Models the Adafruit BusIO write_then_read pattern:
      beginTransmission → write(reg) → endTransmission(false) → requestFrom → read()*n
    """
    slave.handle_event(I2C_START_SEND)   # write direction START
    slave.handle_event(i2c_write(reg))   # set register pointer
    slave.handle_event(I2C_FINISH)       # RSTART (repeated start before read phase)
    slave.handle_event(I2C_START_RECV)   # read direction START
    data = [slave.handle_event(I2C_READ) for _ in range(n)]
    slave.handle_event(I2C_FINISH)       # STOP
    return data


def read_u16_le(regs: bytearray, addr: int) -> int:
    """Read unsigned 16-bit little-endian from register array."""
    return regs[addr] | (regs[addr + 1] << 8)


def bcd_valid(value: int) -> bool:
    """Return True if both nibbles of a BCD byte are in 0–9."""
    return (value >> 4) <= 9 and (value & 0xF) <= 9


# ══════════════════════════════════════════════════════════════════════════════
# BMP280 Slave Tests
# ══════════════════════════════════════════════════════════════════════════════

# The bus vectors the tab model replays too (frontend/src/__tests__/
# bmp280-vectors.test.ts). The format is in the README next to the file, and
# the runner is replay_vector below, with the MPU-6050's.
BMP_VECTORS = json.loads(
    (Path(__file__).parent.parent.parent / 'fixtures' / 'i2c-vectors' / 'bmp280.json')
    .read_text(encoding='utf-8'))
BMP_ADDR = int(BMP_VECTORS['address'], 16)

CTRL_MEAS = 0xF4
NORMAL_X1 = 0x27   # temperature x1, pressure x1, normal mode
FORCED_X1 = 0x25


def adc20(data) -> int:
    return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)


class TestBMP280Slave(unittest.TestCase):

    def setUp(self):
        self.slave = BMP280Slave()

    def write(self, reg: int, *values: int) -> None:
        self.slave.handle_event(I2C_START_SEND)
        self.slave.handle_event(i2c_write(reg))
        for value in values:
            self.slave.handle_event(i2c_write(value))
        self.slave.handle_event(I2C_FINISH)

    def normal_mode(self) -> None:
        """What every driver does before it reads: the chip powers on asleep."""
        self.write(CTRL_MEAS, NORMAL_X1)

    # ── I2C protocol ───────────────────────────────────────────────────────────

    def test_ack_on_start_send(self):
        result = self.slave.handle_event(I2C_START_SEND)
        self.assertEqual(result, 0, 'START_SEND must return 0 (ACK)')

    def test_ack_on_start_recv(self):
        result = self.slave.handle_event(I2C_START_RECV)
        self.assertEqual(result, 0, 'START_RECV must return 0 (ACK)')

    def test_ack_on_write(self):
        self.slave.handle_event(I2C_START_SEND)
        result = self.slave.handle_event(i2c_write(0xD0))
        self.assertEqual(result, 0, 'WRITE must return 0 (ACK)')

    def test_finish_returns_zero(self):
        self.slave.handle_event(I2C_START_SEND)
        result = self.slave.handle_event(I2C_FINISH)
        self.assertEqual(result, 0)

    # ── Chip identity ──────────────────────────────────────────────────────────

    def test_chip_id_register_0xd0(self):
        chip_id = i2c_read_seq(self.slave, 0xD0, 1)[0]
        self.assertEqual(chip_id, 0x58, 'chip_id must be 0x58 for BMP280')

    # ── Calibration registers ──────────────────────────────────────────────────

    def test_calibration_t1_little_endian(self):
        # DIG_T1=27504 stored at 0x88 (LSB) and 0x89 (MSB)
        t1 = read_u16_le(self.slave.regs, 0x88)
        self.assertEqual(t1, BMP280Slave.DIG_T1)

    def test_calibration_p1_little_endian(self):
        p1 = read_u16_le(self.slave.regs, 0x8E)
        self.assertEqual(p1, BMP280Slave.DIG_P1)

    # ── Default measurement values ─────────────────────────────────────────────

    def test_default_temp_is_the_panels(self):
        """Default state: 24 °C, where the sensor panel starts → compensated
        centidegrees ≈ 2400."""
        self.normal_mode()
        adc_t = adc20(i2c_read_seq(self.slave, 0xFA, 3))
        compensated = self.slave._compensate_t(adc_t)
        # Allow ±1 centidegree tolerance (binary-search rounding)
        self.assertAlmostEqual(compensated, 2400, delta=1)

    def test_default_pressure_1013hpa(self):
        """Default state: 1013.25 hPa → compensated Pa within 100 Pa tolerance."""
        self.normal_mode()
        adc_t = adc20(i2c_read_seq(self.slave, 0xFA, 3))
        adc_p = adc20(i2c_read_seq(self.slave, 0xF7, 3))
        compensated_pa = self.slave._compensate_p(adc_p, adc_t)
        target_pa = 1013.25 * 100.0
        self.assertAlmostEqual(compensated_pa, target_pa, delta=100)  # ±1 hPa

    def test_data_registers_hold_the_reset_value_until_a_mode_is_selected(self):
        """The chip powers on in sleep mode and measures nothing there
        (datasheet 3.6.1); table 18 gives the data registers 0x80 0x00 0x00."""
        self.assertEqual(i2c_read_seq(self.slave, 0xF7, 6), [0x80, 0, 0, 0x80, 0, 0])
        self.slave.update(30.0, 900.0)
        self.assertEqual(i2c_read_seq(self.slave, 0xF7, 6), [0x80, 0, 0, 0x80, 0, 0])

    # ── update() changes ADC registers ────────────────────────────────────────

    def test_update_changes_temp_regs(self):
        self.normal_mode()
        before = i2c_read_seq(self.slave, 0xFA, 3)
        self.slave.update(30.0, 1013.25)
        after = i2c_read_seq(self.slave, 0xFA, 3)
        self.assertNotEqual(before, after, 'Temp ADC regs must change after update(30.0,...)')

    def test_update_changes_pressure_regs(self):
        self.normal_mode()
        before = i2c_read_seq(self.slave, 0xF7, 3)
        self.slave.update(25.0, 900.0)
        after = i2c_read_seq(self.slave, 0xF7, 3)
        self.assertNotEqual(before, after, 'Pressure ADC regs must change after update(...,900.0)')

    def test_update_temp_compensates_correctly(self):
        self.normal_mode()
        self.slave.update(40.0, 1013.25)
        adc_t = adc20(i2c_read_seq(self.slave, 0xFA, 3))
        compensated = self.slave._compensate_t(adc_t)
        self.assertAlmostEqual(compensated, 4000, delta=1)

    def test_update_takes_a_sensor_record(self):
        """The worker hands over the record, or the update, as it arrived:
        the panel's names, and whatever else the tab put in."""
        self.slave.update(sensor_type='bmp280', pin=276, addr=0x76, owner='bmp1',
                          temperature=31.5, pressure=990)
        self.assertEqual(self.slave.inputs(), {'temperature': 31.5, 'pressure': 990.0})

    def test_update_changes_what_it_names_and_nothing_else(self):
        self.slave.update(temperature=31.5, pressure=990)
        self.slave.update(cmd='sensor_update', pin=276, pressure=1000.25)
        self.assertEqual(self.slave.inputs(), {'temperature': 31.5, 'pressure': 1000.25})
        self.slave.update(temperature=-5)
        self.assertEqual(self.slave.inputs(), {'temperature': -5.0, 'pressure': 1000.25})

    def test_update_by_the_names_it_had(self):
        self.slave.update(temperature_c=12.0, pressure_hpa=950.0)
        self.assertEqual(self.slave.inputs(), {'temperature': 12.0, 'pressure': 950.0})
        self.slave.update(13.0, 951.0)
        self.assertEqual(self.slave.inputs(), {'temperature': 13.0, 'pressure': 951.0})

    def test_update_leaves_out_what_is_not_a_number(self):
        self.slave.update(temperature=31.5, pressure=990)
        self.slave.update(temperature=None, pressure=float('nan'))
        self.slave.update(temperature='warm', pressure=float('inf'))
        self.slave.update(temperature=True)
        self.assertEqual(self.slave.inputs(), {'temperature': 31.5, 'pressure': 990.0})

    def test_update_takes_a_number_typed_as_text(self):
        """A property dialog stores what was typed."""
        self.slave.update(temperature='24.5', pressure='1001')
        self.assertEqual(self.slave.inputs(), {'temperature': 24.5, 'pressure': 1001.0})

    def test_starts_from_the_values_the_panel_starts_from(self):
        self.assertEqual(self.slave.inputs(), BMP_VECTORS['inputs'])

    # ── Sequential register reads ──────────────────────────────────────────────

    def test_sequential_read_advances_pointer(self):
        """Reading 3 bytes from 0xF7 must yield distinct pressure MSB/LSB/XLSB."""
        self.normal_mode()
        bytes_ = i2c_read_seq(self.slave, 0xF7, 3)
        self.assertEqual(len(bytes_), 3)
        # At least two of the three bytes must differ (non-trivial measurement)
        self.assertFalse(bytes_[0] == bytes_[1] == bytes_[2],
                         'All three pressure ADC bytes should not be identical')

    # ── State machine ─────────────────────────────────────────────────────────

    def test_write_sets_register_ptr(self):
        """Write-then-read: write reg address 0xD0, then read returns chip_id."""
        self.slave.handle_event(I2C_START_SEND)
        self.slave.handle_event(i2c_write(0xD0))   # set ptr to chip_id reg
        self.slave.handle_event(I2C_FINISH)
        self.slave.handle_event(I2C_START_RECV)
        val = self.slave.handle_event(I2C_READ)
        self.assertEqual(val, 0x58)

    def test_finish_resets_first_byte_flag(self):
        """After FINISH, the next transaction's first WRITE must set reg_ptr."""
        self.slave.handle_event(I2C_START_SEND)
        self.slave.handle_event(i2c_write(0xD0))
        self.slave.handle_event(I2C_FINISH)
        # New transaction: WRITE 0xD0 again → should set reg_ptr, not write data
        self.slave.handle_event(I2C_START_SEND)
        self.slave.handle_event(i2c_write(0xD0))
        self.slave.handle_event(I2C_FINISH)
        self.slave.handle_event(I2C_START_RECV)
        val = self.slave.handle_event(I2C_READ)
        self.assertEqual(val, 0x58, 'chip_id should still be 0x58 after FINISH + new transaction')

    # ── What a driver can tell from the chip ──────────────────────────────────

    def test_measuring_is_seen_once_after_a_forced_write(self):
        """SparkFun's and pocketBME280's examples wait for the bit to rise,
        Adafruit's takeForcedMeasurement() for it to fall; neither has a
        timeout."""
        self.write(CTRL_MEAS, FORCED_X1)
        self.assertEqual([i2c_read_seq(self.slave, 0xF3, 1)[0] for _ in range(3)], [0x08, 0, 0])

    def test_forced_mode_is_back_in_sleep_mode(self):
        self.write(CTRL_MEAS, FORCED_X1)
        self.assertEqual(i2c_read_seq(self.slave, CTRL_MEAS, 1), [FORCED_X1 & ~0x03])

    def test_soft_reset_restores_the_power_on_registers_and_reads_zero(self):
        self.write(0xF5, 0x90)
        self.normal_mode()
        self.slave.update(30.0, 900.0)
        self.write(0xE0, 0xB6)
        self.assertEqual(i2c_read_seq(self.slave, 0xE0, 1), [0x00])
        self.assertEqual(i2c_read_seq(self.slave, 0xF3, 3), [0x00, 0x00, 0x00])
        self.assertEqual(i2c_read_seq(self.slave, 0xF7, 6), [0x80, 0, 0, 0x80, 0, 0])
        self.assertEqual(self.slave.inputs(), {'temperature': 30.0, 'pressure': 900.0})

    def test_a_write_is_pairs_of_address_and_data(self):
        self.write(0xF5, 0xA0, CTRL_MEAS, NORMAL_X1)
        self.assertEqual(i2c_read_seq(self.slave, CTRL_MEAS, 2), [NORMAL_X1, 0xA0])

    def test_the_calibration_cannot_be_written(self):
        before = bytes(self.slave.regs[0x88:0xA0])
        self.write(0x88, 0xAA)
        self.assertEqual(bytes(self.slave.regs[0x88:0xA0]), before)

    def test_dump_is_what_a_read_would_find(self):
        self.assertEqual(list(self.slave.dump_registers()[0xF7:0xFD]), [0x80, 0, 0, 0x80, 0, 0])
        self.normal_mode()
        self.slave.update(30.0, 900.0)
        dump = self.slave.dump_registers()
        self.assertEqual(list(dump[0xF7:0xFD]), i2c_read_seq(self.slave, 0xF7, 6))
        self.assertEqual(dump[0xF3], 0x00, 'no trigger bit in a copy')
        self.assertEqual(len(dump), 256)

    def test_rules_are_the_table_the_shared_vectors_carry(self):
        """The twin and the tab model work from one table of facts, and the
        vectors hold the copy both are compared with."""
        def pairs(table: dict) -> dict:
            return {f'{reg:02X}': f'{value:02X}' for reg, value in table.items()}
        mode = BMP280_RULES['mode']
        self.assertEqual({
            'power_on': pairs(BMP280_RULES['power_on']),
            'writable': [f'{reg:02X}' for reg in BMP280_RULES['writable']],
            'reset': pairs(BMP280_RULES['reset']),
            'status': pairs(BMP280_RULES['status']),
            'mode': {key: f'{mode[key]:02X}' for key in ('register', 'mask', 'sleep', 'normal')},
            'sample': [f'{reg:02X}' for reg in BMP280_RULES['sample']],
        }, BMP_VECTORS['rules'])

    def test_the_vectors_are_the_format_this_runner_reads(self):
        self.assertEqual(BMP_VECTORS['format'], 1)
        self.assertEqual(BMP_VECTORS['device'], 'bmp280')
        self.assertGreaterEqual(len(BMP_VECTORS['vectors']), 22)


def _bmp_vector_case(vector: dict, flavour: str):
    def case(self):
        slave = BMP280Slave(BMP_ADDR)
        slave.update(**BMP_VECTORS['inputs'])
        replay_vector(self, slave, vector, flavour)
    case.__doc__ = f'{vector["name"]} ({flavour})'
    return case


# One test per shared vector and bus flavour, named after its place in the
# file. The flavours are BUS_FLAVOURS below, spelled out because the table
# comes after this class.
for _flavour in ('stop-start', 'repeated-start'):
    for _n, _vector in enumerate(BMP_VECTORS['vectors'], start=1):
        setattr(TestBMP280Slave,
                f'test_vector_{_n:02d}_{_flavour.replace("-", "_")}',
                _bmp_vector_case(_vector, _flavour))


# ══════════════════════════════════════════════════════════════════════════════
# DS1307 / DS3231 Slave Tests
# ══════════════════════════════════════════════════════════════════════════════

VECTOR_DIR = Path(__file__).parent.parent.parent / 'fixtures' / 'i2c-vectors'

# The bus vectors the tab models replay too (frontend/src/__tests__/
# rtc-vectors.test.ts). The format is in the README next to the files.
DS1307_VECTORS = json.loads((VECTOR_DIR / 'ds1307.json').read_text(encoding='utf-8'))
DS3231_VECTORS = json.loads((VECTOR_DIR / 'ds3231.json').read_text(encoding='utf-8'))

# How a repeated START reaches the model. QEMU's ESP32 controller ends the
# transfer first (hw/i2c/esp32_i2c.c, I2C_OPCODE_RSTART), so the model hears
# FINISH and then START; the STM32 one delivers the START alone, as the wire
# does. The worker twin answers both guests.
BUS_FLAVOURS = ('stop-start', 'repeated-start')

BUILD_MONTHS = ('Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec')


def wall_clock(text: str) -> int:
    """'2026-09-30T12:34:56.250' as the clock of a model counts it:
    milliseconds since 00:00 of 1 January 1970 of the same calendar. No time
    zone comes into it, so a vector reads the same wherever the test runs."""
    stamp, _, ms = text.partition('.')
    return calendar.timegm(time.strptime(stamp, '%Y-%m-%dT%H:%M:%S')) * 1000 + int(ms or 0)


def build_time(pair) -> tuple:
    """['Sep 29 2026', '23:39:41'], the two strings as the compiler writes them."""
    date, clock = pair
    return (int(date[7:]), BUILD_MONTHS.index(date[:3]) + 1, int(date[4:6]),
            *(int(n) for n in clock.split(':')))


class VectorClock:
    """A clock a vector moves by hand."""

    def __init__(self, start: str) -> None:
        self.now = wall_clock(start)

    def __call__(self) -> int:
        return self.now

    def step(self, step: dict) -> None:
        if 'set' in step:
            self.now = wall_clock(step['set'])
        else:
            self.now += step.get('advance_ms', 0)


def rtc_power_on(cls, vectors: dict, vector: dict):
    """A chip that has just been powered on, under the clock and the firmware
    of the vector."""
    clock = VectorClock(vector.get('clock', vectors['clock']))
    built = [build_time(pair) for pair in vector.get('build_times', vectors['build_times'])]
    slave = cls(dict(vectors['inputs']), clock=clock, build_times=lambda: built)
    return slave, clock


def rtc_at(cls, start: str, record=None, built=()):
    clock = VectorClock(start)
    return cls(record, clock=clock, build_times=lambda: built), clock


def pairs(table: dict) -> dict:
    return {f'{reg:02X}': f'{value:02X}' for reg, value in table.items()}


class TestDS1307Slave(unittest.TestCase):

    def setUp(self):
        self.slave, self.clock = rtc_at(DS1307Slave, '2026-09-30T12:34:56.250')

    def test_ack_on_start_send(self):
        self.assertEqual(self.slave.handle_event(I2C_START_SEND), 0)

    def test_ack_on_write(self):
        self.slave.handle_event(I2C_START_SEND)
        self.assertEqual(self.slave.handle_event(i2c_write(0x00)), 0)

    def test_answers_at_0x68(self):
        self.assertEqual(self.slave.addr, 0x68)
        self.assertEqual(int(DS1307_VECTORS['address'], 16), 0x68)

    def test_the_time_is_the_clocks_with_monday_as_day_1(self):
        """30 September 2026 is a Wednesday. The twin used to count Monday as
        1 and the tab Sunday as 1; both count Monday as 1 now."""
        self.assertEqual(i2c_read_seq(self.slave, 0x00, 7),
                         [0x56, 0x34, 0x12, 0x03, 0x30, 0x09, 0x26])

    def test_a_written_time_is_kept(self):
        """Every write used to be dropped: the sketch set 12:30:00 and read
        the clock of the container."""
        i2c_write_reg(self.slave, 0x00, 0x00, 0x30, 0x12, 0x06, 0x19, 0x01, 0x13)
        self.clock.now += 2000
        self.assertEqual(i2c_read_seq(self.slave, 0x00, 7),
                         [0x02, 0x30, 0x12, 0x06, 0x19, 0x01, 0x13])

    def test_the_pointer_moves_past_a_written_byte(self):
        """A data write used to leave the pointer where it was, so the second
        byte of a burst landed on the first one's register."""
        i2c_write_reg(self.slave, 0x08, 0xAA, 0xBB)
        self.assertEqual(i2c_read_seq(self.slave, 0x08, 2), [0xAA, 0xBB])

    def test_no_record_and_no_clock_is_the_machines_own_time(self):
        """A caller that says nothing (the STM32 worker of the overlay builds
        the twin with no argument) gets the clock of the machine, in UTC."""
        machine = wall_clock('2026-09-30T10:34:56.250') / 1000.0
        with mock.patch('app.services.esp32_i2c_slaves._time.time', return_value=machine):
            slave = DS1307Slave()
            self.assertEqual(i2c_read_seq(slave, 0x00, 7),
                             [0x56, 0x34, 0x10, 0x03, 0x30, 0x09, 0x26])

    def test_rules_are_the_table_the_shared_vectors_carry(self):
        self.assertEqual({
            'power_on': pairs(DS1307_RULES['power_on']),
            'write_mask': pairs(DS1307_RULES['write_mask']),
            'last_register': f'{DS1307_RULES["last_register"]:02X}',
        }, DS1307_VECTORS['rules'])

    def test_the_vectors_are_the_format_this_runner_reads(self):
        self.assertEqual(DS1307_VECTORS['format'], 1)
        self.assertEqual(DS1307_VECTORS['device'], 'ds1307')
        self.assertGreaterEqual(len(DS1307_VECTORS['vectors']), 22)


class TestDS3231Slave(unittest.TestCase):

    def setUp(self):
        self.slave, self.clock = rtc_at(DS3231Slave, '2026-09-30T12:34:56.250')

    def test_time_registers(self):
        self.assertEqual(i2c_read_seq(self.slave, 0x00, 7),
                         [0x56, 0x34, 0x12, 0x03, 0x30, 0x09, 0x26])

    def test_control_register_powers_on_at_0x1c(self):
        """It read 0x00, and RTClib's setAlarm1() refuses to arm an alarm
        unless INTCN reads 1."""
        self.assertEqual(i2c_read_seq(self.slave, 0x0E, 1), [0x1C])

    def test_status_register_powers_on_with_osf_clear(self):
        """A module somebody set, as the DS1307 powers on with CH 0: RTClib's
        lostPower() is false and a sketch prints no "lost power" line."""
        self.assertEqual(i2c_read_seq(self.slave, 0x0F, 1), [0x08])

    def test_temperature_default_25c(self):
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [25, 0x00])

    def test_temperature_quarter_and_half_degree(self):
        self.slave.temperatureC = 25.25
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [25, 0x40])
        self.slave.temperatureC = 25.5
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [25, 0x80])

    def test_temperature_below_zero_is_twos_complement(self):
        """-5.25 C read [0xFB, 0x40], which is -4.75 C."""
        self.slave.temperatureC = -5.25
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [0xFA, 0xC0])

    def test_the_record_seeds_the_temperature(self):
        slave, _ = rtc_at(DS3231Slave, '2026-09-30T12:34:56.250',
                          {'sensor_type': 'ds3231', 'pin': 300, 'addr': 0x68, 'temperature': 31.75})
        self.assertEqual(i2c_read_seq(slave, 0x11, 2), [31, 0xC0])

    def test_update_takes_a_record_or_the_temperature_alone(self):
        """The ESP32 worker hands update() the command it received, and the
        STM32 worker of the overlay calls update(temperature)."""
        self.slave.update(cmd='sensor_update', pin=300, temperature=-10.75)
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [0xF5, 0x40])
        self.slave.update(30.0)
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [30, 0x00])
        # What is not a temperature leaves the last one in place.
        for junk in ('warm', None, float('nan'), True):
            self.slave.update(temperature=junk)
        self.slave.update(cmd='sensor_update', pin=300)
        self.assertEqual(i2c_read_seq(self.slave, 0x11, 2), [30, 0x00])

    def test_rules_are_the_table_the_shared_vectors_carry(self):
        self.assertEqual({
            'power_on': pairs(DS3231_RULES['power_on']),
            'write_mask': pairs(DS3231_RULES['write_mask']),
            'self_clearing': pairs(DS3231_RULES['self_clearing']),
            'write_zero_to_clear': pairs(DS3231_RULES['write_zero_to_clear']),
            'read_only': [[f'{first:02X}', f'{last:02X}']
                          for first, last in DS3231_RULES['read_only']],
            'last_register': f'{DS3231_RULES["last_register"]:02X}',
            'temp_lsb_per_c': DS3231_RULES['temp_lsb_per_c'],
        }, DS3231_VECTORS['rules'])

    def test_the_vectors_are_the_format_this_runner_reads(self):
        self.assertEqual(DS3231_VECTORS['format'], 1)
        self.assertEqual(DS3231_VECTORS['device'], 'ds3231')
        self.assertGreaterEqual(len(DS3231_VECTORS['vectors']), 27)


class TestTabClock(unittest.TestCase):
    """The worker's clock is the server's; the record says what the tab's is."""

    SERVER = 1_790_728_496_250            # 2026-09-30 00:34:56.250 UTC

    def clock(self, record: dict) -> TabClock:
        tab = TabClock(now_ms=lambda: self.SERVER)
        tab.set(record)
        return tab

    def test_the_offset_moves_the_wall_to_the_users_zone(self):
        """The container counts UTC. A user at UTC+2 read a clock two hours
        behind the tab's."""
        self.assertEqual(self.clock({})(), self.SERVER)
        self.assertEqual(self.clock({'utcOffsetMin': 120})(), self.SERVER + 7_200_000)
        self.assertEqual(self.clock({'utcOffsetMin': -330})(), self.SERVER - 19_800_000)

    def test_an_epoch_near_the_servers_is_the_same_clock(self):
        """The record was stamped before the socket opened and the worker
        started: the difference is the delivery, and would show as a clock
        that is behind."""
        for late in (-59_000, -2_500, 0, 800, 60_000):
            tab = self.clock({'epochMs': self.SERVER + late, 'utcOffsetMin': 120})
            self.assertEqual(tab(), self.SERVER + 7_200_000, f'{late} ms')

    def test_an_epoch_far_from_the_servers_is_followed(self):
        """A browser whose clock somebody set to another day shows that day
        in the tab, and so does the worker's copy."""
        day = 86_400_000
        tab = self.clock({'epochMs': self.SERVER - 3 * day, 'utcOffsetMin': 0})
        self.assertEqual(tab(), self.SERVER - 3 * day)
        tab = self.clock({'epochMs': self.SERVER + 61_000})
        self.assertEqual(tab(), self.SERVER + 61_000)

    def test_a_record_that_says_nothing_changes_nothing(self):
        tab = self.clock({'epochMs': self.SERVER - 86_400_000, 'utcOffsetMin': 60})
        was = tab()
        for record in ({}, {'temperature': 30}, {'utcOffsetMin': 'east'}, {'epochMs': None},
                       {'utcOffsetMin': float('inf')}, {'epochMs': True}):
            tab.set(record)
            self.assertEqual(tab(), was, repr(record))

    def test_the_twin_reads_the_wall_of_the_record(self):
        """12:34 in the tab, at UTC+2, while the container says 10:34."""
        server = wall_clock('2026-09-30T10:34:56.250')
        tab = TabClock(now_ms=lambda: server)
        tab.set({'epochMs': server - 1200, 'utcOffsetMin': 120})
        slave = DS1307Slave(clock=tab)
        self.assertEqual(i2c_read_seq(slave, 0x00, 7),
                         [0x56, 0x34, 0x12, 0x03, 0x30, 0x09, 0x26])

    def test_an_update_with_the_clock_moves_a_twin_that_follows_the_host(self):
        server = wall_clock('2026-09-30T10:34:56.250')
        slave = DS3231Slave({'utcOffsetMin': 120}, clock=TabClock(now_ms=lambda: server))
        self.assertEqual(i2c_read_seq(slave, 0x00, 3), [0x56, 0x34, 0x12])
        slave.update(cmd='sensor_update', pin=300, utcOffsetMin=13 * 60)
        self.assertEqual(i2c_read_seq(slave, 0x00, 7),
                         [0x56, 0x34, 0x23, 0x03, 0x30, 0x09, 0x26])
        slave.update(utcOffsetMin=14 * 60)
        self.assertEqual(i2c_read_seq(slave, 0x00, 7),
                         [0x56, 0x34, 0x00, 0x04, 0x01, 0x10, 0x26])


class TestFindBuildTimes(unittest.TestCase):
    """__DATE__ and __TIME__ as they lie in a flash image."""

    def test_the_two_strings_of_a_sketch(self):
        image = b'\xff' * 40 + b'23:39:41\x00Sep 29 2026\x00' + b'\x00' * 9
        self.assertEqual(find_build_times(image), [(2026, 9, 29, 23, 39, 41)])

    def test_the_day_is_padded_with_a_space(self):
        image = b'Oct  1 2026\x0000:00:09\x00'
        self.assertEqual(find_build_times(image), [(2026, 10, 1, 0, 0, 9)])

    def test_every_date_goes_with_every_time(self):
        """An ESP32 image: the bootloader, the application descriptor (two
        16-byte fields padded with NULs) and the sketch."""
        image = (b'\xe9' + b'23:41:52\x00' + b'\xff' * 99
                 + b'23:41:31' + b'\x00' * 8 + b'Sep 29 2026' + b'\x00' * 5
                 + b'velxio-sketch\x00' + b'23:41:51\x00Sep 29 2026\x00')
        self.assertEqual(sorted(find_build_times(image)), [
            (2026, 9, 29, 23, 41, 31), (2026, 9, 29, 23, 41, 51), (2026, 9, 29, 23, 41, 52)])

    def test_midnight_between_two_files_of_one_build(self):
        image = b'Sep 29 2026\x0023:59:58\x00' + b'Sep 30 2026\x0000:00:03\x00'
        self.assertEqual(len(find_build_times(image)), 4)
        self.assertIn((2026, 9, 29, 23, 59, 58), find_build_times(image))

    def test_a_time_right_after_another_time(self):
        """Two files of one build whose __TIME__ literals the linker laid end
        to end: the NUL that ends the first is the byte before the second."""
        image = b'Sep 29 2026\x0023:41:51\x0023:41:52\x00'
        self.assertEqual(sorted(find_build_times(image)), [
            (2026, 9, 29, 23, 41, 51), (2026, 9, 29, 23, 41, 52)])

    def test_what_is_not_the_strings(self):
        for image in (
            b'',
            b'\x00' * 64,
            b'23:39:41\x00',                        # a time and no date
            b'Sep 29 2026\x00',                     # a date and no time
            b'Sep 29 2026 23:39:41\x00',            # no NUL after the date
            b'Sep 29 2026\x0023:39:41 UTC\x00',     # no NUL after the time
            b'Sep 29 2026\x00AA:BB:12:34:56\x00',   # the tail of a MAC address
            b'Sep 29 2026\x0024:00:00\x00',
            b'Sep 29 2026\x0023:60:00\x00',
            b'Sep 32 2026\x0023:39:41\x00',
            b'Sept 9 2026\x0023:39:41\x00',
            b'2026-09-29\x0023:39:41\x00',
        ):
            self.assertEqual(find_build_times(image), [], repr(image))


def _rtc_vector_case(cls, vectors: dict, vector: dict, flavour: str):
    def case(self):
        slave, clock = rtc_power_on(cls, vectors, vector)
        replay_vector(self, slave, vector, flavour, clock=clock)
    case.__doc__ = f'{vector["name"]} ({flavour})'
    return case


# ══════════════════════════════════════════════════════════════════════════════
# I2CWriteSink Tests
# ══════════════════════════════════════════════════════════════════════════════

class TestI2CWriteSink(unittest.TestCase):

    def setUp(self):
        self.emitted: list[dict] = []
        self.sink = I2CWriteSink(addr=0x3C, emit_fn=self.emitted.append)

    def test_ack_on_start_send(self):
        self.assertEqual(self.sink.handle_event(I2C_START_SEND), 0)

    def test_ack_on_start_recv(self):
        self.assertEqual(self.sink.handle_event(I2C_START_RECV), 0)

    def test_ack_on_write(self):
        self.sink.handle_event(I2C_START_SEND)
        for byte in [0x00, 0x21, 0xAB]:
            result = self.sink.handle_event(i2c_write(byte))
            self.assertEqual(result, 0, f'WRITE 0x{byte:02X} must return 0 (ACK)')

    def test_read_returns_0xff(self):
        """Write-only device: READ must return 0xFF (no data to send)."""
        self.sink.handle_event(I2C_START_SEND)
        result = self.sink.handle_event(I2C_READ)
        self.assertEqual(result, 0xFF)

    def test_emits_on_finish(self):
        """After 3 writes + FINISH, emit_fn must be called exactly once."""
        self.sink.handle_event(I2C_START_SEND)
        for b in [0x00, 0x21, 0x7F]:
            self.sink.handle_event(i2c_write(b))
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(len(self.emitted), 1, 'emit_fn must be called once on FINISH')

    def test_emit_payload_addr(self):
        self.sink.handle_event(I2C_START_SEND)
        self.sink.handle_event(i2c_write(0xAB))
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(self.emitted[0]['addr'], 0x3C)

    def test_emit_payload_type(self):
        self.sink.handle_event(I2C_START_SEND)
        self.sink.handle_event(i2c_write(0xAB))
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(self.emitted[0]['type'], 'i2c_transaction')

    def test_emit_payload_data(self):
        bytes_ = [0x00, 0x21, 0x7F]
        self.sink.handle_event(I2C_START_SEND)
        for b in bytes_:
            self.sink.handle_event(i2c_write(b))
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(self.emitted[0]['data'], bytes_)

    def test_no_emit_for_empty_buffer(self):
        """START then immediate FINISH with no writes must NOT emit."""
        self.sink.handle_event(I2C_START_SEND)
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(len(self.emitted), 0, 'Empty buffer must not trigger emit_fn')

    def test_resets_buffer_after_emit(self):
        """Second transaction accumulates fresh bytes, not leftover from first."""
        # First transaction: writes [0xAA]
        self.sink.handle_event(I2C_START_SEND)
        self.sink.handle_event(i2c_write(0xAA))
        self.sink.handle_event(I2C_FINISH)
        # Second transaction: writes [0xBB]
        self.sink.handle_event(I2C_START_SEND)
        self.sink.handle_event(i2c_write(0xBB))
        self.sink.handle_event(I2C_FINISH)
        self.assertEqual(len(self.emitted), 2)
        self.assertEqual(self.emitted[0]['data'], [0xAA])
        self.assertEqual(self.emitted[1]['data'], [0xBB])

    def test_custom_addr_forwarded(self):
        """Sink created with addr=0x27 emits with that addr."""
        sink = I2CWriteSink(addr=0x27, emit_fn=self.emitted.append)
        sink.handle_event(I2C_START_SEND)
        sink.handle_event(i2c_write(0x38))
        sink.handle_event(I2C_FINISH)
        self.assertEqual(self.emitted[0]['addr'], 0x27)

    def test_finish_return_value(self):
        """FINISH always returns 0."""
        self.sink.handle_event(I2C_START_SEND)
        result = self.sink.handle_event(I2C_FINISH)
        self.assertEqual(result, 0)


# ══════════════════════════════════════════════════════════════════════════════
# MPU6050 Slave Tests
# ══════════════════════════════════════════════════════════════════════════════

# The bus vectors the tab model replays too (frontend/src/__tests__/
# protocol-parts.test.ts). The format is in the README next to the file.
MPU_VECTORS = json.loads(
    (Path(__file__).parent.parent.parent / 'fixtures' / 'i2c-vectors' / 'mpu6050.json')
    .read_text(encoding='utf-8'))
MPU_ADDR = int(MPU_VECTORS['address'], 16)

def hex_bytes(text: str) -> list[int]:
    """'6B 00' and '00*107 40' as bytes; every number is hexadecimal but the
    repeat count."""
    out: list[int] = []
    for word in text.split():
        byte, _, times = word.partition('*')
        out.extend([int(byte, 16)] * (int(times) if times else 1))
    return out


def hex_text(data) -> str:
    return ' '.join(f'{b:02X}' for b in data)


def i2c_write_reg(slave, reg: int, *values: int) -> None:
    """beginTransmission, the pointer, the bytes, endTransmission."""
    slave.handle_event(I2C_START_SEND)
    slave.handle_event(i2c_write(reg))
    for value in values:
        slave.handle_event(i2c_write(value))
    slave.handle_event(I2C_FINISH)


def int16(hi: int, lo: int) -> int:
    raw = (hi << 8) | lo
    return raw - 0x10000 if raw & 0x8000 else raw


def replay_vector(case: unittest.TestCase, slave, vector: dict, flavour: str,
                  clock=None) -> None:
    """One vector against one model, as QEMU's events. `clock` is the host's
    clock of a model that has one, for the vector to move."""
    is_open = False

    def start(read: bool) -> None:
        nonlocal is_open
        if is_open and flavour == 'stop-start':
            slave.handle_event(I2C_FINISH)
        is_open = True
        ack = slave.handle_event(I2C_START_RECV if read else I2C_START_SEND)
        case.assertEqual(ack, 0, 'the address is acknowledged')

    def stop() -> None:
        nonlocal is_open
        is_open = False
        slave.handle_event(I2C_FINISH)

    def send(data: list[int]) -> None:
        for byte in data:
            case.assertEqual(slave.handle_event(i2c_write(byte)), 0,
                             f'0x{byte:02x} is acknowledged')

    def recv(n: int) -> list[int]:
        return [slave.handle_event(I2C_READ) & 0xFF for _ in range(n)]

    for i, step in enumerate(vector['steps']):
        op = step['op']
        got = None
        if op == 'write':
            start(False)
            send(hex_bytes(step['data']))
            stop()
        elif op == 'read':
            start(False)
            send([int(step['reg'], 16)])
            start(True)
            got = recv(step['n'])
            stop()
        elif op == 'get':
            start(True)
            got = recv(step['n'])
            stop()
        elif op == 'start':
            start(step['rw'] == 'r')
        elif op == 'send':
            send(hex_bytes(step['data']))
        elif op == 'recv':
            got = recv(step['n'])
        elif op == 'stop':
            stop()
        elif op == 'inputs':
            slave.update(**step['values'])
        elif op == 'clock':
            case.assertIsNotNone(clock, f'step {i}: this model has no clock to move')
            clock.step(step)
        elif op == 'dump':
            at = int(step['at'], 16)
            got = slave.dump_registers()[at:at + len(hex_bytes(step['expect']))]
        else:
            case.fail(f'step {i}: unknown op "{op}"')
        if got is not None:
            case.assertEqual(hex_text(got), hex_text(hex_bytes(step['expect'])),
                             f'step {i} {json.dumps(step)}')


class TestMPU6050Slave(unittest.TestCase):

    def setUp(self):
        self.mpu = MPU6050Slave()

    def wake(self, slave=None):
        """What every driver does first: PWR_MGMT_1 = 0x00 clears SLEEP."""
        i2c_write_reg(slave or self.mpu, 0x6B, 0x00)

    def test_who_am_i_default_addr(self):
        """WHO_AM_I register (0x75) must return 0x68."""
        result = i2c_read_seq(self.mpu, 0x75, 1)
        self.assertEqual(result[0], 0x68)

    def test_start_send_ack(self):
        """START_SEND event must return 0 (ACK)."""
        self.assertEqual(self.mpu.handle_event(I2C_START_SEND), 0)

    def test_start_recv_ack(self):
        """START_RECV event must return 0 (ACK)."""
        self.assertEqual(self.mpu.handle_event(I2C_START_RECV), 0)

    def test_write_then_read_who_am_i(self):
        """Full write-then-read: write reg 0x75, then read returns 0x68."""
        m = MPU6050Slave()
        m.handle_event(I2C_START_SEND)           # beginTransmission
        m.handle_event(i2c_write(0x75))           # write register address
        m.handle_event(I2C_FINISH)                # RSTART
        m.handle_event(I2C_START_RECV)            # requestFrom
        result = m.handle_event(I2C_READ)         # read byte
        self.assertEqual(result, 0x68, f"WHO_AM_I must be 0x68, got 0x{result:02x}")

    def test_detected_pattern(self):
        """detected() pattern: START_SEND then FINISH (no data bytes) returns ACK."""
        m = MPU6050Slave()
        # beginTransmission + endTransmission (no data)
        r1 = m.handle_event(I2C_START_SEND)
        self.assertEqual(r1, 0, "START_SEND must return 0 (ACK) for detected()")
        r2 = m.handle_event(I2C_FINISH)
        self.assertEqual(r2, 0, "FINISH must return 0")

    def test_boots_asleep(self):
        """PWR_MGMT_1 powers on at 0x40 (RM section 3), and a chip nobody woke
        reads zeros however it lies."""
        self.assertEqual(i2c_read_seq(self.mpu, 0x6B, 1), [0x40])
        self.assertEqual(i2c_read_seq(self.mpu, 0x3B, 14), [0] * 14)

    def test_accel_z_default_1g(self):
        """At rest, awake: ACCEL_Z is +1g = 0x4000 at the power-on range of 2 g."""
        self.wake()
        result = i2c_read_seq(self.mpu, 0x3F, 2)
        accel_z = (result[0] << 8) | result[1]
        self.assertEqual(accel_z, 0x4000, f"Expected ACCEL_Z=0x4000, got 0x{accel_z:04x}")

    def test_temperature_default_is_the_panels(self):
        """The panel starts at 24 C, and so does a twin no record has seeded."""
        self.wake()
        hi, lo = i2c_read_seq(self.mpu, 0x41, 2)
        self.assertEqual(int16(hi, lo), round((24 - 36.53) * 340))

    def test_update_accel(self):
        """update() must reflect new accel values in register reads."""
        self.wake()
        self.mpu.update(accel_x=1.0, accel_y=0.0, accel_z=0.0)
        result = i2c_read_seq(self.mpu, 0x3B, 2)
        accel_x = (result[0] << 8) | result[1]
        # 1g at ±2g full-scale = 16384 = 0x4000
        self.assertEqual(accel_x, 0x4000)

    def test_update_changes_only_what_it_names(self):
        """One slider moves at a time: the others stay where they were."""
        self.wake()
        self.mpu.update(gyroX=100, temp=30)
        self.mpu.update(accelX=0.5)
        block = i2c_read_seq(self.mpu, 0x3B, 14)
        self.assertEqual(int16(*block[0:2]), 8192)                          # X, just set
        self.assertEqual(int16(*block[4:6]), 16384)                         # Z, never named
        self.assertEqual(int16(*block[6:8]), round((30 - 36.53) * 340))     # kept
        self.assertEqual(int16(*block[8:10]), 13100)                        # kept

    def test_update_takes_a_sensor_record_as_it_arrives(self):
        """A worker hands over the record whole: the panel's names are read,
        the rest of it is not the chip's."""
        self.wake()
        self.mpu.update(**{
            'sensor_type': 'mpu6050', 'pin': 304, 'addr': 0x68, 'owner': 'imu1',
            'accelX': 0.25, 'accelY': -0.25, 'accelZ': 0.5,
            'gyroX': 10, 'gyroY': -10, 'gyroZ': 20, 'temp': 40,
        })
        self.assertEqual(self.mpu.inputs(), {
            'accelX': 0.25, 'accelY': -0.25, 'accelZ': 0.5,
            'gyroX': 10.0, 'gyroY': -10.0, 'gyroZ': 20.0, 'temp': 40.0,
        })
        block = i2c_read_seq(self.mpu, 0x3B, 14)
        self.assertEqual([int16(*block[i:i + 2]) for i in range(0, 14, 2)],
                         [4096, -4096, 8192, 1180, 1310, -1310, 2620])

    def test_update_leaves_out_what_is_not_a_finite_number(self):
        """As the tab model does: a string, a flag, NaN or infinity is no
        position of a slider."""
        before = self.mpu.inputs()
        self.mpu.update(accelX='1', accelY=True, accelZ=float('nan'),
                        gyroX=float('inf'), gyroY=None, gyroZ=10 ** 400)
        self.assertEqual(self.mpu.inputs(), before)

    def test_update_writes_no_register(self):
        """The panel's values are the world, not registers: what the guest
        owns is untouched, and the sample is worked out at the read."""
        self.wake()
        before = bytes(self.mpu.regs)
        self.mpu.update(accel_x=1.5, gyro_z=200, temp=50)
        self.assertEqual(bytes(self.mpu.regs), before)

    def test_sample_is_latched_at_start_recv(self):
        """The burst is answered from the sample taken when the read began
        (RM section 4.17), not from the values at each byte."""
        self.wake()
        self.mpu.update(accelX=0.5)
        self.mpu.handle_event(I2C_START_SEND)
        self.mpu.handle_event(i2c_write(0x3B))
        self.mpu.handle_event(I2C_FINISH)
        self.mpu.handle_event(I2C_START_RECV)
        self.mpu.update(accelX=-0.5, accelZ=-1, gyroZ=50, temp=0)
        burst = [self.mpu.handle_event(I2C_READ) for _ in range(14)]
        self.mpu.handle_event(I2C_FINISH)
        self.assertEqual(hex_text(burst), '20 00 00 00 40 00 EF 5C 00 00 00 00 00 00')
        # The next read begins a new sample.
        self.assertEqual(int16(*i2c_read_seq(self.mpu, 0x3B, 2)), -8192)

    def test_output_follows_the_selected_range(self):
        """The gallery sketch selects 8 g and 500 deg/s: 1 g is 4096 counts
        and 100 deg/s is 6550. At a fixed 16384 it printed 39.23 m/s2."""
        self.wake()
        self.mpu.update(gyroX=100)
        i2c_write_reg(self.mpu, 0x1C, 0x10)     # AFS_SEL = 2
        i2c_write_reg(self.mpu, 0x1B, 0x08)     # FS_SEL = 1
        block = i2c_read_seq(self.mpu, 0x3B, 14)
        self.assertEqual(int16(*block[4:6]), 4096)
        self.assertEqual(int16(*block[8:10]), 6550)

    def test_just_under_positive_full_scale_stays_positive(self):
        """32767.5 to 32768 counts round to 32768, one more than the register
        holds: it reads 32767, as in the tab model, and not -32768. A value
        from a project file or an API client can land there; no slider does."""
        self.wake()
        self.mpu.update(accelX=1.99998, accelY=-1.99998, accelZ=1.999985,
                        gyroX=250.135, gyroY=-250.135, temp=132.9055)
        block = i2c_read_seq(self.mpu, 0x3B, 14)
        self.assertEqual([int16(*block[i:i + 2]) for i in range(0, 14, 2)],
                         [32767, -32768, 32767, 32767, 32767, -32768, 0])

    def test_a_value_far_off_the_scale_saturates(self):
        """A finite input whose counts overflow a float is still an end of
        the scale, not an error on QEMU's thread."""
        self.wake()
        self.mpu.update(accelX=1e305, accelY=-1e305, gyroX=1e307, temp=-1e306)
        block = i2c_read_seq(self.mpu, 0x3B, 10)
        self.assertEqual([int16(*block[i:i + 2]) for i in range(0, 10, 2)],
                         [32767, -32768, 16384, -32768, 32767])

    def test_sequential_read_14_bytes(self):
        """getEvent() reads 14 bytes from 0x3B: three axes, temperature, three axes."""
        self.wake()
        self.mpu.update(accelX=0.5, accelY=-0.5, gyroX=1, gyroY=-1, gyroZ=2)
        result = i2c_read_seq(self.mpu, 0x3B, 14)
        self.assertEqual(hex_text(result), '20 00 E0 00 40 00 EF 5C 00 83 FF 7D 01 06')

    def test_device_reset_restores_power_on(self):
        """DEVICE_RESET clears itself and puts every register back to its
        power-on value (RM section 4.28), so PWR_MGMT_1 reads 0x40: bit 7
        clear, and asleep again."""
        self.wake()
        i2c_write_reg(self.mpu, 0x19, 0x07, 0x03, 0x18, 0x10)
        self.mpu.update(accelX=0.5)
        i2c_write_reg(self.mpu, 0x6B, 0x80)
        result = i2c_read_seq(self.mpu, 0x6B, 1)
        self.assertEqual(result[0] & 0x80, 0, "DEVICE_RESET bit must auto-clear")
        self.assertEqual(result[0], 0x40, "PWR_MGMT_1 must be back at its power-on value")
        self.assertEqual(i2c_read_seq(self.mpu, 0x19, 4), [0, 0, 0, 0])
        self.assertEqual(i2c_read_seq(self.mpu, 0x75, 1), [0x68])
        # Adafruit's read-modify-write sends 0xC0; nothing of the byte is kept.
        i2c_write_reg(self.mpu, 0x6B, 0xC0)
        self.assertEqual(i2c_read_seq(self.mpu, 0x6B, 1), [0x40])
        # What the panel set is the world around the chip and outlives the reset.
        self.wake()
        self.assertEqual(int16(*i2c_read_seq(self.mpu, 0x3B, 2)), 8192)

    def test_alternate_address(self):
        """MPU6050 at address 0x69 (AD0=HIGH) must still return WHO_AM_I=0x68."""
        mpu69 = MPU6050Slave(addr=0x69)
        result = i2c_read_seq(mpu69, 0x75, 1)
        self.assertEqual(result[0], 0x68)

    def test_reg_ptr_preserved_across_rstart(self):
        """Write sets reg_ptr; FINISH then START_RECV should NOT reset reg_ptr."""
        m = MPU6050Slave()
        m.handle_event(I2C_START_SEND)
        m.handle_event(i2c_write(0x75))   # set reg_ptr = 0x75
        m.handle_event(I2C_FINISH)        # RSTART — must NOT reset reg_ptr
        m.handle_event(I2C_START_RECV)    # read direction START
        val = m.handle_event(I2C_READ)
        self.assertEqual(val, 0x68, "reg_ptr must be preserved across RSTART")

    def test_write_to_reg(self):
        """Writing two bytes sets reg address then reg value."""
        m = MPU6050Slave()
        m.handle_event(I2C_START_SEND)
        m.handle_event(i2c_write(0x6B))   # reg address
        m.handle_event(i2c_write(0x01))   # value
        m.handle_event(I2C_FINISH)
        result = i2c_read_seq(m, 0x6B, 1)
        self.assertEqual(result[0], 0x01)

    def test_rules_are_the_table_the_shared_vectors_carry(self):
        """The twin and the tab model work from one table of facts, and the
        vectors hold the copy both are compared with."""
        def pairs(table: dict) -> dict:
            return {f'{reg:02X}': f'{value:02X}' for reg, value in table.items()}
        self.assertEqual({
            'power_on': pairs(MPU6050_RULES['power_on']),
            'read_only': [[f'{first:02X}', f'{last:02X}']
                          for first, last in MPU6050_RULES['read_only']],
            'self_clearing': pairs(MPU6050_RULES['self_clearing']),
            'accel_lsb_per_g': list(MPU6050_RULES['accel_lsb_per_g']),
            'gyro_lsb_per_dps': list(MPU6050_RULES['gyro_lsb_per_dps']),
            'temp_lsb_per_c': MPU6050_RULES['temp_lsb_per_c'],
            'temp_offset_c': MPU6050_RULES['temp_offset_c'],
        }, MPU_VECTORS['rules'])

    def test_the_vectors_are_the_format_this_runner_reads(self):
        self.assertEqual(MPU_VECTORS['format'], 1)
        self.assertEqual(MPU_VECTORS['device'], 'mpu6050')
        self.assertGreaterEqual(len(MPU_VECTORS['vectors']), 19)


def _vector_case(vector: dict, flavour: str):
    def case(self):
        slave = MPU6050Slave(MPU_ADDR)
        slave.update(**MPU_VECTORS['inputs'])
        replay_vector(self, slave, vector, flavour)
    case.__doc__ = f'{vector["name"]} ({flavour})'
    return case


# One test per shared vector and bus flavour, named after its place in the file.
for _flavour in BUS_FLAVOURS:
    for _n, _vector in enumerate(MPU_VECTORS['vectors'], start=1):
        setattr(TestMPU6050Slave,
                f'test_vector_{_n:02d}_{_flavour.replace("-", "_")}',
                _vector_case(_vector, _flavour))


for _cls, _test, _vectors in ((DS1307Slave, TestDS1307Slave, DS1307_VECTORS),
                               (DS3231Slave, TestDS3231Slave, DS3231_VECTORS)):
    for _flavour in BUS_FLAVOURS:
        for _n, _vector in enumerate(_vectors['vectors'], start=1):
            setattr(_test, f'test_vector_{_n:02d}_{_flavour.replace("-", "_")}',
                    _rtc_vector_case(_cls, _vectors, _vector, _flavour))


# ── Runner ────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=' * 60)
    print('ESP32 I2C Slave Tests')
    print('=' * 60)
    loader = unittest.TestLoader()
    suite  = unittest.TestSuite()
    for cls in [
        TestBMP280Slave,
        TestDS1307Slave,
        TestDS3231Slave,
        TestTabClock,
        TestFindBuildTimes,
        TestI2CWriteSink,
        TestMPU6050Slave,
    ]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
