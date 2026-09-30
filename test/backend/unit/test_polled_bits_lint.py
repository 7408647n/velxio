"""
The polled-bit lint (test/fixtures/i2c-vectors/lint-polled-bits.py) finds the
loops a driver waits on a register bit in, in each of the shapes Arduino
libraries write them, and leaves alone the loops that wait on something else.
"""

import importlib.util
import textwrap
from pathlib import Path

import pytest

LINT = Path(__file__).parent.parent.parent / 'fixtures' / 'i2c-vectors' / 'lint-polled-bits.py'
_spec = importlib.util.spec_from_file_location('lint_polled_bits', LINT)
lint = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lint)


def hits_of(tmp_path: Path, files: dict) -> list:
    lib = tmp_path / 'somechip@1.0.0'
    lib.mkdir()
    for name, text in files.items():
        (lib / name).write_text(textwrap.dedent(text), encoding='utf-8')
    return [(h['register'], h['mask'], h['waits_for'], h['bounded'], h['loop'])
            for h in lint.lint_library(lint.Library(lib.name, str(lib)))]


def test_a_while_on_a_register_read_and_mask(tmp_path):
    assert hits_of(tmp_path, {'chip.h': '#define REG_STATUS 0xF3\n', 'chip.cpp': '''
        void Chip::wait() {
          // while (read8(REG_OTHER) & 1) is a comment
          while (read8(REG_STATUS) & 0x08) delay(1);
        }
    '''}) == [('REG_STATUS (0xF3)', '0x08', 'clear', False, 'while')]


def test_a_do_while_on_a_variable_the_body_reads(tmp_path):
    assert hits_of(tmp_path, {'chip.cpp': '''
        #define CTRL 0x2B
        #define RESET_BIT (1 << 6)
        void Chip::reset() {
          uint8_t v;
          do { v = readRegister(CTRL); } while (v & RESET_BIT);
        }
    '''}) == [('CTRL (0x2B)', 'RESET_BIT (0x40)', 'clear', False, 'do-while')]


def test_an_out_parameter_read_and_a_timeout(tmp_path):
    assert hits_of(tmp_path, {'chip.cpp': '''
        int Chip::ready() {
          uint8_t go = 0;
          uint32_t t = millis();
          while ((go & 0x01) == 0) {
            if (millis() - t > 100) return -1;
            readReg(0x0C, &go);
          }
          return 0;
        }
    '''}) == [('0x0C', '0x01', 'set', True, 'while')]


def test_adafruit_busio_register_bits(tmp_path):
    assert hits_of(tmp_path, {'chip.cpp': '''
        #define PWR_MGMT_1 0x6B
        bool Chip::begin() {
          Adafruit_BusIO_Register power =
              Adafruit_BusIO_Register(i2c_dev, PWR_MGMT_1, 1);
          Adafruit_BusIO_RegisterBits device_reset =
              Adafruit_BusIO_RegisterBits(&power, 1, 7);
          device_reset.write(1);
          while (device_reset.read() == 1) {
            delay(1);
          }
          return true;
        }
    '''}) == [('PWR_MGMT_1 (0x6B)', '0x80', 'clear', False, 'RegisterBits')]


def test_a_status_function_of_the_library(tmp_path):
    assert hits_of(tmp_path, {'chip.cpp': '''
        #define STATUS_BUSY 0x80
        uint8_t Chip::getStatus(void) {
          uint8_t ret;
          if (!i2c_dev->read(&ret, 1)) return 0xFF;
          return ret;
        }
        bool Chip::measure() {
          while (getStatus() & STATUS_BUSY) {
            delay(10);
          }
          return true;
        }
    '''}) == [('(status byte, no pointer)', 'STATUS_BUSY (0x80)', 'clear', False, 'while')]


def test_a_while_1_that_breaks_on_a_bit(tmp_path):
    assert hits_of(tmp_path, {'chip.cpp': '''
        void Chip::waitReady() {
          while (1) {
            if (readReg(0x14) & 0x01) break;
            delay(1);
          }
        }
    '''}) == [('0x14', '0x01', 'set', True, 'while(1)')]


@pytest.mark.parametrize('loop', [
    'while (digitalRead(DOUT) == HIGH) {}',
    'while (Wire.available()) { buf[i++] = Wire.read(); }',
    'while (Serial.read() >= 0) {}',
    'while ((c = pgm_read_byte(s++))) {}',
    'while (i < n) { i++; }',
])
def test_loops_on_something_else(tmp_path, loop):
    assert hits_of(tmp_path, {'chip.cpp': f'void f() {{ {loop} }}\n'}) == []


def test_a_folder_of_libraries_skips_the_retired_ones(tmp_path):
    for name in ('a@1', '.retired'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'a.cpp').write_text('void f() { while (read8(0x01) & 1); }\n')
    assert [lib.name for lib in lint.libraries([str(tmp_path)])] == ['a@1']
