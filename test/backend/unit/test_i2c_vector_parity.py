"""
The parity gate of the shared I2C bus vectors (test/fixtures/i2c-vectors,
format in the README there; project i2c-model-fidelity-2026-09, P5).

A chip Velxio models more than once (the tab model in the frontend, the twin
in app/services/esp32_i2c_slaves.py that answers QEMU and the STM32 worker)
stays one chip only while every copy replays the same vectors and works from
the same rules table. This file fails when that stops being true on the
backend's side:

- a vector file has no twin here, or test_i2c_slaves.py does not replay
  every vector of it in both bus flavours;
- a twin's exported rules table is not the `rules` of its file;
- a register-file model is added to esp32_i2c_slaves.py without a vector
  file (the write sinks, which have no registers, are listed below);
- a chip that the worker also runs as a compiled model
  (app/services/wasm_i2c_models.py, buses/models/, P5) has no vector file,
  test_wasm_i2c_models.py does not replay every vector of it in both bus
  flavours on every path the worker can take to the model, or the model
  powers on with other registers than its twin.

frontend/src/__tests__/i2c-vector-parity.test.ts is the same gate for the
tab models, and also holds the registers the tab tells the Raspberry Pi relay
to ask for (`volatile_reads`, `pointer_stays`) to the files.
"""

import inspect
import json
import unittest
from pathlib import Path

from app.services import esp32_i2c_slaves as slaves
from app.services.esp32_i2c_slaves import (
    BMP280_RULES,
    BMP280Slave,
    DS1307_RULES,
    DS1307Slave,
    DS3231_RULES,
    DS3231Slave,
    MPU6050_RULES,
    MPU6050Slave,
)

from . import test_i2c_slaves as replayers

VECTOR_DIR = Path(__file__).parent.parent.parent / 'fixtures' / 'i2c-vectors'
FLAVOURS = ('repeated-start', 'stop-start')


def _pairs(table: dict) -> dict:
    return {f'{reg:02X}': f'{value:02X}' for reg, value in table.items()}


def _bmp280_rules() -> dict:
    mode = BMP280_RULES['mode']
    return {
        'power_on': _pairs(BMP280_RULES['power_on']),
        'writable': [f'{reg:02X}' for reg in BMP280_RULES['writable']],
        'reset': _pairs(BMP280_RULES['reset']),
        'status': _pairs(BMP280_RULES['status']),
        'mode': {key: f'{mode[key]:02X}' for key in ('register', 'mask', 'sleep', 'normal')},
        'sample': [f'{reg:02X}' for reg in BMP280_RULES['sample']],
    }


def _ds1307_rules() -> dict:
    return {
        'power_on': _pairs(DS1307_RULES['power_on']),
        'write_mask': _pairs(DS1307_RULES['write_mask']),
        'last_register': f'{DS1307_RULES["last_register"]:02X}',
    }


def _ds3231_rules() -> dict:
    return {
        'power_on': _pairs(DS3231_RULES['power_on']),
        'write_mask': _pairs(DS3231_RULES['write_mask']),
        'self_clearing': _pairs(DS3231_RULES['self_clearing']),
        'write_zero_to_clear': _pairs(DS3231_RULES['write_zero_to_clear']),
        'read_only': [[f'{first:02X}', f'{last:02X}']
                      for first, last in DS3231_RULES['read_only']],
        'last_register': f'{DS3231_RULES["last_register"]:02X}',
        'temp_lsb_per_c': DS3231_RULES['temp_lsb_per_c'],
    }


# Every chip with a vector file: its twin, the TestCase of test_i2c_slaves.py
# that replays the file, and the twin's rules table as the file writes it. A
# new vector file fails the gate until it has an entry here.
TWINS = {
    'mpu6050': (MPU6050Slave, 'TestMPU6050Slave', lambda: replayers.rules_as_json(MPU6050_RULES)),
    'bmp280': (BMP280Slave, 'TestBMP280Slave', _bmp280_rules),
    'ds1307': (DS1307Slave, 'TestDS1307Slave', _ds1307_rules),
    'ds3231': (DS3231Slave, 'TestDS3231Slave', _ds3231_rules),
}

# Models in esp32_i2c_slaves.py with no register file to hold to vectors: the
# sink takes whatever a display or an expander is sent and hands it to the tab.
NO_REGISTER_FILE = {'I2CWriteSink'}

FILES = {path.name: json.loads(path.read_text(encoding='utf-8'))
         for path in sorted(VECTOR_DIR.glob('*.json'))}


class TestI2CVectorParity(unittest.TestCase):

    def test_finds_the_vector_files(self):
        self.assertTrue({'bmp280.json', 'ds1307.json', 'ds3231.json', 'mpu6050.json'} <= set(FILES))

    def test_every_vector_file_has_a_twin(self):
        for name, file in FILES.items():
            self.assertEqual(file['device'], name[:-len('.json')], f'{name} names its chip')
            self.assertIn(file['device'], TWINS, f'{name}: no backend twin replays it')
        for device in TWINS:
            self.assertIn(f'{device}.json', FILES, f'{device} has a vector file')

    def test_every_vector_is_replayed_in_both_flavours(self):
        for name, file in FILES.items():
            if file['device'] not in TWINS:
                continue
            case = getattr(replayers, TWINS[file['device']][1])
            for n in range(1, len(file['vectors']) + 1):
                for flavour in FLAVOURS:
                    method = f'test_vector_{n:02d}_{flavour.replace("-", "_")}'
                    self.assertTrue(callable(getattr(case, method, None)),
                                    f'{name}: vector {n} ({flavour}) is not replayed by {case.__name__}')

    def test_the_replayer_reads_the_file_on_disk(self):
        """The module-level copy test_i2c_slaves.py replays is this file."""
        loaded = {v['device']: v for v in vars(replayers).values()
                  if isinstance(v, dict) and 'device' in v and 'vectors' in v}
        for name, file in FILES.items():
            if file['device'] in TWINS:
                self.assertEqual(loaded.get(file['device']), file, name)

    def test_every_twin_exports_the_rules_of_its_file(self):
        for name, file in FILES.items():
            if file['device'] in TWINS:
                self.assertEqual(TWINS[file['device']][2](), file['rules'], name)

    def test_every_register_file_model_has_a_vector_file(self):
        """A class of esp32_i2c_slaves.py that answers the bus is a twin with
        vectors, or listed as having no register file."""
        twins = {cls for cls, _case, _rules in TWINS.values()}
        for name, cls in inspect.getmembers(slaves, inspect.isclass):
            if name.startswith('_') or cls.__module__ != slaves.__name__:
                continue
            if not callable(getattr(cls, 'handle_event', None)):
                continue
            self.assertTrue(cls in twins or name in NO_REGISTER_FILE,
                            f'{name} answers the bus but has no vector file in {VECTOR_DIR.name}')


class TestCompiledModelParity(unittest.TestCase):
    """The compiled models the worker builds from a part's record."""

    @classmethod
    def setUpClass(cls):
        try:
            from app.services import wasm_i2c_models
            from . import test_wasm_i2c_models as runner
        except ImportError as exc:  # no wasmtime in this environment
            raise unittest.SkipTest(f'the compiled models need wasmtime ({exc})')
        cls.models, cls.runner = wasm_i2c_models, runner

    def test_every_compiled_model_has_a_vector_file_and_a_twin(self):
        for device in self.models.SLAVES:
            self.assertIn(f'{device}.json', FILES, f'{device} has a vector file')
            self.assertIn(device, TWINS, f'{device} has a twin to fall back on')
            self.assertIn(device, self.runner.CHIPS, f'{device} is replayed')

    def test_every_vector_is_replayed_in_both_flavours_on_every_path(self):
        case = self.runner.TestWasmModelVectors
        for device in self.models.SLAVES:
            file = FILES[f'{device}.json']
            for n in range(1, len(file['vectors']) + 1):
                for flavour in FLAVOURS:
                    for path in self.runner.PATHS:
                        method = (f'test_{device}_vector_{n:02d}_{flavour.replace("-", "_")}'
                                  f'_{path.replace("-", "_")}')
                        self.assertTrue(callable(getattr(case, method, None)),
                                        f'{device}: vector {n} ({flavour}, {path}) is not replayed')

    def test_every_compiled_model_powers_on_as_its_twin(self):
        clock = lambda: 1_000_000_000_000
        for device, cls in self.models.SLAVES.items():
            wasm = self.runner.WASM[device]
            if device == 'bmp280':
                model, twin = cls(wasm), BMP280Slave()
                first = 0
            else:
                model = cls(wasm, clock=clock)
                twin = TWINS[device][0]({}, clock=clock)
                first = 7   # the time is the host's
            self.assertEqual(bytes(model.dump_registers()[first:]),
                             bytes(twin.dump_registers()[first:]), device)


if __name__ == '__main__':
    unittest.main()
