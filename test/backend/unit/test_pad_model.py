"""
test_pad_model.py: the resolution of a QEMU worker's pad with a module pull
(app/services/pad_model.py), the tab's tiers (board-buses.md, "Pull resistors
on a line"):

  strong (the guest's output, a hosted chip, the tab's injection) beats a
  module pull; a module pull beats the MCU's internal pull the other way (the
  model never reads it); up and down from two modules leaves the pad; a pad
  with no module pull is not touched.

The model writes through a recorder here, which stands in for
qemu_picsimlab_set_pin; test_board_buses_worker_pads.py runs it inside the
real worker, and test/backend/integration/test_esp32_qemu_pads.py on the real
libqemu with a compiled sketch.
"""
from __future__ import annotations

import unittest

from app.services.pad_model import PadModel

DIO, CLK, FREE = 18, 19, 21


class Rig:
    def __init__(self, skip=()) -> None:
        self.writes: list[tuple[int, int]] = []
        self.moved: list[tuple[int, int]] = []
        self.notes: list[str] = []
        self.pads = PadModel(lambda p, v: self.writes.append((p, v)),
                             on_level=lambda p, v: self.moved.append((p, v)),
                             skip=lambda p: p in skip, log=self.notes.append)

    def pulls(self, **by_pin) -> None:
        """pulls(p18='up') -> the tab's map for GPIO 18."""
        self.pads.set_pulls([{'pin': int(k[1:]), 'pull': v, 'owner': f'mod::{k}~pull'}
                             for k, v in by_pin.items()])

    def last(self, pin: int):
        for p, v in reversed(self.writes):
            if p == pin:
                return v
        return None


class GuestRelease(unittest.TestCase):
    def test_a_pad_the_guest_never_drove_reads_the_pull_when_the_map_arrives(self):
        r = Rig()
        r.pulls(p18='up', p19='down')
        self.assertEqual(r.last(DIO), 1)
        self.assertEqual(r.last(CLK), 0)

    def test_the_guest_releasing_its_low_hands_the_line_to_the_pull_up(self):
        """avishorp's TM1637Display: pinMode(OUTPUT) over a latch at 0 is a
        LOW, pinMode(INPUT) is a 1 the module's 10k makes."""
        r = Rig()
        r.pulls(p18='up')
        r.pads.guest_level(DIO, 0)
        r.pads.guest_dir(DIO, True)
        r.writes.clear()
        r.moved.clear()
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [(DIO, 1)])
        self.assertEqual(r.moved, [(DIO, 1)])

    def test_the_guest_taking_the_pad_writes_nothing_whatever_order_qemu_reports_it_in(self):
        """QEMU writes the latch into the input register itself; the ESP32
        reports the level before the direction, the STM32 after."""
        for level_first in (True, False):
            r = Rig()
            r.pulls(p18='up')
            r.writes.clear()
            if level_first:
                r.pads.guest_level(DIO, 0)
                r.pads.guest_dir(DIO, True)
            else:
                r.pads.guest_dir(DIO, True)
                r.pads.guest_level(DIO, 0)
            self.assertEqual(r.writes, [], f'level_first={level_first}')
            self.assertEqual(r.pads.level(DIO), 0)

    def test_a_guest_output_low_stays_low(self):
        r = Rig()
        r.pulls(p18='up')
        r.writes.clear()
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 0)
        self.assertNotIn((DIO, 1), r.writes)
        self.assertEqual(r.pads.level(DIO), 0)

    def test_a_pull_down_takes_a_released_high_line_low(self):
        r = Rig()
        r.pulls(p18='down')
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 1)
        r.writes.clear()
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [(DIO, 0)])

    def test_a_release_to_the_level_the_pad_already_has_writes_nothing(self):
        r = Rig()
        r.pulls(p18='up')
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 1)
        r.writes.clear()
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [])

    def test_a_pad_with_no_module_pull_is_never_touched(self):
        r = Rig()
        r.pulls(p18='up')
        r.writes.clear()
        r.pads.guest_dir(FREE, True)
        r.pads.guest_level(FREE, 0)
        r.pads.guest_dir(FREE, False)
        r.pads.inject(FREE, 1)
        r.pads.chip_drive(FREE, 'chip', 0)
        r.pads.chip_release(FREE, 'chip')
        self.assertEqual(r.writes, [])


class StrongDrivers(unittest.TestCase):
    def test_an_injection_beats_the_pull_until_the_guest_drives_the_pad(self):
        """A button to GND on a pulled-up line reads LOW; the store lets go of
        a pad when the guest reports it an output, and so does the model."""
        r = Rig()
        r.pulls(p18='up')
        r.pads.inject(DIO, 0)
        r.writes.clear()
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [], 'the injection holds a released pad')
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 0)
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [(DIO, 1)], 'the guest took it; its release is the pull')

    def test_a_chip_holding_the_line_low_beats_the_pull_and_its_release_restores_it(self):
        r = Rig()
        r.pulls(p18='up')
        r.pads.chip_drive(DIO, 'tm1637', 0)
        r.writes.clear()
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [])
        r.pads.chip_release(DIO, 'tm1637')
        self.assertEqual(r.writes, [(DIO, 1)])

    def test_a_chip_release_under_a_guest_output_puts_the_guest_level_back(self):
        """The chip's set_pin overwrote GPIO_IN under the guest's output; the
        guest's drive is the wire's again when the chip lets go."""
        r = Rig()
        r.pulls(p18='up')
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 1)
        r.pads.chip_drive(DIO, 'chip', 0)
        r.writes.clear()
        r.pads.chip_release(DIO, 'chip')
        self.assertEqual(r.writes, [(DIO, 1)])

    def test_two_strong_drivers_that_disagree_leave_the_pad(self):
        r = Rig()
        r.pulls(p18='up')
        r.pads.chip_drive(DIO, 'a', 0)
        r.pads.chip_drive(DIO, 'b', 1)
        r.pads.chip_drive(DIO, 'c', 0)
        r.writes.clear()
        r.pads.chip_release(DIO, 'c')
        self.assertEqual(r.writes, [])
        self.assertIsNone(r.pads.level(DIO))

    def test_a_chip_that_leaves_releases_every_pad_it_held(self):
        r = Rig()
        r.pulls(p18='up', p19='up')
        r.pads.chip_drive(DIO, 'chip', 0)
        r.pads.chip_drive(CLK, 'chip', 0)
        r.writes.clear()
        r.pads.chip_gone('chip')
        self.assertEqual(sorted(r.writes), [(DIO, 1), (CLK, 1)])


class TheMap(unittest.TestCase):
    def test_a_pull_up_and_a_pull_down_from_two_modules_keep_the_level_and_say_so_once(self):
        r = Rig()
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 0)
        r.pads.set_pulls([{'pin': DIO, 'pull': 'up', 'owner': 'a::DIO~pull'},
                          {'pin': DIO, 'pull': 'down', 'owner': 'b::DIO~pull'}])
        r.writes.clear()
        r.pads.guest_dir(DIO, False)
        r.pads.guest_dir(DIO, True)
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [])
        self.assertEqual(len(r.notes), 1, r.notes)

    def test_a_pull_that_goes_away_leaves_the_pad_alone(self):
        r = Rig()
        r.pulls(p18='up')
        r.pads.set_pulls([])
        r.writes.clear()
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 0)
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [])
        self.assertFalse(r.pads.managed(DIO))

    def test_a_map_without_the_half_changes_nothing(self):
        r = Rig()
        r.pulls(p18='up')
        r.pads.set_pulls(None)
        self.assertTrue(r.pads.managed(DIO))

    def test_malformed_entries_are_ignored(self):
        r = Rig()
        r.pads.set_pulls([{'pin': 'x', 'pull': 'up'}, {'pin': 5, 'pull': 'sideways'},
                          {'pin': -1, 'pull': 'up'}, 'nonsense', {'pin': 7, 'pull': 'up'}])
        self.assertEqual(r.pads.pulls(), {7: {'pull@7': 'up'}})

    def test_a_pad_something_else_owns_is_never_written(self):
        """A DHT22's data pin, a keypad wire: its model answers on its own clock."""
        r = Rig(skip={DIO})
        r.pulls(p18='up')
        r.pads.guest_dir(DIO, True)
        r.pads.guest_level(DIO, 0)
        r.pads.guest_dir(DIO, False)
        self.assertEqual(r.writes, [])


if __name__ == '__main__':
    unittest.main()
