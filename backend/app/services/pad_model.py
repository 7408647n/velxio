"""
pad_model.py: the level of a board pad in a QEMU worker, for the pads a
module's pull resistor is on.

QEMU's pin injection (qemu_picsimlab_set_pin) writes a level into the guest's
input register and nothing else: it has no float state and no strength. So
when the guest lets go of a pad (pinMode(INPUT) after driving it), or a chip
hosted in the worker does (vx_pin_set_mode(VX_INPUT)), the register keeps
the last level anybody wrote. On a board a module's resistor takes the line
there: the Grove 4-Digit Display's 10k pull-ups on CLK and DIO are what
avishorp's TM1637Display relies on for every 1 it sends, OneWire and
SoftwareWire with pull-ups off do the same. The tab models that resistor on
its board-pin nets (busNets.setBoardPinPull, board-buses.md "Pull resistors on
a line"), but the guest's register is the worker's, so the tab sends the
pulls of every board pin in the bus map (`pulls`) and this model puts them on
the pad.

Resolution, the tab's tiers:

  1. Strong drivers: the guest's own output (its latch while the pad is an
     output), a chip hosted here driving the pad, a level the tab injected
     (a button, a line model, the SPICE connector). One of them, or several
     that agree, decide the level. Two that disagree are contention and the
     pad is left as the last writer put it.
  2. No strong driver: the module pulls. All up is HIGH, all down is LOW; an
     up and a down from two modules is a divider at mid-rail, no logic level,
     and the pad is left where it was (the tab says so once).
  3. The MCU's own internal pull is never consulted: a module pull on the pad
     beats it either way (the smaller resistor sets the divider), and a pad
     with no module pull is not managed here at all, so everything that was
     true of it before stays true.

An injection is strong until the guest drives the pad as an output, which is
the tab's rule too: the store releases a host-held pad when the guest
reports the pad an output (useSimulatorStore, onPinDir).

The model writes only when the resolved level differs from the level it last
knows the pad at, and only for a managed pad. Everything else is
bookkeeping: the guest's output callback is on the hot path and costs a dict
store here.

Threading: every caller holds QEMU's IO-thread lock (the guest callbacks run
on QEMU's thread; the command loop and the chip timer thread take the lock
before they touch a pin), so the model takes no lock of its own.
"""
from __future__ import annotations

from typing import Callable, Iterable, Optional

UP = 'up'
DOWN = 'down'


class PadModel:
    """Per-worker pad state for the pads a module pull is on.

    Args:
        write:    (pin, level) -> None, puts a level on the guest's input
                  register (qemu_picsimlab_set_pin with the worker's slot).
        on_level: (pin, level) -> None, told after the model itself moved a
                  pad, so the worker can show the level to what reads pads
                  in the worker (chip pin reads and watches, SPI selects).
        skip:     (pin) -> bool, a pad something else in the worker owns
                  outright (a line sensor's data pin, a keypad wire): the
                  model never writes it.
        log:      (str) -> None, for the one-off notes.
    """

    def __init__(
        self,
        write: Callable[[int, int], None],
        on_level: Optional[Callable[[int, int], None]] = None,
        skip: Optional[Callable[[int], bool]] = None,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._write = write
        self._on_level = on_level
        self._skip = skip
        self._log = log or (lambda _msg: None)
        # pin -> {owner: 'up' | 'down'}: the module pulls the tab mapped.
        self._pulls: dict[int, dict[str, str]] = {}
        # What the guest programmed: 1 = output. A pad never reported is an
        # input, as every pad is out of reset.
        self._dir: dict[int, int] = {}
        # The guest's output latch, per pad, as its output callback reported.
        self._latch: dict[int, int] = {}
        # pin -> {chip owner: level}: chips hosted here driving the pad.
        self._chips: dict[int, dict[object, int]] = {}
        # pin -> level the tab injected, until the guest drives the pad.
        self._inject: dict[int, int] = {}
        # The level the model last knows the pad's input register at.
        self._pad: dict[int, int] = {}
        # Pads whose two opposite module pulls were already reported.
        self._fight_noted: set[int] = set()

    # ── the tab's map ──────────────────────────────────────────────────────

    def set_pulls(self, entries: Optional[Iterable[dict]]) -> None:
        """Replace the module pulls with the `pulls` half of the tab's map:
        [{'pin': N, 'pull': 'up' | 'down', 'owner': str}, ...]. None leaves
        them as they are (a map that carries another half only)."""
        if entries is None:
            return
        fresh: dict[int, dict[str, str]] = {}
        for e in entries:
            if not isinstance(e, dict):
                continue
            try:
                pin = int(e.get('pin'))
            except (TypeError, ValueError):
                continue
            pull = e.get('pull')
            if pin < 0 or pull not in (UP, DOWN):
                continue
            owner = str(e.get('owner') or f'pull@{pin}')
            fresh.setdefault(pin, {})[owner] = pull
        changed = [p for p in set(fresh) | set(self._pulls)
                   if fresh.get(p) != self._pulls.get(p)]
        self._pulls = fresh
        for pin in changed:
            self._fight_noted.discard(pin)
            # A pad that lost its last pull is not managed any more and keeps
            # the level it has, which is what an unpulled pad did before.
            self._resolve(pin)

    def managed(self, pin: int) -> bool:
        return pin in self._pulls

    def pulls(self) -> dict[int, dict[str, str]]:
        return {p: dict(o) for p, o in self._pulls.items()}

    # ── the guest ─────────────────────────────────────────────────────────

    def guest_level(self, pin: int, level: int) -> None:
        """The guest's output latch moved (QEMU reports it only for a pad
        that is an output, and writes it into the input register itself)."""
        v = level & 1
        self._latch[pin] = v
        self._pad[pin] = v

    def guest_dir(self, pin: int, output: bool) -> None:
        """The guest made the pad an output (True) or released it (False)."""
        self._dir[pin] = 1 if output else 0
        if output:
            # The host lets go of a pad the guest drives (the tab's rule).
            # Nothing to write: QEMU puts the output latch into the input
            # register itself when the pad becomes an output.
            self._inject.pop(pin, None)
            return
        self._resolve(pin)

    # ── the tab's injections ──────────────────────────────────────────────

    def inject(self, pin: int, level: int) -> None:
        """The tab put a level on the pad; the caller wrote it."""
        v = level & 1
        self._inject[pin] = v
        self._pad[pin] = v

    # ── chips hosted in the worker ────────────────────────────────────────

    def chip_drive(self, pin: int, owner: object, level: int) -> None:
        """A chip drove the pad (vx_pin_write, or an output mode that carries
        a level); the caller wrote it."""
        v = level & 1
        self._chips.setdefault(pin, {})[owner] = v
        self._pad[pin] = v

    def chip_release(self, pin: int, owner: object) -> None:
        """A chip let go of the pad (vx_pin_set_mode to an input)."""
        held = self._chips.get(pin)
        if not held or owner not in held:
            return
        del held[owner]
        if not held:
            del self._chips[pin]
        self._resolve(pin)

    def chip_gone(self, owner: object) -> None:
        """A chip left the worker: every pad it drove is released."""
        for pin in [p for p, held in self._chips.items() if owner in held]:
            self.chip_release(pin, owner)

    # ── resolution ─────────────────────────────────────────────────────────

    def level(self, pin: int) -> Optional[int]:
        """The level a managed pad resolves to now, or None when nothing
        decides one (not managed, contention, or two opposite pulls)."""
        pulls = self._pulls.get(pin)
        if not pulls:
            return None
        strong: set[int] = set()
        if self._dir.get(pin) == 1:
            if pin not in self._latch:
                # An output whose level QEMU has not reported yet (the STM32
                # reports the direction before the level): the guest drives
                # it, and QEMU writes that level in a moment.
                return None
            strong.add(self._latch[pin])
        held = self._chips.get(pin)
        if held:
            strong.update(held.values())
        if pin in self._inject:
            strong.add(self._inject[pin])
        if strong:
            return next(iter(strong)) if len(strong) == 1 else None
        kinds = set(pulls.values())
        if len(kinds) != 1:
            if pin not in self._fight_noted:
                self._fight_noted.add(pin)
                self._log(f'[pads] GPIO {pin}: a pull-up and a pull-down from two modules '
                          f'({", ".join(sorted(pulls))}) and nothing driving: the pad '
                          'keeps its level')
            return None
        return 1 if UP in kinds else 0

    def _resolve(self, pin: int) -> None:
        if self._skip is not None and self._skip(pin):
            return
        target = self.level(pin)
        if target is None or self._pad.get(pin) == target:
            return
        self._pad[pin] = target
        self._write(pin, target)
        if self._on_level is not None:
            self._on_level(pin, target)

    def resolve_all(self) -> None:
        """Re-assert every managed pad (after something outside the model
        wrote pads it manages, such as a reset)."""
        self._pad.clear()
        for pin in list(self._pulls):
            self._resolve(pin)
