"""
test_board_buses_f4_worker.py: the ESP32 QEMU worker's SPI bus table
(project/board-buses-2026-09, F4-SPEC "Protocolo").

The worker runs unmodified, as esp32_lib_manager starts it, on the same rig as
test_board_buses_repro_worker.py: libqemu is replaced at the ctypes boundary
and the test plays the guest by calling the worker's own callbacks the way the
C side does. Every model is real, compiled with the production chip flags.

What is under test is the arbitration itself, which F4 moved from a walk of
"whoever registered first" into one chip-select table beside the guest:

  - the MISO of a byte is the answer of the ONE selected responder;
  - several selected is the wired-AND plus a contention diagnostic, which is
    what a real bus with two drivers reads back;
  - none selected is the line's idle level;
  - a responder pinned to a controller does not answer on another one;
  - a chip select the peripheral drives itself is heard through its own event,
    not through the GPIO channel, which QEMU never moves for that pad;
  - and the byte reaches the browser either way, because the worker does not
    know what SINKS the tab has.

Wiring is the repro suite's, so the two files describe one board.
"""
from __future__ import annotations

import pytest

from .test_board_buses_repro_worker import (  # noqa: F401  (worker fixture)
    CHIP_A_CS,
    CHIP_B_CS,
    EPAPER_SINK,
    EPD_CS,
    EPD_DC,
    MISO,
    MOSI,
    SCK,
    TFT_CS,
    TFT_DC,
    TOUCH_CS,
    Worker,
    _clock_tft,
    _wasm,
    worker,
)

CASET_CMD = [0x2A]
CASET_DATA = [0x00, 0x00, 0x00, 0xEF]
PIXELS = [(i * 37) & 0xFF for i in range(640)]

# A second chip select the SPI peripheral could drive itself. CS0 is the index
# the ESP32's SPI bridge reports for the first one.
HW_CS0 = 0

# The two numberings of the classic ESP32's general-purpose SPI controllers:
# the SoC's units, which the tab's bus map uses, and the ids QEMU's
# picsimlab_spi shim reports them by (attach order, spi[2] first).
HSPI, VSPI = 2, 3
QEMU_HSPI, QEMU_VSPI = 0, 1
# gpio_sig_map.h: VSPICS0_OUT_IDX, and the "plain GPIO output" out_sel value.
VSPICS0_OUT = 68
GPIO_OUT = 256


def probe_entry(owner: str, cs: dict, sig: int, bus_id=None) -> dict:
    """The bus-map entry the tab sends for a responder with a portable model.

    spi-probe answers `sig` for the first byte of a transaction, `sig + 1` for
    the second, and so on, and logs what it received when its select rises. Two
    of them with different `sig` values say, in the MISO alone, which one
    answered a byte.
    """
    pin_map = {'SCK': SCK, 'MOSI': MOSI, 'MISO': MISO}
    if 'gpio' in cs:
        pin_map['CS'] = int(cs['gpio'])
    return {
        'owner': owner,
        'bus_id': bus_id,
        'cs': cs,
        'model': {
            'wasm_b64': _wasm('spi-probe'),
            'pin_map': pin_map,
            'attrs': {'sig': sig},
            'blobs': {},
        },
    }


def pin_cs(gpio: int) -> dict:
    return {'kind': 'pin', 'gpio': gpio, 'active_low': True}


def free_entry(owner: str, cs: dict, sig: int, bus_id=None) -> dict:
    """A responder with NO select line of its own
    (fixtures/board_buses_worker/free-probe.c). The bus table alone decides
    whether it is clocked, which is what the `none` and `const` selects are:
    nothing in the circuit ever moves a pin for them."""
    return {
        'owner': owner,
        'bus_id': bus_id,
        'cs': cs,
        'model': {
            'wasm_b64': _wasm('free-probe'),
            'pin_map': {'SCK': SCK, 'MOSI': MOSI, 'MISO': MISO},
            'attrs': {'sig': sig},
            'blobs': {},
        },
    }


def hw_select(w: Worker) -> None:
    """Assert the peripheral's own chip select, from its idle level.

    The deassert first is what the silicon does (CS idles high between
    transactions) and what the model needs: a chip arms on the FALLING edge of
    its select, so a first event that is already low is not an edge at all.
    """
    w.cs(HW_CS0, 1)
    w.cs(HW_CS0, 0)


def select(w: Worker, gpio: int) -> None:
    """Assert a GPIO chip select, from a known deselected state.

    The high first is not ceremony: a pad the guest has never driven has no
    level at all, and the worker reads that as NOT selected, because a chip
    whose select nobody drives does not answer on a bench either."""
    w.pin(gpio, 1)
    w.pin(gpio, 0)


def diags(w: Worker, code: str | None = None) -> list[dict]:
    return [e for e in w.events('bus_diag') if code is None or e.get('code') == code]


# ── The table: one selected, several, none ──────────────────────────────────


class TestChipSelectArbitration:
    """F4-SPEC: selected = the mapped responders whose CS is active. One: its
    MISO. Several: the AND plus a contention event. None: 0xFF."""

    def test_setup_two_responders_load_and_each_hears_its_own_select(self, worker):
        """The rig: both models are in the worker and each one's watch fires on
        its own pin, so an idle answer below cannot come from a chip that never
        loaded."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0xB0)]})
        assert w.chip_log().count('spi probe ready') == 2
        for cs in (CHIP_A_CS, CHIP_B_CS):
            w.pin(cs, 1)
            w.pin(cs, 0)
            w.pin(cs, 1)
        assert w.wait_for(lambda: any(t.startswith('probe a0 rx=') for t in w.chip_log()))
        assert w.wait_for(lambda: any(t.startswith('probe b0 rx=') for t in w.chip_log()))

    def test_the_selected_responder_answers_and_the_other_one_does_not(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0xB0)]})
        w.pin(CHIP_A_CS, 1)
        w.pin(CHIP_B_CS, 1)
        w.pin(CHIP_A_CS, 0)
        assert w.spi([0x11, 0x22, 0x33]) == [0xA0, 0xA1, 0xA2]
        w.pin(CHIP_A_CS, 1)
        w.pin(CHIP_B_CS, 0)
        assert w.spi([0x11, 0x22, 0x33]) == [0xB0, 0xB1, 0xB2]
        w.pin(CHIP_B_CS, 1)
        assert w.wait_for(lambda: 'probe a0 rx=11 22 33' in w.chip_log(), 2.0)
        assert w.wait_for(lambda: 'probe b0 rx=11 22 33' in w.chip_log(), 2.0)

    def test_nothing_selected_reads_the_lines_idle_level(self, worker):
        """A bus whose devices are all deselected reads its pull-up, not the
        last answer anyone gave (D-012)."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        select(w, CHIP_A_CS)
        assert w.spi([0x11]) == [0xA0]
        w.pin(CHIP_A_CS, 1)
        assert w.spi([0x11, 0x22]) == [0xFF, 0xFF]

    def test_a_chip_select_moved_between_two_bytes_is_seen_on_the_second(self, worker):
        """Selection is kept on edges, not looked up per byte, so the edge has
        to land between the two bytes and not after both."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        w.pin(CHIP_A_CS, 1)
        first = w.spi([0x11])
        w.pin(CHIP_A_CS, 0)
        second = w.spi([0x22])
        assert (first, second) == ([0xFF], [0xA0])

    def test_a_responder_with_no_select_line_is_always_on_the_bus(self, worker):
        """cs `none` is a chip with no select leg (a 74HC595). It answers from
        the first byte, with nobody driving anything."""
        w = worker(bus_map={'spi': [free_entry('always', {'kind': 'none'}, 0x5A)]})
        assert w.spi([0x11, 0x22]) == [0x5A, 0x5B]

    def test_a_select_tied_to_a_rail_is_read_from_the_map(self, worker):
        """cs `const` is a select wired to GND or VCC: no pin ever moves, so
        the only place its level exists is the map."""
        w = worker(bus_map={'spi': [free_entry('gnd', {'kind': 'const', 'active': True}, 0x11),
                                    free_entry('vcc', {'kind': 'const', 'active': False}, 0x22)]})
        assert w.spi([0x00, 0x00]) == [0x11, 0x12]


class TestContention:
    """Two responders selected at once: the guest reads the wired-AND and the
    tab is told, once."""

    def test_two_selected_responders_read_back_the_wired_and(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0x0F),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0x33)]})
        select(w, CHIP_A_CS)
        select(w, CHIP_B_CS)
        assert w.spi([0x00]) == [0x0F & 0x33]

    def test_the_contention_is_reported_once_with_both_owners(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0x0F),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0x33)]})
        select(w, CHIP_A_CS)
        select(w, CHIP_B_CS)
        w.spi([0x00] * 8)
        assert w.wait_for(lambda: bool(diags(w, 'spi-contention')), 2.0)
        events = diags(w, 'spi-contention')
        assert len(events) == 1, f'one line per contention, not per byte: {events}'
        assert events[0]['owners'] == ['a', 'b']

    def test_one_selected_responder_is_not_a_contention(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0x0F),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0x33)]})
        select(w, CHIP_A_CS)
        w.pin(CHIP_B_CS, 1)
        w.spi([0x00] * 8)
        w.sync()
        assert diags(w, 'spi-contention') == []


class TestHardwareChipSelect:
    """A select the SPI peripheral drives itself. QEMU moves no GPIO for that
    pad, so the op 0x01 event is the only place its level exists."""

    def test_the_peripheral_select_selects_the_responder(self, worker):
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0)
        w = worker(bus_map={'spi': [entry]})
        assert w.spi([0x11]) == [0xFF], 'idle until the peripheral asserts its select'
        hw_select(w)
        assert w.spi([0x11, 0x22]) == [0xC0, 0xC1]
        w.cs(HW_CS0, 1)
        assert w.spi([0x33]) == [0xFF]

    def test_the_peripheral_select_reaches_the_models_own_watch(self, worker):
        """The chip arms on its select falling. With a peripheral-driven pad
        there is no GPIO edge to arm it, so the worker has to hand the model
        the edge it would see on a bench."""
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0)
        w = worker(bus_map={'spi': [entry]})
        hw_select(w)
        w.spi([0x11, 0x22])
        w.cs(HW_CS0, 1)
        assert w.wait_for(lambda: 'probe c0 rx=11 22' in w.chip_log(), 2.0), w.chip_log()

    def test_the_table_alone_gates_a_model_that_is_always_armed(self, worker):
        """A chip with no select line of its own (free-probe) is armed from
        the moment it loads, so the ONLY thing deciding whether it answers is
        this table's reading of the peripheral's chip select. Without this the
        model's own arming hides the table: a table that called every
        hardware select asserted would still look right, because a chip that
        arms on a falling edge has nothing to say until it gets one."""
        w = worker(bus_map={'spi': [
            free_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'active_low': True}, 0x70)]})
        assert w.spi([0x11]) == [0xFF], 'the peripheral has not asserted its select'
        w.cs(HW_CS0, 0)
        assert w.spi([0x11, 0x22]) == [0x70, 0x71]
        w.cs(HW_CS0, 1)
        assert w.spi([0x33]) == [0xFF]

    def test_a_gpio_on_that_pad_does_not_select_it_while_the_peripheral_drives_it(self, worker):
        """The matrix routes the pad to the peripheral's CS0 (setHwCs(true)):
        the pad belongs to the peripheral, and a level the tab injects on it as
        a plain GPIO must not be read as a chip select."""
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.guest('matrix', out_sel={str(CHIP_A_CS): VSPICS0_OUT})
        w.pin(CHIP_A_CS, 1)
        w.pin(CHIP_A_CS, 0)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xFF]
        w.cs(HW_CS0, 1, bus=QEMU_VSPI)
        w.cs(HW_CS0, 0, bus=QEMU_VSPI)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xC0]

    def test_a_pad_the_sketch_drives_as_a_gpio_follows_the_gpio(self, worker):
        """GPIO 5 is VSPI's CS0 in the pin table, so the tab sends it as `hw`,
        but SD.begin(5) drives it with digitalWrite and the matrix leaves the
        pad a GPIO (signal 256). The peripheral still toggles its own CS0
        around every byte, on no pad at all. The device follows the pad:
        selected while the GPIO is low, whatever CS0 does, and deselected while
        it is high even inside a peripheral transaction (the 74 dummy clocks an
        SD card needs with CS high). This is the 2026-09-27 velxio.dev
        regression: the card followed CS0 and never saw its command frame."""
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.guest('matrix', out_sel={str(CHIP_A_CS): GPIO_OUT})
        w.pin(CHIP_A_CS, 1)
        w.cs(HW_CS0, 0, bus=QEMU_VSPI)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xFF], 'CS0 asserted, the pad is high'
        w.cs(HW_CS0, 1, bus=QEMU_VSPI)
        w.pin(CHIP_A_CS, 0)
        for _ in range(3):
            # The per-byte transactions arduino-esp32 makes: CS0 around each.
            w.cs(HW_CS0, 0, bus=QEMU_VSPI)
            w.spi([0x22], bus=QEMU_VSPI)
            w.cs(HW_CS0, 1, bus=QEMU_VSPI)
        w.pin(CHIP_A_CS, 1)
        # One frame: the model's own select watch heard the GPIO, not the
        # three CS0 pulses, so it logs the three bytes as one transaction.
        assert w.wait_for(lambda: 'probe c0 rx=22 22 22' in w.chip_log(), 2.0), w.chip_log()

    def test_without_a_matrix_a_pad_written_as_a_gpio_follows_the_gpio(self, worker):
        """An older libqemu cannot show the matrix. A pad the guest wrote as a
        GPIO then follows what it wrote; one it never wrote follows the
        peripheral (the tests above that start from a fresh pad)."""
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.pin(CHIP_A_CS, 1)
        w.cs(HW_CS0, 0, bus=QEMU_VSPI)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xFF]
        w.pin(CHIP_A_CS, 0)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xC0]

    def test_hspi_cs0_does_not_select_a_device_on_vspi(self, worker):
        """Both controllers report their CS0 as index 0; the controller is what
        tells them apart."""
        entry = probe_entry('hw', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0xC0, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.guest('matrix', out_sel={str(CHIP_A_CS): VSPICS0_OUT})
        w.cs(HW_CS0, 1, bus=QEMU_HSPI)
        w.cs(HW_CS0, 0, bus=QEMU_HSPI)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xFF]
        w.cs(HW_CS0, 1, bus=QEMU_VSPI)
        w.cs(HW_CS0, 0, bus=QEMU_VSPI)
        assert w.spi([0x11], bus=QEMU_VSPI) == [0xC0]


class TestBusId:
    """A board with two SPI controllers: a device is on ONE of them.

    The map names a controller by the SoC's unit (HSPI = 2, VSPI = 3 on the
    classic ESP32, what the tab's pin tables say); QEMU's events name it by
    the attach order of its host shim (HSPI = 0, VSPI = 1). The worker
    translates once, at the callback; a map in QEMU's numbering is wrong."""

    def test_a_responder_pinned_to_a_controller_ignores_the_other_one(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0, bus_id=HSPI),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0xB0, bus_id=VSPI)]})
        select(w, CHIP_A_CS)
        select(w, CHIP_B_CS)
        assert w.spi([0x00], bus=QEMU_HSPI) == [0xA0]
        assert w.spi([0x00], bus=QEMU_VSPI) == [0xB0]

    def test_the_only_selected_responder_is_still_deaf_to_the_other_controller(self, worker):
        # One selected model is served by a shortcut that skips the table (a
        # streamed card sector is 515 bytes of it). The shortcut must not skip
        # the controller check with it: this chip is on VSPI and hears
        # nothing clocked on HSPI.
        w = worker(bus_map={'spi': [probe_entry('b', pin_cs(CHIP_B_CS), 0xB0, bus_id=VSPI)]})
        select(w, CHIP_B_CS)
        assert w.spi([0x00], bus=QEMU_HSPI) == [0xFF]
        assert w.spi([0x00], bus=QEMU_VSPI) == [0xB0]

    def test_the_tabs_vspi_is_qemus_bus_1(self, worker):
        """The exact entry the tab sent on velxio.dev (bus_id 3) against the
        id QEMU reports VSPI bytes with (1). Before the translation the two
        never matched and the card answered nothing."""
        w = worker(bus_map={'spi': [probe_entry('sd', pin_cs(CHIP_A_CS), 0x5D, bus_id=VSPI)]})
        select(w, CHIP_A_CS)
        assert w.spi([0x00, 0x00], bus=QEMU_VSPI) == [0x5D, 0x5E]

    def test_a_write_only_block_reaches_the_controller_that_clocked_it(self, worker):
        """esp32_spi.c passes a literal 0 as the id of every write-only batch,
        whichever controller clocked it. The block belongs to the controller of
        the last event, not to HSPI: arduino-esp32 writes an SD command in bulk
        on VSPI."""
        w = worker(bus_map={'spi': [probe_entry('sd', pin_cs(CHIP_A_CS), 0x5D, bus_id=VSPI)]})
        select(w, CHIP_A_CS)
        w.spi([0x01], bus=QEMU_VSPI)
        w.batch([0x40, 0x00, 0x00, 0x00, 0x00, 0x95])
        w.pin(CHIP_A_CS, 1)
        assert w.wait_for(lambda: 'probe 5d rx=01 40 00 00 00 00 95' in w.chip_log(), 2.0), \
            w.chip_log()

    def test_two_responders_on_different_controllers_are_not_a_contention(self, worker):
        """Both are selected, but a byte on one controller never reaches the
        other's chip, so there is nothing to fight over."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0x0F, bus_id=HSPI),
                                    probe_entry('b', pin_cs(CHIP_B_CS), 0x33, bus_id=VSPI)]})
        select(w, CHIP_A_CS)
        select(w, CHIP_B_CS)
        assert w.spi([0x00] * 4, bus=QEMU_HSPI) == [0x0F, 0x10, 0x11, 0x12]
        w.sync()
        assert diags(w, 'spi-contention') == []

    def test_a_responder_that_names_no_controller_answers_on_any(self, worker):
        """The tab cannot always tell which peripheral a wire ends on; a null
        controller means "wherever the bytes come from", which is the honest
        answer and the one that keeps a one-controller board working."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        select(w, CHIP_A_CS)
        assert w.spi([0x00], bus=QEMU_HSPI) == [0xA0]
        assert w.spi([0x00], bus=QEMU_VSPI) == [0xA1]


class TestPeripheralSelectStaysInTheWorker:
    """The peripheral asserts CS0-CS2 around every transaction whether or not
    a pad carries them. The tab reads such an edge as its CS pad moving, so
    it hears one only when the matrix routes that select to a pad."""

    def test_an_unrouted_select_is_not_relayed(self, worker):
        entry = probe_entry('sd', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0x5D, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.guest('matrix', out_sel={str(CHIP_A_CS): GPIO_OUT})
        for idx in (0, 1, 2):
            w.cs(idx, 0, bus=QEMU_VSPI)
            w.cs(idx, 1, bus=QEMU_VSPI)
        w.flush()
        assert w.events('spi_event') == []

    def test_a_routed_select_is_relayed_in_the_maps_numbering(self, worker):
        entry = probe_entry('sd', {'kind': 'hw', 'index': HW_CS0, 'gpio': CHIP_A_CS,
                                   'active_low': True}, 0x5D, bus_id=VSPI)
        w = worker(bus_map={'spi': [entry]})
        w.guest('matrix', out_sel={str(CHIP_A_CS): VSPICS0_OUT})
        w.cs(HW_CS0, 0, bus=QEMU_VSPI)
        w.flush()
        assert w.events('spi_event') == [{'type': 'spi_event', 'bus': VSPI, 'event': 0x01}]


class TestTheBusIsNotSwallowed:
    """Whatever answers a byte, every SINK in the tab still gets it: the worker
    does not know what the tab has, so it forwards the traffic and the tab's
    own fabric decides by chip select."""

    def test_a_selected_responder_does_not_keep_the_tft_bytes_from_the_tab(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        select(w, CHIP_A_CS)
        _clock_tft(w)
        assert w.spi_stream() == bytes(CASET_CMD + CASET_DATA + PIXELS)

    def test_a_deselected_responder_does_not_either(self, worker):
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        w.pin(CHIP_A_CS, 1)
        _clock_tft(w)
        assert w.spi_stream() == bytes(CASET_CMD + CASET_DATA + PIXELS)

    def test_a_mapped_responder_leaves_a_tab_sink_its_bytes(self, worker):
        """A sink the tab keeps (the e-paper panel decodes there) and a
        responder the tab sent are members of the same table; neither shuts
        the other out: the responder answers, and the byte still reaches the
        tab because the sink's select is low."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0), EPAPER_SINK]})
        select(w, CHIP_A_CS)
        w.pin(EPD_CS, 0)
        w.pin(EPD_DC, 0)
        assert w.spi([0x20]) == [0xA0], 'the responder answers, the sink does not drive MISO'
        w.pin(EPD_CS, 1)
        w.flush()
        assert w.spi_stream() == bytes([0x20]), 'the sink in the tab got the byte'

    def test_a_bulk_write_reaches_every_selected_member_and_the_tab(self, worker):
        """The block path has to end where byte-by-byte ends: the browser gets
        the same bytes, in order, under the sink's select."""
        w = worker(bus_map={'spi': [probe_entry('a', pin_cs(CHIP_A_CS), 0xA0), EPAPER_SINK]})
        w.pin(CHIP_A_CS, 1)
        w.pin(EPD_CS, 0)
        w.pin(EPD_DC, 1)
        w.batch(PIXELS)
        w.pin(EPD_DC, 0)
        w.batch([0x20])
        w.pin(EPD_CS, 1)
        w.flush()
        assert w.spi_stream() == bytes(PIXELS + [0x20])


class TestBadEntries:
    """A map the tab got wrong must not take the bus down with it."""

    def test_an_entry_without_a_model_is_skipped_and_the_rest_load(self, worker):
        w = worker(bus_map={'spi': [
            {'owner': 'nomodel', 'bus_id': None, 'cs': pin_cs(CHIP_B_CS), 'model': {}},
            probe_entry('a', pin_cs(CHIP_A_CS), 0xA0),
        ]})
        select(w, CHIP_A_CS)
        assert w.spi([0x11]) == [0xA0]

    def test_a_model_that_will_not_load_is_skipped_and_the_rest_load(self, worker):
        bad = probe_entry('bad', pin_cs(CHIP_B_CS), 0x00)
        bad['model']['wasm_b64'] = 'bm90IHdhc20='  # "not wasm"
        w = worker(bus_map={'spi': [bad, probe_entry('a', pin_cs(CHIP_A_CS), 0xA0)]})
        select(w, CHIP_A_CS)
        assert w.spi([0x11]) == [0xA0]
