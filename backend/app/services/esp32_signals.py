"""ESP32 GPIO Matrix output signal source IDs.

Lifted from the ESP32 Technical Reference Manual (Espressif ESP32 TRM
section 4.11, "IO_MUX and GPIO Matrix"). Each output GPIO has a
configuration register `GPIO_FUNCx_OUT_SEL_CFG_REG[x]` whose low 9
bits (`FUNCx_OUT_SEL`) select one of 256 internal peripheral signals
to drive the pin. The constants below name the signals that velxio
actually emulates today; add more here when a new peripheral wants
to participate in the SignalRouter.

The signal ID range mirrors the QEMU plugin's interpretation of
`gpio_out_sel`; the existing worker code at
`esp32_worker.py:_refresh_ledc_gpio_map` already reads these values
out of the matrix via `qemu_picsimlab_get_internals(2)`.
"""

from __future__ import annotations


# ── LEDC (PWM peripheral) ─────────────────────────────────────────────────
# Per ESP32 Technical Reference Manual section 4.11, Table 4-3 (GPIO
# Matrix output signals):
#   71-78 → LEDC_HS_SIG_OUT[0..7]  (high-speed channels 0-7)
#   79-86 → LEDC_LS_SIG_OUT[0..7]  (low-speed  channels 0-7)
#
# The legacy worker code at esp32_worker.py:426 used the off-by-one
# range 72-87; that masked itself because the channel index encoded in
# the 0x5000 duty callback (0..15) was internally consistent with the
# bogus signal-id math, so single-servo demos still appeared to work.
# Multi-servo projects (e.g. solar-tracker, project 5218f9e3) exposed
# the bug — `ledcAttachPin(13, 0)` actually writes signal 71 to
# gpio_out_sel[13], which the off-by-one scan REJECTED, so channel 0
# resolved to GPIO 12 (the next servo's pin, whose signal 72 WAS in
# range and was misinterpreted as channel 0).
SIG_LEDC_HS_CH0_OUT_IDX = 71  # add N for HS channel N (0..7)
SIG_LEDC_HS_CH_LAST     = 78
SIG_LEDC_LS_CH0_OUT_IDX = 79  # add N for LS channel N (0..7)
SIG_LEDC_LS_CH_LAST     = 86


# RMT_SIG_OUT0_IDX per chip, read from the IDF headers shipped in this image
# (components/soc/<chip>/include/soc/gpio_sig_map.h). Add the RMT TX channel
# number to get the signal a GPIO must carry for that channel to reach the pad.
#
# Needed because the WS2812 frames the RMT decoder produces are useless to the
# frontend without the GPIO they went out on: a NeoPixel PART on the canvas is
# keyed by its DIN pin, and an RMT channel number cannot reach one.
RMT_SIG_OUT0_IDX_BY_CHIP = {
    'esp32': 87,
    'esp32-s3': 81,
    'esp32-c3': 51,
}


def rmt_signal_base(machine: str) -> int | None:
    """The chip's RMT_SIG_OUT0_IDX, from a machine string like 'esp32-s3'."""
    if not machine:
        return None
    if 'c3' in machine:
        return RMT_SIG_OUT0_IDX_BY_CHIP['esp32-c3']
    if 's3' in machine:
        return RMT_SIG_OUT0_IDX_BY_CHIP['esp32-s3']
    return RMT_SIG_OUT0_IDX_BY_CHIP['esp32']


# I2CEXTn_SDA_OUT_IDX per chip, by controller (ESP-IDF
# components/soc/<chip>/include/soc/gpio_sig_map.h). The worker reads which
# controller a pad carries off the matrix, because on these chips any GPIO can
# be SDA: `Wire1.begin(25, 26)` is the whole of Wire1's pin assignment, and the
# tab has no table that could say it (project board-buses-2026-09, F5).
I2C_SDA_OUT_IDX_BY_CHIP = {
    'esp32': {30: 0, 96: 1},
    'esp32-s3': {90: 0, 92: 1},
    'esp32-c3': {54: 0},
}


def i2c_sda_signals(machine: str) -> dict[int, int]:
    """Matrix signal id -> I2C controller, for the chip a machine string names."""
    if not machine:
        return {}
    if 'c3' in machine:
        return I2C_SDA_OUT_IDX_BY_CHIP['esp32-c3']
    if 's3' in machine:
        return I2C_SDA_OUT_IDX_BY_CHIP['esp32-s3']
    return I2C_SDA_OUT_IDX_BY_CHIP['esp32']


def ledc_signal_for_channel(channel: int) -> int:
    """Map a velxio-style unified LEDC channel index (0..15) to its
    GPIO Matrix signal source id.

    The ESP32 LEDC hardware has two channel groups: 8 high-speed (HS)
    and 8 low-speed (LS). velxio unifies them into a single 0..15
    space where ch 0-7 = HS, ch 8-15 = LS (matches the encoding the
    QEMU plugin emits on the 0x5000 duty callback).
    """
    if not 0 <= channel < 16:
        raise ValueError(f"ledc channel out of range: {channel}")
    if channel < 8:
        return SIG_LEDC_HS_CH0_OUT_IDX + channel
    return SIG_LEDC_LS_CH0_OUT_IDX + (channel - 8)


def channel_for_ledc_signal(signal_id: int) -> int | None:
    """Inverse of :func:`ledc_signal_for_channel`.  Returns None when
    the signal id is not an LEDC channel."""
    if SIG_LEDC_HS_CH0_OUT_IDX <= signal_id <= SIG_LEDC_HS_CH_LAST:
        return signal_id - SIG_LEDC_HS_CH0_OUT_IDX
    if SIG_LEDC_LS_CH0_OUT_IDX <= signal_id <= SIG_LEDC_LS_CH_LAST:
        return 8 + (signal_id - SIG_LEDC_LS_CH0_OUT_IDX)
    return None


# ── Sentinel for "GPIO not routed to any peripheral" ──────────────────────
# When `gpio_out_sel[N]` carries this value the pin is driven by
# normal GPIO output (the value in the GPIO_OUT_REG bit N), not a
# peripheral signal.
SIG_GPIO_DIRECT_OUT_IDX = 256


__all__ = [
    "SIG_LEDC_HS_CH0_OUT_IDX",
    "SIG_LEDC_HS_CH_LAST",
    "SIG_LEDC_LS_CH0_OUT_IDX",
    "SIG_LEDC_LS_CH_LAST",
    "SIG_GPIO_DIRECT_OUT_IDX",
    "RMT_SIG_OUT0_IDX_BY_CHIP",
    "rmt_signal_base",
    "ledc_signal_for_channel",
    "channel_for_ledc_signal",
]


# UnTXD_OUT_IDX per chip, by controller (the same gpio_sig_map.h files). The
# worker reads which UART a pad carries off the matrix so a chip's RX leg,
# wired to the pad the guest transmits on, follows a `Serial1.begin(9600,
# SERIAL_8N1, 16, 17)` that moved the port (project board-buses-2026-09, F6).
# Only the TX signals: a pad's input select (UnRXD_IN) is not in gpio_out_sel.
UART_TX_OUT_IDX_BY_CHIP = {
    'esp32': {14: 0, 17: 1, 198: 2},
    'esp32-s3': {12: 0, 15: 1, 18: 2},
    'esp32-c3': {6: 0, 9: 1},
}


def uart_tx_signals(machine: str) -> dict[int, int]:
    """Matrix signal id -> UART controller, for the chip a machine string names."""
    if not machine:
        return {}
    if 'c3' in machine:
        return UART_TX_OUT_IDX_BY_CHIP['esp32-c3']
    if 's3' in machine:
        return UART_TX_OUT_IDX_BY_CHIP['esp32-s3']
    return UART_TX_OUT_IDX_BY_CHIP['esp32']


# ── SPI: one numbering at the worker's boundary (project board-buses-2026-09) ─
# The bus map the tab sends names each controller by the SoC's own unit, the
# number the pin tables, the in-browser engines and the datasheets use: GP-SPI2
# (HSPI / FSPI) = 2, GP-SPI3 (VSPI) = 3. QEMU names it otherwise: its
# picsimlab_spi host shim numbers controllers in ATTACH order
# (hw/ssi/picsimlab_spi.c `spi_id++`), and the classic machine attaches spi[2]
# first (esp32_picsimlab.c), so HSPI reports 0 and VSPI 1; the CS irq handler
# reports `n >> 2` over the same two. The S3 machine models only GP-SPI2 and
# attaches one shim (id 0). The C3 machine attaches none today; the entry is
# what it would report. The worker translates QEMU's id into the unit here and
# nowhere else, before anything compares it with the map.
SPI_UNIT_BY_QEMU_ID_BY_CHIP = {
    'esp32': {0: 2, 1: 3},
    'esp32-s3': {0: 2},
    'esp32-c3': {0: 2},
}

# The chip-select OUTPUT signals of each controller, CS0 first (gpio_sig_map.h:
# HSPICS0/1/2 = 11, 61, 62 and VSPICS0/1/2 = 68, 69, 70 on the ESP32; FSPICS0-5
# = 110-115 and SPI3_CS0-2 = 71, 72, 127 on the S3; FSPICS0-5 = 68-73 on the
# C3). A pad whose gpio_out_sel carries one of these is driven by that
# controller's own chip select; any other value (SIG_GPIO_DIRECT_OUT_IDX above
# all) means the pad is a GPIO, and the level the guest writes to it is what a
# device on the wire sees.
SPI_CS_OUT_IDX_BY_CHIP = {
    'esp32': {2: (11, 61, 62), 3: (68, 69, 70)},
    'esp32-s3': {2: (110, 111, 112, 113, 114, 115), 3: (71, 72, 127)},
    'esp32-c3': {2: (68, 69, 70, 71, 72, 73)},
}


def _chip_of(machine: str) -> str:
    if 'c3' in machine:
        return 'esp32-c3'
    if 's3' in machine:
        return 'esp32-s3'
    return 'esp32'


def spi_units(machine: str) -> dict[int, int]:
    """QEMU's picsimlab_spi id -> the SoC's SPI unit, for a machine string."""
    if not machine:
        return {}
    return SPI_UNIT_BY_QEMU_ID_BY_CHIP[_chip_of(machine)]


def spi_cs_signals(machine: str) -> dict[int, tuple[int, ...]]:
    """SPI unit -> its CS0, CS1... output signal ids, for a machine string."""
    if not machine:
        return {}
    return SPI_CS_OUT_IDX_BY_CHIP[_chip_of(machine)]
