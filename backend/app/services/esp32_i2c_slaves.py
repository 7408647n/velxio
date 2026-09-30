"""
esp32_i2c_slaves.py — Standalone I2C slave state machines for ESP32 QEMU simulation.

Each class emulates the I2C register map of a real sensor, handling the picsimlab
I2C event protocol as defined in hw/i2c/picsimlab_i2c.c:

  picsimlab_i2c_ev(event)  → passes raw QEMU i2c_event enum value:
    0x00 = I2C_START_RECV  — firmware doing requestFrom  (read  direction START)
    0x01 = I2C_START_SEND  — firmware doing beginTransmission (write direction START)
    0x02 = I2C_START_SEND_ASYNC  (rarely used)
    0x03 = I2C_FINISH      — end of transaction (STOP or RSTART between write+read)
    0x04 = I2C_NACK

  picsimlab_i2c_tx(data)   → event = (data << 8) | (I2C_NACK+1) = (data<<8)|0x05
  picsimlab_i2c_rx()       → event = I2C_NACK+2 = 0x06  (return data byte to firmware)

ACK convention (matches QEMU i2c core):
  return 0  → ACK  (success, device present / byte accepted)
  return ≠0 → NACK (error)
  For READ events: return value is the data byte delivered to the firmware.
"""

import math as _math
import re as _re
import threading as _threading
import time as _time


# ── Protocol constants ────────────────────────────────────────────────────────

I2C_START_RECV = 0x00   # firmware called requestFrom  (read  direction START)
I2C_START_SEND = 0x01   # firmware called beginTransmission (write direction START)
I2C_FINISH     = 0x03   # end of transaction (STOP or repeated-START between phases)
I2C_WRITE      = 0x05   # firmware sent a byte; data = (event >> 8) & 0xFF
I2C_READ       = 0x06   # firmware requesting a byte; return the data byte


# ── MPU-6050 IMU ──────────────────────────────────────────────────────────────

# What the MPU-6050 does with a byte written to it, and how it turns motion
# into counts, as one table. It is MPU6050_RULES of the tab model
# (frontend/src/simulation/parts/ProtocolParts.ts) and the `rules` of
# test/fixtures/i2c-vectors/mpu6050.json, which the tests hold both copies
# against. Sections are those of the register map, RM-MPU-6000A-00 rev 4.2.
MPU6050_RULES = {
    # Every register powers on at 0x00 but these: the factory trims, asleep,
    # and its id (section 3; AN-OFFS 7.2). The accelerometer's factory trims
    # (OTP) are not zero on any part, and the low bit of each low byte is not
    # a trim but the product revision, which InvenSense's eMPL mpu_init()
    # reads (bit 0 of 0x07, 0x09, 0x0B; 2 is a part at full sensitivity, 0
    # fails with -6).
    'power_on': {0x06: 0xFA, 0x07: 0x38, 0x08: 0x04, 0x09: 0xB3, 0x0A: 0x05, 0x0B: 0xDC,
                 0x6B: 0x40, 0x75: 0x68},
    # Inclusive ranges a write leaves as they are: I2C_MST_STATUS, INT_STATUS,
    # the sample block with the external sensor data behind it, FIFO_COUNT
    # and WHO_AM_I (sections 4.13, 4.16 to 4.20, 4.30 and 4.32).
    'read_only': ((0x36, 0x36), (0x3A, 0x3A), (0x3B, 0x60), (0x72, 0x73), (0x75, 0x75)),
    # Bits that start something and are never stored, so the next read finds
    # them at 0. All of SIGNAL_PATH_RESET, which is write-only (4.26). The
    # resets of USER_CTRL (4.27) and its bit 3, where i2cdevlib and InvenSense's
    # own driver reset the DMP; i2cdevlib sets one bit at a time with a
    # read-modify-write, so a bit that stuck would fire again on every later
    # write. DEVICE_RESET in PWR_MGMT_1 (4.28).
    'self_clearing': {0x68: 0xFF, 0x6A: 0x0F, 0x6B: 0x80},
    # Counts per g by AFS_SEL (4.17) and per degree per second by FS_SEL
    # (4.19). The table and not 131 / 2^n, which gives 32.75 and 16.375: the
    # drivers divide by 32.8 and 16.4.
    'accel_lsb_per_g': (16384, 8192, 4096, 2048),
    'gyro_lsb_per_dps': (131, 65.5, 32.8, 16.4),
    # What one count of the offset registers weighs. The gyro offsets
    # XG/YG/ZG_OFFS_USR (0x13-0x18) are in the +-1000 deg/s format, 32.8 per
    # deg/s (AN-OFFS 6.2). The accelerometer trims (0x06-0x0B) at 2048 per g,
    # the +-16 g format, bit 0 left out: the application note says +-8 g, but
    # the calibrations that converge on real parts write in +-16 g units
    # (i2cdevlib PID(): reading at +-2 g / 8; Luis Rodenas'
    # MPU6050_calibration the same), and at +-8 g their loop gain would be 2
    # and never settle. Decision O3, pending a bench measurement. An offset
    # counts from the factory trim, so the chip at power-on reads the panel.
    'gyro_offset_lsb_per_dps': 32.8,
    'accel_offset_lsb_per_g': 2048,
    # TEMP_OUT = (T - 36.53) * 340 (4.18).
    'temp_lsb_per_c': 340,
    'temp_offset_c': 36.53,
    # What another die of the family changes, selected by the record's
    # `variant`. The MPU-9250 (the Grove IMU 9DOF v2.0 and 10DOF bricks)
    # answers WHO_AM_I 0x71 and reads TEMP_OUT = (T - 21) * 333.87
    # (RM-MPU-9250A-00 rev 1.6, sections 4.22 and 4.39; PS-MPU-9250A-01
    # 3.4.2). Its AK8963 magnetometer is not modelled.
    'variants': {
        'mpu9250': {'who_am_i': 0x71, 'temp_lsb_per_c': 333.87, 'temp_offset_c': 21},
    },
    # Bits a read of the register takes with it: "each bit will clear after
    # the register is read" (4.16). With INT_RD_CLEAR set in INT_PIN_CFG, a
    # read of any register clears them (4.14).
    'clear_on_read': {0x3A: 0xFF},
    # The gyroscope output rate the sample rate is divided from, in Hz: 8 kHz
    # with the low-pass filter off (DLPF_CFG 0 or 7), 1 kHz with it on.
    # Sample rate = rate / (1 + SMPLRT_DIV) (4.2, 4.3).
    'gyro_rate_hz': {'dlpf_off': 8000, 'dlpf_on': 1000},
    # With CYCLE set (and SLEEP clear) the chip wakes at LP_WAKE_CTRL
    # (PWR_MGMT_2 bits 7:6) to take one sample and sleeps between (4.28, 4.29).
    'cycle_rate_hz': (1.25, 5, 20, 40),
    # How long INT stays active per interrupt with LATCH_INT_EN clear (4.14).
    'int_pulse_us': 50,
    # Bytes the FIFO holds; past that the oldest go and FIFO_OFLOW_INT is set
    # (4.31, PS 7.17).
    'fifo_size': 1024,
    # The DMP memory behind BANK_SEL (0x6D), MEM_START_ADDR (0x6E) and
    # MEM_R_W (0x6F): 32 banks of 256 bytes, the bank field of BANK_SEL being
    # 5 bits (i2cdevlib setMemoryBank, InvenSense eMPL mpu_write_mem).
    # Undocumented in the register map.
    'dmp_banks': 32,
    # DMP memory that is not zero at power-on, by bank * 256 + address: the
    # hardware revision i2cdevlib's dmpInitialize() reads at user bank 16,
    # byte 6. Parts report 0xA5 or 0x4D there (jrowberg/i2cdevlib issues 246
    # and 371); 0xA5 is the one in its MotionApps comments.
    'dmp_rom': {0x1006: 0xA5},
}

# Motion and temperature at the chip, under the names of the panel's sliders
# (g, degrees per second, degrees Celsius), which are the names the sensor
# record and its updates carry. At rest on the bench, as the panel starts.
MPU6050_INPUTS = {
    'accelX': 0.0, 'accelY': 0.0, 'accelZ': 1.0,
    'gyroX': 0.0, 'gyroY': 0.0, 'gyroZ': 0.0,
    'temp': 24.0,
}

# The keyword names update() had before it took the record's.
_MPU_INPUT_NAMES = {
    **{name: name for name in MPU6050_INPUTS},
    'accel_x': 'accelX', 'accel_y': 'accelY', 'accel_z': 'accelZ',
    'gyro_x': 'gyroX', 'gyro_y': 'gyroY', 'gyro_z': 'gyroZ',
}

_MPU_XA_OFFS        = 0x06   # three signed words of accelerometer trim
_MPU_XG_OFFS_USR    = 0x13   # three signed words of gyro offset
_MPU_SMPLRT_DIV     = 0x19
_MPU_CONFIG         = 0x1A
_MPU_GYRO_CONFIG    = 0x1B
_MPU_ACCEL_CONFIG   = 0x1C
_MPU_INT_PIN_CFG    = 0x37
_MPU_INT_LEVEL      = 0x80   # active low, against active high
_MPU_INT_OPEN       = 0x40   # open drain, against push-pull
_MPU_LATCH_INT_EN   = 0x20   # held until cleared, against a pulse
_MPU_INT_RD_CLEAR   = 0x10
_MPU_INT_ENABLE     = 0x38
_MPU_INT_STATUS     = 0x3A
_MPU_DATA_RDY_INT   = 0x01
_MPU_FIFO_OFLOW_INT = 0x10
# The interrupt sources the model raises, as bits of INT_ENABLE and INT_STATUS.
_MPU_INT_SOURCES    = _MPU_DATA_RDY_INT | _MPU_FIFO_OFLOW_INT
_MPU_FIFO_EN        = 0x23
# The FIFO_EN bits in the order their data enters the FIFO, which is the
# order of the registers (4.6, 4.31): ACCEL (0x3B-0x40), TEMP, XG, YG, ZG, as
# (bit, offset in the sample block, bytes). SLV0-2 push the external sensor
# data of the auxiliary master, which the model has none of.
_MPU_FIFO_SOURCES   = ((0x08, 0, 6), (0x80, 6, 2), (0x40, 8, 2), (0x20, 10, 2), (0x10, 12, 2))
_MPU_FIFO_COUNT_H   = 0x72
_MPU_FIFO_COUNT_L   = 0x73
_MPU_FIFO_R_W       = 0x74
# USER_CTRL: the FIFO takes samples, and the trigger that empties it (4.27).
_MPU_USER_FIFO_EN   = 0x40
_MPU_FIFO_RESET     = 0x04
# The DMP memory port: bank, address in the bank, and the byte there.
_MPU_BANK_SEL       = 0x6D
_MPU_MEM_START_ADDR = 0x6E
_MPU_MEM_R_W        = 0x6F
# ACCEL_XOUT_H to GYRO_ZOUT_L: three axes, the die temperature, three axes.
_MPU_SAMPLE_FIRST   = 0x3B
_MPU_SAMPLE_LAST    = 0x48
_MPU_SAMPLE_SIZE    = _MPU_SAMPLE_LAST - _MPU_SAMPLE_FIRST + 1
_MPU_USER_CTRL      = 0x6A
_MPU_SIG_COND_RESET = 0x01
_MPU_PWR_MGMT_1     = 0x6B
_MPU_DEVICE_RESET   = 0x80
_MPU_SLEEP          = 0x40
_MPU_CYCLE          = 0x20
_MPU_TEMP_DIS       = 0x08
# PWR_MGMT_2: LP_WAKE_CTRL in bits 7:6, then STBY_XA, YA, ZA, XG, YG, ZG
# (4.29). The standby bits as the fields of the sample block they freeze
# (ax, ay, az, temp, gx, gy, gz: bit n is field n).
_MPU_PWR_MGMT_2     = 0x6C
_MPU_STBY_FIELDS    = ((0x20, 0x01), (0x10, 0x02), (0x08, 0x04),
                       (0x04, 0x10), (0x02, 0x20), (0x01, 0x40))
_MPU_TEMP_FIELD     = 0x08
_MPU_ALL_FIELDS     = 0x7F

_MPU_READ_ONLY = bytearray(256)
for _first, _last in MPU6050_RULES['read_only']:
    _MPU_READ_ONLY[_first:_last + 1] = b'\x01' * (_last - _first + 1)
_MPU_SELF_CLEARING = bytearray(256)
for _reg, _mask in MPU6050_RULES['self_clearing'].items():
    _MPU_SELF_CLEARING[_reg] = _mask
# Registers the pointer stays on after each byte: FIFO_R_W reads and writes
# the FIFO, one byte per access (4.31).
_MPU_POINTER_STAYS = bytearray(256)
_MPU_POINTER_STAYS[_MPU_FIFO_R_W] = 1
# MEM_R_W moves the memory address instead, so a DMP upload bursts into the
# memory and not over FIFO_COUNT and WHO_AM_I behind it.
_MPU_POINTER_STAYS[_MPU_MEM_R_W] = 1
_MPU_CLEAR_ON_READ = bytearray(256)
for _reg, _mask in MPU6050_RULES['clear_on_read'].items():
    _MPU_CLEAR_ON_READ[_reg] = _mask
_MPU_INT_PULSE_NS = MPU6050_RULES['int_pulse_us'] * 1000


def parse_ad0(value):
    """The `ad0` property as a level: True (high), False (low), or None when
    it says nothing and the AD0 net decides. parseAd0 in the tab's model
    reads the property the same way; both are held to the cases of
    test/fixtures/i2c-vectors/mpu6050.json."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return True if value == 1 else False if value == 0 else None
    if not isinstance(value, str):
        return None
    v = value.strip().lower()
    if v in ('1', 'true', 'high', 'on', 'vcc'):
        return True
    if v in ('0', 'false', 'low', 'off', 'gnd'):
        return False
    return None


def parse_variant(value) -> str:
    """The `variant` property (parseMpuVariant in the tab): a die of
    MPU6050_RULES['variants'], spelled with or without the dash, or the
    MPU-6050."""
    if not isinstance(value, str):
        return 'mpu6050'
    v = value.strip().lower().replace('-', '')
    return v if v in MPU6050_RULES['variants'] else 'mpu6050'


def mpu6050_address(record: dict) -> int:
    """The address a sensor record puts the chip at. The tab resolves the
    AD0 net and sends `addr`; a record without it (an older tab, a hand-made
    config) is read by its `ad0` property, low when that says nothing."""
    addr = record.get('addr')
    if addr is not None:
        return int(addr) & 0x7F
    return 0x69 if parse_ad0(record.get('ad0')) else 0x68


def _mpu_word(regs, reg: int) -> int:
    """The signed big-endian word at `reg`."""
    raw = (regs[reg] << 8) | regs[reg + 1]
    return raw - 0x10000 if raw & 0x8000 else raw


# The accelerometer trims as words, by the register of their high byte.
_MPU_FACTORY_TRIM = {
    at: _mpu_word([MPU6050_RULES['power_on'].get(r, 0) for r in range(256)], at)
    for at in (_MPU_XA_OFFS, _MPU_XA_OFFS + 2, _MPU_XA_OFFS + 4)
}


def _mpu_counts(value: float) -> int:
    """A physical value as the counts of a 16-bit output register.

    Half a count rounds away from zero, so a tilt one way and the same tilt
    the other way read the same size (round() sends 65.5 to 66 and 196.5 to
    196), and what does not fit stays at the end of the scale, as the
    converter's output does. Rounded first and held to the scale after: half
    a count under positive full scale rounds to 32768, which is one more than
    the register holds and would read as the negative end.
    """
    size = abs(value)
    if size < 32768:
        counts = _math.floor(size)
        if size - counts >= 0.5:
            counts += 1
    else:
        # Infinity included, which floor() does not take: a finite input
        # times a sensitivity can overflow.
        counts = 32768
    return min(counts, 32767) if value >= 0 else -counts


class MPU6050Slave:
    """MPU-6050 6-axis IMU (address 0x68 or 0x69), modelled where a driver can
    tell the difference from the chip. The twin of VirtualMPU6050 in the tab:
    both replay test/fixtures/i2c-vectors/mpu6050.json.

      - It powers on asleep (PWR_MGMT_1 = 0x40), so a sketch that reads
        without waking it reads zeros, as it does on the bench.
      - DEVICE_RESET puts every register back to its power-on value and is
        gone before the next read. Adafruit_MPU6050::reset() polls that bit
        with no timeout.
      - What the panel sets is the world around the chip, not a register: it
        survives a reset, and the sample block 0x3B-0x48 is worked out from it
        and from the full-scale ranges the sketch selected, when a read
        begins. The whole burst is answered from that one sample (4.17), so a
        slider moving while it is read cannot mix two instants.
      - Asleep, the block holds what it held when the chip fell asleep. So
        does an axis in standby (PWR_MGMT_2) and the temperature with
        TEMP_DIS; in CYCLE mode the block moves only at each wake-up, at
        LP_WAKE_CTRL.
      - Awake, it takes a sample every sample period of the guest's time
        (now_ns), whether the sketch talks to it or not. Each one sets
        DATA_RDY_INT when DATA_RDY_EN is set, which a read of INT_STATUS
        clears, and moves the INT pad the way INT_PIN_CFG says (int_pad,
        int_wake_ns). Nothing runs in the background for it: the samples due
        are counted when the chip is next looked at, from the time that has
        passed. With no clock (a libqemu that does not export it) one period
        passes per register pointer the sketch writes, so a driver that waits
        for DATA_RDY still finds it.
      - With USER_CTRL.FIFO_EN set, each sample also pushes the sources
        FIFO_EN selects into a 1024-byte FIFO, in register order. FIFO_COUNT
        is latched when its high byte is read, and FIFO_R_W pops a byte per
        read without moving the pointer (an empty FIFO repeats the last one).
      - BANK_SEL, MEM_START_ADDR and MEM_R_W reach 32 banks of DMP memory,
        the address advancing within its bank and the pointer staying on
        MEM_R_W, with the ROM byte i2cdevlib reads as the hardware revision.
        The DMP itself does not run: an image written there produces no
        packets.
    """

    def __init__(self, addr: int = 0x68, now_ns=None, variant: str = 'mpu6050'):
        self.addr       = addr
        # WHO_AM_I and the temperature line of the die.
        self._die = MPU6050_RULES['variants'].get(parse_variant(variant)) or {
            'who_am_i': MPU6050_RULES['power_on'][0x75],
            'temp_lsb_per_c': MPU6050_RULES['temp_lsb_per_c'],
            'temp_offset_c': MPU6050_RULES['temp_offset_c'],
        }
        # The guest's clock, in ns, as a callable: what the chip measures its
        # sample period on. A worker hands over what it reads the guest's
        # time from (QEMU_CLOCK_VIRTUAL); None is a host that keeps no time.
        # It is never the host's clock: a guest that waits 40 ms has to find
        # 40 ms of samples however slowly the emulator ran them.
        self._now_ns    = now_ns
        self.regs       = bytearray(256)
        self.reg_ptr    = 0
        self.first_byte = True
        # Replaced whole by update(), never changed in place: the panel moves
        # on the worker's command thread while QEMU's thread reads.
        self._inputs = dict(MPU6050_INPUTS)
        # The sample the read in progress is answered from.
        self._sample = bytes(_MPU_SAMPLE_SIZE)
        # What the fields that do not sample hold: the block as it was when
        # SLEEP, a standby bit or TEMP_DIS stopped them, or at the last CYCLE
        # wake-up. Zeros after power-on and reset.
        self._held = bytes(_MPU_SAMPLE_SIZE)
        # No START was heard for the read that comes next: latch on its first byte.
        self._latch_due = True
        # Called when what the INT pad does, or the time it moves next, may
        # have changed: the worker that drives the pin reads int_pad() and
        # int_wake_ns() again.
        self.on_int_change = None
        # The guest time the sample periods are counted from, in ns: when the
        # chip woke up or its rate changed. None until the chip is next
        # looked at.
        self._epoch_ns = None
        self._taken = 0           # samples since the epoch
        self._period_ns = 0.0     # the period _taken was counted with
        self._pulse_end_ns = None # where the INT pulse of the last sample ends
        self._fifo = bytearray()
        self._fifo_last = 0       # what an empty FIFO answers: the byte read last
        self._dmp_mem = bytearray(MPU6050_RULES['dmp_banks'] * 256)
        # The panel moves on the worker's command thread and QEMU's thread
        # runs the bus: the samples due are taken of the world before a move,
        # under one lock with the bus events.
        self._lock = _threading.RLock()
        self._power_on()

    @staticmethod
    def address_of(record: dict) -> int:
        """The address a worker's sensor record puts the chip at (see
        mpu6050_address)."""
        return mpu6050_address(record)

    def handle_event(self, event: int) -> int:
        with self._lock:
            result = self._handle_event(event)
        cb = self.on_int_change
        if cb is not None and (event & 0xFF) != I2C_FINISH:
            cb()
        return result

    def _handle_event(self, event: int) -> int:
        op   = event & 0xFF          # low byte = operation type
        data = (event >> 8) & 0xFF   # high byte = data byte (for WRITE)

        if op in (I2C_START_RECV, I2C_START_SEND):
            # reg_ptr is NOT reset here — a write-then-read (repeated START)
            # relies on reg_ptr having been set by the preceding WRITE phase.
            self.first_byte = True
            self._sync()
            if op == I2C_START_RECV:
                self._latch()
            return 0   # ACK (0 = success in QEMU convention)

        elif op == I2C_WRITE:
            self._latch_due = True
            if self.first_byte:
                # First byte after START is the register address pointer
                self.reg_ptr    = data
                self.first_byte = False
                # Every register access starts with a pointer, on every host
                # and in both ways a repeated START is delivered: where there
                # is no time to read, this is the chip's tick.
                if self._now_ns is None:
                    self._tick()
                else:
                    self._sync()
            else:
                self._sync()
                reg = self.reg_ptr
                self.reg_ptr = reg if _MPU_POINTER_STAYS[reg] else (reg + 1) & 0xFF
                self._write_register(reg, data)
            return 0   # ACK

        elif op == I2C_READ:
            self._sync()
            if self._latch_due:
                self._latch()
            reg = self.reg_ptr
            self.reg_ptr = reg if _MPU_POINTER_STAYS[reg] else (reg + 1) & 0xFF
            value = self._read_register(reg)
            # What the read takes with it goes at the read, not at the FINISH:
            # QEMU ends a transfer before a repeated START.
            cleared = (0xFF if self.regs[_MPU_INT_PIN_CFG] & _MPU_INT_RD_CLEAR
                       else _MPU_CLEAR_ON_READ[reg])
            if cleared:
                self.regs[_MPU_INT_STATUS] &= ~cleared & 0xFF
            return value

        else:                         # I2C_FINISH, I2C_NACK, unknown
            # The pointer survives: i2cdevlib writes it in one transaction and
            # reads in the next, and QEMU ends every write phase this way, the
            # one before a repeated START included (hw/i2c/esp32_i2c.c,
            # I2C_OPCODE_RSTART calls i2c_end_transfer).
            self.first_byte = True
            self._latch_due = True
            return 0

    def update(self, /, **inputs) -> None:
        """The panel moved. Only the values it names change, and no register
        does: the sample block is worked out when a read begins.

        Takes a sensor record or an update as they arrive (accelX ... gyroZ in
        g and degrees per second, temp in degrees Celsius); whatever else the
        record carries, and anything that is not a finite number, is left out.
        """
        with self._lock:
            # The samples due until now were taken of the world as it was.
            self._sync()
            self._set_inputs(inputs)

    def _set_inputs(self, inputs: dict) -> None:
        changed = dict(self._inputs)
        for name, value in inputs.items():
            key = _MPU_INPUT_NAMES.get(name)
            if key is None or isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            try:
                number = float(value)
            except OverflowError:
                continue
            if _math.isfinite(number):
                changed[key] = number
        self._inputs = changed

    def inputs(self) -> dict:
        return dict(self._inputs)

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now: the sample block
        encoded from the panel's values (or what a sleeping chip holds), and
        no trigger bit. Nothing is cleared by it: it is not a read on the bus."""
        with self._lock:
            self._sync()
            out = bytearray(self.regs)
            out[_MPU_SAMPLE_FIRST:_MPU_SAMPLE_LAST + 1] = self._output()
            # The count as it stands, not as a read of the high byte last
            # latched it.
            out[_MPU_FIFO_COUNT_H] = len(self._fifo) >> 8
            out[_MPU_FIFO_COUNT_L] = len(self._fifo) & 0xFF
            return out

    def board_reset(self) -> None:
        """The MCU was reset: the chip kept its supply and goes on sampling,
        but the guest's clock started again, so the periods are counted from
        where it stands now."""
        with self._lock:
            self._restart_sampling()

    def int_pad(self) -> str:
        """What the INT pad does at this instant: 'high', 'low', or 'z' when
        an open-drain pad lets go. Active is high or low by INT_LEVEL; an
        open-drain pad (INT_OPEN) only ever pulls low (4.14)."""
        with self._lock:
            self._sync()
            cfg = self.regs[_MPU_INT_PIN_CFG]
            high = self._int_active() != bool(cfg & _MPU_INT_LEVEL)
            if not high:
                return 'low'
            return 'z' if cfg & _MPU_INT_OPEN else 'high'

    def int_wake_ns(self):
        """The guest time, in ns, at which the pad moves next with nobody
        touching the chip: the end of the pulse under way, or the next sample
        that raises an enabled interrupt. None when nothing is due: a latched
        interrupt waits for the sketch, and so does a chip with no clock."""
        with self._lock:
            # Asleep, nothing moves: no sample, and going to sleep ended any
            # pulse. The clock is not asked either (see _restart_sampling).
            if self._asleep():
                return None
            self._sync()
            now = self._now()
            if now is None:
                return None
            if self._latched():
                if self._int_active():
                    return None
            elif self._pulse_end_ns is not None and now < self._pulse_end_ns:
                return self._pulse_end_ns
            if self._epoch_ns is None:
                return None
            if not self.regs[_MPU_INT_ENABLE] & _MPU_INT_SOURCES:
                return None
            return self._epoch_ns + (self._taken + 1) * self._period_ns

    def _asleep(self) -> bool:
        return (self.regs[_MPU_PWR_MGMT_1] & _MPU_SLEEP) != 0

    def _latched(self) -> bool:
        return bool(self.regs[_MPU_INT_PIN_CFG] & _MPU_LATCH_INT_EN)

    def _int_active(self) -> bool:
        if self._latched():
            return bool(self.regs[_MPU_INT_STATUS] & self.regs[_MPU_INT_ENABLE]
                        & _MPU_INT_SOURCES)
        now = self._now()
        return (self._pulse_end_ns is not None and now is not None
                and now < self._pulse_end_ns)

    def _now(self):
        """The guest's time in ns, or None where the host keeps none."""
        if self._now_ns is None:
            return None
        return self._now_ns()

    def _sample_period_ns(self) -> float:
        """The sample period the registers select, in ns (4.2)."""
        if self._cycling():
            return 1e9 / MPU6050_RULES['cycle_rate_hz'][self.regs[_MPU_PWR_MGMT_2] >> 6]
        dlpf = self.regs[_MPU_CONFIG] & 0x07
        rate = MPU6050_RULES['gyro_rate_hz']['dlpf_off' if dlpf in (0, 7) else 'dlpf_on']
        return (1 + self.regs[_MPU_SMPLRT_DIV]) * 1e9 / rate

    def _restart_sampling(self) -> None:
        """The sample periods are counted from this instant: the chip woke
        up, its rate changed, or its clock started again. Where the time
        cannot be read yet, from the next look at the chip."""
        # Asleep, the clock is not asked at all: a worker builds the twin
        # before QEMU is initialised.
        self._epoch_ns = None if self._asleep() else self._now()
        self._taken = 0
        self._period_ns = self._sample_period_ns()
        self._pulse_end_ns = None

    def _sync(self) -> None:
        """Take the samples that came due since the chip was last looked at.
        Called before anything reads or changes what a sample depends on, so
        every sample is taken of the registers and the world of its instant."""
        if self._asleep():
            return
        now = self._now()
        if now is None:
            self._epoch_ns = None
            return
        if self._epoch_ns is None or now < self._epoch_ns:
            # A clock that was not there when the chip woke, or one that
            # started again: the first sample is one period from here.
            self._epoch_ns = now
            self._taken = 0
            return
        due = int((now - self._epoch_ns) // self._period_ns)
        if due <= self._taken:
            return
        n = due - self._taken
        self._taken = due
        self._sampled(n, self._epoch_ns + due * self._period_ns)

    def _tick(self) -> None:
        """One sample period passed, on a host where nothing measures it."""
        if not self._asleep():
            self._sampled(1, None)

    def _sampled(self, n: int, at_ns) -> None:
        """`n` samples were taken, the last of them at `at_ns` of the guest's
        time. Only an enabled source raises its status bit: the register map
        does not say whether a disabled one latches (4.15, 4.16), and every
        driver that polls DATA_RDY enables it first."""
        # A CYCLE wake-up samples what is not in standby, and holds it until
        # the next.
        if self._cycling():
            live = self._encode()
            stby = self._standby_fields()
            held = bytearray(self._held)
            for f in range(7):
                if not stby & (1 << f):
                    held[2 * f:2 * f + 2] = live[2 * f:2 * f + 2]
            self._held = bytes(held)
        events = _MPU_DATA_RDY_INT
        if self._fifo_samples(n):
            events |= _MPU_FIFO_OFLOW_INT
        raised = self.regs[_MPU_INT_ENABLE] & _MPU_INT_SOURCES & events
        if not raised:
            return
        self.regs[_MPU_INT_STATUS] |= raised
        # A pulse has a length only where there is a clock to measure it on.
        if at_ns is not None and not self._latched():
            self._pulse_end_ns = at_ns + _MPU_INT_PULSE_NS

    def _fifo_samples(self, n: int) -> bool:
        """Push `n` samples of the sources FIFO_EN selects, while USER_CTRL
        lets the FIFO take them. Returns whether bytes were lost to a full
        FIFO. The samples of one call are of one instant, so only as many as
        can still be in the FIFO afterwards are pushed."""
        if not self.regs[_MPU_USER_CTRL] & _MPU_USER_FIFO_EN:
            return False
        sources = self.regs[_MPU_FIFO_EN]
        if not sources:
            return False
        block = self._output()
        packet = bytearray()
        for bit, at, size in _MPU_FIFO_SOURCES:
            if sources & bit:
                packet += block[at:at + size]
        if not packet:
            return False
        cap = MPU6050_RULES['fifo_size']
        lost = len(self._fifo) + n * len(packet) > cap
        pushes = min(n, -(-cap // len(packet)) + 1)
        self._fifo += packet * pushes
        if len(self._fifo) > cap:
            del self._fifo[:len(self._fifo) - cap]
        return lost

    def _fifo_push(self, value: int) -> None:
        """One byte into the FIFO; when it is full the oldest goes (4.31)."""
        self._fifo.append(value)
        if len(self._fifo) > MPU6050_RULES['fifo_size']:
            del self._fifo[0]

    def _fifo_pop(self) -> int:
        if self._fifo:
            self._fifo_last = self._fifo.pop(0)
        return self._fifo_last

    def _power_on(self) -> None:
        self.regs[:] = bytes(256)
        for reg, value in MPU6050_RULES['power_on'].items():
            self.regs[reg] = value
        self.regs[0x75] = self._die['who_am_i']
        self._held = bytes(_MPU_SAMPLE_SIZE)
        self._fifo = bytearray()
        self._fifo_last = 0
        self._dmp_mem[:] = bytes(len(self._dmp_mem))
        for at, value in MPU6050_RULES['dmp_rom'].items():
            self._dmp_mem[at] = value
        self._restart_sampling()

    def _dmp_cell(self) -> int:
        """Where MEM_R_W reads or writes, and the address moved on past it.
        The address wraps within its bank: eMPL refuses to cross one and
        i2cdevlib selects the next bank itself."""
        at = ((self.regs[_MPU_BANK_SEL] & 0x1F) << 8) | self.regs[_MPU_MEM_START_ADDR]
        self.regs[_MPU_MEM_START_ADDR] = (self.regs[_MPU_MEM_START_ADDR] + 1) & 0xFF
        return at % len(self._dmp_mem)

    def _read_register(self, reg: int) -> int:
        if reg == _MPU_FIFO_COUNT_H:
            # Both bytes are latched when the high one is read (4.30).
            self.regs[_MPU_FIFO_COUNT_H] = len(self._fifo) >> 8
            self.regs[_MPU_FIFO_COUNT_L] = len(self._fifo) & 0xFF
        if reg == _MPU_FIFO_R_W:
            return self._fifo_pop()
        if reg == _MPU_MEM_R_W:
            return self._dmp_mem[self._dmp_cell()]
        if _MPU_SAMPLE_FIRST <= reg <= _MPU_SAMPLE_LAST:
            return self._sample[reg - _MPU_SAMPLE_FIRST]
        return self.regs[reg]

    def _write_register(self, reg: int, value: int) -> None:
        if _MPU_READ_ONLY[reg]:
            return
        if reg == _MPU_PWR_MGMT_1 and value & _MPU_DEVICE_RESET:
            # Nothing of the byte is kept, SLEEP and CLKSEL included: Adafruit's
            # read-modify-write sends 0xC0 and then has to read 0x40.
            self._power_on()
            return
        # SIG_COND_RESET clears the sensor registers too (4.27), which shows on
        # a sleeping chip; one that is awake has a new sample by the next read.
        if reg == _MPU_USER_CTRL and value & _MPU_SIG_COND_RESET:
            self._held = bytes(_MPU_SAMPLE_SIZE)
        # FIFO_RESET empties it whether or not it is enabled: the register map
        # says "while FIFO_EN equals 0", and i2cdevlib resets it enabled and
        # counts on it (MotionApps resetFIFO).
        if reg == _MPU_USER_CTRL and value & _MPU_FIFO_RESET:
            self._fifo = bytearray()
        if reg == _MPU_FIFO_R_W:
            self._fifo_push(value)
            return
        if reg == _MPU_MEM_R_W:
            self._dmp_mem[self._dmp_cell()] = value
            return
        stored = value & ~_MPU_SELF_CLEARING[reg] & 0xFF
        # What sampled until now holds this instant from here on if the write
        # stops it; what did not keeps what it held.
        if reg in (_MPU_PWR_MGMT_1, _MPU_PWR_MGMT_2):
            self._held = self._output()
        was_asleep = self._asleep()
        self.regs[reg] = stored
        # Waking up, or another rate: the first sample is one period from here.
        if self._asleep() != was_asleep or self._sample_period_ns() != self._period_ns:
            self._restart_sampling()

    def _latch(self) -> None:
        self._sample = self._output()
        self._latch_due = False

    def _cycling(self) -> bool:
        """CYCLE with SLEEP clear: one sample per wake-up (4.28)."""
        return self.regs[_MPU_PWR_MGMT_1] & (_MPU_CYCLE | _MPU_SLEEP) == _MPU_CYCLE

    def _standby_fields(self) -> int:
        """The fields of the sample block that are not sampling, as bits
        (ax = bit 0)."""
        fields = 0
        stby = self.regs[_MPU_PWR_MGMT_2]
        for bit, field in _MPU_STBY_FIELDS:
            if stby & bit:
                fields |= field
        if self.regs[_MPU_PWR_MGMT_1] & _MPU_TEMP_DIS:
            fields |= _MPU_TEMP_FIELD
        return fields

    def _output(self) -> bytes:
        """The sample block as the chip's registers hold it now."""
        frozen = (_MPU_ALL_FIELDS if self._asleep() or self._cycling()
                  else self._standby_fields())
        if frozen == _MPU_ALL_FIELDS:
            return bytes(self._held)
        out = bytearray(self._encode())
        for f in range(7):
            if frozen & (1 << f):
                out[2 * f:2 * f + 2] = self._held[2 * f:2 * f + 2]
        return bytes(out)

    def _encode(self) -> bytes:
        inputs = self._inputs
        accel = MPU6050_RULES['accel_lsb_per_g'][(self.regs[_MPU_ACCEL_CONFIG] >> 3) & 3]
        gyro  = MPU6050_RULES['gyro_lsb_per_dps'][(self.regs[_MPU_GYRO_CONFIG] >> 3) & 3]
        regs = self.regs

        # Offsets act before the output registers, the FIFO and the DMP
        # (AN-OFFS 4).
        def trim(reg: int) -> float:
            return (((_mpu_word(regs, reg) & ~1) - (_MPU_FACTORY_TRIM[reg] & ~1)) * accel
                    / MPU6050_RULES['accel_offset_lsb_per_g'])

        def drift(reg: int) -> float:
            return _mpu_word(regs, reg) * gyro / MPU6050_RULES['gyro_offset_lsb_per_dps']

        block = (
            inputs['accelX'] * accel + trim(_MPU_XA_OFFS),
            inputs['accelY'] * accel + trim(_MPU_XA_OFFS + 2),
            inputs['accelZ'] * accel + trim(_MPU_XA_OFFS + 4),
            (inputs['temp'] - self._die['temp_offset_c']) * self._die['temp_lsb_per_c'],
            inputs['gyroX'] * gyro + drift(_MPU_XG_OFFS_USR),
            inputs['gyroY'] * gyro + drift(_MPU_XG_OFFS_USR + 2),
            inputs['gyroZ'] * gyro + drift(_MPU_XG_OFFS_USR + 4),
        )
        out = bytearray()
        for value in block:
            counts = _mpu_counts(value) & 0xFFFF
            out.append(counts >> 8)
            out.append(counts & 0xFF)
        return bytes(out)


# ── BMP280 Barometric Pressure + Temperature Sensor ───────────────────────────

# What the BMP280 holds at power-on and what it does with a byte written to
# it, as one table. It is BMP280_RULES of the tab model
# (frontend/src/simulation/I2CBusManager.ts) and the `rules` of
# test/fixtures/i2c-vectors/bmp280.json, which the tests hold both copies
# against. Sections are those of the datasheet, BST-BMP280-DS001 rev 1.26.
BMP280_RULES = {
    # Every register powers on at 0x00 but the id and the msb of the two data
    # words, which hold 0x80000 until a measurement replaces it (4.2, table
    # 18). The calibration block 0x88-0x9F is the part's own.
    'power_on': {0xD0: 0x58, 0xF7: 0x80, 0xFA: 0x80},
    # The registers a write changes: ctrl_meas and config. The calibration,
    # the id, status and the data registers are read-only, and the rest of the
    # map is reserved (4.2, the "Type" row of table 18).
    'writable': (0xF4, 0xF5),
    # The reset register (4.3.2) keeps nothing and reads 0x00. This one word
    # runs the power-on reset, any other does nothing.
    'reset': {0xE0: 0xB6},
    # Bits the chip sets by itself in status (4.3.3). `measuring` is 1 while a
    # conversion runs: the first read after one starts finds it, the next does
    # not. im_update, bit 0, is up for the NVM copy that is over before a
    # master can ask.
    'status': {0xF3: 0x08},
    # mode[1:0] of ctrl_meas (3.6, table 10): 01 and 10 are both forced mode.
    'mode': {'register': 0xF4, 'mask': 0x03, 'sleep': 0, 'normal': 3},
    # press and temp, 20 bits each, msb first (4.3.6, 4.3.7).
    'sample': (0xF7, 0xFC),
}

# Pressure and temperature at the chip, under the names of the panel's sliders
# (hPa, degrees Celsius), which are the names the sensor record and its
# updates carry. Where the panel starts.
BMP280_INPUTS = {'temperature': 24.0, 'pressure': 1013.25}

_BMP_RESET        = 0xE0
_BMP_RESET_WORD   = BMP280_RULES['reset'][_BMP_RESET]
_BMP_STATUS       = 0xF3
_BMP_MEASURING    = BMP280_RULES['status'][_BMP_STATUS]
_BMP_CTRL_MEAS    = BMP280_RULES['mode']['register']
_BMP_MODE         = BMP280_RULES['mode']['mask']
_BMP_SLEEP        = BMP280_RULES['mode']['sleep']
_BMP_NORMAL       = BMP280_RULES['mode']['normal']
_BMP_SAMPLE_FIRST, _BMP_SAMPLE_LAST = BMP280_RULES['sample']


def _bmp_number(value) -> 'float | None':
    """A value of a sensor record as a finite number, or None for anything
    else. A number typed into a property dialog may arrive as its text, which
    the worker always took."""
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    return number if _math.isfinite(number) else None


class BMP280Slave:
    """BMP280 (address 0x76 or 0x77), modelled where a driver can tell the
    difference from the chip. The twin of VirtualBMP280 in the tab: both
    replay test/fixtures/i2c-vectors/bmp280.json.

      - It powers on in sleep mode and measures nothing there (3.6.1): until
        the sketch selects a mode, the data registers hold their reset value
        0x80000.
      - Forced mode is one measurement, and the chip is back in sleep mode
        when it is done (3.6.2). A conversion takes no time here, so the mode
        bits read 00 at once and the measurement is what the panel said at
        the write. esp-idf-lib and M5Unit-ENV wait for those bits, BMP280_DEV
        starts the next conversion only from sleep mode.
      - In normal mode the chip measures by itself (3.6.3): a read finds what
        the panel says when it begins, and the whole burst is answered from
        that one measurement (3.10), so a slider moving while it is read
        cannot mix two of them.
      - `measuring` reads 1 once after a mode write that starts a conversion.
        SparkFun's and pocketBME280's examples wait for it to rise with no
        timeout, Adafruit's takeForcedMeasurement() waits for it to fall.
        Without a clock the cycles of normal mode that follow are not seen in
        status.
      - A write is pairs of register address and register data, and the
        address does not count up (5.2.1, figure 7). A read counts up from
        the last address written (5.2.2).
      - What the panel sets is the world around the chip, not a register: it
        survives a soft reset.

    Uses BMP280 datasheet Section 8.2 example calibration constants.
    Implements Bosch compensation formulas with binary-search inversion
    to find raw ADC values from the desired temperature / pressure.
    """

    # Section 8.2 calibration constants
    DIG_T1 =  27504; DIG_T2 =  26435; DIG_T3 =   -1000
    DIG_P1 =  36477; DIG_P2 = -10685; DIG_P3 =    3024
    DIG_P4 =   2855; DIG_P5 =    140; DIG_P6 =      -7
    DIG_P7 =  15500; DIG_P8 = -14600; DIG_P9 =    6000

    def __init__(self, addr: int = 0x76):
        self.addr       = addr
        self.regs       = bytearray(256)
        self.reg_ptr    = 0
        # The next byte written is a register address.
        self.first_byte = True
        self._temp_c    = BMP280_INPUTS['temperature']
        self._press_hpa = BMP280_INPUTS['pressure']
        # What a measurement taken now puts in the data registers. Replaced
        # whole by update(), never changed in place: the panel moves on the
        # worker's command thread while QEMU's thread reads.
        self._live = bytes(_BMP_SAMPLE_LAST - _BMP_SAMPLE_FIRST + 1)
        # A conversion started and status has not been read since.
        self._measuring = False
        # The data registers hold a measurement and not their reset value.
        self._measured = False
        # No START was heard for the read that comes next: latch on its first byte.
        self._latch_due = True
        self._init_calibration()
        self._power_on()
        self._update_measurements()

    # ── calibration register layout ───────────────────────────────────────────
    def _wu16(self, a: int, v: int) -> None:
        self.regs[a] = v & 0xFF; self.regs[a + 1] = (v >> 8) & 0xFF

    def _ws16(self, a: int, v: int) -> None:
        self._wu16(a, v & 0xFFFF)

    def _init_calibration(self) -> None:
        self._wu16(0x88, self.DIG_T1); self._ws16(0x8A, self.DIG_T2); self._ws16(0x8C, self.DIG_T3)
        self._wu16(0x8E, self.DIG_P1); self._ws16(0x90, self.DIG_P2); self._ws16(0x92, self.DIG_P3)
        self._ws16(0x94, self.DIG_P4); self._ws16(0x96, self.DIG_P5); self._ws16(0x98, self.DIG_P6)
        self._ws16(0x9A, self.DIG_P7); self._ws16(0x9C, self.DIG_P8); self._ws16(0x9E, self.DIG_P9)

    # ── Bosch compensation formulas ───────────────────────────────────────────
    def _t_fine(self, adc_t: int) -> int:
        v1 = (((adc_t >> 3) - (self.DIG_T1 << 1)) * self.DIG_T2) >> 11
        s  = (adc_t >> 4) - self.DIG_T1
        v2 = ((s * s >> 12) * self.DIG_T3) >> 14
        return v1 + v2

    def _compensate_t(self, adc_t: int) -> int:
        return (self._t_fine(adc_t) * 5 + 128) >> 8

    def _compensate_p(self, adc_p: int, adc_t: int) -> float:
        tf = self._t_fine(adc_t)
        v1 = tf / 2.0 - 64000.0
        v2 = v1 * v1 * self.DIG_P6 / 32768.0
        v2 = v2 + v1 * self.DIG_P5 * 2.0
        v2 = v2 / 4.0 + self.DIG_P4 * 65536.0
        v1 = (self.DIG_P3 * v1 * v1 / 524288.0 + self.DIG_P2 * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * self.DIG_P1
        if v1 == 0:
            return 0.0
        p = 1048576.0 - adc_p
        p = (p - v2 / 4096.0) * 6250.0 / v1
        p = p + (self.DIG_P9 * p * p / 2147483648.0 + p * self.DIG_P8 / 32768.0 + self.DIG_P7) / 16.0
        return p

    def _find_adc_t(self, target_centideg: int) -> int:
        lo, hi = 0, (1 << 20) - 1
        while lo < hi:
            mid = (lo + hi) >> 1
            if self._compensate_t(mid) < target_centideg:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def _find_adc_p(self, target_pa: float, adc_t: int) -> int:
        lo, hi = 0, (1 << 20) - 1
        while lo < hi:
            mid = (lo + hi) >> 1
            if self._compensate_p(mid, adc_t) > target_pa:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def _encode20(self, v: int) -> tuple:
        return (v >> 12) & 0xFF, (v >> 4) & 0xFF, (v & 0xF) << 4

    def _update_measurements(self) -> None:
        """The raw ADC values of the panel's temperature and pressure. They
        reach the data registers with a measurement, not here."""
        # Half a hundredth of a degree rounds up, as Math.round does in the
        # tab: round() sends 2412.5 to 2412 and the two copies would differ.
        adc_t = self._find_adc_t(_math.floor(self._temp_c * 100 + 0.5))
        adc_p = self._find_adc_p(self._press_hpa * 100.0, adc_t)
        self._live = bytes((*self._encode20(adc_p), *self._encode20(adc_t)))

    def update(self, temperature_c=None, pressure_hpa=None, /, **inputs) -> None:
        """The panel moved. Only the values it names change, and no register
        does: they are measured in the mode the sketch selected.

        Takes a sensor record or an update as they arrive (temperature in
        degrees Celsius, pressure in hPa); whatever else the record carries,
        and anything that is not a finite number, is left out. The two
        positional values are how update() was called before it took the
        record's names.
        """
        named = {'temperature': temperature_c, 'pressure': pressure_hpa}
        for name, legacy in (('temperature', 'temperature_c'), ('pressure', 'pressure_hpa')):
            for key in (legacy, name):
                if key in inputs:
                    named[name] = inputs[key]
        temp_c = _bmp_number(named['temperature'])
        press  = _bmp_number(named['pressure'])
        if temp_c is not None:
            self._temp_c = temp_c
        if press is not None:
            self._press_hpa = press
        self._update_measurements()

    def inputs(self) -> dict:
        return {'temperature': self._temp_c, 'pressure': self._press_hpa}

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now: in normal mode the
        data registers encoded from the panel's values, and no `measuring`."""
        out = bytearray(self.regs)
        if self._mode() == _BMP_NORMAL:
            out[_BMP_SAMPLE_FIRST:_BMP_SAMPLE_LAST + 1] = self._live
        return out

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            # reg_ptr is NOT reset here: a write-then-read (repeated START)
            # relies on reg_ptr having been set by the preceding WRITE phase.
            self.first_byte = True
            if op == I2C_START_RECV:
                self._latch()
            return 0
        elif op == I2C_WRITE:
            self._latch_due = True
            if self.first_byte:
                self.reg_ptr = data; self.first_byte = False
            else:
                # The byte after a register's data is the next register's
                # address. The pointer stays where the pair put it:
                # esp-idf-lib's bmp280_is_measuring sends 0xF3 0xF4 and reads
                # status and ctrl_meas back.
                self.first_byte = True
                self._write_register(self.reg_ptr, data)
            return 0
        elif op == I2C_READ:
            if self._latch_due:
                self._latch()
            reg = self.reg_ptr
            self.reg_ptr = (reg + 1) & 0xFF
            if reg == _BMP_STATUS:
                status = _BMP_MEASURING if self._measuring else 0
                self._measuring = False
                return status
            return self.regs[reg]
        else:
            # I2C_FINISH, I2C_NACK, unknown. The pointer survives:
            # Seeed_BMP280 writes it in one transaction and reads in the
            # next, and QEMU ends every write phase this way, the one before
            # a repeated START included.
            self.first_byte = True
            self._latch_due = True
            return 0

    def _mode(self) -> int:
        return self.regs[_BMP_CTRL_MEAS] & _BMP_MODE

    def _power_on(self) -> None:
        """The power-on reset, which the reset word runs too. The calibration
        is NVM and the panel is not the chip's."""
        for reg in BMP280_RULES['writable']:
            self.regs[reg] = 0
        for reg in range(_BMP_SAMPLE_FIRST, _BMP_SAMPLE_LAST + 1):
            self.regs[reg] = 0
        for reg, value in BMP280_RULES['power_on'].items():
            self.regs[reg] = value
        self._measuring = False
        self._measured = False

    def _write_register(self, reg: int, value: int) -> None:
        if reg == _BMP_RESET:
            if value == _BMP_RESET_WORD:
                self._power_on()
            return
        if reg not in BMP280_RULES['writable']:
            return
        if reg != _BMP_CTRL_MEAS:
            self.regs[reg] = value
            return
        mode = value & _BMP_MODE
        if mode == _BMP_SLEEP:
            # The chip measured until now, so what it holds asleep is this instant.
            if self._mode() == _BMP_NORMAL:
                self._measure()
            self._measuring = False
            self.regs[reg] = value
            return
        self._measuring = True
        if mode == _BMP_NORMAL:
            self.regs[reg] = value
            return
        self._measure()
        self.regs[reg] = value & ~_BMP_MODE & 0xFF

    def _measure(self) -> None:
        self.regs[_BMP_SAMPLE_FIRST:_BMP_SAMPLE_LAST + 1] = self._live
        self._measured = True

    def _latch(self) -> None:
        if self._mode() == _BMP_NORMAL:
            self._measure()
        self._latch_due = False


# ── DS1307 / DS3231 Real-Time Clock ──────────────────────────────────────────

# What the two clock chips do with a byte written to them, as tables. They are
# DS1307_RULES and DS3231_RULES of the tab models
# (frontend/src/simulation/I2CBusManager.ts) and the `rules` of
# test/fixtures/i2c-vectors/ds1307.json and ds3231.json, which the tests hold
# both copies against. DS1307: datasheet REV 3/15. DS3231: datasheet 19-5170
# rev 10.
DS1307_RULES = {
    # CONTROL powers on with RS1 and RS0 set ("typically set to a 1", Control
    # Register). The RAM is modelled as zeros; the datasheet leaves it open.
    #
    # CH powers on at 0: the clock runs. The datasheet has CH at 1 on a chip
    # that never had power, with the time stopped at 00:00:00 of 01/01/00, and
    # a sketch that only reads the clock would show that forever (seven
    # examples of the gallery only read it). So the part comes as a module
    # somebody set: running, and on the host's time.
    'power_on': {0x07: 0x03},
    # The bits of each register that exist; the others always read 0 (Table 2).
    'write_mask': {
        0x00: 0xFF, 0x01: 0x7F, 0x02: 0x7F, 0x03: 0x07,
        0x04: 0x3F, 0x05: 0x1F, 0x06: 0xFF, 0x07: 0x93,
    },
    # The address pointer wraps to 0x00 after the last byte of the RAM.
    'last_register': 0x3F,
}

DS3231_RULES = {
    # CONTROL 0x1C: oscillator on, 8.192 kHz selected, INTCN set, both alarm
    # interrupts off. STATUS 0x08: EN32kHz set, OSF clear.
    #
    # OSF powers on at 0, as the DS1307's CH does, and for the same reason:
    # the part is a module somebody set and whose battery kept it running,
    # which is why it shows the host's time. The datasheet sets OSF "the first
    # time power is applied", and a module fresh from the bag would say
    # lostPower() on every Run: in the 2026-09 corpus about 64 projects would
    # print a "lost power" line each time and 2 would blank their clock. The
    # chip also sets OSF when its oscillator stops (VCC and VBAT both too low,
    # EOSC in battery mode), and this model has no such case: it runs from
    # VCC with its oscillator on. So OSF only ever reads 0 here.
    'power_on': {0x0E: 0x1C, 0x0F: 0x08},
    # The bits of each register a write stores as written. Bit 7 of the
    # seconds does not exist. CONV is left out of CONTROL (self_clearing), and
    # of STATUS only EN32kHz is a plain read/write bit.
    'write_mask': {
        0x00: 0x7F, 0x01: 0x7F, 0x02: 0x7F, 0x03: 0x07,
        0x04: 0x3F, 0x05: 0x9F, 0x06: 0xFF,
        0x07: 0xFF, 0x08: 0xFF, 0x09: 0xFF, 0x0A: 0xFF,
        0x0B: 0xFF, 0x0C: 0xFF, 0x0D: 0xFF,
        0x0E: 0xDF, 0x0F: 0x08, 0x10: 0xFF,
    },
    # CONV starts a temperature conversion and is never stored: the conversion
    # takes no time here, so the next read finds CONV and BSY at 0.
    'self_clearing': {0x0E: 0x20},
    # OSF, A2F and A1F: "This bit can only be written to logic 0. Attempting
    # to write to logic 1 leaves the value unchanged."
    'write_zero_to_clear': {0x0F: 0x83},
    # The temperature registers.
    'read_only': ((0x11, 0x12),),
    # The address pointer wraps to 0x00 after the temperature's low byte.
    'last_register': 0x12,
    # Quarter degrees, ten bits, two's complement: -128.00 to +127.75 C.
    'temp_lsb_per_c': 4,
}

_RTC_MS_DAY = 86_400_000

_BUILD_MONTHS = ('Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec')
# `__DATE__` and `__TIME__` as C lays them down: each ends its literal, so a
# NUL follows. `__DATE__` pads the day with a space ("Sep  1 2026"). A time is
# not the tail of something longer made of digits and colons (a MAC address).
# Both patterns start with a character class and name no month: megabytes of
# image go through them on the thread that runs the guest.
_BUILD_DATE = _re.compile(rb'([A-Z][a-z]{2}) ([ 0-3][0-9]) ([0-9]{4})\x00')
_BUILD_TIME = _re.compile(rb'([01][0-9]|2[0-3]):([0-5][0-9]):([0-5][0-9])\x00')


def find_build_times(image: bytes) -> list:
    """When a firmware image was compiled, read from the image itself: every
    `__DATE__` paired with every `__TIME__` found in it, as (year, month, day,
    hour, minute, second).

    A sketch that sets a clock to "now" writes those two strings, which
    RTClib parses when the sketch runs, so both are in the image as text (an
    ELF or a merged flash image here). The image is the only thing that says
    when it was built: the build cache answers a compile request with a build
    that can be two weeks old. An image carries more than one such string (an
    ESP32 build has the time its bootloader and its application descriptor
    were compiled next to the sketch's, seconds apart) and nothing in it says
    which one the sketch reads. The tab does the same with the images it
    holds (frontend/src/simulation/firmwareBuildTime.ts).
    """
    # The padding behind a flash image holds nothing.
    image = bytes(image).rstrip(b'\xff')
    dates = {}
    for m in _BUILD_DATE.finditer(image):
        month, day = m.group(1).decode(), int(m.group(2))
        if month in _BUILD_MONTHS and 1 <= day <= 31:
            dates[m.group(0)] = (int(m.group(3)), _BUILD_MONTHS.index(month) + 1, day)
    if not dates:
        return []
    times = {}
    for m in _BUILD_TIME.finditer(image):
        if m.start() == 0 or image[m.start() - 1] not in b'0123456789:':
            times[m.group(0)] = tuple(int(g) for g in m.groups())
    return [date + time for date in dates.values() for time in times.values()]


def _rtc_bcd(n: int) -> int:
    return (((n // 10) % 10) << 4 | (n % 10)) & 0xFF


def _rtc_bin(bcd: int) -> int:
    return ((bcd >> 4) & 0xF) * 10 + (bcd & 0xF)


def _rtc_days_of(year: int, month: int, day: int) -> int:
    """Days since 1 January 1970 of a date. Written out and not left to
    datetime, which refuses what the tab's copy counts through: a month 0 or
    a 31 June a sketch wrote."""
    m0 = month - 1
    m = m0 % 12                       # 0 = January
    y = year + m0 // 12 - (1 if m < 2 else 0)
    era = y // 400
    yoe = y - era * 400
    doy = (153 * (m + (-2 if m > 1 else 10)) + 2) // 5
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    return era * 146097 + doe - 719468 + (day - 1)


def _rtc_date_of(days: int) -> tuple:
    z = days + 719468
    era = z // 146097
    doe = z - era * 146097
    yoe = (doe - doe // 1460 + doe // 36524 - doe // 146096) // 365
    doy = doe - (365 * yoe + yoe // 4 - yoe // 100)
    mp = (5 * doy + 2) // 153
    month = mp + 3 if mp < 10 else mp - 9
    return yoe + era * 400 + (1 if month <= 2 else 0), month, doy - (153 * mp + 2) // 5 + 1


def _rtc_weekday_after(weekday: int, days: int) -> int:
    """The day-of-week register after `days` midnights. It counts 1 to 7 and
    back to 1, from whatever it holds: the chip gives the numbers no meaning.
    A 0, which is what RTClib writes to a DS1307, becomes 1 at the first
    midnight."""
    if days == 0:
        return weekday
    if weekday == 0:
        return 0 if days < 0 else (days - 1) % 7 + 1
    return (weekday - 1 + days) % 7 + 1


def _rtc_hour_of(register: int) -> int:
    """Hours as the register holds them: bit 6 selects 12-hour mode, bit 5 is
    PM there."""
    if register & 0x40:
        return _rtc_bin(register & 0x1F) % 12 + (12 if register & 0x20 else 0)
    return _rtc_bin(register & 0x3F)


def _rtc_hour_register(hour: int, twelve_hour: bool) -> int:
    if not twelve_hour:
        return _rtc_bcd(hour)
    return 0x40 | (0x20 if hour >= 12 else 0) | _rtc_bcd(hour % 12 or 12)


def _rtc_alarm_matched(start: int, to: int, weekday_at_start: int,
                       second, minute, hour, day_register: int) -> bool:
    """Whether an alarm's registers matched the clock at one of the seconds it
    counted through, (start, to]. "The match is tested on the once-per-second
    update of the time and date registers", so a minute nobody read the chip
    in is looked through here, from one field to the next and not second by
    second.

    `second`, `minute` and `hour` are what the field has to be, or None where
    the alarm's mask bit leaves it out; an hour of -1 cannot match (the alarm
    is in 12-hour form and the clock is not, or the reverse). `day_register`
    is the alarm's day/date register as written.
    """
    first_day = start // _RTC_MS_DAY

    def day_matches(days: int) -> bool:
        if day_register & 0x80:
            return True
        if day_register & 0x40:
            return _rtc_weekday_after(weekday_at_start, days - first_day) == day_register & 0x0F
        return _rtc_date_of(days)[2] == _rtc_bin(day_register & 0x3F)

    # Longer ago than a year the chip would have matched as well; nobody waits.
    t = max(start, to - 400 * _RTC_MS_DAY) + 1000
    while t <= to:
        days = t // _RTC_MS_DAY
        day = days * _RTC_MS_DAY
        if not day_matches(days):
            t = day + _RTC_MS_DAY
            continue
        h = (t - day) // 3_600_000
        if hour is not None and h != hour:
            t = day + hour * 3_600_000 if h < hour else day + _RTC_MS_DAY
            continue
        m = (t - day) // 60_000 % 60
        if minute is not None and m != minute:
            hour_start = day + h * 3_600_000
            t = hour_start + minute * 60_000 if m < minute else hour_start + 3_600_000
            continue
        s = (t - day) // 1000 % 60
        if second is not None and s != second:
            minute_start = day + h * 3_600_000 + m * 60_000
            t = minute_start + second * 1000 if s < second else minute_start + 60_000
            continue
        return True
    return False


class TabClock:
    """The clock of the tab, told from the worker's.

    The worker's own clock is the server's, in the server's time zone (UTC in
    the container), and the tab's model shows the browser's. The sensor record
    of a clock chip carries both halves of the difference: `utcOffsetMin`,
    how far the browser's zone is from UTC in minutes east, and `epochMs`,
    the epoch as the browser counted it when the board was run.

    The record is stamped in the tab and read here some time later (the
    socket, the worker starting), and that time would show as a clock that is
    behind. Two clocks that are set from the network differ by less than the
    delivery takes, so a difference of under a minute is taken as none and
    the worker's epoch is used; a browser whose clock is further off than
    that is a clock somebody set, and it is followed.
    """

    SAME_CLOCK_MS = 60_000

    def __init__(self, now_ms=None) -> None:
        self._now_ms = now_ms or (lambda: _time.time() * 1000.0)
        self._skew_ms = 0.0
        self._offset_ms = 0.0

    def set(self, record: dict) -> None:
        """Take what a sensor record or an update says about the clock."""
        offset = _finite(record.get('utcOffsetMin'))
        if offset is not None:
            self._offset_ms = offset * 60_000.0
        epoch = _finite(record.get('epochMs'))
        if epoch is not None:
            skew = epoch - self._now_ms()
            self._skew_ms = 0.0 if abs(skew) <= self.SAME_CLOCK_MS else skew

    def __call__(self) -> int:
        """Milliseconds since 00:00 of 1 January 1970 of the calendar on the
        user's wall."""
        return int(self._now_ms() + self._skew_ms + self._offset_ms)


def _finite(value):
    """A number of a sensor record as a float, or None for anything else."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if _math.isfinite(number) else None


class _RtcCounters:
    """The counters of a clock chip: seconds to year as the registers hold
    them, and when they last moved.

    Until the sketch sets a time the counters are the host's clock, read again
    at every START. A time the sketch writes is kept and counted from, as the
    chip does, with one exception (project i2c-model-fidelity-2026-09,
    decision D7): a time that is the compile time of the firmware. That is
    what `rtc.adjust(DateTime(F(__DATE__), F(__TIME__)))` writes, the line of
    every RTClib example, and it means "now": counted from, it would show the
    hour of the compile server, in its time zone and as old as the build. The
    model takes it as a clock that was set when the firmware was built and
    has run since, which is the host's clock.

    The rule, to the second. When a write phase that wrote any of the
    registers 0x00 to 0x02 or 0x04 to 0x06 ends, the six of them are read as a
    date and a time (year 2000 + YY; CH, the century bit and the 12-hour bits
    taken out). If they are one of the firmware's build times, the counters
    follow the host's clock from then on. The day of week is not compared:
    the strings carry none, and RTClib writes 0 there to a DS1307. It is kept
    as written and moved on by the days between the two dates, as the
    midnights in between would have. Anything else written is kept. So is
    everything a firmware writes whose image holds no build time.
    """

    def __init__(self, clock, build_times, has_clock_halt: bool) -> None:
        self._clock = clock
        self._build_times = build_times
        # DS1307: bit 7 of the seconds is CH, and it stops the clock.
        self._has_clock_halt = has_clock_halt
        # Seconds, minutes, hours, day of week, date, month, year.
        self.regs = bytearray(7)
        # The counters are the host's clock.
        self._following = True
        # A time register was written in the write phase that is open.
        self._written = False
        # The seconds the clock counted through, for the alarms: (start, to],
        # and the weekday at `start`.
        self.on_count = None
        now = self._clock()
        self._show(now // 1000 * 1000)
        # No sketch has said what the numbers mean yet. Monday = 1 is what
        # RTClib writes to a DS3231 and compares an alarm on a weekday with
        # (dowToDS3231), and what the Seeed DS1307 library calls MON.
        self.regs[3] = (now // _RTC_MS_DAY + 3) % 7 + 1
        # Host time at which the second the registers show began.
        self._tick_at = now // 1000 * 1000

    @property
    def halted(self) -> bool:
        return self._has_clock_halt and bool(self.regs[0] & 0x80)

    def sync(self) -> None:
        """Bring the counters to the present."""
        now = self._clock()
        if self._following:
            to = now // 1000 * 1000
            self._count(self.time(), to, True)
            self._tick_at = to
            return
        if self.halted:
            return
        seconds = (now - self._tick_at) // 1000
        if seconds <= 0:
            # The host's clock was set back: the chip does not count backwards.
            if now < self._tick_at:
                self._tick_at = now
            return
        start = self.time()
        self._count(start, start + seconds * 1000, True)
        self._tick_at += seconds * 1000

    def write(self, reg: int, value: int) -> None:
        """A byte written to one of the seven registers, already cut to the
        bits that exist."""
        self.regs[reg] = value
        # The day of week is a counter of its own: writing it sets no time.
        if reg == 3:
            return
        self._following = False
        self._written = True
        # "The countdown chain is reset whenever the seconds register is
        # written."
        if reg == 0:
            self._tick_at = self._clock()

    def commit(self) -> None:
        """The write phase ended: what was written is a time now."""
        if not self._written:
            return
        self._written = False
        if self.halted:
            return
        was_set = self.time()
        days = was_set // _RTC_MS_DAY
        ms = was_set - days * _RTC_MS_DAY
        written = _rtc_date_of(days) + (ms // 3_600_000, ms // 60_000 % 60, ms // 1000 % 60)
        if not any(tuple(built) == written for built in self._build_times()):
            return
        self._following = True
        to = self._clock() // 1000 * 1000
        # Set back then, running since: no alarm is owed for the time in between.
        self._count(was_set, to, False)
        self._tick_at = to

    def time(self) -> int:
        """The time the registers show, as the clock counts it."""
        r = self.regs
        days = _rtc_days_of(2000 + _rtc_bin(r[6]), _rtc_bin(r[5] & 0x1F), _rtc_bin(r[4] & 0x3F))
        return (days * _RTC_MS_DAY + _rtc_hour_of(r[2]) * 3_600_000
                + _rtc_bin(r[1] & 0x7F) * 60_000 + _rtc_bin(r[0] & 0x7F) * 1000)

    def _count(self, start: int, to: int, alarms: bool) -> None:
        if to == start:
            return
        weekday = self.regs[3]
        self.regs[3] = _rtc_weekday_after(weekday, to // _RTC_MS_DAY - start // _RTC_MS_DAY)
        self._show(to)
        if alarms and to > start and self.on_count is not None:
            self.on_count(start, to, weekday)

    def _show(self, time: int) -> None:
        """Put a time in the registers. CH, the 12-hour mode and the day of
        week stay."""
        r = self.regs
        days = time // _RTC_MS_DAY
        ms = time - days * _RTC_MS_DAY
        year, month, day = _rtc_date_of(days)
        # The year register counts 00 to 99, and the century bit of the
        # DS3231 turns over with it.
        centuries = (year - 2000) // 100
        r[0] = (r[0] & 0x80) | _rtc_bcd(ms // 1000 % 60)
        r[1] = _rtc_bcd(ms // 60_000 % 60)
        r[2] = _rtc_hour_register(ms // 3_600_000, bool(r[2] & 0x40))
        r[4] = _rtc_bcd(day)
        r[5] = ((r[5] & 0x80) ^ (0x80 if centuries & 1 else 0)) | _rtc_bcd(month)
        r[6] = _rtc_bcd((year - 2000) % 100)


class _RtcSlave:
    """What the DS1307 and the DS3231 have in common on the bus: a register
    pointer that wraps, and seven time registers that are answered from a
    copy.

    "When reading or writing the time and date registers, secondary (user)
    buffers are used to prevent errors when the internal registers update.
    [...] the user buffers are synchronized to the internal registers on any
    START and when the register pointer rolls over to zero." So a burst that
    starts at 12:34:59 reads 12:34:59 to its last byte, however long the
    guest takes over it, and never 12:35:59.

    `record` is the sensor record of the part, which carries the tab's clock
    (TabClock). `clock` replaces it, for a test: a TabClock over a machine
    clock of the test's own, or any callable that returns the host's wall
    time in milliseconds. `build_times` is a callable that
    returns the build times of the firmware (find_build_times); it is asked
    when the sketch sets the clock, and not before.
    """

    LAST_REGISTER = 0x3F
    HAS_CLOCK_HALT = False

    def __init__(self, record=None, *, clock=None, build_times=None) -> None:
        self.addr       = 0x68
        self.reg_ptr    = 0
        self.first_byte = True
        # What the record and its updates say about the clock lands here. A
        # clock of another kind is not moved by them.
        self.tab_clock  = clock if isinstance(clock, TabClock) else TabClock()
        if isinstance(record, dict):
            self.tab_clock.set(record)
        self._counters = _RtcCounters(clock or self.tab_clock, build_times or (lambda: ()),
                                      self.HAS_CLOCK_HALT)
        # The time registers as the START of this transfer found them.
        self._latched = bytes(7)
        # No START was heard for the transfer that comes next: it begins at
        # its first byte.
        self._latch_due = True

    def _read_register(self, reg: int) -> int:
        """A register behind the time, as the transfer in progress is answered."""
        raise NotImplementedError

    def _write_register(self, reg: int, value: int) -> None:
        raise NotImplementedError

    def _latch_inputs(self) -> None:
        """What else is sampled when a transfer begins."""

    def _register_now(self, reg: int) -> int:
        """A register behind the time as it is now, whatever a transfer in
        progress was told."""
        return self._read_register(reg)

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            # reg_ptr is NOT reset here: a write-then-read (repeated START)
            # relies on it having been set by the preceding WRITE phase.
            self.first_byte = True
            self._begin()
            return 0

        elif op == I2C_WRITE:
            if self.first_byte and self._latch_due:
                self._begin()
            self._latch_due = True
            if self.first_byte:
                # Past the last register the datasheets say nothing: the
                # DS1307 has six address bits to count with, the DS3231 is
                # given the byte.
                self.reg_ptr = data & 0x3F if self.LAST_REGISTER == 0x3F else data
                self.first_byte = False
            else:
                reg = self.reg_ptr
                self.reg_ptr = self._after(reg)
                self._write_register(reg, data)
            return 0

        elif op == I2C_READ:
            if self._latch_due:
                self._begin()
            reg = self.reg_ptr
            value = self._latched[reg] if reg < 7 else self._read_register(reg)
            self.reg_ptr = self._after(reg)
            if self.reg_ptr == 0:
                self._latch()
            return value & 0xFF

        else:                         # I2C_FINISH, I2C_NACK, unknown
            # The pointer survives: the Seeed library writes it in one
            # transaction and reads in the next, and QEMU ends every write
            # phase this way, the one before a repeated START included. What
            # a write phase wrote to the time registers is a time from here.
            self._counters.commit()
            self.first_byte = True
            self._latch_due = True
            return 0

    def update(self, /, **record) -> None:
        """The tab sent the part's record again, or a part of it."""
        self.tab_clock.set(record)

    def dump_registers(self) -> bytearray:
        """The registers as a read would find them now."""
        self._counters.sync()
        out = bytearray(256)
        for reg in range(7, self.LAST_REGISTER + 1):
            out[reg] = self._register_now(reg) & 0xFF
        out[0:7] = self._counters.regs
        return out

    def _begin(self) -> None:
        # A repeated START ends a write phase as a STOP does.
        self._counters.commit()
        self._latch()
        self._latch_due = False

    def _latch(self) -> None:
        self._counters.sync()
        self._latched = bytes(self._counters.regs)
        self._latch_inputs()

    def _after(self, reg: int) -> int:
        return 0 if reg == self.LAST_REGISTER else (reg + 1) & 0xFF


class DS1307Slave(_RtcSlave):
    """DS1307: the clock with 56 bytes of battery-backed RAM, at 0x68. The
    twin of VirtualDS1307 in the tab: both replay
    test/fixtures/i2c-vectors/ds1307.json.

      - The time is the host's until the sketch sets one; then it is the
        sketch's, counted from the moment it was written (_RtcCounters has
        the one exception, a firmware's own build time).
      - CH, bit 7 of the seconds, stops the clock where it is, and RTClib's
        isrunning() reads it. Clearing it starts the clock from there.
      - CONTROL and the RAM at 0x08 to 0x3F keep what is written to them
        (RTClib readnvram and writenvram).
      - The pointer wraps from 0x3F to 0x00.
    """

    LAST_REGISTER = DS1307_RULES['last_register']
    HAS_CLOCK_HALT = True

    def __init__(self, record=None, *, clock=None, build_times=None) -> None:
        super().__init__(record, clock=clock, build_times=build_times)
        # CONTROL at 0x07 and the RAM behind it, under their own addresses.
        self._ram = bytearray(self.LAST_REGISTER + 1)
        for reg, value in DS1307_RULES['power_on'].items():
            self._ram[reg] = value

    def _read_register(self, reg: int) -> int:
        return self._ram[reg]

    def _write_register(self, reg: int, value: int) -> None:
        mask = DS1307_RULES['write_mask'].get(reg, 0xFF)
        if reg < 7:
            self._counters.write(reg, value & mask)
        else:
            self._ram[reg] = value & mask


class DS3231Slave(_RtcSlave):
    """DS3231: the temperature-compensated clock with two alarms, at 0x68. The
    twin of VirtualDS3231 in the tab: both replay
    test/fixtures/i2c-vectors/ds3231.json.

      - The time registers are the DS1307's, without CH: powered from VCC the
        oscillator runs whatever EOSC says (Control Register, bit 7).
      - CONTROL powers on at 0x1C. RTClib's setAlarm1() and setAlarm2() refuse
        to arm an alarm unless INTCN reads 1, and CONV is gone by the next
        read (Makuna's Rtc polls it after forcing a conversion).
      - STATUS powers on with OSF clear (DS3231_RULES['power_on'] says why),
        so RTClib's lostPower() is false. OSF, A1F and A2F can only be
        written to 0.
      - A1F and A2F are set when the clock counts through a second the
        alarm's registers match, whether or not the interrupt is enabled, and
        stay until the sketch writes them to 0 (RTClib alarmFired,
        clearAlarm). The INT/SQW pin is not driven.
      - The temperature is the panel's, in quarter degrees, two's complement:
        q = round(T x 4), 0x11 = q >> 2, 0x12 = (q & 3) << 6. It is read only.
      - The pointer wraps from 0x12 to 0x00.
    """

    LAST_REGISTER = DS3231_RULES['last_register']

    def __init__(self, record=None, *, clock=None, build_times=None) -> None:
        super().__init__(record, clock=clock, build_times=build_times)
        self.temperatureC = 25.0
        # Alarm 1 (0x07-0x0A), alarm 2 (0x0B-0x0D), CONTROL, STATUS and the
        # aging offset.
        self._regs = bytearray(self.LAST_REGISTER + 1)
        for reg, value in DS3231_RULES['power_on'].items():
            self._regs[reg] = value
        # The temperature as the START of this transfer found it.
        self._temperature = bytes(2)
        self._counters.on_count = self._check_alarms
        if isinstance(record, dict):
            self._set_temperature(record.get('temperature'))

    def update(self, temperature=None, /, **record) -> None:
        """The panel moved, or the tab sent the record again. The temperature
        is `temperature` of a record, in degrees Celsius, or the one argument
        of a caller that has nothing else to say."""
        super().update(**record)
        self._set_temperature(record.get('temperature', temperature))

    def _set_temperature(self, value) -> None:
        celsius = _finite(value)
        if celsius is not None:
            self.temperatureC = celsius

    def _read_register(self, reg: int) -> int:
        if reg in (0x11, 0x12):
            return self._temperature[reg - 0x11]
        return self._regs[reg] if reg <= 0x10 else 0x00

    def _write_register(self, reg: int, value: int) -> None:
        mask = DS3231_RULES['write_mask'].get(reg)
        if mask is None:
            return
        if reg < 7:
            self._counters.write(reg, value & mask)
            return
        flags = DS3231_RULES['write_zero_to_clear'].get(reg, 0)
        self._regs[reg] = (self._regs[reg] & flags & value) | (value & mask)

    def _latch_inputs(self) -> None:
        self._temperature = self._temperature_registers()

    def _register_now(self, reg: int) -> int:
        if reg in (0x11, 0x12):
            return self._temperature_registers()[reg - 0x11]
        return self._read_register(reg)

    def _temperature_registers(self) -> bytes:
        limit = 128 * DS3231_RULES['temp_lsb_per_c']
        celsius = _finite(self.temperatureC)
        quarters = (celsius or 0.0) * DS3231_RULES['temp_lsb_per_c']
        # Half a step rounds away from zero, as in the other twin (round()
        # sends 99.5 to 100 and 98.5 to 98).
        size = _math.floor(abs(quarters) + 0.5) if abs(quarters) < limit else limit
        q = max(-limit, min(limit - 1, size if quarters >= 0 else -size))
        return bytes(((q >> 2) & 0xFF, (q & 3) << 6))

    def _check_alarms(self, start: int, to: int, weekday: int) -> None:
        r = self._regs
        twelve_hour = bool(self._counters.regs[2] & 0x40)

        def field(reg: int):
            return None if reg & 0x80 else _rtc_bin(reg & 0x7F)

        def hour(reg: int):
            if reg & 0x80:
                return None
            return _rtc_hour_of(reg & 0x7F) if bool(reg & 0x40) == twelve_hour else -1

        if _rtc_alarm_matched(start, to, weekday,
                              field(r[0x07]), field(r[0x08]), hour(r[0x09]), r[0x0A]):
            r[0x0F] |= 0x01
        # Alarm 2 has no seconds register: it matches at second 00.
        if _rtc_alarm_matched(start, to, weekday, 0, field(r[0x0B]), hour(r[0x0C]), r[0x0D]):
            r[0x0F] |= 0x02


# ── I2C Write Sink (relay for write-only devices: SSD1306, PCF8574) ──────────

class I2CWriteSink:
    """ACKs all I2C writes, emits complete transaction to frontend on FINISH.

    The echo names the part it belongs to (`owner`, the component id the
    record carries) whenever the record gave one, so the tab hands each write
    phase to that part only: two panels at one address on the two controllers
    of a board no longer draw each other's frames.

    Without a `port` the sink is a write-only panel, which drives nothing, so
    a read is 0xFF. With one it is the PCF8574 of an expander part or an LCD
    backpack. That chip has no registers: every byte written goes to the port
    latch at its acknowledge, and a read returns the pins (PCF8574 datasheet,
    "Writing to the port" / "Reading from the port"). A latch bit of 0 drives
    its pin low; a 1 releases it to the weak pull-up, and the pin then reads
    what the outside drives. `port` is that outside: 0xFF when nothing pulls a
    pin down, 0xF7 on an LCD backpack, whose backlight transistor holds P3
    low. The tab's model (VirtualPCF8574, I2CBusManager.ts) reads the same
    `latch & port`. hd44780_I2Cexp tells the chip from an MCP23008, and finds
    the backpack's wiring, from these reads; against a sink that read 0xFF it
    took the backpack for an MCP23008.

    One class for both, so the workers' checks by class name (no per-event
    log or trace for a sink) keep holding.
    """

    def __init__(self, addr: int, emit_fn, owner: str | None = None,
                 port: int | None = None) -> None:
        self.addr  = addr
        self.owner = owner
        self.port  = None if port is None else port & 0xFF
        self.latch = 0xFF   # power-on: every pin released (quasi-input)
        self._emit = emit_fn
        self._buf: list[int] = []

    @staticmethod
    def from_record(stype: str, record: dict, emit_fn) -> 'I2CWriteSink':
        """The worker's copy of a display or an expander part, from its record.

        One place for every worker that hosts these parts: the record's type
        picks the default address and, for 'pcf8574', the port latch; its
        `owner` goes on every echo; its `portState` is the outside of an
        expander's pins. A static method so a worker reaches it through the
        class it already imports.
        """
        addr = int(record.get('addr', 0x3C if stype == 'ssd1306' else 0x27))
        owner = record.get('owner')
        owner = str(owner) if owner else None
        port = None
        if stype == 'pcf8574':
            port = int(record.get('portState', 0xFF)) & 0xFF
        return I2CWriteSink(addr, emit_fn, owner, port)

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            self._buf = []; return 0
        elif op == I2C_WRITE:
            self._buf.append(data)
            self.latch = data
            return 0
        elif op == I2C_READ:
            if self.port is None:
                return 0xFF   # write-only device
            return self.latch & self.port
        else:             # I2C_FINISH — emit accumulated transaction
            if self._buf:
                msg = {'type': 'i2c_transaction',
                       'addr': self.addr, 'data': list(self._buf)}
                if self.owner:
                    msg['owner'] = self.owner
                self._emit(msg)
                self._buf = []
            return 0
