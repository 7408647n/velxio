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

import datetime as _datetime
import math as _math
import threading as _threading


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
    # Every register powers on at 0x00 but these: asleep, and its id (section 3).
    'power_on': {0x6B: 0x40, 0x75: 0x68},
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
    # TEMP_OUT = (T - 36.53) * 340 (4.18).
    'temp_lsb_per_c': 340,
    'temp_offset_c': 36.53,
    # Bits a read of the register takes with it: "each bit will clear after
    # the register is read" (4.16). With INT_RD_CLEAR set in INT_PIN_CFG, a
    # read of any register clears them (4.14).
    'clear_on_read': {0x3A: 0xFF},
    # The gyroscope output rate the sample rate is divided from, in Hz: 8 kHz
    # with the low-pass filter off (DLPF_CFG 0 or 7), 1 kHz with it on.
    # Sample rate = rate / (1 + SMPLRT_DIV) (4.2, 4.3).
    'gyro_rate_hz': {'dlpf_off': 8000, 'dlpf_on': 1000},
    # How long INT stays active per interrupt with LATCH_INT_EN clear (4.14).
    'int_pulse_us': 50,
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
# The interrupt sources the model raises, as bits of INT_ENABLE and INT_STATUS.
_MPU_INT_SOURCES    = _MPU_DATA_RDY_INT
# ACCEL_XOUT_H to GYRO_ZOUT_L: three axes, the die temperature, three axes.
_MPU_SAMPLE_FIRST   = 0x3B
_MPU_SAMPLE_LAST    = 0x48
_MPU_SAMPLE_SIZE    = _MPU_SAMPLE_LAST - _MPU_SAMPLE_FIRST + 1
_MPU_USER_CTRL      = 0x6A
_MPU_SIG_COND_RESET = 0x01
_MPU_PWR_MGMT_1     = 0x6B
_MPU_DEVICE_RESET   = 0x80
_MPU_SLEEP          = 0x40

_MPU_READ_ONLY = bytearray(256)
for _first, _last in MPU6050_RULES['read_only']:
    _MPU_READ_ONLY[_first:_last + 1] = b'\x01' * (_last - _first + 1)
_MPU_SELF_CLEARING = bytearray(256)
for _reg, _mask in MPU6050_RULES['self_clearing'].items():
    _MPU_SELF_CLEARING[_reg] = _mask
_MPU_CLEAR_ON_READ = bytearray(256)
for _reg, _mask in MPU6050_RULES['clear_on_read'].items():
    _MPU_CLEAR_ON_READ[_reg] = _mask
_MPU_INT_PULSE_NS = MPU6050_RULES['int_pulse_us'] * 1000


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
      - Asleep, the block holds what it held when the chip fell asleep.
      - Awake, it takes a sample every sample period of the guest's time
        (now_ns), whether the sketch talks to it or not. Each one sets
        DATA_RDY_INT when DATA_RDY_EN is set, which a read of INT_STATUS
        clears, and moves the INT pad the way INT_PIN_CFG says (int_pad,
        int_wake_ns). Nothing runs in the background for it: the samples due
        are counted when the chip is next looked at, from the time that has
        passed. With no clock (a libqemu that does not export it) one period
        passes per register pointer the sketch writes, so a driver that waits
        for DATA_RDY still finds it.
    """

    def __init__(self, addr: int = 0x68, now_ns=None):
        self.addr       = addr
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
        # What the block held when SLEEP was set; zeros after power-on and reset.
        self._last_awake = bytes(_MPU_SAMPLE_SIZE)
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
        # The panel moves on the worker's command thread and QEMU's thread
        # runs the bus: the samples due are taken of the world before a move,
        # under one lock with the bus events.
        self._lock = _threading.RLock()
        self._power_on()

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
                self.reg_ptr = (reg + 1) & 0xFF
                self._write_register(reg, data)
            return 0   # ACK

        elif op == I2C_READ:
            self._sync()
            if self._latch_due:
                self._latch()
            reg = self.reg_ptr
            self.reg_ptr = (reg + 1) & 0xFF
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
            out[_MPU_SAMPLE_FIRST:_MPU_SAMPLE_LAST + 1] = (
                self._last_awake if self._asleep() else self._encode())
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
        raised = self.regs[_MPU_INT_ENABLE] & _MPU_INT_SOURCES & _MPU_DATA_RDY_INT
        if not raised:
            return
        self.regs[_MPU_INT_STATUS] |= raised
        # A pulse has a length only where there is a clock to measure it on.
        if at_ns is not None and not self._latched():
            self._pulse_end_ns = at_ns + _MPU_INT_PULSE_NS

    def _power_on(self) -> None:
        self.regs[:] = bytes(256)
        for reg, value in MPU6050_RULES['power_on'].items():
            self.regs[reg] = value
        self._last_awake = bytes(_MPU_SAMPLE_SIZE)
        self._restart_sampling()

    def _read_register(self, reg: int) -> int:
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
            self._last_awake = bytes(_MPU_SAMPLE_SIZE)
        stored = value & ~_MPU_SELF_CLEARING[reg] & 0xFF
        # The chip sampled until now, so what it holds asleep is this instant.
        if reg == _MPU_PWR_MGMT_1 and stored & _MPU_SLEEP and not self._asleep():
            self._last_awake = self._encode()
        was_asleep = self._asleep()
        self.regs[reg] = stored
        # Waking up, or another rate: the first sample is one period from here.
        if self._asleep() != was_asleep or self._sample_period_ns() != self._period_ns:
            self._restart_sampling()

    def _latch(self) -> None:
        self._sample = self._last_awake if self._asleep() else self._encode()
        self._latch_due = False

    def _encode(self) -> bytes:
        inputs = self._inputs
        accel = MPU6050_RULES['accel_lsb_per_g'][(self.regs[_MPU_ACCEL_CONFIG] >> 3) & 3]
        gyro  = MPU6050_RULES['gyro_lsb_per_dps'][(self.regs[_MPU_GYRO_CONFIG] >> 3) & 3]
        block = (
            inputs['accelX'] * accel,
            inputs['accelY'] * accel,
            inputs['accelZ'] * accel,
            (inputs['temp'] - MPU6050_RULES['temp_offset_c']) * MPU6050_RULES['temp_lsb_per_c'],
            inputs['gyroX'] * gyro,
            inputs['gyroY'] * gyro,
            inputs['gyroZ'] * gyro,
        )
        out = bytearray()
        for value in block:
            counts = _mpu_counts(value) & 0xFFFF
            out.append(counts >> 8)
            out.append(counts & 0xFF)
        return bytes(out)


# ── BMP280 Barometric Pressure + Temperature Sensor ───────────────────────────

class BMP280Slave:
    """Full BMP280 register-map I2C slave (address 0x76 or 0x77).

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
        self.first_byte = True
        self._temp_c    = 25.0
        self._press_hpa = 1013.25
        self._init_calibration()
        self._update_measurements()

    # ── calibration register layout ───────────────────────────────────────────
    def _wu16(self, a: int, v: int) -> None:
        self.regs[a] = v & 0xFF; self.regs[a + 1] = (v >> 8) & 0xFF

    def _ws16(self, a: int, v: int) -> None:
        self._wu16(a, v & 0xFFFF)

    def _init_calibration(self) -> None:
        self.regs[0xD0] = 0x58  # chip_id BMP280 (production silicon; BME280 uses 0x60)
        self.regs[0xF3] = 0x00  # status  (done)
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
        adc_t = self._find_adc_t(round(self._temp_c * 100))
        adc_p = self._find_adc_p(self._press_hpa * 100.0, adc_t)
        pm, pl, px = self._encode20(adc_p)
        tm, tl, tx = self._encode20(adc_t)
        self.regs[0xF7] = pm; self.regs[0xF8] = pl; self.regs[0xF9] = px
        self.regs[0xFA] = tm; self.regs[0xFB] = tl; self.regs[0xFC] = tx

    def update(self, temperature_c: float, pressure_hpa: float) -> None:
        self._temp_c    = temperature_c
        self._press_hpa = pressure_hpa
        self._update_measurements()

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            self.first_byte = True; return 0
        elif op == I2C_WRITE:
            if self.first_byte:
                self.reg_ptr = data; self.first_byte = False
            else:
                self.regs[self.reg_ptr] = data
                self.reg_ptr = (self.reg_ptr + 1) & 0xFF
            return 0
        elif op == I2C_READ:
            val = self.regs[self.reg_ptr]
            self.reg_ptr = (self.reg_ptr + 1) & 0xFF
            return val
        else:
            self.first_byte = True; return 0


# ── DS1307 / DS3231 Real-Time Clock ──────────────────────────────────────────

class DS1307Slave:
    """DS1307 I2C RTC — returns current system time in BCD (address 0x68)."""

    def __init__(self) -> None:
        self.reg_ptr    = 0
        self.first_byte = True

    @staticmethod
    def _bcd(n: int) -> int:
        return ((n // 10) << 4) | (n % 10)

    def _read_reg(self, reg: int) -> int:
        now = _datetime.datetime.now()
        if   reg == 0x00: return self._bcd(now.second)
        elif reg == 0x01: return self._bcd(now.minute)
        elif reg == 0x02: return self._bcd(now.hour)
        elif reg == 0x03: return self._bcd(now.weekday() + 1)  # Mon=1..Sun=7
        elif reg == 0x04: return self._bcd(now.day)
        elif reg == 0x05: return self._bcd(now.month)
        elif reg == 0x06: return self._bcd(now.year % 100)
        return 0x00

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            self.first_byte = True; return 0
        elif op == I2C_WRITE:
            if self.first_byte:
                self.reg_ptr = data; self.first_byte = False
            return 0
        elif op == I2C_READ:
            val = self._read_reg(self.reg_ptr)
            self.reg_ptr = (self.reg_ptr + 1) & 0x3F
            return val
        else:
            self.first_byte = True; return 0


class DS3231Slave(DS1307Slave):
    """DS3231 I2C RTC with on-chip temperature (address 0x68)."""

    def __init__(self) -> None:
        super().__init__()
        self.temperatureC = 25.0

    def _read_reg(self, reg: int) -> int:
        if reg == 0x0E: return 0x00   # Control
        if reg == 0x0F: return 0x00   # Status (OSF cleared)
        if reg == 0x11:               # Temp MSB (signed integer °C)
            return int(self.temperatureC) & 0xFF
        if reg == 0x12:               # Temp LSB (fractional bits 7:6)
            frac = abs(self.temperatureC) - int(abs(self.temperatureC))
            return (round(frac / 0.25) & 0x03) << 6
        return super()._read_reg(reg)


# ── I2C Write Sink (relay for write-only devices: SSD1306, PCF8574) ──────────

class I2CWriteSink:
    """ACKs all I2C writes, emits complete transaction to frontend on FINISH."""

    def __init__(self, addr: int, emit_fn) -> None:
        self.addr  = addr
        self._emit = emit_fn
        self._buf: list[int] = []

    def handle_event(self, event: int) -> int:
        op   = event & 0xFF
        data = (event >> 8) & 0xFF

        if op in (I2C_START_RECV, I2C_START_SEND):
            self._buf = []; return 0
        elif op == I2C_WRITE:
            self._buf.append(data); return 0
        elif op == I2C_READ:
            return 0xFF   # write-only device
        else:             # I2C_FINISH — emit accumulated transaction
            if self._buf:
                self._emit({'type': 'i2c_transaction',
                            'addr': self.addr, 'data': list(self._buf)})
                self._buf = []
            return 0
