# I2C bus vectors

One file per chip. Each file is a list of bus operations and the bytes every
read must return, so every model of that chip can replay it: the tab model in
TypeScript, the backend twin in Python, anything that comes later. Two models
that pass the same file cannot drift apart again.

| File | Chip | Replayed by |
|---|---|---|
| `mpu6050.json` | InvenSense MPU-6050 | `frontend/src/__tests__/protocol-parts.test.ts` (tab model), `test/backend/unit/test_i2c_slaves.py` (backend twin) |
| `ds1307.json` | DS1307 real-time clock | `frontend/src/__tests__/rtc-vectors.test.ts` (tab model), `test/backend/unit/test_i2c_slaves.py` (backend twin) |
| `ds3231.json` | DS3231 real-time clock | `frontend/src/__tests__/rtc-vectors.test.ts` (tab model), `test/backend/unit/test_i2c_slaves.py` (backend twin) |
| `bmp280.json` | Bosch BMP280 | `frontend/src/__tests__/bmp280-vectors.test.ts` (tab model), `test/backend/unit/test_i2c_slaves.py` (backend twin) |

## File

```json
{
  "format": 1,
  "device": "mpu6050",
  "address": "68",
  "rules": { },
  "inputs": { "accelZ": 1, "temp": 24 },
  "vectors": [ { "name": "...", "spec": "...", "steps": [ ] } ]
}
```

- `address`: the 7-bit address the vectors talk to.
- `rules`: the chip's write rules as a table (power-on values, read-only
  ranges, self-clearing masks, sensitivities). A model that exports the same
  table is tested against this copy, so the two agree on the facts and not only
  on the cases below.
- `inputs`: the physical inputs every vector starts from, in the units of the
  sensor panel (g, deg/s, deg C for the MPU-6050; deg C and hPa for the
  BMP280).
- `vectors`: each one starts from a chip that has just been powered on.
  `name` says what is proved, `spec` where the behaviour comes from (datasheet
  section or driver source). A vector with a `driver` field is the traffic of
  that driver, read from its source.

A chip that keeps the time has two more fields, in the file and, where one
vector needs its own, in the vector:

```json
{
  "clock": "2026-09-30T12:34:56.250",
  "build_times": [["Sep 29 2026", "23:39:41"]]
}
```

- `clock`: where the host's clock is when the chip powers on, as the calendar
  on the user's wall reads it. It has no time zone: a model counts wall time,
  and the runner hands it a clock that only the vector moves. No vector
  depends on when or where it runs.
- `build_times`: the `__DATE__` and `__TIME__` pairs of the firmware that is
  running, as the compiler writes the two strings. A model that is set to one
  of them stays on the host's clock; see `RtcCounters` in
  `frontend/src/simulation/I2CBusManager.ts`. An empty list is a firmware
  whose image says nothing about when it was built.

## Numbers

Every string is hexadecimal with no prefix (`"6B"`), every JSON number is
decimal (`"n": 14`). A byte list is bytes separated by spaces, and `XX*N`
repeats a byte N times (N is decimal): `"00*107 40"` is 107 zero bytes and
then 0x40. The empty string is no bytes.

## Steps

| Step | On the bus |
|---|---|
| `{ "op": "write", "data": "6B 00" }` | START for writing, the bytes, STOP. The first byte is the register pointer. No bytes at all is an address probe |
| `{ "op": "read", "reg": "3B", "n": 14, "expect": "..." }` | START for writing, the pointer, repeated START for reading, `n` bytes, STOP |
| `{ "op": "get", "n": 1, "expect": "..." }` | START for reading, `n` bytes, STOP: a read from wherever the pointer is |
| `{ "op": "start", "rw": "w" }` | START, or a repeated START when a transaction is open; `"r"` for reading |
| `{ "op": "send", "data": "3B" }` | bytes written inside the open transaction |
| `{ "op": "recv", "n": 6, "expect": "..." }` | bytes read inside the open transaction |
| `{ "op": "stop" }` | STOP |
| `{ "op": "inputs", "values": { "accelX": 0.5 } }` | not a bus event: the panel moves. Only the named inputs change |
| `{ "op": "clock", "advance_ms": 1000 }` | not a bus event: the host's clock moves on by that many milliseconds |
| `{ "op": "clock", "set": "2026-10-01T00:00:00.000" }` | not a bus event: the host's clock is put at that date and time |
| `{ "op": "dump", "at": "3B", "expect": "..." }` | not a bus event: the register file as a host that mirrors it would copy it, compared from register `at`. A model with no dump skips the step |

The chip acknowledges its address and every byte written to it. A runner
fails the vector when it does not.

## Bus flavours

Every vector must pass in both:

| Flavour | A repeated START is delivered as |
|---|---|
| `repeated-start` | a START with no STOP before it. This is the wire, and what the bus fabric in the tab delivers |
| `stop-start` | a STOP and then a START. QEMU delivers it to a device model this way (`I2C_FINISH`, then `I2C_START_RECV` or `I2C_START_SEND`) |

The runner applies the flavour, the vectors do not change: in `stop-start`, a
`start` step that finds a transaction open is preceded by a STOP. So a model
may not do at STOP what belongs to a read (the register pointer has to survive
it), and may not depend on a STOP to begin a new read.

## Writing a runner

A runner needs, per vector, a new model at `address` with `inputs` applied
(and with the clock and the build times, for a chip that keeps the time),
and then a loop over `steps`. The TypeScript one is about forty lines
(`replayVector` in `protocol-parts.test.ts`, the one that moves a clock in
`frontend/src/__tests__/helpers/i2cVectors.ts`, and the one any model's test can
import in `frontend/src/__tests__/helpers/busVectors.ts`), and so is the Python
one (`replay_vector` in `test_i2c_slaves.py`). For a QEMU device model the
events are `I2C_START_SEND` and `I2C_START_RECV` for `start`, `I2C_WRITE` for
each byte of `send`, `I2C_READ` for each byte of `recv`, `I2C_FINISH` for
`stop`.

## Rounding

An input times a sensitivity is rarely a whole number of counts. The rule is
round half away from zero, then saturate at the ends of the register
(-32768 and 32767 for a 16-bit output), so +x and -x give counts of the same
size. JavaScript's `Math.round` and Python's `round` both do something else at
exactly one half, and a gyro slider that moves in steps of 1 deg/s lands there
at 65.5 LSB per deg/s, which is why a vector pins it.

The BMP280 has no sensitivity: its raw values are the ones the compensation
formulas of the datasheet turn back into the input, found by searching them.
The search aims at the temperature in hundredths of a degree, and half a
hundredth rounds up (24.125 deg C is 2413), which is what `Math.round` does
and Python's `round` does not. A vector pins that too.
