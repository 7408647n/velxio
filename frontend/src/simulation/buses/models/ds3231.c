/*
 * ds3231.c: the DS3231 real-time clock as one portable model, on the
 * velxio-chip.h ABI.
 *
 * Project i2c-model-fidelity-2026-09, P5 (decision O4, open with the owner):
 * the evidence for moving the I2C register models to ONE compiled model, the
 * way buses/models/microsd.c already replaced three SD card twins. Today the
 * chip exists twice, kept equal only by replaying the same bus vectors:
 *
 *   frontend/src/simulation/I2CBusManager.ts        VirtualDS3231 (the tab)
 *   backend/app/services/esp32_i2c_slaves.py        DS3231Slave (the worker)
 *
 * This file is a line-for-line port of the two, and it replays the same file,
 * test/fixtures/i2c-vectors/ds3231.json, in both hosts. It is NOT the model a
 * user runs: the part hosts it only behind a flag that is off by default
 * (simulation/parts/wasmI2cModels.ts, backend/app/services/wasm_i2c_models.py).
 *
 * ── What the chip needs from a host, and how it gets it ─────────────────
 *
 * A clock chip keeps the host's time until the sketch sets one (decision D7),
 * so it needs three things the ABI carries no call for. They come in as
 * attributes the host answers live, at the moment the chip reads them, so the
 * model asks exactly when the two copies call their `clock()` and
 * `buildTimes()`, and never more often:
 *
 *   host_ms      the host's wall clock as the calendar on the user's wall
 *                reads it, milliseconds since 00:00 of 1 January 1970 of that
 *                calendar (RtcOptions.clock, TabClock). A double holds it
 *                exactly until the year 287,396.
 *   temperature  the sensor panel, degrees Celsius.
 *   build_times  string: the firmware's __DATE__ and __TIME__ pairs as
 *                "YYYYMMDDhhmmss", any other character between them. Asked
 *                only when the sketch has written a time, because finding
 *                them means scanning the firmware image.
 *
 * The guest clock (vx_sim_now_nanos) is not this chip's clock: the DS3231
 * counts wall time, whatever the guest's speed, as both copies do.
 *
 * ── The register file a host mirrors ────────────────────────────────────
 *
 * A host that answers the guest from a copy of the registers (the Raspberry
 * Pi relay) takes them from chip_dump_registers(): 256 bytes, the time brought
 * to the present, the temperature as it is now rather than as the transfer in
 * progress latched it. This is VirtualRtc.dumpRegisters().
 */
#include "velxio-chip.h"

#define LAST_REGISTER 0x12u
#define MS_DAY        86400000LL
/* "No constraint" for an alarm field whose mask bit is set. An hour of -1 is
 * an alarm in the other hour mode than the clock, which never matches. */
#define ANY           (-100)
#define MAX_BUILDS    16

/* The write rules, DS3231_RULES in I2CBusManager.ts (datasheet 19-5170 rev 10,
 * Figure 1 and the Control and Status registers). 0 = not writable. */
static const uint8_t WRITE_MASK[LAST_REGISTER + 1] = {
  0x7F, 0x7F, 0x7F, 0x07, 0x3F, 0x9F, 0xFF,   /* 00-06: the time; no CH bit */
  0xFF, 0xFF, 0xFF, 0xFF,                     /* 07-0A: alarm 1 */
  0xFF, 0xFF, 0xFF,                           /* 0B-0D: alarm 2 */
  0xDF,                                       /* 0E: CONTROL, CONV never stored */
  0x08,                                       /* 0F: STATUS, only EN32kHz */
  0xFF,                                       /* 10: aging offset */
  0x00, 0x00,                                 /* 11-12: temperature, read only */
};
/* OSF, A2F and A1F can only be written to 0. */
#define STATUS_WRITE_ZERO_TO_CLEAR 0x83u

typedef struct {
  int64_t year, month, day, hour, minute, second;
} build_time;

static struct {
  vx_attr a_host_ms, a_temperature, a_build_times;

  /* RtcCounters: seconds, minutes, hours, day of week, date, month, year. */
  uint8_t time[7];
  /* Host time at which the second the registers show began. */
  int64_t tick_at;
  /* The counters are the host's clock. */
  bool following;
  /* A time register was written in the write phase that is open. */
  bool written;

  /* Alarm 1 (07-0A), alarm 2 (0B-0D), CONTROL, STATUS and the aging offset. */
  uint8_t regs[LAST_REGISTER + 1];

  /* VirtualRtc: the time and the temperature as the START of this transfer
   * found them, and where the transfer is. */
  uint8_t latched[7];
  uint8_t temperature[2];
  bool latch_due;
  bool first_byte;
  uint8_t pointer;

  uint8_t dump[256];
  char builds_text[MAX_BUILDS * 16];
} rtc;

/* ── Arithmetic the two copies do in doubles and Python ints ─────────────── */

static int64_t fdiv(int64_t a, int64_t b) {
  int64_t q = a / b;
  if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
  return q;
}

static int64_t fmod_(int64_t a, int64_t b) { return a - fdiv(a, b) * b; }

static uint8_t bcd(int64_t n) { return (uint8_t)((((n / 10) % 10) << 4) | (n % 10)); }

static int64_t bin(uint8_t v) { return ((v >> 4) & 0xF) * 10 + (v & 0xF); }

/* Days since 1 January 1970 of a date, and back: the same civil-calendar
 * arithmetic as rtcDaysOf and _rtc_days_of, a month 0 or a 31 June a sketch
 * wrote included, so the three hosts count to the same day. */
static int64_t days_of(int64_t year, int64_t month, int64_t day) {
  int64_t m0 = month - 1;
  int64_t m = fmod_(m0, 12);
  int64_t y = year + fdiv(m0, 12) - (m < 2 ? 1 : 0);
  int64_t era = fdiv(y, 400);
  int64_t yoe = y - era * 400;
  int64_t doy = fdiv(153 * (m + (m > 1 ? -2 : 10)) + 2, 5);
  int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468 + (day - 1);
}

static void date_of(int64_t days, int64_t* year, int64_t* month, int64_t* day) {
  int64_t z = days + 719468;
  int64_t era = fdiv(z, 146097);
  int64_t doe = z - era * 146097;
  int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int64_t mp = (5 * doy + 2) / 153;
  *month = mp < 10 ? mp + 3 : mp - 9;
  *year = yoe + era * 400 + (*month <= 2 ? 1 : 0);
  *day = doy - (153 * mp + 2) / 5 + 1;
}

/* The day-of-week register after `days` midnights. It counts 1 to 7 and back
 * to 1 from whatever it holds; a 0 becomes 1 at the first midnight. */
static uint8_t weekday_after(uint8_t weekday, int64_t days) {
  if (days == 0) return weekday;
  if (weekday == 0) return days < 0 ? 0 : (uint8_t)(((days - 1) % 7) + 1);
  return (uint8_t)(fmod_((int64_t)weekday - 1 + days, 7) + 1);
}

/* Hours as the register holds them: bit 6 selects 12-hour mode, bit 5 is PM. */
static int64_t hour_of(uint8_t reg) {
  if (reg & 0x40) return (bin(reg & 0x1F) % 12) + ((reg & 0x20) ? 12 : 0);
  return bin(reg & 0x3F);
}

static uint8_t hour_register(int64_t hour, bool twelve_hour) {
  if (!twelve_hour) return bcd(hour);
  return (uint8_t)(0x40 | (hour >= 12 ? 0x20 : 0) | bcd(hour % 12 ? hour % 12 : 12));
}

/* ── The host's clock ────────────────────────────────────────────────────── */

static int64_t clock_ms(void) {
  double ms = vx_attr_read(rtc.a_host_ms);
  return (int64_t)__builtin_floor(ms);
}

/* ── RtcCounters ─────────────────────────────────────────────────────────── */

static int64_t shown_time(void) {
  const uint8_t* r = rtc.time;
  int64_t days = days_of(2000 + bin(r[6]), bin(r[5] & 0x1F), bin(r[4] & 0x3F));
  return days * MS_DAY + hour_of(r[2]) * 3600000LL + bin(r[1] & 0x7F) * 60000LL +
         bin(r[0] & 0x7F) * 1000LL;
}

/* Put a time in the registers. The 12-hour mode and the day of week stay. */
static void show(int64_t t) {
  uint8_t* r = rtc.time;
  int64_t days = fdiv(t, MS_DAY);
  int64_t ms = t - days * MS_DAY;
  int64_t year, month, day;
  date_of(days, &year, &month, &day);
  /* The year register counts 00 to 99, and the century bit turns over with it. */
  int64_t centuries = fdiv(year - 2000, 100);
  r[0] = (uint8_t)((r[0] & 0x80) | bcd((ms / 1000) % 60));
  r[1] = bcd((ms / 60000) % 60);
  r[2] = hour_register(ms / 3600000, (r[2] & 0x40) != 0);
  r[4] = bcd(day);
  r[5] = (uint8_t)(((r[5] & 0x80) ^ ((centuries & 1) ? 0x80 : 0)) | bcd(month));
  r[6] = bcd(fmod_(year - 2000, 100));
}

static void check_alarms(int64_t from, int64_t to, uint8_t weekday);

static void count(int64_t from, int64_t to, bool alarms) {
  if (to == from) return;
  uint8_t weekday = rtc.time[3];
  rtc.time[3] = weekday_after(weekday, fdiv(to, MS_DAY) - fdiv(from, MS_DAY));
  show(to);
  if (alarms && to > from) check_alarms(from, to, weekday);
}

/* Bring the counters to the present. */
static void sync(void) {
  int64_t now = clock_ms();
  if (rtc.following) {
    int64_t to = fdiv(now, 1000) * 1000;
    count(shown_time(), to, true);
    rtc.tick_at = to;
    return;
  }
  int64_t seconds = fdiv(now - rtc.tick_at, 1000);
  if (seconds <= 0) {
    /* The host's clock was set back: the chip does not count backwards. */
    if (now < rtc.tick_at) rtc.tick_at = now;
    return;
  }
  int64_t from = shown_time();
  count(from, from + seconds * 1000, true);
  rtc.tick_at += seconds * 1000;
}

/* A byte written to one of the seven registers, already cut to its bits. */
static void counters_write(uint8_t reg, uint8_t value) {
  rtc.time[reg] = value;
  /* The day of week is a counter of its own: writing it sets no time. */
  if (reg == 3) return;
  rtc.following = false;
  rtc.written = true;
  /* "The countdown chain is reset whenever the seconds register is written." */
  if (reg == 0) rtc.tick_at = clock_ms();
}

/* "YYYYMMDDhhmmss", any non-digit between two of them. */
static int read_build_times(build_time* out) {
  uint32_t len = vx_attr_string_read(rtc.a_build_times, rtc.builds_text, sizeof rtc.builds_text);
  if (len >= sizeof rtc.builds_text) len = sizeof rtc.builds_text - 1;
  rtc.builds_text[len] = 0;
  int n = 0;
  const char* p = rtc.builds_text;
  while (*p && n < MAX_BUILDS) {
    int digits = 0;
    int64_t f[14];
    while (digits < 14 && p[digits] >= '0' && p[digits] <= '9') {
      f[digits] = p[digits] - '0';
      digits++;
    }
    if (digits == 14) {
      out[n].year = f[0] * 1000 + f[1] * 100 + f[2] * 10 + f[3];
      out[n].month = f[4] * 10 + f[5];
      out[n].day = f[6] * 10 + f[7];
      out[n].hour = f[8] * 10 + f[9];
      out[n].minute = f[10] * 10 + f[11];
      out[n].second = f[12] * 10 + f[13];
      n++;
      p += 14;
    } else {
      p += digits ? digits : 1;
    }
  }
  return n;
}

/* The write phase ended: what was written is a time now. A time that is the
 * compile time of the firmware means "now" (decision D7), and the counters go
 * back to the host's clock; anything else is kept. */
static void commit(void) {
  if (!rtc.written) return;
  rtc.written = false;
  int64_t set = shown_time();
  int64_t days = fdiv(set, MS_DAY);
  int64_t ms = set - days * MS_DAY;
  int64_t year, month, day;
  date_of(days, &year, &month, &day);
  int64_t hour = ms / 3600000, minute = (ms / 60000) % 60, second = (ms / 1000) % 60;
  build_time builds[MAX_BUILDS];
  int n = read_build_times(builds);
  bool built = false;
  for (int i = 0; i < n && !built; i++) {
    const build_time* b = &builds[i];
    built = b->year == year && b->month == month && b->day == day && b->hour == hour &&
            b->minute == minute && b->second == second;
  }
  if (!built) return;
  rtc.following = true;
  int64_t to = fdiv(clock_ms(), 1000) * 1000;
  /* Set back then, running since: no alarm is owed for the time in between. */
  count(set, to, false);
  rtc.tick_at = to;
}

/* ── Alarms ──────────────────────────────────────────────────────────────── */

static bool day_matches(int64_t days, int64_t first_day, uint8_t weekday_at_from,
                        uint8_t day_register) {
  if (day_register & 0x80) return true;
  if (day_register & 0x40)
    return weekday_after(weekday_at_from, days - first_day) == (day_register & 0x0F);
  int64_t y, m, d;
  date_of(days, &y, &m, &d);
  return d == bin(day_register & 0x3F);
}

/* Whether an alarm's registers matched the clock at one of the seconds it
 * counted through, (from, to]: rtcAlarmMatched, field by field and not second
 * by second. */
static bool alarm_matched(int64_t from, int64_t to, uint8_t weekday_at_from, int64_t second,
                          int64_t minute, int64_t hour, uint8_t day_register) {
  int64_t first_day = fdiv(from, MS_DAY);
  /* Longer ago than a year the chip would have matched as well; nobody waits. */
  int64_t t = (from > to - 400 * MS_DAY ? from : to - 400 * MS_DAY) + 1000;
  while (t <= to) {
    int64_t days = fdiv(t, MS_DAY);
    int64_t day = days * MS_DAY;
    if (!day_matches(days, first_day, weekday_at_from, day_register)) {
      t = day + MS_DAY;
      continue;
    }
    int64_t h = (t - day) / 3600000;
    if (hour != ANY && h != hour) {
      t = h < hour ? day + hour * 3600000 : day + MS_DAY;
      continue;
    }
    int64_t m = ((t - day) / 60000) % 60;
    if (minute != ANY && m != minute) {
      int64_t hour_start = day + h * 3600000;
      t = m < minute ? hour_start + minute * 60000 : hour_start + 3600000;
      continue;
    }
    int64_t s = ((t - day) / 1000) % 60;
    if (second != ANY && s != second) {
      int64_t minute_start = day + h * 3600000 + m * 60000;
      t = s < second ? minute_start + second * 1000 : minute_start + 60000;
      continue;
    }
    return true;
  }
  return false;
}

static int64_t alarm_field(uint8_t reg) { return (reg & 0x80) ? ANY : bin(reg & 0x7F); }

static int64_t alarm_hour(uint8_t reg, bool twelve_hour) {
  if (reg & 0x80) return ANY;
  return (((reg & 0x40) != 0) == twelve_hour) ? hour_of(reg & 0x7F) : -1;
}

/* A1F and A2F are set when the clock counts through a matching second,
 * whether or not the interrupt is enabled. The INT/SQW pin is not driven. */
static void check_alarms(int64_t from, int64_t to, uint8_t weekday) {
  uint8_t* r = rtc.regs;
  bool twelve = (rtc.time[2] & 0x40) != 0;
  if (alarm_matched(from, to, weekday, alarm_field(r[0x07]), alarm_field(r[0x08]),
                    alarm_hour(r[0x09], twelve), r[0x0A]))
    r[0x0F] |= 0x01;
  /* Alarm 2 has no seconds register: it matches at second 00. */
  if (alarm_matched(from, to, weekday, 0, alarm_field(r[0x0B]), alarm_hour(r[0x0C], twelve),
                    r[0x0D]))
    r[0x0F] |= 0x02;
}

/* ── Temperature ─────────────────────────────────────────────────────────── */

/* Quarter degrees, two's complement: q = round(T x 4), half away from zero,
 * saturated at -128.00 and +127.75. 0x11 = q >> 2, 0x12 = (q & 3) << 6. */
static void temperature_registers(uint8_t out[2]) {
  double quarters = vx_attr_read(rtc.a_temperature) * 4.0;
  int32_t q = 0;
  if (quarters == quarters && quarters - quarters == 0.0) {
    double size = __builtin_floor(__builtin_fabs(quarters) + 0.5);
    if (size > 512.0) size = 512.0;
    q = quarters < 0 ? -(int32_t)size : (int32_t)size;
    if (q > 511) q = 511;
  }
  out[0] = (uint8_t)((q >> 2) & 0xFF);
  out[1] = (uint8_t)((q & 3) << 6);
}

/* ── VirtualRtc: the bus ─────────────────────────────────────────────────── */

static uint8_t read_register(uint8_t reg) {
  if (reg == 0x11 || reg == 0x12) return rtc.temperature[reg - 0x11];
  return reg <= 0x10 ? rtc.regs[reg] : 0x00;
}

static void write_register(uint8_t reg, uint8_t value) {
  if (reg > LAST_REGISTER || WRITE_MASK[reg] == 0) return;
  uint8_t mask = WRITE_MASK[reg];
  if (reg < 7) {
    counters_write(reg, value & mask);
    return;
  }
  uint8_t flags = reg == 0x0F ? STATUS_WRITE_ZERO_TO_CLEAR : 0;
  rtc.regs[reg] = (uint8_t)((rtc.regs[reg] & flags & value) | (value & mask));
}

static void latch(void) {
  sync();
  for (int i = 0; i < 7; i++) rtc.latched[i] = rtc.time[i];
  temperature_registers(rtc.temperature);
}

/* "The user buffers are synchronized to the internal registers on any START
 * and when the register pointer rolls over to zero." A repeated START ends a
 * write phase as a STOP does. */
static void begin(void) {
  commit();
  latch();
  rtc.latch_due = false;
}

static uint8_t after(uint8_t reg) { return reg == LAST_REGISTER ? 0 : (uint8_t)(reg + 1); }

static bool on_connect(void* ud, uint8_t addr, bool is_read) {
  (void)ud;
  (void)addr;
  (void)is_read;
  /* The pointer is not reset: a write-then-read relies on it. */
  rtc.first_byte = true;
  begin();
  return true;
}

static bool on_write(void* ud, uint8_t byte) {
  (void)ud;
  /* For a host that does not say where a transfer begins: the first byte
   * after a STOP begins one, and after a write the next byte read does. */
  if (rtc.first_byte && rtc.latch_due) begin();
  rtc.latch_due = true;
  if (rtc.first_byte) {
    /* Past the last register the datasheet says nothing: the chip is given
     * the byte. */
    rtc.pointer = byte;
    rtc.first_byte = false;
    return true;
  }
  uint8_t reg = rtc.pointer;
  rtc.pointer = after(reg);
  write_register(reg, byte);
  return true;
}

static uint8_t on_read(void* ud) {
  (void)ud;
  if (rtc.latch_due) begin();
  uint8_t reg = rtc.pointer;
  uint8_t value = reg < 7 ? rtc.latched[reg] : read_register(reg);
  rtc.pointer = after(reg);
  if (rtc.pointer == 0) latch();
  return value;
}

/* The pointer survives the STOP: QEMU ends every write phase this way, the
 * one before a repeated START included. What a write phase wrote to the time
 * registers is a time from here. */
static void on_stop(void* ud) {
  (void)ud;
  commit();
  rtc.first_byte = true;
  rtc.latch_due = true;
}

/* The registers as a read would find them now, for a host that answers the
 * guest from a copy. Returns the address of 256 bytes in linear memory. */
__attribute__((export_name("chip_dump_registers"))) uint8_t* chip_dump_registers(void) {
  sync();
  for (int i = 0; i < 256; i++) rtc.dump[i] = 0;
  uint8_t now_temperature[2];
  temperature_registers(now_temperature);
  for (uint8_t reg = 7; reg <= LAST_REGISTER; reg++)
    rtc.dump[reg] = (reg == 0x11 || reg == 0x12) ? now_temperature[reg - 0x11] : rtc.regs[reg];
  for (int i = 0; i < 7; i++) rtc.dump[i] = rtc.time[i];
  return rtc.dump;
}

void chip_setup(void) {
  rtc.a_host_ms = vx_attr_register("host_ms", 0);
  rtc.a_temperature = vx_attr_register("temperature", 25);
  rtc.a_build_times = vx_attr_register_string("build_times", "");

  /* CONTROL 0x1C and STATUS 0x08 with OSF clear: a module somebody set and
   * whose battery kept it running (DS3231_RULES.power_on says why). */
  rtc.regs[0x0E] = 0x1C;
  rtc.regs[0x0F] = 0x08;

  /* Power-on: the host's time. Monday = 1 is what RTClib writes to a DS3231
   * and compares an alarm on a weekday with (dowToDS3231). */
  int64_t now = clock_ms();
  rtc.following = true;
  show(fdiv(now, 1000) * 1000);
  rtc.time[3] = (uint8_t)(fmod_(fdiv(now, MS_DAY) + 3, 7) + 1);
  rtc.tick_at = fdiv(now, 1000) * 1000;
  rtc.latch_due = true;
  rtc.first_byte = true;

  vx_i2c_config cfg = {
    .address = 0x68,
    .scl = vx_pin_register("SCL", VX_INPUT),
    .sda = vx_pin_register("SDA", VX_INPUT),
    .on_connect = on_connect,
    .on_read = on_read,
    .on_write = on_write,
    .on_stop = on_stop,
    .user_data = 0,
  };
  vx_i2c_attach(&cfg);
}
