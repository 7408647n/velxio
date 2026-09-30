/**
 * I2C parts answered by their compiled model (project
 * i2c-model-fidelity-2026-09, P5, decision O4).
 *
 * The microSD card already runs as ONE model in C (buses/models/microsd.c)
 * that the tab, the QEMU workers and the Linux-board host all run. The two
 * real-time clocks and the BMP280 now do too: buses/models/ds1307.c and
 * ds3231.c (on the shared buses/models/rtc.h) and bmp280.c, hosted here by
 * ChipRuntime in place of VirtualDS1307, VirtualDS3231 and VirtualBMP280, and
 * in the worker from the same bytes (backend/app/services/wasm_i2c_models.py),
 * which the part puts in the worker's record (`wasmB64`).
 *
 * On by default. `?i2cwasm=off` or localStorage `velxio.i2cwasm = 'off'` goes
 * back to the hand-written models in both hosts; a comma list
 * (`?i2cwasm=ds3231`) runs only the models it names. A model that cannot be
 * built keeps the hand-written one for that part, and a record without the
 * bytes keeps the worker's Python twin, so neither the flag nor a broken
 * build can cost a user the part.
 *
 * The bytes are compiled into the bundle (buses/models/i2cModelBytes
 * .generated.ts, written by build.sh): a part attaches synchronously and has
 * to answer the guest's first START, so its model cannot wait for a fetch.
 */
import { ChipInstance } from '../customChips/ChipRuntime';
import { PinManager } from '../PinManager';
import { I2C_MODEL_WASM_B64 } from '../buses/models/i2cModelBytes.generated';
import {
  DS1307_RULES,
  DS3231_RULES,
  hostWallClock,
  type I2CDevice,
  type RtcDateTime,
  type RtcOptions,
} from '../I2CBusManager';

export type WasmI2cModelName = keyof typeof I2C_MODEL_WASM_B64;

/** The models that run compiled unless the flag says otherwise. */
const DEFAULT_ON: readonly WasmI2cModelName[] = ['ds1307', 'ds3231', 'bmp280'];
/** Flag values that name no model at all. */
const OFF = new Set(['off', 'none', '0', 'false']);

let testOverride: ReadonlySet<string> | null = null;

/** Test seam: the models the flag names, or null for the real flag. */
export function setWasmI2cModelsForTest(names: readonly string[] | null): void {
  testOverride = names === null ? null : new Set(names);
}

/** Whether `name` runs from its compiled model. */
export function wasmI2cModelEnabled(name: string): boolean {
  if (testOverride !== null) return testOverride.has(name);
  let v: string | null = null;
  try {
    if (typeof window !== 'undefined' && window.location) {
      v = new URLSearchParams(window.location.search).get('i2cwasm');
    }
    if (v === null && typeof localStorage !== 'undefined')
      v = localStorage.getItem('velxio.i2cwasm');
  } catch {
    /* SecurityError on localStorage, missing globals in tests */
  }
  if (v === null || v.trim() === '') return (DEFAULT_ON as readonly string[]).includes(name);
  const names = v
    .split(',')
    .map((s) => s.trim().toLowerCase())
    .filter(Boolean);
  if (names.some((n) => OFF.has(n))) return false;
  return names.includes(name);
}

const modules = new Map<string, WebAssembly.Module | null>();

function decodeB64(b64: string): Uint8Array {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/**
 * The compiled model, built once per page on first use: a few kilobytes,
 * which every current browser compiles synchronously on the main thread.
 * Null when it cannot be built, and then the part keeps its hand-written
 * model (said once).
 */
export function wasmI2cModule(name: string): WebAssembly.Module | null {
  if (modules.has(name)) return modules.get(name)!;
  const b64 = (I2C_MODEL_WASM_B64 as Record<string, string>)[name];
  let m: WebAssembly.Module | null = null;
  if (b64) {
    try {
      m = new WebAssembly.Module(decodeB64(b64) as BufferSource);
    } catch (e) {
      console.warn(`[i2c-models] ${name}.wasm could not be compiled; the part keeps its own model`, e);
    }
  }
  modules.set(name, m);
  return m;
}

/** The model's bytes for the worker's record. */
export function wasmI2cModelB64(name: string): string | null {
  return (I2C_MODEL_WASM_B64 as Record<string, string>)[name] ?? null;
}

/** Test seam: hand a compiled model over directly (null: it cannot be built). */
export function primeWasmI2cModel(name: string, module: WebAssembly.Module | null): void {
  modules.set(name, module);
}

/** Test seam: forget the compiled models. */
export function resetWasmI2cModelsForTest(): void {
  modules.clear();
}

const pad2 = (n: number) => String(n).padStart(2, '0');

/** Build times as the model reads them: "YYYYMMDDhhmmss" each. */
function buildTimesText(times: readonly RtcDateTime[]): string {
  return times
    .map(
      (b) =>
        `${String(b.year).padStart(4, '0')}${pad2(b.month)}${pad2(b.day)}` +
        `${pad2(b.hour)}${pad2(b.minute)}${pad2(b.second)}`,
    )
    .join(' ');
}

/**
 * A clock chip from buses/models/rtc.h, with the API of VirtualRtc: the same
 * options (the host's clock, the build times), the bus, and the register dump
 * the Pi relay mirrors. Every call on the bus goes through ChipRuntime's own
 * I2C device, the one the fabric reaches a custom chip through, so the model
 * hears what a custom chip would.
 *
 * The host's clock and the panel's temperature are pushed into the model's
 * `rtc_inputs` (the export chip_inputs), the clock before every event: the
 * model never calls out for them. Only the build times are asked for, when a
 * sketch has written a time, because finding them scans the firmware image.
 */
abstract class WasmRtc implements I2CDevice {
  public address = 0x68;
  /** The register the pointer wraps after (I2cTarget.pointerWrapsAfter). */
  readonly pointerWrapsAfter: number;

  protected readonly chip: ChipInstance;
  private readonly dev: NonNullable<ReturnType<ChipInstance['i2cDevice']>>;
  private readonly clock: () => number;
  private readonly inputsAt: number;
  private view: DataView | null = null;

  protected constructor(
    module: WebAssembly.Module,
    options: RtcOptions,
    temperature: number,
    rules: { readonly power_on: Readonly<Record<number, number>>; readonly last_register: number },
  ) {
    this.pointerWrapsAfter = rules.last_register;
    this.clock = options.clock ?? hostWallClock;
    const buildTimes = options.buildTimes ?? (() => []);
    this.chip = ChipInstance.createSync({
      wasm: module,
      // The chip is on no board's pins: the part's own attach puts it on the
      // bus, as it put the hand-written model there.
      pinManager: new PinManager(),
      liveAttrs: (name) =>
        name === 'build_times' ? buildTimesText(buildTimes()) : undefined,
    });
    this.inputsAt = (this.chip.exports.chip_inputs as () => number)();
    // The registers behind the time power on as the rules table of the chip
    // says, the table the vectors hold the model to (rtc.h, chip_power_on).
    const table = (this.chip.exports.chip_power_on as () => number)();
    const bytes = new Uint8Array(this.chip.memory!.buffer, table, this.pointerWrapsAfter + 1);
    for (const [reg, value] of Object.entries(rules.power_on)) {
      const r = Number(reg);
      if (r >= 7 && r < bytes.length) bytes[r] = value;
    }
    this.pushTemperature(temperature);
    // Power-on reads the clock.
    this.pushClock();
    this.chip.start();
    const dev = this.chip.i2cDevice(this.address);
    if (!dev) throw new Error('the RTC model attached no I2C device at 0x68');
    this.dev = dev;
  }

  /** rtc_inputs, re-read if the memory ever grew under it. */
  private inputs(): DataView {
    const buffer = this.chip.memory!.buffer;
    if (this.view === null || this.view.buffer !== buffer) {
      this.view = new DataView(buffer, this.inputsAt, 16);
    }
    return this.view;
  }

  private pushClock(): void {
    this.inputs().setFloat64(0, this.clock(), true);
  }

  protected pushTemperature(celsius: number): void {
    this.inputs().setFloat64(8, celsius, true);
  }

  start(read: boolean): void {
    this.pushClock();
    this.dev.connect(this.address, read);
  }

  writeByte(value: number): boolean {
    this.pushClock();
    return this.dev.writeByte(value);
  }

  readByte(): number {
    this.pushClock();
    return this.dev.readByte();
  }

  stop(): void {
    this.pushClock();
    this.dev.stop();
  }

  /** The registers as a read would find them now, for a host that answers from a copy. */
  dumpRegisters(): Uint8Array {
    this.pushClock();
    const ptr = (this.chip.exports.chip_dump_registers as () => number)();
    return new Uint8Array(this.chip.memory!.buffer, ptr, 256).slice();
  }

  dispose(): void {
    this.chip.dispose();
  }
}

/** The DS1307 from buses/models/ds1307.c, in place of VirtualDS1307. */
export class WasmDS1307 extends WasmRtc {
  constructor(module: WebAssembly.Module, options: RtcOptions = {}) {
    super(module, options, 25, DS1307_RULES);
  }
}

/** The DS3231 from buses/models/ds3231.c, in place of VirtualDS3231. */
export class WasmDS3231 extends WasmRtc {
  private celsius = 25.0;

  constructor(module: WebAssembly.Module, options: RtcOptions = {}) {
    super(module, options, 25, DS3231_RULES);
  }

  /** The panel's temperature, pushed into the model as it moves. */
  get temperatureC(): number {
    return this.celsius;
  }

  set temperatureC(celsius: number) {
    this.celsius = celsius;
    this.pushTemperature(celsius);
  }
}

/**
 * The BMP280 from buses/models/bmp280.c, in place of VirtualBMP280: the same
 * address choice, the panel's two values, the note to the board's monitor
 * when the data registers are read before the chip ever measured, and the
 * register dump the Pi relay mirrors.
 *
 * The panel's values and the address are pushed into the model's `bmp280_io`
 * (the export chip_inputs); the model counts the reads of a chip that never
 * measured there (asleep_reads), and the part says it once a run.
 */
export class WasmBMP280 implements I2CDevice {
  public address: number;
  /** The data registers were read before the chip ever measured, for the first time in this run. */
  onAsleepRead: (() => void) | null = null;

  private readonly chip: ChipInstance;
  private readonly dev: NonNullable<ReturnType<ChipInstance['i2cDevice']>>;
  private readonly ioAt: number;
  private view: DataView | null = null;
  private asleepReads = 0;
  private asleepReadSaid = false;
  // Where the sensor panel starts (sensorControlConfig, bmp280), as bmp280.c.
  private temperature = 24.0;
  private pressure = 1013.25;

  constructor(module: WebAssembly.Module, address = 0x76) {
    this.address = address === 0x77 ? 0x77 : 0x76;
    this.chip = ChipInstance.createSync({ wasm: module, pinManager: new PinManager() });
    this.ioAt = (this.chip.exports.chip_inputs as () => number)();
    this.io().setUint32(16, this.address, true);
    this.io().setFloat64(0, this.temperature, true);
    this.io().setFloat64(8, this.pressure, true);
    this.chip.start();
    const dev = this.chip.i2cDevice(this.address);
    if (!dev) throw new Error(`the BMP280 model attached no I2C device at 0x${this.address.toString(16)}`);
    this.dev = dev;
  }

  /** bmp280_io, re-read if the memory ever grew under it. */
  private io(): DataView {
    const buffer = this.chip.memory!.buffer;
    if (this.view === null || this.view.buffer !== buffer) {
      this.view = new DataView(buffer, this.ioAt, 24);
    }
    return this.view;
  }

  get temperatureC(): number {
    return this.temperature;
  }
  set temperatureC(v: number) {
    this.temperature = v;
    this.io().setFloat64(0, v, true);
  }

  get pressureHPa(): number {
    return this.pressure;
  }
  set pressureHPa(v: number) {
    this.pressure = v;
    this.io().setFloat64(8, v, true);
  }

  start(read: boolean): void {
    this.dev.connect(this.address, read);
  }

  writeByte(value: number): boolean {
    return this.dev.writeByte(value);
  }

  readByte(): number {
    const value = this.dev.readByte();
    const reads = this.io().getUint32(20, true);
    if (reads !== this.asleepReads) {
      this.asleepReads = reads;
      if (!this.asleepReadSaid) {
        this.asleepReadSaid = true;
        this.onAsleepRead?.();
      }
    }
    return value;
  }

  stop(): void {
    this.dev.stop();
  }

  /** A new run reads a chip that still has not measured: its monitor is told as well. */
  boardReset(): void {
    this.asleepReadSaid = false;
  }

  /** The registers as a read would find them now, for a host that answers from a copy. */
  dumpRegisters(): Uint8Array {
    const ptr = (this.chip.exports.chip_dump_registers as () => number)();
    return new Uint8Array(this.chip.memory!.buffer, ptr, 256).slice();
  }

  dispose(): void {
    this.chip.dispose();
  }
}
