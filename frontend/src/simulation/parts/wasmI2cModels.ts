/**
 * I2C parts answered by their compiled model instead of a hand-written one
 * (project i2c-model-fidelity-2026-09, P5, decision O4 open with the owner).
 *
 * The microSD card already runs as ONE model in C (buses/models/microsd.c)
 * that the tab, the QEMU workers and the Linux-board host all run. The I2C
 * register models still exist twice, in TypeScript here and in Python in the
 * worker (esp32_i2c_slaves.py), kept equal only by the bus vectors of
 * test/fixtures/i2c-vectors. This module is the tab's half of the evidence
 * that they can move the same way: buses/models/ds3231.c, hosted by
 * ChipRuntime, as a drop-in for VirtualDS3231.
 *
 * Behind a flag that is OFF by default, so nothing changes for a user:
 * `?i2cwasm=ds3231` or localStorage `velxio.i2cwasm = 'ds3231'` (a comma
 * list names several). With it the tab answers from the model and puts the
 * same bytes in the worker's record (`wasmB64`), which makes the worker run
 * them too (esp32_i2c_slaves.rtc_slave).
 */
import { ChipInstance } from '../customChips/ChipRuntime';
import { PinManager } from '../PinManager';
import { loadBusChip, busChipB64 } from '../buses/busChips';
import { hostWallClock, type I2CDevice, type RtcDateTime, type RtcOptions } from '../I2CBusManager';

let testOverride: ReadonlySet<string> | null = null;

/** Test seam: the models the flag names, or null for the real flag. */
export function setWasmI2cModelsForTest(names: readonly string[] | null): void {
  testOverride = names === null ? null : new Set(names);
}

/** Whether the flag asks for `name` to run from its compiled model. */
export function wasmI2cModelEnabled(name: string): boolean {
  if (testOverride !== null) return testOverride.has(name);
  try {
    let v: string | null = null;
    if (typeof window !== 'undefined' && window.location) {
      v = new URLSearchParams(window.location.search).get('i2cwasm');
    }
    if (v === null && typeof localStorage !== 'undefined')
      v = localStorage.getItem('velxio.i2cwasm');
    return (
      !!v &&
      v
        .split(',')
        .map((s) => s.trim())
        .includes(name)
    );
  } catch {
    /* SecurityError on localStorage, missing globals in tests */
    return false;
  }
}

const modules = new Map<string, WebAssembly.Module>();
const compiling = new Map<string, Promise<WebAssembly.Module | null>>();

/**
 * Fetch and compile a model once per page. A part attaches synchronously and
 * has to answer the guest's first START, so the compile is done ahead of it:
 * the registry starts it when the flag is on, and a part that finds it not
 * ready yet keeps the hand-written model for that run.
 */
export function prepareWasmI2cModel(name: string): Promise<WebAssembly.Module | null> {
  const have = modules.get(name);
  if (have) return Promise.resolve(have);
  let p = compiling.get(name);
  if (!p) {
    p = loadBusChip(name).then(async (bytes) => {
      if (!bytes) return null;
      const m = await WebAssembly.compile(bytes as BufferSource);
      modules.set(name, m);
      return m;
    });
    compiling.set(name, p);
  }
  return p;
}

/** The compiled model, if it is ready. */
export function wasmI2cModule(name: string): WebAssembly.Module | null {
  return modules.get(name) ?? null;
}

/** The model's bytes for the worker's record, if they are here. */
export function wasmI2cModelB64(name: string): string | null {
  return busChipB64(name);
}

/** Test seam: hand the compiled model over directly. */
export function primeWasmI2cModel(name: string, module: WebAssembly.Module): void {
  modules.set(name, module);
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
 * The DS3231 from buses/models/ds3231.c, with the API of VirtualDS3231: the
 * same options (the host's clock, the build times), `temperatureC`, the bus,
 * and the register dump the Pi relay mirrors. Every call on the bus goes
 * through ChipRuntime's own I2C device, the one the fabric reaches a custom
 * chip through, so the model hears what a custom chip would.
 */
export class WasmDS3231 implements I2CDevice {
  public address = 0x68;
  public temperatureC = 25.0;

  private readonly chip: ChipInstance;
  private readonly dev: NonNullable<ReturnType<ChipInstance['i2cDevice']>>;

  constructor(module: WebAssembly.Module, options: RtcOptions = {}) {
    const clock = options.clock ?? hostWallClock;
    const buildTimes = options.buildTimes ?? (() => []);
    this.chip = ChipInstance.createSync({
      wasm: module,
      // The chip is on no board's pins: the part's own attach puts it on the
      // bus, as it put VirtualDS3231 there.
      pinManager: new PinManager(),
      liveAttrs: (name) => {
        if (name === 'host_ms') return clock();
        if (name === 'temperature') return this.temperatureC;
        // Asked only when the sketch wrote a time: finding them scans the image.
        if (name === 'build_times') return buildTimesText(buildTimes());
        return undefined;
      },
    });
    this.chip.start();
    const dev = this.chip.i2cDevice(this.address);
    if (!dev) throw new Error('ds3231.wasm attached no I2C device at 0x68');
    this.dev = dev;
  }

  start(read: boolean): void {
    this.dev.connect(this.address, read);
  }

  writeByte(value: number): boolean {
    return this.dev.writeByte(value);
  }

  readByte(): number {
    return this.dev.readByte();
  }

  stop(): void {
    this.dev.stop();
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
