/**
 * The compiled DS3231 (simulation/buses/models/ds3231.c) against the bus
 * vectors the two hand-written copies replay: test/fixtures/i2c-vectors/
 * ds3231.json. Project i2c-model-fidelity-2026-09, P5 (decision O4): the tab's
 * half of the evidence that one model can stand in for both copies. The
 * worker's half replays the same file against the same .wasm
 * (test/backend/unit/test_wasm_i2c_models.py).
 *
 * The model is hosted the way the part hosts it behind its flag
 * (simulation/parts/wasmI2cModels.ts): ChipRuntime, the chip's own I2C
 * device, the host's clock and the build times as live attributes.
 */
import { describe, it, expect, afterEach, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { VirtualDS3231, type RtcOptions } from '../simulation/I2CBusManager';
import { i2cTargetOf } from '../simulation/parts/i2cPart';
import {
  WasmDS3231,
  primeWasmI2cModel,
  setWasmI2cModelsForTest,
  wasmI2cModelEnabled,
} from '../simulation/parts/wasmI2cModels';
import { primeBusChip, resetBusChipsForTest } from '../simulation/buses/busChips';
import { PartSimulationRegistry } from '../simulation/parts/PartSimulationRegistry';
import '../simulation/parts/ProtocolParts';
import {
  BUS_FLAVOURS,
  VectorClock,
  buildTime,
  loadVectors,
  replayVector,
  type BusVector,
  type VectorHost,
} from './helpers/i2cVectors';

const models = (p: string) =>
  fileURLToPath(new URL(`../simulation/buses/models/${p}`, import.meta.url));
const WASM = readFileSync(
  fileURLToPath(new URL('../../public/bus-chips/ds3231.wasm', import.meta.url)),
);
const MODULE = new WebAssembly.Module(WASM);
const FILE = loadVectors('ds3231');

type Rtc = WasmDS3231 | VirtualDS3231;

function powerOn(
  vector: BusVector,
  make: (o: RtcOptions) => Rtc,
): { dev: Rtc; clock: VectorClock } {
  const clock = new VectorClock(vector.clock ?? FILE.clock!);
  const built = (vector.build_times ?? FILE.build_times ?? []).map(buildTime);
  const dev = make({ clock: clock.read, buildTimes: () => built });
  dev.temperatureC = FILE.inputs.temperature;
  return { dev, clock };
}

const setInputs = (dev: Rtc, values: Record<string, number>): void => {
  if ('temperature' in values) dev.temperatureC = values.temperature;
};

/** On the fabric's target contract, as a board whose firmware runs in the tab reaches it. */
function fabricHost(dev: Rtc, clock: VectorClock): VectorHost {
  const target = i2cTargetOf(dev);
  return {
    start: (read) => target.start(dev.address, read),
    write: (byte) => target.write(byte),
    read: () => target.read(),
    stop: () => target.stop(),
    inputs: (values) => setInputs(dev, values),
    dump: () => target.dumpRegisters!(),
    clock: (step) => clock.step(step),
  };
}

/** A host that never says where a transfer begins (rtc-vectors.test.ts, bareHost). */
function bareHost(dev: Rtc, clock: VectorClock): VectorHost {
  return {
    start: (read) => {
      if (!read) dev.stop();
      return true;
    },
    write: (byte) => dev.writeByte(byte),
    read: () => dev.readByte(),
    stop: () => dev.stop(),
    inputs: (values) => setInputs(dev, values),
    dump: () => dev.dumpRegisters(),
    clock: (step) => clock.step(step),
  };
}

describe('ds3231.wasm: the artifact', () => {
  it('was built from the source next to it (buses/models/build.sh)', () => {
    const manifest = JSON.parse(readFileSync(models('manifest.json'), 'utf-8'));
    const sha = createHash('sha256')
      .update(readFileSync(models('ds3231.c')))
      .digest('hex');
    expect(manifest.ds3231.sourceSha256).toBe(sha);
  });

  it('answers at the address of the vectors', () => {
    expect(new WasmDS3231(MODULE, {}).address).toBe(parseInt(FILE.address, 16));
  });
});

describe('ds3231.wasm: the bus vectors the two copies replay', () => {
  for (const flavour of BUS_FLAVOURS) {
    for (const vector of FILE.vectors) {
      it(`[${flavour}] ${vector.name}`, () => {
        const { dev, clock } = powerOn(vector, (o) => new WasmDS3231(MODULE, o));
        replayVector(fabricHost(dev, clock), vector, flavour);
      });

      it(`[${flavour}, a host that hears no START] ${vector.name}`, () => {
        const { dev, clock } = powerOn(vector, (o) => new WasmDS3231(MODULE, o));
        replayVector(bareHost(dev, clock), vector, flavour);
      });
    }
  }
});

describe('ds3231.wasm: the flag', () => {
  afterEach(() => {
    setWasmI2cModelsForTest(null);
    resetBusChipsForTest();
  });

  /** A board whose firmware runs in a QEMU worker: the part files a record. */
  const workerSim = () => ({
    registerSensor: vi.fn(),
    updateSensor: vi.fn(),
    unregisterSensor: vi.fn(),
  });
  const element = () =>
    ({ addEventListener: vi.fn(), removeEventListener: vi.fn() }) as unknown as HTMLElement;
  const attach = (sim: ReturnType<typeof workerSim>, id: string) =>
    PartSimulationRegistry.get('ds3231')!.attachEvents!(element(), sim as never, () => null, id);

  it('is off unless asked for', () => {
    expect(wasmI2cModelEnabled('ds3231')).toBe(false);
    setWasmI2cModelsForTest(['ds3231']);
    expect(wasmI2cModelEnabled('ds3231')).toBe(true);
    expect(wasmI2cModelEnabled('mpu6050')).toBe(false);
  });

  it('off: the worker record carries no model, and the worker keeps its twin', () => {
    primeBusChip('ds3231', new Uint8Array(WASM));
    primeWasmI2cModel('ds3231', MODULE);
    const sim = workerSim();
    attach(sim, 'ds-wasm-off')();
    const [type, , props] = sim.registerSensor.mock.calls[0];
    expect(type).toBe('ds3231');
    expect(props).not.toHaveProperty('wasmB64');
  });

  it('on: the worker record carries the bytes the tab runs', () => {
    setWasmI2cModelsForTest(['ds3231']);
    primeBusChip('ds3231', new Uint8Array(WASM));
    primeWasmI2cModel('ds3231', MODULE);
    const sim = workerSim();
    attach(sim, 'ds-wasm-on')();
    const [, , props] = sim.registerSensor.mock.calls[0];
    expect(Buffer.from(props.wasmB64 as string, 'base64').equals(WASM)).toBe(true);
    expect(props).toMatchObject({ addr: 0x68, owner: 'ds-wasm-on', temperature: 25 });
  });
});
