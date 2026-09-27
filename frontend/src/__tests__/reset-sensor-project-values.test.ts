/**
 * Reset brings every interactive sensor back to the value the PROJECT has for
 * it, never to the SensorControlPanel's generic default, and never rewrites
 * component.properties.
 *
 * What a sensor holds after Reset:
 *   - its project value: what the user configured (properties, or
 *     properties.attrs on a custom chip), as loaded or as last edited in the
 *     property dialog;
 *   - the panel default only for a control the project leaves unset.
 * A slider moved during the run is a live input, not a project edit. Some parts
 * mirror it into properties while running (the SPICE netlist reads the NTC's
 * temperature and the photodiode's lux from there), and Reset takes those back
 * out; everything else in properties is left exactly as it was.
 *
 * The BMP280 cases run real firmware (fixtures/avr-bmp280-temp, raw register
 * reads and the datasheet compensation) on avr8js through the store's own
 * lifecycle (compileBoardProgram, startBoard, resetBoard). Two things stand in
 * for the browser, both taken from DynamicComponent / SimulatorCanvas: the
 * effect that copies component.properties onto the element, and the one that
 * re-attaches every part when hexEpoch moves, plus the canvas listener that
 * routes a part's property-change event into the store.
 */
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

// Node environment with the browser globals the parts and the store reach for:
// timers, and an event target so emitPropertyChange reaches the canvas listener.
const win = Object.assign(new EventTarget(), {
  setTimeout: (fn: () => void, ms: number) => setTimeout(fn, ms),
  clearTimeout: (id: ReturnType<typeof setTimeout>) => clearTimeout(id),
});
vi.stubGlobal('window', win);
vi.stubGlobal('requestAnimationFrame', () => 0);
vi.stubGlobal('cancelAnimationFrame', () => {});

import { useSimulatorStore, getBoardSimulator } from '../store/useSimulatorStore';
import type { AVRSimulator } from '../simulation/AVRSimulator';
import { PartSimulationRegistry } from '../simulation/parts/PartSimulationRegistry';
import '../simulation/parts';
import { traceDetailed } from '../simulation/PinTrace';
import { PROPERTY_CHANGE_EVENT, type PropertyChangeDetail } from '../simulation/parts/partUtils';
import {
  dispatchSensorUpdate,
  getLastSensorValues,
  registerSensorUpdate,
  unregisterSensorUpdate,
} from '../simulation/SensorUpdateRegistry';
import {
  SENSOR_CONTROLS,
  projectSensorValues,
  registerInstanceSensorControlResolver,
  type SensorControlDef,
} from '../simulation/sensorControlConfig';

const HEX = readFileSync(
  fileURLToPath(new URL('./fixtures/avr-bmp280-temp/avr-bmp280-temp.ino.hex', import.meta.url)),
  'utf-8',
);

// SimulatorCanvas: one listener routes every runtime property change.
const onPropertyChange = (evt: Event) => {
  const { componentId, propName, value } = (evt as CustomEvent<PropertyChangeDetail>).detail;
  useSimulatorStore.getState().applyLivePropertyChange(componentId, propName, value);
};
win.addEventListener(PROPERTY_CHANGE_EVENT, onPropertyChange);

// ── Bench: one board of the real store, parts mounted the way the canvas does ─

interface PartSpec {
  id: string;
  metadataId: string;
  properties?: Record<string, unknown>;
}

interface Mounted extends PartSpec {
  el: Record<string, unknown>;
  applied: Record<string, unknown>;
  cleanup?: () => void;
}

let seq = 0;

class Bench {
  readonly id = `arduino-uno-rs${++seq}`;
  out = '';
  readonly parts: Mounted[] = [];
  private wireSeq = 0;
  private readonly unsubscribe: () => void;

  constructor() {
    useSimulatorStore.getState().addBoard('arduino-uno', 0, 0, this.id);
    useSimulatorStore.getState().setActiveBoardId(this.id);
    this.sim.onSerialData = (ch: string) => {
      this.out += ch;
    };
    // DynamicComponent: the property sync effect is declared before the
    // attach effect, so a hexEpoch bump copies properties onto the element
    // first and then re-attaches (every cleanup before any attach).
    let epoch = useSimulatorStore.getState().hexEpoch;
    this.unsubscribe = useSimulatorStore.subscribe((s) => {
      for (const p of this.parts) this.sync(p);
      if (s.hexEpoch !== epoch) {
        epoch = s.hexEpoch;
        for (const p of this.parts) this.detach(p);
        for (const p of this.parts) this.attach(p);
      }
    });
  }

  get sim(): AVRSimulator {
    return getBoardSimulator(this.id) as unknown as AVRSimulator;
  }

  wire(comp: string, pin: string, boardPin: string): void {
    useSimulatorStore.getState().addWire({
      id: `${this.id}-w${++this.wireSeq}`,
      start: { componentId: comp, pinName: pin, x: 0, y: 0 },
      end: { componentId: this.id, pinName: boardPin, x: 0, y: 0 },
      waypoints: [],
      color: '#0a0',
    } as never);
  }

  mount(spec: PartSpec): void {
    const st = useSimulatorStore.getState();
    st.setComponents([
      ...st.components,
      { id: spec.id, metadataId: spec.metadataId, x: 0, y: 0, properties: spec.properties ?? {} },
    ] as never);
    // An event target, like the element: some parts listen on it.
    const el = new EventTarget() as unknown as Record<string, unknown>;
    const m: Mounted = { ...spec, el, applied: {} };
    this.parts.push(m);
    this.sync(m);
    this.attach(m);
  }

  props(id: string): Record<string, unknown> {
    return useSimulatorStore.getState().components.find((c) => c.id === id)!.properties;
  }

  /** DynamicComponent's sync: assign only the values that changed. */
  private sync(p: Mounted): void {
    const comp = useSimulatorStore.getState().components.find((c) => c.id === p.id);
    if (!comp) return;
    for (const [k, v] of Object.entries(comp.properties)) {
      if (p.applied[k] !== v) {
        p.applied[k] = v;
        p.el[k] = v;
      }
    }
  }

  private attach(p: Mounted): void {
    const logic = PartSimulationRegistry.get(p.metadataId)!;
    const getPin = (name: string) => traceDetailed(useSimulatorStore.getState(), p.id, name, 0).arduinoPin;
    p.cleanup = logic.attachEvents!(p.el as never, this.sim as never, getPin, p.id) ?? undefined;
  }

  private detach(p: Mounted): void {
    p.cleanup?.();
    p.cleanup = undefined;
  }

  load(): void {
    useSimulatorStore.getState().compileBoardProgram(this.id, HEX);
  }

  run(): void {
    useSimulatorStore.getState().startBoard(this.id);
  }

  reset(): void {
    useSimulatorStore.getState().resetBoard(this.id);
  }

  /** Step the CPU until a complete "T=" line arrives after `from`; returns it. */
  nextReading(from: number, budget = 4_000_000): string {
    for (let i = 0; i < budget; i++) {
      this.sim.step();
      if ((i & 0x3ff) === 0) {
        const m = /(T=[^\r\n]*)\r?\n/.exec(this.out.slice(from));
        if (m) return m[1];
      }
    }
    throw new Error(`no reading; serial after mark: ${JSON.stringify(this.out.slice(from))}`);
  }

  /** The reading once the value has settled (skips a line already in flight). */
  reading(): string {
    const first = this.nextReading(this.out.length);
    return this.nextReading(this.out.indexOf(first, this.out.length - first.length - 2) + first.length);
  }

  adcVolts(channel: number): number {
    return (this.sim.getADC() as { channelValues: number[] }).channelValues[channel];
  }

  dispose(): void {
    this.unsubscribe();
    for (const p of this.parts) this.detach(p);
    useSimulatorStore.getState().removeBoard(this.id);
  }
}

let benches: Bench[] = [];
const bench = () => {
  const b = new Bench();
  benches.push(b);
  return b;
};

beforeEach(() => {
  benches = [];
  useSimulatorStore.setState({ components: [], wires: [] } as never);
});
afterEach(() => {
  for (const b of benches) b.dispose();
  useSimulatorStore.setState({ components: [], wires: [] } as never);
});

function bmp280(b: Bench, properties: Record<string, unknown>): void {
  b.wire('bmp', 'VCC', '5V');
  b.wire('bmp', 'GND', 'GND.1');
  b.wire('bmp', 'SDA', 'A4');
  b.wire('bmp', 'SCL', 'A5');
  b.mount({ id: 'bmp', metadataId: 'bmp280', properties });
}

// ── BMP280 on real firmware ──────────────────────────────────────────────────

describe('Reset keeps a BMP280 at the temperature the project gave it', () => {
  it('setup: the sketch reads the project temperature (35 C) on the first run', () => {
    const b = bench();
    bmp280(b, { temperature: '35' });
    b.load();
    b.run();
    expect(b.reading()).toBe('T=35.0');
  });

  it('after Reset the sketch still reads 35 C, not the panel default (24 C)', () => {
    const b = bench();
    bmp280(b, { temperature: '35' });
    b.load();
    b.run();
    expect(b.reading()).toBe('T=35.0');

    b.reset();
    b.run();
    expect(b.reading()).toBe('T=35.0');
  });

  it('Reset does not rewrite component.properties (saving afterwards keeps the 35)', () => {
    const b = bench();
    bmp280(b, { temperature: '35' });
    b.load();
    b.run();
    const before = b.props('bmp');

    b.reset();
    expect(b.props('bmp')).toBe(before);
    expect(b.props('bmp')).toEqual({ temperature: '35' });
  });

  it('a slider moved during the run is a live input: Reset returns the part to 35 C', () => {
    const b = bench();
    bmp280(b, { temperature: '35' });
    b.load();
    b.run();
    expect(b.reading()).toBe('T=35.0');

    dispatchSensorUpdate('bmp', { temperature: 50 });
    expect(b.reading()).toBe('T=50.0');

    b.reset();
    b.run();
    expect(b.reading()).toBe('T=35.0');
    expect(b.props('bmp')).toEqual({ temperature: '35' });
    // The open panel re-reads this cache (or the project, once the re-attach
    // cleared it) when sensorResetNonce remounts it: the slider shows 35.
    const cached = getLastSensorValues('bmp');
    if (cached) expect(cached.temperature).toBe(35);
  });

  it('a value edited in the property dialog is the project value from then on', () => {
    const b = bench();
    bmp280(b, { temperature: '35' });
    b.load();
    b.run();
    dispatchSensorUpdate('bmp', { temperature: 50 });

    const st = useSimulatorStore.getState();
    st.updateComponent('bmp', { properties: { ...b.props('bmp'), temperature: '12' } });

    b.reset();
    b.run();
    expect(b.reading()).toBe('T=12.0');
    expect(b.props('bmp')).toEqual({ temperature: '12' });
  });
});

// ── NTC: what 608c2538 fixed (injected ADC voltage + SPICE property) ─────────

const ntcVolts = (tempC: number, vcc = 5) => {
  const rNtc = 10_000 * Math.exp(3950 * (1 / (tempC + 273.15) - 1 / 298.15));
  return vcc * (rNtc / (rNtc + 10_000));
};

function ntc(b: Bench, properties: Record<string, unknown>): void {
  b.wire('ntc', 'VCC', '5V');
  b.wire('ntc', 'GND', 'GND.1');
  b.wire('ntc', 'OUT', 'A0');
  b.mount({ id: 'ntc', metadataId: 'ntc-temperature-sensor', properties });
}

describe('Reset returns an NTC to its resting value', () => {
  it('no project temperature: back to 25 C (2.5 V) and the SPICE property is gone again', () => {
    const b = bench();
    ntc(b, {});
    b.load();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(25), 6);

    dispatchSensorUpdate('ntc', { temperature: 80 });
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(80), 6);
    // Live mirror for the SPICE netlist (componentToSpice reads it).
    expect(b.props('ntc').temperature).toBe(80);

    b.reset();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(25), 6);
    expect(b.props('ntc')).toEqual({});
  });

  it('project temperature 40 C: the first run and every Reset inject 40 C', () => {
    const b = bench();
    ntc(b, { temperature: 40 });
    b.load();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(40), 6);

    dispatchSensorUpdate('ntc', { temperature: 80 });
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(80), 6);

    b.reset();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(40), 6);
    expect(b.props('ntc')).toEqual({ temperature: 40 });
  });

  it('a value set in the property dialog mid-run is the project value, even after more live moves', () => {
    const b = bench();
    ntc(b, {});
    b.load();
    b.run();
    dispatchSensorUpdate('ntc', { temperature: 80 });
    const st = useSimulatorStore.getState();
    st.updateComponent('ntc', { properties: { ...b.props('ntc'), temperature: 60 } });
    dispatchSensorUpdate('ntc', { temperature: 90 });
    expect(b.props('ntc')).toEqual({ temperature: 90 });

    b.reset();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(ntcVolts(60), 6);
    expect(b.props('ntc')).toEqual({ temperature: 60 });
  });
});

describe('Reset returns a photoresistor to the project illumination', () => {
  it('project lux 100: 0.5 V on the first run, after a live move and after Reset', () => {
    const b = bench();
    b.wire('ldr', 'VCC', '5V');
    b.wire('ldr', 'GND', 'GND.1');
    b.wire('ldr', 'AO', 'A0');
    b.mount({ id: 'ldr', metadataId: 'photoresistor-sensor', properties: { lux: 100 } });
    b.load();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(0.5, 6);

    dispatchSensorUpdate('ldr', { lux: 900 });
    expect(b.adcVolts(0)).toBeCloseTo(4.5, 6);

    b.reset();
    b.run();
    expect(b.adcVolts(0)).toBeCloseTo(0.5, 6);
    expect(b.props('ldr')).toEqual({ lux: 100 });
  });
});

// ── Control keys that are not property names, and custom chips ───────────────

describe('project values for controls stored under another name', () => {
  it('ir-receiver: address / command come from irAddress / irCommand', () => {
    const def = SENSOR_CONTROLS['ir-receiver'];
    expect(
      projectSensorValues(
        { id: 'ir', metadataId: 'ir-receiver', properties: { irAddress: '0x10', irCommand: '0x2A' } },
        def,
      ),
    ).toEqual({ address: 0x10, command: 0x2a });
  });

  it('a control the project leaves unset falls back to the panel default', () => {
    const def = SENSOR_CONTROLS.bmp280;
    expect(
      projectSensorValues({ id: 'b', metadataId: 'bmp280', properties: { temperature: '35' } }, def),
    ).toEqual({ temperature: 35, pressure: 1013.25 });
    expect(projectSensorValues({ id: 'b', metadataId: 'bmp280', properties: {} }, def)).toEqual(
      def.defaultValues,
    );
  });

  it('custom chip: Reset replays properties.attrs, not the chip.json defaults, and leaves attrs alone', () => {
    const chipDef: SensorControlDef = {
      title: 'CO2',
      controls: [
        { type: 'slider', key: 'ppm', label: 'ppm', min: 400, max: 5000, step: 1, unit: '', defaultValue: 400 },
      ],
      defaultValues: { ppm: 400 },
    };
    registerInstanceSensorControlResolver((c) => (c.metadataId === 'custom-chip' ? chipDef : undefined));
    const b = bench();
    const attrs = { ppm: 1200 };
    useSimulatorStore
      .getState()
      .setComponents([
        { id: 'chip', metadataId: 'custom-chip', x: 0, y: 0, properties: { attrs } },
      ] as never);
    const seen: Array<Record<string, unknown>> = [];
    registerSensorUpdate('chip', (v) => seen.push(v));

    b.reset();
    expect(seen.at(-1)).toEqual({ ppm: 1200 });
    expect(b.props('chip').attrs).toBe(attrs);

    // A live slider move the overlay mirrors into attrs is taken back out.
    useSimulatorStore.getState().applyLivePropertyChange('chip', 'attrs', { ppm: 3000 });
    expect(b.props('chip').attrs).toEqual({ ppm: 3000 });
    b.reset();
    expect(seen.at(-1)).toEqual({ ppm: 1200 });
    expect(b.props('chip').attrs).toEqual({ ppm: 1200 });
    unregisterSensorUpdate('chip');
  });
});
