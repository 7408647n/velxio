/**
 * Multi-driver bus net registry — Phase 1 of the multi-chip digital bus track
 * (project/multichip-bus/). Sits between the chip runtime and the PinManager
 * for chip-to-chip BUS net keys (syntheticNetPin), and, since board-buses F7,
 * for the board pins a chip drives.
 *
 * Phase 0 gave every endpoint of a chip-to-chip net one shared PinManager key,
 * but PinManager is last-writer-wins — wrong for a bus where several chips can
 * drive one data line and the rest release it (Hi-Z). This registry tracks each
 * chip pin's (value, strength) contribution per net, resolves the net with the
 * 4-valued rules (busLogic.resolveNet), and pushes the RESOLVED level into the
 * PinManager so readers see the bus, not the last writer. Contention (two strong
 * drivers disagree -> X) is surfaced as a console warning, like Proteus.
 *
 * A board pin is a net like any other, with one more driver on it: the MCU's
 * pad. What the guest does to the pad is on the PinManager's pad channel
 * (driving low, driving high, or released with a pull), so the resolution
 * here is the wired-AND every open-drain line relies on: a chip that pulls
 * the pin low wins over the pull-up, and a chip that goes back to VX_INPUT
 * stops contributing, and the pull restores the level. The resolved level
 * goes to the PinManager and, through the sink the chip host registers,
 * into the guest's input register. It used to be that a chip wrote the
 * PinManager and the guest straight from vx_pin_write, and its release wrote
 * nothing at all, so the board read the chip's last level forever (finding
 * chip-release-to-input-keeps-board-pin-driven).
 *
 * Single-chip-to-component synthetic pins keep the legacy direct PinManager
 * path untouched.
 */
import { resolveNet, resolvedToBool, Strength, HIGHZ_DRIVE, type Drive } from './busLogic';
import { publishNetLevel, resetBusKernel } from './busKernel';

interface PinManagerLike {
  triggerPinChange(pin: number, state: boolean, source?: 'mcu' | 'external'): void;
}

/**
 * A board's PinManager as a board-pin net sees it: the level channel plus the
 * pad channel the engines report the guest's drive on. A manager without the
 * pad channel (a test double) has pads that are never driving and never
 * pulled, so a chip's own drive alone decides its pins.
 */
export interface BoardPinHost extends PinManagerLike {
  onPinChange?(pin: number, cb: (pin: number, state: boolean) => void): () => void;
  peekPad?(pin: number): { drive: 'low' | 'high' | 'z'; pull: 0 | 1 | 2 } | undefined;
  onPadChange?(pin: number, cb: () => void): () => void;
  /**
   * What an engine without the pad channel still reports: the direction and
   * the internal pull the guest programmed (PinManager.setPinDirection /
   * setPinPull, fed by every ESP32 bridge's onPinDir / onPinPull). Read only
   * when peekPad has nothing for the pin.
   */
  getPinDirection?(pin: number): 0 | 1 | undefined;
  getPinPull?(pin: number): 0 | 1 | 2;
  /** The level the channel carries now: what an output pad of such an
   *  engine last put on it, read when a net forms over that pad. */
  getPinState?(pin: number): boolean;
  /** The net answers every level proposed for the pin (PinManager.claimLevel). */
  claimLevel?(
    pin: number,
    resolve: (proposed: boolean, source: 'mcu' | 'external') => boolean | undefined,
  ): () => void;
}

// netKey -> (driverId -> Drive). driverId = `${componentId}::${pinName}`.
const nets = new Map<number, Map<string, Drive>>();
// Nets currently flagged as in contention — so we warn once per onset, not per
// re-resolve, and clear when the contention is gone.
const inContention = new Set<number>();

function recompute(pm: PinManagerLike, netKey: number): void {
  const drivers = nets.get(netKey);
  const resolved = resolveNet(drivers ? drivers.values() : []);

  if (resolved.v === 'X') {
    if (!inContention.has(netKey)) {
      inContention.add(netKey);
      console.warn(
        `[chipbus] bus contention on net ${netKey}: two strong drivers disagree (resolved X)`,
      );
    }
  } else {
    inContention.delete(netKey);
  }

  // PinManager is boolean; push the projected level (only a driven 1 is high)
  // through the settle kernel so the change propagates as a bounded delta-cycle
  // pass rather than a recursive cascade.
  publishNetLevel(pm, netKey, resolvedToBool(resolved));
}

/** Set (or replace) one chip pin's contribution to a bus net and re-resolve. */
export function setBusDrive(
  pm: PinManagerLike,
  netKey: number,
  driverId: string,
  drive: Drive,
): void {
  let m = nets.get(netKey);
  if (!m) {
    m = new Map();
    nets.set(netKey, m);
  }
  m.set(driverId, drive);
  recompute(pm, netKey);
}

// ── Board pins ──────────────────────────────────────────────────────────────

/** Where a resolved level goes besides the PinManager: the guest's input
 *  register, through the host of the chip that drives the pin. */
type BoardPinSink = (level: boolean) => void;

interface BoardNet {
  /** The chip pins holding this board pin, by driver id. A pin that lets go
   *  (VX_INPUT) is removed, not kept at Hi-Z: an empty net is torn down. */
  drivers: Map<string, Drive>;
  sinks: Map<string, BoardPinSink>;
  /** Off the pad and level channels when the last driver leaves. */
  unwatch: () => void;
  /** Warned about contention with the pad; cleared when it ends. */
  contended: boolean;
  /** The level this net last put on the channel, to tell its own echo from
   *  a write by someone else. */
  lastLevel: boolean | undefined;
  /** An engine without the pad channel: the level its output pad drives,
   *  from the levels it reports (it reports one only when its latch moves). */
  mcuLevel: boolean | undefined;
}

// One table per PinManager: board pin numbers repeat on every board, so D2
// of two Unos in one project are two nets, never one.
const boardNets = new Map<BoardPinHost, Map<number, BoardNet>>();

/** The MCU's pad as a driver of its own pin: strong while it drives, a pull
 *  while released with one, nothing while released and floating. A pad the
 *  engine never reported on has not been touched by the guest: floating,
 *  unless the engine reports pulls on their own channel (reportedPull). */
function padDrive(host: BoardPinHost, pin: number): Drive {
  const pad = host.peekPad?.(pin);
  if (!pad) return reportedPull(host, pin);
  if (pad.drive === 'low') return { value: 0, strength: Strength.STRONG };
  if (pad.drive === 'high') return { value: 1, strength: Strength.STRONG };
  if (pad.pull === 1) return { value: 1, strength: Strength.PULL };
  if (pad.pull === 2) return { value: 0, strength: Strength.PULL };
  return HIGHZ_DRIVE;
}

/**
 * The pad of an engine that has no pad channel (the in-browser ESP32 engines
 * and the QEMU bridge): the pull the guest enabled on an input, from the
 * direction and pull channels those engines do report. An output stays
 * Hi-Z here, as before: those engines put their driven level on the level
 * channel, and proposeBoardPin passes it as the wire's.
 *
 * Without this the pull of INPUT_PULLUP was invisible to the net, so a chip
 * that released an open-collector line (vx_pin_set_mode(VX_INPUT), the A3144
 * and every IRQ idiom) resolved it to Z, and a Z leaves the wire where the
 * chip's last pull put it: the ESP32 read LOW for the rest of the run and
 * never saw a second falling edge (finding
 * esp32-open-collector-release-reads-low).
 */
function reportedPull(host: BoardPinHost, pin: number): Drive {
  if (host.getPinDirection?.(pin) === 1) return HIGHZ_DRIVE;
  const pull = host.getPinPull?.(pin) ?? 0;
  if (pull === 1) return { value: 1, strength: Strength.PULL };
  if (pull === 2) return { value: 0, strength: Strength.PULL };
  return HIGHZ_DRIVE;
}

/**
 * What the wire holds with `proposed` put on it through `source`.
 *
 * A level the engine reports ('mcu') on a pad it drives is the wire's: it
 * passes, and the chips' contention with it is reported by the pad channel's
 * own resolution. An engine without a pad channel (the ESP32 bridges) reports
 * only the pads its firmware drives, so its level is its drive too. Anything
 * else (a part's injection, the pull of a pad the guest configured as an
 * input) is a proposal the net's resolution answers, which a strong driver
 * decides: undefined for an injection on a pad the guest drives (it moves
 * nothing), the proposal itself when nothing on the net holds the wire.
 */
function proposeBoardPin(
  host: BoardPinHost,
  pin: number,
  net: BoardNet,
  proposed: boolean,
  source: 'mcu' | 'external',
): boolean | undefined {
  const pad = padDrive(host, pin);
  if (pad.strength === Strength.STRONG) return source === 'mcu' ? proposed : undefined;
  if (source === 'mcu' && !host.peekPad?.(pin)) {
    net.mcuLevel = proposed;
    return proposed;
  }
  const resolved = resolveNet([...net.drivers.values(), pad]);
  if (resolved.v === 'X' || resolved.v === 'Z') return proposed;
  const level = resolved.v === '1';
  net.lastLevel = level;
  // An overridden proposal came through a door that moved the guest's input
  // register before asking (the AVR's setPinState writes PIN first): put the
  // wire's level back into the guest through every chip's sink, as the
  // re-assertion after the write used to. The sink's own write comes back
  // here with the resolved level and changes nothing.
  if (level !== proposed) for (const sink of net.sinks.values()) sink(level);
  return level;
}

function recomputeBoardPin(host: BoardPinHost, pin: number, net: BoardNet): void {
  const pad = padDrive(host, pin);
  const resolved = resolveNet([...net.drivers.values(), pad]);
  if (resolved.v === 'X') {
    if (!net.contended) {
      net.contended = true;
      console.warn(
        `[chipbus] contention on board pin ${pin}: a chip and the MCU drive it to opposite levels`,
      );
    }
    return;
  }
  net.contended = false;
  // While the MCU drives the pad its level is the wire's, and the engine has
  // already put it on the level channel; there is nothing to feed back into
  // the guest (its own output reads its latch). A released pad with nothing
  // pulling it keeps the level it had, as a floating input does.
  if (pad.strength === Strength.STRONG) return;
  let level: boolean;
  if (resolved.v === 'Z') {
    // An engine without the pad channel keeps an output pad at HIGHZ here
    // and reports its level only when the latch moves. When the chips let go
    // of such a pad, the wire is the MCU's output again, not the chip's last
    // level: a TM1637 that ACKs by pulling DIO low while the sketch's latch
    // is HIGH would otherwise leave the line low, and the next 1 bit, which
    // moves no latch, never reaches the chip (finding
    // chip-release-leaves-esp32-output-at-chip-level).
    const mcu = net.mcuLevel;
    if (mcu === undefined || host.peekPad?.(pin) || host.getPinDirection?.(pin) !== 1) return;
    if (net.lastLevel === mcu) return;
    level = mcu;
  } else {
    level = resolved.v === '1';
  }
  const sinks = [...net.sinks.values()];
  publishNetLevel(host, pin, level, (lvl) => {
    net.lastLevel = lvl;
    for (const sink of sinks) sink(lvl);
    host.triggerPinChange(pin, lvl, 'external');
  });
}

/**
 * Set (or withdraw) one chip pin's drive of a board pin and re-resolve it.
 * `sink` is the chip host's way into the guest's input register for this pin
 * (its digital-write hook); every chip on the pin registers its own, and the
 * resolved level goes through each, which is how each host claims the pin it
 * drives. A Hi-Z drive withdraws the chip: `vx_pin_set_mode(VX_INPUT)` is the
 * release idiom, and a released chip is simply not on the net.
 */
export function setBoardPinDrive(
  host: BoardPinHost,
  pin: number,
  driverId: string,
  drive: Drive,
  sink: BoardPinSink,
): void {
  let byPin = boardNets.get(host);
  if (!byPin) {
    byPin = new Map();
    boardNets.set(host, byPin);
  }
  let net = byPin.get(pin);
  if (drive.strength <= Strength.HIGHZ) {
    if (!net || !net.drivers.has(driverId)) return;
    net.drivers.delete(driverId);
    // The pull, or another chip, has the line now: say so before leaving,
    // and through the leaving chip's own door into the guest as well. Its
    // sink goes after the resolution, or a chip that was alone on the pin
    // would restore the level on the PinManager only, and the sketch would
    // read the chip's last level forever.
    recomputeBoardPin(host, pin, net);
    net.sinks.delete(driverId);
    if (net.drivers.size === 0) dropBoardNet(host, pin, net);
    return;
  }
  if (!net) {
    const created: BoardNet = {
      drivers: new Map(),
      sinks: new Map(),
      unwatch: () => {},
      contended: false,
      lastLevel: undefined,
      // The channel carries the output pad's level until a chip drives it.
      mcuLevel:
        !host.peekPad?.(pin) && host.getPinDirection?.(pin) === 1 ? host.getPinState?.(pin) : undefined,
    };
    // The pad decides the resolution as much as the chips do, so a pinMode
    // in the sketch re-resolves the pin. And the net claims the pin's level:
    // an engine that writes an input pin's latch onto the channel (the AVR's
    // PORT bit for INPUT_PULLUP), or a part injecting a level, is answered by
    // the resolution before anything reaches the channel, so a second chip
    // watching the pin never sees the latch as an edge. A host without the
    // claim (a test double) is watched instead and re-asserted after the
    // write, glitch included; the net's own publication comes back through
    // the same channel and is not a reason to resolve again.
    const offPad = host.onPadChange?.(pin, () => recomputeBoardPin(host, pin, created)) ?? (() => {});
    const offLevel = host.claimLevel
      ? host.claimLevel(pin, (proposed, source) => proposeBoardPin(host, pin, created, proposed, source))
      : (host.onPinChange?.(pin, (_p, state) => {
          if (state !== created.lastLevel) recomputeBoardPin(host, pin, created);
        }) ?? (() => {}));
    created.unwatch = () => {
      offPad();
      offLevel();
    };
    net = created;
    byPin.set(pin, net);
  }
  net.drivers.set(driverId, drive);
  net.sinks.set(driverId, sink);
  recomputeBoardPin(host, pin, net);
}

function dropBoardNet(host: BoardPinHost, pin: number, net: BoardNet): void {
  net.unwatch();
  const byPin = boardNets.get(host);
  byPin?.delete(pin);
  if (byPin && byPin.size === 0) boardNets.delete(host);
}

/** Remove every driver a chip contributes (on dispose) and re-resolve its nets. */
export function clearBusDriversForChip(pm: PinManagerLike, componentId: string): void {
  const prefix = `${componentId}::`;
  for (const [netKey, m] of nets) {
    let changed = false;
    for (const id of [...m.keys()]) {
      if (id.startsWith(prefix)) {
        m.delete(id);
        changed = true;
      }
    }
    if (changed) recompute(pm, netKey);
  }
  const byPin = boardNets.get(pm as BoardPinHost);
  if (!byPin) return;
  for (const [pin, net] of [...byPin.entries()]) {
    const gone = [...net.drivers.keys()].filter((id) => id.startsWith(prefix));
    if (gone.length === 0) continue;
    for (const id of gone) net.drivers.delete(id);
    // Same order as a release: the level the others leave goes into the
    // guest through the departing chip's door too, then the door closes.
    recomputeBoardPin(pm as BoardPinHost, pin, net);
    for (const id of gone) net.sinks.delete(id);
    if (net.drivers.size === 0) dropBoardNet(pm as BoardPinHost, pin, net);
  }
}

/** Test seam: wipe all bus-net driver state (and the settle kernel). */
export function resetBusNets(): void {
  nets.clear();
  inContention.clear();
  for (const byPin of boardNets.values()) for (const net of byPin.values()) net.unwatch();
  boardNets.clear();
  resetBusKernel();
}
