/**
 * attachOnboardLeds (simulation/onboardLeds.ts): from the levels a board's
 * PinManager reports, and the WS2812 frames its engine decodes, to the
 * visual each on-board LED should show, polarity applied (issue #374).
 *
 * A real PinManager drives every case, through the same triggerPinChange
 * the ESP32 bridges call from gpio_change.
 */
import { describe, it, expect, vi } from 'vitest';
import { PinManager } from '../simulation/PinManager';
import { attachOnboardLeds, sameLedVisual } from '../simulation/onboardLeds';
import type { OnboardLedVisual } from '../types/board';

type Pixel = { r: number; g: number; b: number };

function rig(kind: string) {
  const pm = new PinManager();
  const frames = new Map<number, (px: readonly Pixel[]) => void>();
  const seen: Array<[string, OnboardLedVisual]> = [];
  const last = () => seen[seen.length - 1];
  const observeWs2812 = vi.fn((pin: number, sink: (px: readonly Pixel[]) => void) => {
    frames.set(pin, sink);
    return () => {
      frames.delete(pin);
    };
  });
  const onPinChange = vi.fn((pin: number, cb: (p: number, s: boolean) => void) =>
    pm.onPinChange(pin, cb),
  );
  const detach = attachOnboardLeds(
    kind,
    {
      onPinChange,
      getPinState: (pin) => pm.getPinState(pin),
      getOutputPins: () => pm.getOutputPins(),
      observeWs2812,
    },
    (id, visual) => seen.push([id, visual]),
  );
  const mcu = (pin: number, high: boolean) => pm.triggerPinChange(pin, high, 'mcu');
  const frame = (pin: number, px: Pixel[]) => frames.get(pin)?.(px);
  return { pm, seen, last, mcu, frame, detach, onPinChange, observeWs2812 };
}

describe('attachOnboardLeds: single-colour LEDs follow their own pin with the board polarity', () => {
  it('esp32 (DevKit V1): GPIO2 HIGH lights, LOW darkens', () => {
    const r = rig('esp32');
    r.mcu(2, true);
    expect(r.last()).toEqual(['led', true]);
    r.mcu(2, false);
    expect(r.last()).toEqual(['led', false]);
  });

  it.each([
    ['xiao-esp32-s3', 21],
    ['wemos-lolin32-lite', 22],
    ['aitewinrobot-esp32c3-supermini', 8],
  ])('%s: GPIO%i LOW lights, HIGH darkens (active LOW)', (kind, pin) => {
    const r = rig(kind);
    r.mcu(pin, false);
    expect(r.last()).toEqual(['led', true]);
    r.mcu(pin, true);
    expect(r.last()).toEqual(['led', false]);
  });

  it('esp32-cam: the flash on GPIO4 is active HIGH and the red LED on GPIO33 active LOW, each its own', () => {
    const r = rig('esp32-cam');
    r.mcu(4, true);
    expect(r.last()).toEqual(['flash', true]);
    r.mcu(33, false);
    expect(r.last()).toEqual(['led', true]);
    r.mcu(33, true);
    expect(r.last()).toEqual(['led', false]);
    expect(r.seen.filter(([id]) => id === 'flash')).toEqual([['flash', true]]);
  });

  it('arduino-nano-esp32: the D13 LED is GPIO48 active HIGH', () => {
    const r = rig('arduino-nano-esp32');
    r.mcu(48, true);
    expect(r.last()).toEqual(['led', true]);
    r.mcu(48, false);
    expect(r.last()).toEqual(['led', false]);
  });

  it('pin 13 means nothing to any ESP32 kind (the old default)', () => {
    for (const kind of [
      'esp32',
      'esp32-cam',
      'wemos-lolin32-lite',
      'esp32-s3',
      'xiao-esp32-s3',
      'arduino-nano-esp32',
      'esp32-c3',
      'aitewinrobot-esp32c3-supermini',
    ]) {
      const r = rig(kind);
      r.mcu(13, true);
      r.mcu(13, false);
      expect(r.seen).toEqual([]);
    }
  });
});

describe('attachOnboardLeds: the plain RGB LED of the Nano ESP32', () => {
  it('each colour is its own active-LOW pad, and a pad the sketch never drove stays off', () => {
    const r = rig('arduino-nano-esp32');
    r.mcu(46, false); // LED_RED on
    expect(r.last()).toEqual(['rgb', { r: 255, g: 0, b: 0 }]);
    r.mcu(45, false); // LED_BLUE on; LED_GREEN (GPIO0) untouched
    expect(r.last()).toEqual(['rgb', { r: 255, g: 0, b: 255 }]);
    r.mcu(46, true); // LED_RED off
    expect(r.last()).toEqual(['rgb', { r: 0, g: 0, b: 255 }]);
    r.mcu(0, false); // LED_GREEN on
    expect(r.last()).toEqual(['rgb', { r: 0, g: 255, b: 255 }]);
  });
});

describe('attachOnboardLeds: the addressable LED of the two Espressif DevKits', () => {
  it.each([
    ['esp32-s3', 48],
    ['esp32-c3', 8],
  ])('%s: the first pixel of a frame on GPIO%i colours it, a level on that pin does not', (kind, pin) => {
    const r = rig(kind);
    expect(r.observeWs2812).toHaveBeenCalledWith(pin, expect.any(Function));
    r.frame(pin, [
      { r: 0, g: 64, b: 0 },
      { r: 255, g: 255, b: 255 },
    ]);
    expect(r.last()).toEqual(['rgb', { r: 0, g: 64, b: 0 }]);
    r.mcu(pin, true);
    r.mcu(pin, false);
    expect(r.seen).toHaveLength(1);
    expect(r.onPinChange).not.toHaveBeenCalled();
  });

  it('an empty frame changes nothing', () => {
    const r = rig('esp32-s3');
    r.frame(48, []);
    expect(r.seen).toEqual([]);
  });
});

describe('attachOnboardLeds: boards with no user LED subscribe to nothing', () => {
  it.each(['esp32-devkit-c-v4', 'xiao-esp32-c3', 'raspberry-pi-4', 'stm32-bluepill', 'no-such-kind'])(
    '%s',
    (kind) => {
      const r = rig(kind);
      for (const pin of [2, 8, 13, 21, 22, 48]) {
        r.mcu(pin, true);
        r.mcu(pin, false);
      }
      expect(r.onPinChange).not.toHaveBeenCalled();
      expect(r.observeWs2812).not.toHaveBeenCalled();
      expect(r.seen).toEqual([]);
    },
  );
});

describe('attachOnboardLeds: the kinds that already worked', () => {
  it.each([
    ['arduino-uno', 13],
    ['arduino-nano', 13],
    ['arduino-mega', 13],
    ['raspberry-pi-pico', 25],
    ['pi-pico-w', 25],
    ['attiny85', 1],
  ])('%s follows pin %i, HIGH is lit', (kind, pin) => {
    const r = rig(kind);
    r.mcu(pin, true);
    expect(r.last()).toEqual(['led', true]);
    r.mcu(pin, false);
    expect(r.last()).toEqual(['led', false]);
    expect(r.onPinChange).toHaveBeenCalledTimes(1);
  });
});

describe('attachOnboardLeds: lifecycle and helpers', () => {
  it('the returned function detaches every subscription', () => {
    const r = rig('esp32-cam');
    r.detach();
    r.mcu(4, true);
    r.mcu(33, false);
    expect(r.seen).toEqual([]);
  });

  it('sameLedVisual compares by value', () => {
    expect(sameLedVisual(undefined, false)).toBe(false);
    expect(sameLedVisual(true, true)).toBe(true);
    expect(sameLedVisual({ r: 1, g: 2, b: 3 }, { r: 1, g: 2, b: 3 })).toBe(true);
    expect(sameLedVisual({ r: 1, g: 2, b: 3 }, { r: 1, g: 2, b: 4 })).toBe(false);
    expect(sameLedVisual(true, { r: 255, g: 255, b: 255 })).toBe(false);
  });
});
