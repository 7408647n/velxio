/**
 * On-board LEDs: from the levels and frames a board's engine reports to what
 * the board element should draw.
 *
 * The canvas used to subscribe every board to pin 13 and hand the raw level
 * to the element, which is right for an Uno and for nothing else: the ESP32
 * kinds never lit (issue #374), and a board whose LED sits between 3V3 and
 * the pin would have shown the inverse of the hardware. The table in
 * types/board.ts now says which pin, and this module applies the polarity so
 * the element only ever sees "lit" or a colour.
 */
import { onboardLedsFor, type OnboardLedVisual } from '../types/board';

/** The three feeds an on-board LED can hang off; a PinManager is the first two. */
export interface OnboardLedSources {
  /** Digital levels the engine reports on this board's pads. */
  onPinChange: (pin: number, cb: (pin: number, state: boolean) => void) => () => void;
  getPinState: (pin: number) => boolean;
  /** Pads the MCU has driven, so a channel the sketch never touched reads as off. */
  getOutputPins: () => ReadonlySet<number>;
  /** Decoded WS2812 frames going out on one pin of this board. */
  observeWs2812: (
    pin: number,
    sink: (pixels: ReadonlyArray<{ r: number; g: number; b: number }>) => void,
  ) => () => void;
}

/**
 * Subscribe the LEDs of `boardKind` to `src` and report each one's visual
 * through `setLed` whenever it changes. Returns the unsubscribe.
 *
 * Levels are not latched here: a board that declares no LED gets no
 * subscription, and a pin that has not reported yet leaves its LED as the
 * caller had it (unlit), which is what an unconfigured pad looks like.
 */
export function attachOnboardLeds(
  boardKind: string,
  src: OnboardLedSources,
  setLed: (ledId: string, visual: OnboardLedVisual) => void,
): () => void {
  const unsubs: (() => void)[] = [];
  for (const led of onboardLedsFor(boardKind)) {
    switch (led.kind) {
      case 'gpio':
        unsubs.push(
          src.onPinChange(led.pin, (_pin, state) => {
            setLed(led.id, led.activeLow ? !state : state);
          }),
        );
        break;
      case 'rgb-gpio': {
        // Each colour is its own pad. On an active-LOW LED an untouched pad
        // must not count as "low, therefore on": the Nano ESP32 boots with
        // all three channels dark until the sketch writes them, so only a pad
        // the MCU has driven contributes.
        const channelOn = (pin: number): boolean => {
          if (!src.getOutputPins().has(pin)) return false;
          const level = src.getPinState(pin);
          return led.activeLow ? !level : level;
        };
        const paint = () => {
          setLed(led.id, {
            r: channelOn(led.pins.r) ? 255 : 0,
            g: channelOn(led.pins.g) ? 255 : 0,
            b: channelOn(led.pins.b) ? 255 : 0,
          });
        };
        for (const pin of [led.pins.r, led.pins.g, led.pins.b]) {
          unsubs.push(src.onPinChange(pin, paint));
        }
        break;
      }
      case 'ws2812':
        unsubs.push(
          src.observeWs2812(led.pin, (pixels) => {
            // The on-board LED is the first pixel of whatever goes out on its
            // pin; on the DevKitC-1 the same GPIO also reaches the header, so
            // a strip wired there shows the rest of the frame on its own.
            const px = pixels[0];
            if (px) setLed(led.id, { r: px.r, g: px.g, b: px.b });
          }),
        );
        break;
    }
  }
  return () => unsubs.forEach((u) => u());
}

/** Two visuals that would draw the same thing. */
export function sameLedVisual(a: OnboardLedVisual | undefined, b: OnboardLedVisual): boolean {
  if (typeof a === 'boolean' || typeof b === 'boolean' || a === undefined) return a === b;
  return a.r === b.r && a.g === b.g && a.b === b.b;
}
