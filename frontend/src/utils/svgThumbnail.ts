/**
 * A part's SVG thumbnail, ready for a box of any size.
 *
 * The catalogue thumbnails are drawn on a 64x64 canvas and most of them say
 * so only with width="64" height="64", with no viewBox. Shown at 64px that is
 * fine, but the property dialog's header and the picker's hover card show
 * them in a 40px box (max-width / max-height 100%), and an SVG without a
 * viewBox does not scale when its viewport shrinks: it is cropped. The
 * header showed the top-left 40x40 of the art (the BMP280's board without
 * its chip, a capacitor cut in half). The viewBox the drawing implies is
 * added here, so the whole picture scales into the box.
 *
 * Returns null for a thumbnail that is not inline SVG (empty: the part is
 * drawn by its live element instead).
 */
export function scalableSvgThumbnail(thumbnail: string | null | undefined): string | null {
  const t = (thumbnail ?? '').trim();
  if (!t.startsWith('<svg')) return null;
  const end = t.indexOf('>');
  if (end < 0) return null;
  const open = t.slice(0, end);
  if (/\sviewBox\s*=/.test(open)) return t;
  const w = /\swidth\s*=\s*["']?([\d.]+)["']?/.exec(open);
  const h = /\sheight\s*=\s*["']?([\d.]+)["']?/.exec(open);
  if (!w || !h) return t;
  return `<svg viewBox="0 0 ${w[1]} ${h[1]}"${t.slice('<svg'.length)}`;
}
