/**
 * RGB (0-255) → HSV (H 0-179, S/V 0-255) — OpenCV-style hue for matching Java/OpenCV pipeline.
 */
export function rgbToHsv(r, g, b) {
  const rn = r / 255;
  const gn = g / 255;
  const bn = b / 255;
  const max = Math.max(rn, gn, bn);
  const min = Math.min(rn, gn, bn);
  const d = max - min;

  let h = 0;
  if (d !== 0) {
    if (max === rn) h = 60 * (((gn - bn) / d) % 6);
    else if (max === gn) h = 60 * ((bn - rn) / d + 2);
    else h = 60 * ((rn - gn) / d + 4);
  }
  if (h < 0) h += 360;

  const s = max === 0 ? 0 : (d / max) * 255;
  const v = max * 255;
  return [Math.round(h / 2), Math.round(s), Math.round(v)];
}

/** @param {number[]} pixelHsv @param {number[]} low @param {number[]} high */
export function hsvInRange(pixelHsv, low, high) {
  const [h, s, v] = pixelHsv;
  return h >= low[0] && h <= high[0] && s >= low[1] && s <= high[1] && v >= low[2] && v <= high[2];
}

/** @param {number[]} hsv @param {import('./config.js').ColorTarget} target */
export function matchesTarget(hsv, target) {
  if (hsvInRange(hsv, target.hsvLow, target.hsvHigh)) return true;
  if (target.hsvLowWrap && target.hsvHighWrap) {
    return hsvInRange(hsv, target.hsvLowWrap, target.hsvHighWrap);
  }
  return false;
}

/** HSV range and optional brightness fallback for holed / off-white balls. */
export function matchesPixel(r, g, b, target) {
  const hsv = rgbToHsv(r, g, b);
  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);

  // Bright neutrals are never treated as green plate.
  if (target.rejectGreen && isGreenish(hsv, max)) return false;

  if (matchesTarget(hsv, target)) return true;

  if (target.brightnessMin != null) {
    const maxSat = target.maxSaturation ?? 85;
    if (hsv[1] > maxSat) return false;
    return max >= target.brightnessMin && max - min <= (target.brightnessSpread ?? 60);
  }
  return false;
}

/** Green plate / carpet — but not white ball highlights. */
function isGreenish(hsv, maxRgb) {
  const [h, s] = hsv;
  if (maxRgb >= 175 && s < 90) return false;
  return s >= 60 && h >= 35 && h <= 92;
}
