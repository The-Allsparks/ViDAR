/**
 * Interior / plate validation gates — mirrors Python `contour_detect.py`.
 */

/** @param {[number, number, number, number][]} corners @param {number} u @param {number} v */
export function bilinear(corners, u, v) {
  const [p0, p1, p2, p3] = corners;
  const x =
    (1 - u) * (1 - v) * p0[0] +
    u * (1 - v) * p1[0] +
    u * v * p2[0] +
    (1 - u) * v * p3[0];
  const y =
    (1 - u) * (1 - v) * p0[1] +
    u * (1 - v) * p1[1] +
    u * v * p2[1] +
    (1 - u) * v * p3[1];
  return [x, y];
}

/**
 * @param {Uint8ClampedArray} data
 * @param {number} width
 * @param {number} height
 * @param {number} cx
 * @param {number} cy
 * @param {number} radius
 * @param {{ interiorBright?: number, interiorSpread?: number, holeDarkMax?: number, brightnessMin?: number, brightnessSpread?: number, holeDarkMax?: number }} target
 */
export function interiorScore(data, width, height, cx, cy, radius, target) {
  const brightMin = target.interiorBright ?? target.brightnessMin ?? 90;
  const spreadMax = target.interiorSpread ?? target.brightnessSpread ?? 60;
  const holeDarkMax = target.holeDarkMax ?? 45;
  const ri = Math.floor(radius);
  const icx = Math.round(cx);
  const icy = Math.round(cy);
  let inside = 0;
  let bright = 0;
  let darkHole = 0;

  for (let dy = -ri; dy <= ri; dy++) {
    for (let dx = -ri; dx <= ri; dx++) {
      if (dx * dx + dy * dy > radius * radius) continue;
      const x = icx + dx;
      const y = icy + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      const i = (y * width + x) * 4;
      const b = data[i];
      const g = data[i + 1];
      const r = data[i + 2];
      const maxC = Math.max(r, g, b);
      const minC = Math.min(r, g, b);
      inside += 1;
      if (maxC >= brightMin && maxC - minC <= spreadMax) bright += 1;
      if (maxC < holeDarkMax) darkHole += 1;
    }
  }
  if (inside === 0) return 0;
  const brightRatio = bright / inside;
  const holeBonus = darkHole > inside * 0.05 ? 0.15 : 0;
  return Math.min(1, brightRatio + holeBonus);
}

/**
 * @param {Uint8ClampedArray} data
 * @param {number} width
 * @param {number} height
 * @param {{ cx: number, cy: number, w: number, h: number, angleDeg?: number }} box
 * @param {{ whiteSampleGrid?: number, whiteBrightMin?: number, whiteSpreadMax?: number }} target
 */
export function whiteDigitRatio(data, width, height, box, target) {
  const grid = target.whiteSampleGrid ?? 5;
  const brightMin = target.whiteBrightMin ?? 175;
  const spreadMax = target.whiteSpreadMax ?? 55;
  const angle = ((box.angleDeg ?? 0) * Math.PI) / 180;
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const hw = box.w / 2;
  const hh = box.h / 2;
  const corners = [
    [box.cx + (-hw * cos - -hh * sin), box.cy + (-hw * sin + -hh * cos)],
    [box.cx + (hw * cos - -hh * sin), box.cy + (hw * sin + -hh * cos)],
    [box.cx + (hw * cos - hh * sin), box.cy + (hw * sin + hh * cos)],
    [box.cx + (-hw * cos - hh * sin), box.cy + (-hw * sin + hh * cos)],
  ];

  let samples = 0;
  let white = 0;
  for (let gy = 1; gy < grid; gy++) {
    for (let gx = 1; gx < grid; gx++) {
      const [px, py] = bilinear(corners, gx / grid, gy / grid);
      const x = Math.round(px);
      const y = Math.round(py);
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      const i = (y * width + x) * 4;
      const b = data[i];
      const g = data[i + 1];
      const r = data[i + 2];
      const maxC = Math.max(r, g, b);
      const minC = Math.min(r, g, b);
      samples += 1;
      if (maxC >= brightMin && maxC - minC <= spreadMax) white += 1;
    }
  }
  return samples === 0 ? 0 : white / samples;
}
