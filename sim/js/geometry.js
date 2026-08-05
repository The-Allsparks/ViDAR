/** Uncertainty-weighted range fusion (matches Java VidarGeometry). */

/** @param {number} diameterIn @param {number} focalPx @param {number} radiusPx */
export function distanceFromSizeInches(diameterIn, focalPx, radiusPx) {
  if (!radiusPx || radiusPx <= 0 || !focalPx || !diameterIn) return null;
  return (diameterIn * focalPx) / (2 * radiusPx);
}

/** @param {number} physicalWidthIn @param {number} focalPx @param {number} pixelWidth */
export function distanceFromWidthInches(physicalWidthIn, focalPx, pixelWidth) {
  if (!pixelWidth || pixelWidth <= 0 || !focalPx || !physicalWidthIn) return null;
  return (physicalWidthIn * focalPx) / pixelWidth;
}

/** @param {number} cy @param {{ floorLut: { cy: number, distIn: number }[] }} profile */
export function distanceFromFloorInches(cy, profile) {
  const lut = profile.floorLut;
  if (!lut?.length) return null;

  const xs = lut.map((p) => p.cy);
  const ys = lut.map((p) => p.distIn);

  if (cy <= xs[0]) return ys[0];
  const last = xs.length - 1;
  if (cy >= xs[last]) return ys[last];

  for (let i = 0; i < last; i++) {
    if (cy >= xs[i] && cy <= xs[i + 1]) {
      const t = (cy - xs[i]) / (xs[i + 1] - xs[i]);
      return ys[i] + t * (ys[i + 1] - ys[i]);
    }
  }
  return null;
}

/**
 * @param {number | null} dSize
 * @param {number | null} dFloor
 * @param {{ maxRangeMismatchRatio?: number }} [opts]
 */
export function fuseRangeWeighted(dSize, dFloor, opts = {}) {
  const maxMismatch = opts.maxRangeMismatchRatio ?? 0.28;
  /** @type {{ source: string, distance: number, weight: number, uncertainty: number, rejected?: string }[]} */
  const sources = [];

  if (dSize != null && dSize > 0) {
    const unc = dSize * 0.08;
    sources.push({ source: "SIZE", distance: dSize, weight: 1 / (unc * unc), uncertainty: unc });
  }
  if (dFloor != null && dFloor > 0) {
    const unc = dFloor * 0.12;
    sources.push({ source: "FLOOR", distance: dFloor, weight: 1 / (unc * unc), uncertainty: unc });
  }

  const valid = sources.filter((s) => s.weight > 0);
  if (!valid.length) {
    return { distanceIn: null, uncertaintyIn: null, confidence: 0, sources };
  }

  const weightSum = valid.reduce((a, s) => a + s.weight, 0);
  const fused = valid.reduce((a, s) => a + s.weight * s.distance, 0) / weightSum;
  const unc = Math.sqrt(valid.reduce((a, s) => a + s.weight * s.uncertainty ** 2, 0) / weightSum);

  let confidence = Math.min(1, weightSum / valid.length);
  if (valid.length >= 2) {
    let maxDiff = 0;
    for (const a of valid) {
      for (const b of valid) {
        const denom = Math.max(a.distance, b.distance);
        if (denom > 0) maxDiff = Math.max(maxDiff, Math.abs(a.distance - b.distance) / denom);
      }
    }
    if (maxDiff > maxMismatch) confidence *= Math.max(0.2, 1 - maxDiff);
  }

  return { distanceIn: fused, uncertaintyIn: unc, confidence, sources };
}

/** @deprecated Use fuseRangeWeighted */
export function fuseRangeInches(dSize, dFloor) {
  const r = fuseRangeWeighted(dSize, dFloor);
  return r.distanceIn;
}

/** @deprecated Use fuseRangeWeighted */
export function rangeConfidence(dSize, dFloor, maxMismatch = 0.28) {
  return fuseRangeWeighted(dSize, dFloor, { maxRangeMismatchRatio: maxMismatch }).confidence;
}

/** @param {number | null} rangeIn @param {number} bearingDeg */
export function robotXYInches(rangeIn, bearingDeg) {
  if (rangeIn == null || Number.isNaN(rangeIn)) return { x: null, y: null };
  const rad = (bearingDeg * Math.PI) / 180;
  return { x: rangeIn * Math.cos(rad), y: rangeIn * Math.sin(rad) };
}

/**
 * @param {import('./detection.js').Detection} det
 * @param {import('./config.js').GeometryConfig} geometry
 * @param {number} [cameraIndex]
 */
export function applyBallGeometry(det, geometry, cameraIndex) {
  if (det.category !== "element" || !det.radius) return det;

  const idx = cameraIndex ?? geometry.activeCameraIndex ?? 0;
  const profile = geometry.cameras?.[idx] ?? geometry.cameras?.[0];
  if (!profile) return det;

  const dSizeIn = distanceFromSizeInches(geometry.ballDiameterIn, profile.focalLengthPx, det.radius);
  const dFloorIn = distanceFromFloorInches(det.cy, profile);
  const fused = fuseRangeWeighted(dSizeIn, dFloorIn, { maxRangeMismatchRatio: geometry.maxRangeMismatchRatio });
  const { x: robotXIn, y: robotYIn } = robotXYInches(fused.distanceIn, profile.bearingDeg);

  return {
    ...det,
    dSizeIn,
    dFloorIn,
    rangeIn: fused.distanceIn,
    rangeUncertaintyIn: fused.uncertaintyIn,
    confidence: fused.confidence,
    rangeSources: fused.sources,
    robotXIn,
    robotYIn,
    cameraName: profile.name,
    bearingDeg: profile.bearingDeg,
  };
}

/** @param {import('./detection.js').Detection[]} detections @param {import('./config.js').GeometryConfig | undefined} geometry */
export function enrichBallGeometry(detections, geometry) {
  if (!geometry) return detections;
  return detections.map((d) =>
    d.category === "element" && (d.shape === "circle" || d.radius)
      ? applyBallGeometry(d, geometry)
      : d,
  );
}

/** @param {import('./detection.js').Detection | null | undefined} ball @param {import('./config.js').GeometryConfig | undefined} geometry */
export function rangeDrivePower(ball, geometry) {
  if (!ball?.rangeIn || ball.confidence == null) return 0;
  const minConf = geometry?.minBallConfidence ?? 0.35;
  if (ball.confidence < minConf) return 0;

  const stop = geometry?.pickupStopIn ?? 14;
  const maxRange = geometry?.seekMaxRangeIn ?? 72;
  if (ball.rangeIn <= stop || ball.rangeIn > maxRange) return 0;

  const raw = (ball.rangeIn - stop) * 0.025;
  return Math.min(0.35, raw);
}
