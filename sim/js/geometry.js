/** Uncertainty-weighted range fusion (parity with Java VidarRangeFusion / Python geometry). */

import { distanceFromGroundPlane, hasGroundPlaneExtrinsics } from "./ground-plane.js";

const HOUGH_MIN_RADIUS = 8;
const DEFAULT_MAX_MISMATCH = 0.28;

/** @param {number} diameter @param {number} focalPx @param {number} radiusPx */
export function distanceFromSize(diameter, focalPx, radiusPx) {
  if (!radiusPx || radiusPx <= 0 || !focalPx || !diameter) return null;
  return (diameter * focalPx) / (2 * radiusPx);
}

/** @param {number} physicalWidthIn @param {number} focalPx @param {number} pixelWidth */
export function distanceFromWidth(physicalWidthIn, focalPx, pixelWidth) {
  if (!pixelWidth || pixelWidth <= 0 || !focalPx || !physicalWidthIn) return null;
  return (physicalWidthIn * focalPx) / pixelWidth;
}

/** @param {number} cy @param {{ floorLut: { cy: number, dist: number }[] }} profile */
export function distanceFromFloor(cy, profile) {
  const lut = profile.floorLut;
  if (!lut?.length) return null;

  const xs = lut.map((p) => p.cy);
  const ys = lut.map((p) => p.dist);

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

/** @param {number} horizonRowPx */
export function profileHorizonConfidence(horizonRowPx) {
  return Math.max(0.3, 1.0 - horizonRowPx / 120.0);
}

/**
 * @typedef {Object} RangeEstimate
 * @property {string} source
 * @property {number | null} distance
 * @property {number} weight
 * @property {number} uncertainty
 * @property {boolean} valid
 * @property {string} [rejectionReason]
 */

/** @returns {RangeEstimate} */
export function buildGroundPlaneEstimate(dGround, cyPx, horizonConfidence, nearHorizon) {
  void cyPx;
  if (dGround == null || !Number.isFinite(dGround) || dGround <= 0) {
    return rejected("GROUND_PLANE", "invalid_geometry");
  }
  if (nearHorizon) {
    return rejected("GROUND_PLANE", "near_horizon");
  }
  const baseUncertainty = (dGround * 0.1) / Math.max(0.25, horizonConfidence);
  const weight = 1 / (baseUncertainty * baseUncertainty);
  return valid("GROUND_PLANE", dGround, weight, baseUncertainty);
}

/** @returns {RangeEstimate} */
export function buildSizeEstimate(
  dSize,
  radiusPx,
  circleFitQuality,
  partialOcclusion,
  touchesBoundary,
) {
  if (dSize == null || !Number.isFinite(dSize) || dSize <= 0) {
    return rejected("SIZE", "invalid_distance");
  }
  if (radiusPx < HOUGH_MIN_RADIUS) {
    return rejected("SIZE", "radius_too_small");
  }
  let baseUncertainty = (dSize * 0.08) / Math.max(0.3, circleFitQuality ?? 0.9);
  if (partialOcclusion) baseUncertainty *= 1.8;
  if (touchesBoundary) baseUncertainty *= 2.0;
  const weight = 1 / (baseUncertainty * baseUncertainty);
  return valid("SIZE", dSize, weight, baseUncertainty);
}

/** @returns {RangeEstimate} */
export function buildFloorEstimate(dFloor, cyPx, horizonConfidence, nearHorizon) {
  void cyPx;
  if (dFloor == null || !Number.isFinite(dFloor) || dFloor <= 0) {
    return rejected("FLOOR", "invalid_lut");
  }
  if (nearHorizon) {
    return rejected("FLOOR", "near_horizon");
  }
  const baseUncertainty = (dFloor * 0.12) / Math.max(0.2, horizonConfidence);
  const weight = 1 / (baseUncertainty * baseUncertainty);
  return valid("FLOOR", dFloor, weight, baseUncertainty);
}

/** @returns {RangeEstimate} */
export function buildPlateWidthEstimate(
  dWidth,
  pixelWidth,
  rectangularity,
  whiteRatio,
  partialVisibility,
  touchesRoiBoundary,
  rotationPenalty,
) {
  if (dWidth == null || !Number.isFinite(dWidth) || dWidth <= 0) {
    return rejected("PLATE_WIDTH", "invalid_width");
  }
  if (pixelWidth < 20) {
    return rejected("PLATE_WIDTH", "width_too_small");
  }
  let baseUncertainty = dWidth * 0.1;
  baseUncertainty *= 2.0 - Math.min(1.0, rectangularity ?? 0.8);
  baseUncertainty *= 1.5 - Math.min(0.5, whiteRatio ?? 0.3);
  if (partialVisibility) baseUncertainty *= 1.6;
  if (touchesRoiBoundary) baseUncertainty *= 1.4;
  baseUncertainty *= 1.0 + (rotationPenalty ?? 0);
  const weight = 1 / (baseUncertainty * baseUncertainty);
  return valid("PLATE_WIDTH", dWidth, weight, baseUncertainty);
}

/** @param {string} source @param {string} reason */
function rejected(source, reason) {
  return {
    source,
    distance: null,
    weight: 0,
    uncertainty: 0,
    valid: false,
    rejectionReason: reason,
  };
}

/** @param {string} source @param {number} distance @param {number} weight @param {number} uncertainty */
function valid(source, distance, weight, uncertainty) {
  return { source, distance, weight, uncertainty, valid: true };
}

/** @param {RangeEstimate[]} estimates */
function estimatesToSources(estimates) {
  return estimates.map((e) => ({
    source: e.source,
    distance: e.distance,
    weight: e.weight,
    uncertainty: e.uncertainty,
    rejected: e.valid ? undefined : e.rejectionReason,
  }));
}

/**
 * Ground-plane range is authoritative when valid; heuristics cross-check or fall back.
 * @param {(RangeEstimate | null | undefined)[]} estimates
 * @param {number} [maxRangeMismatchRatio]
 */
export function fuseRangeEstimates(estimates, maxRangeMismatchRatio = DEFAULT_MAX_MISMATCH) {
  /** @type {RangeEstimate[]} */
  const valid = [];
  let firstAny = null;
  let secondAny = null;
  let anyCount = 0;

  for (const est of estimates) {
    if (!est) continue;
    if (anyCount === 0) {
      firstAny = est;
      anyCount = 1;
    } else if (anyCount === 1) {
      secondAny = est;
      anyCount = 2;
    }
    if (est.valid && valid.length < 4) {
      valid.push(est);
    }
  }

  if (!valid.length) {
    const sources = estimatesToSources(estimates.filter(Boolean));
    if (anyCount === 0) {
      return invalidResult(sources);
    }
    if (anyCount === 1) {
      return {
        distance: null,
        uncertaintyIn: null,
        confidence: 0,
        sources,
        sourceCount: 1,
        primarySource: firstAny?.source ?? null,
      };
    }
    return {
      distance: null,
      uncertaintyIn: null,
      confidence: 0,
      sources,
      sourceCount: 2,
      primarySource: firstAny?.source ?? null,
    };
  }

  const ground = valid.find((e) => e.source === "GROUND_PLANE") ?? null;
  const heuristics = valid.filter((e) => e.source !== "GROUND_PLANE");
  if (ground) {
    return fuseGeometryPrimary(ground, heuristics, maxRangeMismatchRatio, estimates.filter(Boolean));
  }
  return fuseHeuristicsWeighted(valid, maxRangeMismatchRatio, estimates.filter(Boolean));
}

/** @param {{ source: string }[]} allEstimates */
function fuseGeometryPrimary(ground, heuristics, maxRangeMismatchRatio, allEstimates) {
  let maxDiffVsGround = 0;
  let agreeCount = 0;
  let disagreeCount = 0;
  /** @type {RangeEstimate | null} */
  let bestHeuristic = null;

  for (const h of heuristics) {
    const denom = Math.max(ground.distance, h.distance);
    const rel = denom > 0 ? Math.abs(ground.distance - h.distance) / denom : 0;
    maxDiffVsGround = Math.max(maxDiffVsGround, rel);
    if (rel <= maxRangeMismatchRatio) {
      agreeCount++;
    } else {
      disagreeCount++;
    }
    if (!bestHeuristic || h.weight > bestHeuristic.weight) {
      bestHeuristic = h;
    }
  }

  let confidence;
  let uncertainty = ground.uncertainty;
  if (!heuristics.length) {
    confidence = Math.min(0.75, Math.max(0.35, 0.55 * Math.min(1.0, ground.weight)));
  } else if (disagreeCount > 0 && maxDiffVsGround > maxRangeMismatchRatio) {
    confidence = Math.max(0.15, 0.7 * (1.0 - maxDiffVsGround));
    uncertainty = ground.uncertainty * (1.0 + maxDiffVsGround);
  } else {
    confidence = Math.min(1.0, 0.65 + 0.12 * agreeCount);
  }

  return {
    distance: ground.distance,
    uncertaintyIn: uncertainty,
    confidence,
    sources: estimatesToSources(allEstimates),
    sourceCount: 1 + heuristics.length,
    primarySource: "GROUND_PLANE",
  };
}

/** @param {RangeEstimate[]} valid @param {{ source: string }[]} allEstimates */
function fuseHeuristicsWeighted(valid, maxRangeMismatchRatio, allEstimates) {
  let weightSum = 0;
  let weightedDist = 0;
  let varianceSum = 0;
  for (const est of valid) {
    weightSum += est.weight;
    weightedDist += est.weight * est.distance;
    varianceSum += est.weight * est.uncertainty ** 2;
  }
  if (weightSum <= 0) {
    return invalidResult(estimatesToSources(allEstimates));
  }

  const fused = weightedDist / weightSum;
  const uncertainty = Math.sqrt(varianceSum / weightSum);

  let maxPairDiff = 0;
  for (let i = 0; i < valid.length; i++) {
    for (let j = i + 1; j < valid.length; j++) {
      const a = valid[i].distance;
      const b = valid[j].distance;
      const denom = Math.max(a, b);
      if (denom > 0) {
        maxPairDiff = Math.max(maxPairDiff, Math.abs(a - b) / denom);
      }
    }
  }
  let disagreementPenalty = 1.0;
  if (valid.length > 1 && maxPairDiff > maxRangeMismatchRatio) {
    disagreementPenalty = Math.max(0.2, 1.0 - maxPairDiff);
  }
  const confidence = Math.min(1.0, (weightSum / valid.length) * disagreementPenalty);

  let top0 = valid[0];
  let top1 = valid.length > 1 ? valid[1] : null;
  for (let i = 1; i < valid.length; i++) {
    if (valid[i].weight > top0.weight) {
      top1 = top0;
      top0 = valid[i];
    } else if (!top1 || valid[i].weight > top1.weight) {
      top1 = valid[i];
    }
  }

  return {
    distance: fused,
    uncertaintyIn: uncertainty,
    confidence,
    sources: estimatesToSources(allEstimates),
    sourceCount: valid.length,
    primarySource: top0.source,
  };
}

/** @param {ReturnType<typeof estimatesToSources>} sources */
function invalidResult(sources) {
  return {
    distance: null,
    uncertaintyIn: null,
    confidence: 0,
    sources,
    sourceCount: 0,
    primarySource: null,
  };
}

/**
 * Legacy SIZE+FLOOR-only inverse-variance average (rollback via {@code legacyAverageOnly}).
 * @param {number | null} dSize
 * @param {number | null} dFloor
 * @param {number} maxMismatch
 */
function fuseLegacyAverage(dSize, dFloor, maxMismatch) {
  /** @type {{ source: string, distance: number, weight: number, uncertainty: number }[]} */
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
    return invalidResult(sources);
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
  return {
    distance: fused,
    uncertaintyIn: unc,
    confidence,
    sources,
    sourceCount: valid.length,
    primarySource: valid[0].source,
  };
}

/**
 * @param {number | null} dSize
 * @param {number | null} dFloor
 * @param {{
 *   dGround?: number | null,
 *   dWidth?: number | null,
 *   maxRangeMismatchRatio?: number,
 *   cyPx?: number,
 *   radiusPx?: number,
 *   pixelWidth?: number,
 *   horizonRowPx?: number,
 *   nearHorizon?: boolean,
 *   circleFitQuality?: number,
 *   legacyAverageOnly?: boolean,
 * }} [opts]
 */
export function fuseRangeWeighted(dSize, dFloor, opts = {}) {
  const maxMismatch = opts.maxRangeMismatchRatio ?? DEFAULT_MAX_MISMATCH;
  if (opts.legacyAverageOnly) {
    return fuseLegacyAverage(dSize, dFloor, maxMismatch);
  }

  const horizonRowPx = opts.horizonRowPx ?? 12;
  const horizonConf = profileHorizonConfidence(horizonRowPx);
  const nearHorizon = opts.nearHorizon ?? false;
  const cyPx = opts.cyPx ?? 0;
  const radiusPx = opts.radiusPx ?? 0;

  /** @type {RangeEstimate[]} */
  const estimates = [];
  if (dSize != null) {
    estimates.push(
      buildSizeEstimate(dSize, radiusPx, opts.circleFitQuality ?? 0.9, false, false),
    );
  }
  if (dFloor != null) {
    estimates.push(buildFloorEstimate(dFloor, cyPx, horizonConf, nearHorizon));
  }
  if (opts.dGround != null) {
    estimates.push(buildGroundPlaneEstimate(opts.dGround, cyPx, horizonConf, nearHorizon));
  }
  if (opts.dWidth != null) {
    estimates.push(
      buildPlateWidthEstimate(
        opts.dWidth,
        opts.pixelWidth ?? 0,
        0.8,
        0.3,
        false,
        false,
        0.1,
      ),
    );
  }
  return fuseRangeEstimates(estimates, maxMismatch);
}

/** @param {number | null} range @param {number} bearingDeg */
export function robotXYInches(range, bearingDeg) {
  if (range == null || Number.isNaN(range)) return { x: null, y: null };
  const rad = (bearingDeg * Math.PI) / 180;
  return { x: range * Math.cos(rad), y: range * Math.sin(rad) };
}

/**
 * @param {import('./detection.js').Detection} det
 * @param {import('./config.js').GeometryConfig} geometry
 * @param {number} [cameraIndex]
 */
export function applyElementGeometry(det, geometry, cameraIndex) {
  if (det.category !== "element" || !det.radius) return det;

  const idx = cameraIndex ?? geometry.activeCameraIndex ?? 0;
  const profile = geometry.cameras?.[idx] ?? geometry.cameras?.[0];
  if (!profile) return det;

  const elementDiameter = geometry.elementDiameter ?? 5.0;
  const nearHorizon = det.cy <= profile.horizonRowPx + 8;
  const dSize = distanceFromSize(elementDiameter, profile.focalLengthPx, det.radius);
  const dFloor = distanceFromFloor(det.cy, profile);
  const targetHeight = elementDiameter * 0.5;
  const dGround = hasGroundPlaneExtrinsics(profile)
    ? distanceFromGroundPlane(det.cx, det.cy, profile, targetHeight)
    : null;

  const fused = fuseRangeWeighted(dSize, dFloor, {
    dGround,
    cyPx: det.cy,
    radiusPx: det.radius,
    horizonRowPx: profile.horizonRowPx,
    nearHorizon,
    maxRangeMismatchRatio: geometry.maxRangeMismatchRatio,
    legacyAverageOnly: geometry.legacyRangeFusion === true,
    circleFitQuality: det.circularity ?? 0.9,
  });
  const { x: robotX, y: robotY } = robotXYInches(fused.distance, profile.bearingDeg);

  return {
    ...det,
    elementId: det.elementId ?? det.name,
    dSize,
    dFloor,
    dGround,
    groundRejected: dGround == null && !hasGroundPlaneExtrinsics(profile)
      ? "missing_extrinsics"
      : undefined,
    range: fused.distance,
    rangeUncertaintyIn: fused.uncertaintyIn,
    confidence: fused.confidence,
    rangeSources: fused.sources,
    primaryRangeSource: fused.primarySource,
    rangeSourceCount: fused.sourceCount,
    robotX,
    robotY,
    cameraName: profile.name,
    bearingDeg: profile.bearingDeg,
  };
}

/** @param {import('./detection.js').Detection} det @param {import('./config.js').GeometryConfig} geometry @param {number} [cameraIndex] */
export function applyPlateGeometry(det, geometry, cameraIndex) {
  if (det.category !== "robot") return det;

  const idx = cameraIndex ?? geometry.activeCameraIndex ?? 0;
  const profile = geometry.cameras?.[idx] ?? geometry.cameras?.[0];
  if (!profile) return det;

  const plateWidthIn = 12;
  const nearHorizon = det.cy <= profile.horizonRowPx + 8;
  const dWidth = distanceFromWidth(plateWidthIn, profile.focalLengthPx, det.w);
  const dFloor = distanceFromFloor(det.cy, profile);
  const dGround = hasGroundPlaneExtrinsics(profile)
    ? distanceFromGroundPlane(det.cx, det.cy, profile, 0)
    : null;

  const fused = fuseRangeWeighted(null, dFloor, {
    dGround,
    dWidth,
    pixelWidth: det.w,
    cyPx: det.cy,
    horizonRowPx: profile.horizonRowPx,
    nearHorizon,
    maxRangeMismatchRatio: geometry.maxRangeMismatchRatio,
    legacyAverageOnly: geometry.legacyRangeFusion === true,
  });
  const { x: robotX, y: robotY } = robotXYInches(fused.distance, profile.bearingDeg);

  return {
    ...det,
    dSize: dWidth,
    dFloor,
    dGround,
    range: fused.distance,
    confidence: fused.confidence,
    rangeSources: fused.sources,
    primaryRangeSource: fused.primarySource,
    robotX,
    robotY,
    cameraName: profile.name,
    bearingDeg: profile.bearingDeg,
  };
}

/** @param {import('./detection.js').Detection[]} detections @param {import('./config.js').GeometryConfig | undefined} geometry */
export function enrichElementGeometry(detections, geometry) {
  if (!geometry) return detections;
  return detections.map((d) => {
    if (d.category === "element" && (d.shape === "circle" || d.radius)) {
      return applyElementGeometry(d, geometry);
    }
    if (d.category === "robot") {
      return applyPlateGeometry(d, geometry);
    }
    return d;
  });
}

/** @param {import('./detection.js').Detection | null | undefined} element @param {import('./config.js').GeometryConfig | undefined} geometry */
export function rangeDrivePower(element, geometry) {
  if (!element?.range || element.confidence == null) return 0;
  const minConf = geometry?.minElementConfidence ?? 0.35;
  if (element.confidence < minConf) return 0;

  const stop = geometry?.pickupStop ?? 14;
  const maxRange = geometry?.seekMaxRange ?? geometry?.seekMaxRangeIn ?? 72;
  if (element.range <= stop || element.range > maxRange) return 0;

  const raw = (element.range - stop) * 0.025;
  return Math.min(0.35, raw);
}

export { distanceFromGroundPlane, hasGroundPlaneExtrinsics };
