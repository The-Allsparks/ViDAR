/** @typedef {{ name: string, label: string, color: string, shape?: string, hsvLow: number[], hsvHigh: number[], hsvLowWrap?: number[], hsvHighWrap?: number[], minArea?: number, requireCircularity?: boolean, minCircularity?: number, maxAspectRatio?: number, morphClosePasses?: number, brightnessMin?: number, brightnessSpread?: number, maxSaturation?: number, rejectGreen?: boolean, minInteriorScore?: number, minWhiteRatio?: number }} ColorTarget */

/** @typedef {{ name: string, bearingDeg: number, horizonRowPx: number, focalLengthPx: number, floorLut: { cy: number, dist: number }[] }} CameraProfile */

/** @typedef {{ elementDiameter?: number, maxRangeMismatchRatio?: number, minElementConfidence?: number, pickupStop?: number, seekMaxRange?: number, activeCameraIndex?: number, cameras: CameraProfile[] }} GeometryConfig */

/** @typedef {{ enabled?: boolean, minSampleIntervalMs?: number, scoutWidth?: number, bandLeftMax?: number, bandRightMin?: number, scoutMinWidthPx?: number, decodeMinTagWidthPx?: number, poseGateDeg?: number, tagSizeIn?: number, desiredTagId?: number, decimationMin?: number, decimationMax?: number, expectedBearingDeg?: number | null, cameraBearingDeg?: number }} TagConfig */

/** @typedef {{ captureWidth: number, captureHeight: number, downscaleRatio: number, verticalCropOffset: number, verticalCropHeight: number, processWidth?: number, processHeight?: number, crop?: { x: number, y: number, w: number, h: number }, minBlobArea: number, maxBlobArea: number, useCenterRoi: boolean, roiUnity: number, avoidCenterRadius: number, tag?: TagConfig, geometry?: GeometryConfig, elements: ColorTarget[], robots: ColorTarget[] }} VidarTuning */

/** @param {string} path */
export async function loadTuning(path = "vidar-tuning.json") {
  const res = await fetch(path);
  if (!res.ok) {
    throw new Error(`Failed to load ${path}: ${res.status}`);
  }
  const raw = await res.json();
  return resolveProcessing(/** @type {VidarTuning} */ (raw));
}

/** @param {number} v @param {number} lo @param {number} hi */
export function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

/** @param {VidarTuning} tuning */
export function resolveProcessing(tuning) {
  const capW = tuning.captureWidth;
  const capH = tuning.captureHeight;
  const offset = clamp(Math.round(tuning.verticalCropOffset ?? 0), 0, capH - 1);
  const cropH = clamp(
    Math.round(tuning.verticalCropHeight ?? capH),
    1,
    capH - offset,
  );
  const ratio = clamp(Number(tuning.downscaleRatio ?? 0.5), 0.1, 1.0);

  const crop = { x: 0, y: offset, w: capW, h: cropH };
  const processWidth = Math.max(8, Math.round(crop.w * ratio));
  const processHeight = Math.max(8, Math.round(crop.h * ratio));

  return {
    ...tuning,
    verticalCropOffset: offset,
    verticalCropHeight: cropH,
    downscaleRatio: ratio,
    crop,
    processWidth,
    processHeight,
  };
}

/** @param {VidarTuning} base @param {{ downscaleRatio: number, verticalCropOffset: number, verticalCropHeight: number, element?: Record<string, number> }} ui */
export function tuningFromUi(base, ui) {
  let tuning = resolveProcessing({
    ...base,
    downscaleRatio: ui.downscaleRatio,
    verticalCropOffset: ui.verticalCropOffset,
    verticalCropHeight: ui.verticalCropHeight,
  });

  if (ui.element && tuning.elements[0]) {
    const src = tuning.elements[0];
    const hsvLow = [...src.hsvLow];
    const hsvHigh = [...src.hsvHigh];
    const { hsvMinValue, ...elementPatch } = ui.element;
    if (hsvMinValue != null) hsvLow[2] = hsvMinValue;

    tuning = {
      ...tuning,
      elements: [{
        ...src,
        ...elementPatch,
        hsvLow,
        hsvHigh,
      }],
    };
  }

  return tuning;
}

export function roiRect(width, height, unity = 0.75) {
  const halfW = (width * unity) / 2;
  const halfH = (height * unity) / 2;
  const cx = width / 2;
  const cy = height / 2;
  return {
    x: Math.round(cx - halfW),
    y: Math.round(cy - halfH),
    w: Math.round(halfW * 2),
    h: Math.round(halfH * 2),
  };
}

export function scaleRect(rect, fromW, fromH, toW, toH) {
  const sx = toW / fromW;
  const sy = toH / fromH;
  return {
    x: rect.x * sx,
    y: rect.y * sy,
    w: rect.w * sx,
    h: rect.h * sy,
  };
}

/** @param {number} x @param {number} y @param {{ x: number, y: number, w: number, h: number }} crop @param {number} processW @param {number} processH */
export function processToCapture(x, y, crop, processW, processH) {
  return {
    x: crop.x + (x / processW) * crop.w,
    y: crop.y + (y / processH) * crop.h,
  };
}
