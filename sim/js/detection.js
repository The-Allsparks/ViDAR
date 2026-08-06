import { matchesPixel } from "./color.js";
import { enrichElementGeometry } from "./geometry.js";
import { roiRect } from "./config.js";
import { detectContourTarget } from "./contour-processor.js";

/**
 * @typedef {{ category: string, name: string, label: string, color: string, shape: string, cx: number, cy: number, area: number, x: number, y: number, w: number, h: number, circularity?: number, radius?: number, aspectRatio?: number, interior?: number, detector?: string, range?: number | null, dSize?: number | null, dFloor?: number | null, confidence?: number, robotX?: number | null, robotY?: number | null, cameraName?: string, bearingDeg?: number }} Detection
 */

/**
 * @typedef {{ detectElement: boolean, detectRedPlate: boolean, detectBluePlate: boolean, grayscaleProcess?: boolean }} DetectOptions
 */

/**
 * @param {ImageData} colorImageData
 * @param {import('./config.js').VidarTuning} tuning
 * @param {{ x: number, y: number, w: number, h: number }} roi
 * @param {DetectOptions} [options]
 * @returns {{ detections: Detection[], elementMask: Uint8Array | null, activeDetectors: string }}
 */
export function detectBlobs(colorImageData, tuning, roi, options) {
  const opts = options ?? {
    detectElement: true,
    detectRedPlate: true,
    detectBluePlate: true,
    grayscaleProcess: false,
  };

  const { width, height, data } = colorImageData;
  const elementFrame = opts.grayscaleProcess ? grayscaleCopy(colorImageData) : colorImageData;
  const elementData = elementFrame.data;
  const roiX0 = Math.max(0, roi.x);
  const roiY0 = Math.max(0, roi.y);
  const roiX1 = Math.min(width, roi.x + roi.w);
  const roiY1 = Math.min(height, roi.y + roi.h);
  const roiBox = { x: roiX0, y: roiY0, w: roiX1 - roiX0, h: roiY1 - roiY0 };

  /** @type {Detection[]} */
  const all = [];
  /** @type {Uint8Array | null} */
  let elementMask = null;

  const elementTarget = tuning.elements.find((t) => t.shape === "circle") ?? tuning.elements[0];
  if (opts.detectElement && elementTarget) {
    all.push(...detectContourTarget(elementData, width, height, roiBox, elementTarget, tuning));
    elementMask = buildRawMask(elementData, width, height, roiX0, roiY0, roiX1, roiY1, elementTarget);
  }

  for (const target of tuning.robots) {
    if (target.name === "plate_red" && !opts.detectRedPlate) continue;
    if (target.name === "plate_blue" && !opts.detectBluePlate) continue;
    all.push(...detectContourTarget(elementData, width, height, roiBox, target, tuning));
  }

  all.sort((a, b) => b.area - a.area);
  const withGeometry = enrichElementGeometry(all, tuning.geometry);

  const parts = [];
  if (opts.detectElement) parts.push(opts.grayscaleProcess ? "contour@gray" : "contour");
  if (opts.detectRedPlate) parts.push("red");
  if (opts.detectBluePlate) parts.push("blue");

  return { detections: withGeometry, elementMask, activeDetectors: parts.join("+") || "none" };
}

function grayscaleCopy(source) {
  const { width, height, data } = source;
  const out = new ImageData(width, height);
  for (let i = 0; i < width * height; i++) {
    const si = i * 4;
    const g = Math.round(0.299 * data[si] + 0.587 * data[si + 1] + 0.114 * data[si + 2]);
    out.data[si] = g;
    out.data[si + 1] = g;
    out.data[si + 2] = g;
    out.data[si + 3] = 255;
  }
  return out;
}

function buildRawMask(data, width, height, roiX0, roiY0, roiX1, roiY1, target) {
  const mask = new Uint8Array(width * height);
  for (let y = roiY0; y < roiY1; y++) {
    for (let x = roiX0; x < roiX1; x++) {
      const i = (y * width + x) * 4;
      if (matchesPixel(data[i], data[i + 1], data[i + 2], target)) {
        mask[y * width + x] = 1;
      }
    }
  }
  return mask;
}

/** Debug: element mask preview for contour tuning. */
export function buildElementMaskPreview(imageData, tuning, roi) {
  const target = tuning.elements[0];
  if (!target) return null;
  const { width, height, data } = imageData;
  const roiX0 = Math.max(0, roi.x);
  const roiY0 = Math.max(0, roi.y);
  const roiX1 = Math.min(width, roi.x + roi.w);
  const roiY1 = Math.min(height, roi.y + roi.h);
  return buildRawMask(data, width, height, roiX0, roiY0, roiX1, roiY1, target);
}

/** @param {import('./config.js').VidarTuning} tuning @param {number} w @param {number} h */
export function computeRoi(tuning, w, h) {
  if (!tuning.useCenterRoi) {
    return { x: 0, y: 0, w, h };
  }
  return roiRect(w, h, tuning.roiUnity);
}

/** @param {Detection[]} detections @param {number} [frameH] @param {import('./config.js').GeometryConfig} [geometry] */
export function bestByCategory(detections, frameH = 480, geometry) {
  /** @type {{ element: Detection | null, robot: Detection | null }} */
  const best = { element: null, robot: null };
  let bestElementScore = -1;
  const minConf = geometry?.minElementConfidence ?? 0.35;

  for (const d of detections) {
    if (d.category === "element") {
      const conf = d.confidence ?? 1;
      if (conf < minConf) continue;

      const floorWeight = 0.25 + 0.75 * (d.cy / frameH);
      const rangeBoost = d.range != null && d.range > 0 ? Math.min(2, 48 / d.range) : 1;
      const score = d.area * floorWeight * floorWeight * conf * rangeBoost;
      if (score > bestElementScore) {
        bestElementScore = score;
        best.element = d;
      }
    }
    if (d.category === "robot" && !best.robot) best.robot = d;
  }
  return best;
}

/** @param {Uint8Array} mask @param {number} width @param {number} height */
export function maskToCanvas(mask, width, height) {
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(width, height);
  for (let i = 0; i < mask.length; i++) {
    const o = i * 4;
    if (mask[i]) {
      img.data[o] = 255;
      img.data[o + 1] = 80;
      img.data[o + 2] = 200;
      img.data[o + 3] = 180;
    }
  }
  ctx.putImageData(img, 0, 0);
  return canvas;
}
