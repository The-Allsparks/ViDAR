import { matchesPixel } from "./color.js";
import { enrichBallGeometry } from "./geometry.js";
import { roiRect } from "./config.js";
import { detectHoughBalls, houghEdgePreview } from "./detection-hough.js";

/**
 * @typedef {{ category: string, name: string, label: string, color: string, shape: string, cx: number, cy: number, area: number, x: number, y: number, w: number, h: number, circularity?: number, radius?: number, aspectRatio?: number, votes?: number, interior?: number, detector?: string, rangeIn?: number | null, dSizeIn?: number | null, dFloorIn?: number | null, confidence?: number, robotXIn?: number | null, robotYIn?: number | null, cameraName?: string, bearingDeg?: number }} Detection
 */

/**
 * @typedef {{ ballDetector: 'hsv' | 'hough' | 'both', detectBall: boolean, detectRedPlate: boolean, detectBluePlate: boolean, grayscaleProcess?: boolean }} DetectOptions
 */

/**
 * @param {ImageData} colorImageData
 * @param {import('./config.js').VidarTuning} tuning
 * @param {{ x: number, y: number, w: number, h: number }} roi
 * @param {DetectOptions} [options]
 * @returns {{ detections: Detection[], ballMask: Uint8Array | null, activeDetectors: string }}
 */
export function detectBlobs(colorImageData, tuning, roi, options) {
  const opts = options ?? {
    ballDetector: "hsv",
    detectBall: true,
    detectRedPlate: true,
    detectBluePlate: true,
    grayscaleProcess: false,
  };

  const { width, height, data } = colorImageData;
  const ballFrame = opts.grayscaleProcess ? grayscaleCopy(colorImageData) : colorImageData;
  const ballData = ballFrame.data;
  const roiX0 = Math.max(0, roi.x);
  const roiY0 = Math.max(0, roi.y);
  const roiX1 = Math.min(width, roi.x + roi.w);
  const roiY1 = Math.min(height, roi.y + roi.h);
  const roiBox = { x: roiX0, y: roiY0, w: roiX1 - roiX0, h: roiY1 - roiY0 };

  /** @type {Detection[]} */
  const all = [];
  /** @type {Uint8Array | null} */
  let ballMask = null;

  const ballTarget = tuning.elements.find((t) => t.shape === "circle") ?? tuning.elements[0];
  if (opts.detectBall && ballTarget) {
    const runHsv = opts.ballDetector === "hsv" || opts.ballDetector === "both";
    const runHough = opts.ballDetector === "hough" || opts.ballDetector === "both";

    if (runHough) {
      const houghDets = detectHoughBalls(ballFrame, ballTarget, roiBox).map((d) => ({
        ...d,
        label: opts.ballDetector === "both" ? "Ball (Hough)" : d.label,
        color: opts.ballDetector === "both" ? "#b8d4ff" : d.color,
      }));
      all.push(...houghDets);
      if (opts.ballDetector !== "hsv") {
        ballMask = houghEdgePreview(ballFrame, ballTarget, roiBox);
      }
    }
    if (runHsv) {
      const { detections: hsvDets, mask } = detectBallHsv(
        ballData, width, height, roiX0, roiY0, roiX1, roiY1, ballTarget, tuning,
      );
      all.push(...hsvDets.map((d) => ({
        ...d,
        label: opts.ballDetector === "both" ? "Ball (HSV)" : d.label,
        color: opts.ballDetector === "both" ? "#88ffcc" : d.color,
        detector: "hsv",
      })));
      if (runHsv && (opts.ballDetector === "hsv" || opts.ballDetector === "both")) {
        ballMask = mask;
      }
    }
  }

  for (const target of tuning.robots) {
    if (target.name === "plate_red" && !opts.detectRedPlate) continue;
    if (target.name === "plate_blue" && !opts.detectBluePlate) continue;

    const mask = buildRawMask(data, width, height, roiX0, roiY0, roiX1, roiY1, target);
    applyMorph(target, mask, width, height, roiX0, roiY0, roiX1, roiY1);
    const blobs = connectedComponents(mask, width, height, roiX0, roiY0, roiX1, roiY1);
    all.push(...blobsToDetections(blobs, "robot", target, tuning));
  }

  all.sort((a, b) => b.area - a.area);

  const withGeometry = enrichBallGeometry(all, tuning.geometry);

  const parts = [];
  if (opts.detectBall) parts.push(opts.grayscaleProcess ? `${opts.ballDetector}@gray` : opts.ballDetector);
  if (opts.detectRedPlate) parts.push("red");
  if (opts.detectBluePlate) parts.push("blue");
  const activeDetectors = parts.join("+") || "none";

  return { detections: withGeometry, ballMask, activeDetectors };
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

/**
 * @param {{ area: number, cx: number, cy: number, minX: number, minY: number, maxX: number, maxY: number, perimeter: number }[]} blobs
 */
function blobsToDetections(blobs, category, target, tuning) {
  /** @type {Detection[]} */
  const out = [];
  for (const blob of blobs) {
    const minArea = target.minArea ?? tuning.minBlobArea;
    if (blob.area < minArea || blob.area > tuning.maxBlobArea) continue;

    const w = blob.maxX - blob.minX + 1;
    const h = blob.maxY - blob.minY + 1;
    const aspectRatio = Math.max(w, h) / Math.max(1, Math.min(w, h));
    const circularity = blob.perimeter > 0
      ? (4 * Math.PI * blob.area) / (blob.perimeter * blob.perimeter)
      : 0;

    if (target.shape === "circle") {
      const maxAspect = target.maxAspectRatio ?? 2.0;
      if (aspectRatio > maxAspect) continue;
      if (target.requireCircularity !== false) {
        const minCirc = target.minCircularity ?? 0.15;
        if (circularity < minCirc) continue;
      }
    }

    const radius = Math.sqrt(blob.area / Math.PI);
    out.push({
      category,
      name: target.name,
      label: target.label,
      color: target.color,
      shape: target.shape ?? "plate",
      cx: blob.cx,
      cy: blob.cy,
      area: blob.area,
      x: blob.minX,
      y: blob.minY,
      w,
      h,
      circularity,
      radius,
      aspectRatio,
    });
  }
  return out;
}

function detectBallHsv(data, width, height, roiX0, roiY0, roiX1, roiY1, target, tuning) {
  const mask = buildRawMask(data, width, height, roiX0, roiY0, roiX1, roiY1, target);
  applyMorph(target, mask, width, height, roiX0, roiY0, roiX1, roiY1);
  const blobs = connectedComponents(mask, width, height, roiX0, roiY0, roiX1, roiY1);
  const detections = blobsToDetections(blobs, "element", target, tuning);
  return { detections, mask: mask.slice() };
}

/** Debug: white-threshold mask before morph (for holed-ball review). */
export function buildBallMaskPreview(imageData, tuning, roi) {
  const target = tuning.elements[0];
  if (!target) return null;
  const { width, height, data } = imageData;
  const roiX0 = Math.max(0, roi.x);
  const roiY0 = Math.max(0, roi.y);
  const roiX1 = Math.min(width, roi.x + roi.w);
  const roiY1 = Math.min(height, roi.y + roi.h);
  const mask = buildRawMask(data, width, height, roiX0, roiY0, roiX1, roiY1, target);
  applyMorph(target, mask, width, height, roiX0, roiY0, roiX1, roiY1);
  return mask;
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

function applyMorph(target, mask, width, height, x0, y0, x1, y1) {
  if (target.shape === "circle") {
    const passes = target.morphClosePasses ?? 5;
    morphClose(mask, width, height, x0, y0, x1, y1, passes);
  } else {
    dilate3x3(mask, width, height, x0, y0, x1, y1);
  }
}

/** @param {import('./config.js').VidarTuning} tuning @param {number} w @param {number} h */
export function computeRoi(tuning, w, h) {
  if (!tuning.useCenterRoi) {
    return { x: 0, y: 0, w, h };
  }
  return roiRect(w, h, tuning.roiUnity);
}

function rgbToHsvFast(r, g, b) {
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

function dilate3x3(mask, width, height, x0, y0, x1, y1) {
  const copy = mask.slice();
  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      if (!copy[y * width + x]) continue;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx >= x0 && nx < x1 && ny >= y0 && ny < y1) {
            mask[ny * width + nx] = 1;
          }
        }
      }
    }
  }
}

function erode3x3(mask, width, height, x0, y0, x1, y1) {
  const copy = mask.slice();
  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      if (!copy[y * width + x]) continue;
      let keep = true;
      for (let dy = -1; dy <= 1 && keep; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx < x0 || nx >= x1 || ny < y0 || ny >= y1 || !copy[ny * width + nx]) {
            keep = false;
            break;
          }
        }
      }
      if (!keep) mask[y * width + x] = 0;
    }
  }
}

function morphClose(mask, width, height, x0, y0, x1, y1, passes) {
  for (let i = 0; i < passes; i++) {
    dilate3x3(mask, width, height, x0, y0, x1, y1);
  }
  for (let i = 0; i < passes; i++) {
    erode3x3(mask, width, height, x0, y0, x1, y1);
  }
}

function connectedComponents(mask, width, height, x0, y0, x1, y1) {
  const labels = new Int32Array(width * height);
  let nextLabel = 1;
  /** @type {{ area: number, sumX: number, sumY: number, minX: number, minY: number, maxX: number, maxY: number, perimeter: number }[]} */
  const stats = [null];

  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      const idx = y * width + x;
      if (!mask[idx] || labels[idx]) continue;

      const label = nextLabel++;
      const stack = [idx];
      labels[idx] = label;
      stats[label] = {
        area: 0,
        sumX: 0,
        sumY: 0,
        minX: x,
        minY: y,
        maxX: x,
        maxY: y,
        perimeter: 0,
      };

      while (stack.length) {
        const cur = stack.pop();
        const cy = Math.floor(cur / width);
        const cx = cur - cy * width;
        const s = stats[label];
        s.area += 1;
        s.sumX += cx;
        s.sumY += cy;
        s.minX = Math.min(s.minX, cx);
        s.minY = Math.min(s.minY, cy);
        s.maxX = Math.max(s.maxX, cx);
        s.maxY = Math.max(s.maxY, cy);

        let edge = false;
        const dirs = [[-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [1, -1], [-1, 1], [1, 1]];
        for (const [dx, dy] of dirs) {
          const nx = cx + dx;
          const ny = cy + dy;
          if (nx < x0 || nx >= x1 || ny < y0 || ny >= y1 || !mask[ny * width + nx]) {
            edge = true;
            break;
          }
        }
        if (edge) s.perimeter += 1;

        if (cx > x0 && !labels[cur - 1] && mask[cur - 1]) {
          labels[cur - 1] = label;
          stack.push(cur - 1);
        }
        if (cx + 1 < x1 && !labels[cur + 1] && mask[cur + 1]) {
          labels[cur + 1] = label;
          stack.push(cur + 1);
        }
        if (cy > y0 && !labels[cur - width] && mask[cur - width]) {
          labels[cur - width] = label;
          stack.push(cur - width);
        }
        if (cy + 1 < y1 && !labels[cur + width] && mask[cur + width]) {
          labels[cur + width] = label;
          stack.push(cur + width);
        }
      }
    }
  }

  return stats
    .filter(Boolean)
    .map((s) => ({
      area: s.area,
      cx: s.sumX / s.area,
      cy: s.sumY / s.area,
      minX: s.minX,
      minY: s.minY,
      maxX: s.maxX,
      maxY: s.maxY,
      perimeter: s.perimeter,
    }));
}

/** @param {Detection[]} detections @param {number} [frameH] @param {import('./config.js').GeometryConfig} [geometry] */
export function bestByCategory(detections, frameH = 480, geometry) {
  /** @type {{ element: Detection | null, robot: Detection | null }} */
  const best = { element: null, robot: null };
  let bestBallScore = -1;
  const minConf = geometry?.minBallConfidence ?? 0.35;

  for (const d of detections) {
    if (d.category === "element") {
      const conf = d.confidence ?? 1;
      if (conf < minConf) continue;

      const floorWeight = 0.25 + 0.75 * (d.cy / frameH);
      const voteBoost = d.votes ? Math.min(2, d.votes / 20) : 1;
      const rangeBoost = d.rangeIn != null && d.rangeIn > 0 ? Math.min(2, 48 / d.rangeIn) : 1;
      const score = d.area * floorWeight * floorWeight * voteBoost * conf * rangeBoost;
      if (score > bestBallScore) {
        bestBallScore = score;
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
