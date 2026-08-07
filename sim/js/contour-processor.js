/**
 * Unified contour detection for the browser sim — aligned with Python `ContourProcessor`.
 * Uses HSV mask + blob geometry + interior / white-digit gates (not full-frame Hough).
 */
import { matchesPixel } from "./color.js";
import { interiorScore, whiteDigitRatio } from "./contour-detect.js";

/**
 * @param {Uint8ClampedArray} data
 * @param {number} width
 * @param {number} height
 * @param {{ x: number, y: number, w: number, h: number }} roi
 * @param {import('./config.js').ColorTarget} target
 * @param {import('./config.js').VidarTuning} tuning
 */
export function detectContourTarget(data, width, height, roi, target, tuning) {
  const x0 = Math.max(0, roi.x);
  const y0 = Math.max(0, roi.y);
  const x1 = Math.min(width, roi.x + roi.w);
  const y1 = Math.min(height, roi.y + roi.h);
  const mask = buildMask(data, width, height, x0, y0, x1, y1, target);
  applyMorph(target, mask, width, height, x0, y0, x1, y1);
  const blobs = connectedComponents(mask, width, height, x0, y0, x1, y1);
  return blobsToContourDetections(blobs, target, tuning, data, width, height);
}

function buildMask(data, width, height, x0, y0, x1, y1, target) {
  const mask = new Uint8Array(width * height);
  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      const i = (y * width + x) * 4;
      if (matchesPixel(data[i], data[i + 1], data[i + 2], target)) {
        mask[y * width + x] = 1;
      }
    }
  }
  return mask;
}

function applyMorph(target, mask, width, height, x0, y0, x1, y1) {
  const passes = target.morphClosePasses ?? (target.shape === "circle" ? 5 : 1);
  if (target.shape === "circle") {
    morphClose(mask, width, height, x0, y0, x1, y1, passes);
  } else {
    dilate3x3(mask, width, height, x0, y0, x1, y1);
  }
}

function blobsToContourDetections(blobs, target, tuning, data, width, height) {
  const category = target.shape === "circle" ? "element" : "robot";
  const minArea = target.minArea ?? tuning.minBlobArea;
  const minInterior = target.minInteriorScore ?? 0;
  const minWhite = target.minWhiteRatio ?? 0;
  /** @type {import('./detection.js').Detection[]} */
  const out = [];

  for (const blob of blobs) {
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
      const minCirc = target.minCircularity ?? 0.45;
      if (circularity < minCirc) continue;
      const radius = Math.sqrt(blob.area / Math.PI);
      if (minInterior > 0) {
        const interior = interiorScore(data, width, height, blob.cx, blob.cy, radius, target);
        if (interior < minInterior) continue;
        blob.interior = interior;
      }
    } else {
      const minAspect = target.minAspect ?? 1.15;
      const maxAspect = target.maxAspect ?? 4.5;
      if (aspectRatio < minAspect || aspectRatio > maxAspect) continue;
      if (minWhite > 0) {
        const ratio = whiteDigitRatio(data, width, height, {
          cx: blob.cx,
          cy: blob.cy,
          w,
          h,
          angleDeg: 0,
        }, target);
        if (ratio < minWhite) continue;
        blob.whiteRatio = ratio;
      }
    }

    out.push({
      category,
      name: target.name,
      elementId: target.name,
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
      radius: target.shape === "circle" ? Math.sqrt(blob.area / Math.PI) : undefined,
      aspectRatio,
      interior: blob.interior,
      detector: "contour",
    });
  }
  return out;
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
          if (nx >= x0 && nx < x1 && ny >= y0 && ny < y1) mask[ny * width + nx] = 1;
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
  for (let i = 0; i < passes; i++) dilate3x3(mask, width, height, x0, y0, x1, y1);
  for (let i = 0; i < passes; i++) erode3x3(mask, width, height, x0, y0, x1, y1);
}

function connectedComponents(mask, width, height, x0, y0, x1, y1) {
  const labels = new Int32Array(width * height);
  let nextLabel = 1;
  const stats = [null];

  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      const idx = y * width + x;
      if (!mask[idx] || labels[idx]) continue;
      const label = nextLabel++;
      const stack = [idx];
      labels[idx] = label;
      stats[label] = {
        area: 0, sumX: 0, sumY: 0, minX: x, minY: y, maxX: x, maxY: y, perimeter: 0,
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
        for (const [dx, dy] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
          const nx = cx + dx;
          const ny = cy + dy;
          if (nx < x0 || nx >= x1 || ny < y0 || ny >= y1 || !mask[ny * width + nx]) {
            edge = true;
            break;
          }
        }
        if (edge) s.perimeter += 1;
        if (cx > x0 && !labels[cur - 1] && mask[cur - 1]) { labels[cur - 1] = label; stack.push(cur - 1); }
        if (cx + 1 < x1 && !labels[cur + 1] && mask[cur + 1]) { labels[cur + 1] = label; stack.push(cur + 1); }
        if (cy > y0 && !labels[cur - width] && mask[cur - width]) { labels[cur - width] = label; stack.push(cur - width); }
        if (cy + 1 < y1 && !labels[cur + width] && mask[cur + width]) { labels[cur + width] = label; stack.push(cur + width); }
      }
    }
  }

  return stats.filter(Boolean).map((s) => ({
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
