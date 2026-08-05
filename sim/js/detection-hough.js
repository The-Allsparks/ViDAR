/**
 * Hough circle detection for holed / perforated balls where HSV blob masks fragment.
 * @param {ImageData} imageData
 * @param {import('./config.js').ColorTarget} target
 * @param {{ x: number, y: number, w: number, h: number }} roi
 * @returns {import('./detection.js').Detection[]}
 */
export function detectHoughBalls(imageData, target, roi) {
  const { width, height, data } = imageData;
  const x0 = Math.max(0, roi.x);
  const y0 = Math.max(0, roi.y);
  const x1 = Math.min(width, roi.x + roi.w);
  const y1 = Math.min(height, roi.y + roi.h);

  const gray = new Float32Array(width * height);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = (y * width + x) * 4;
      gray[y * width + x] = 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2];
    }
  }
  boxBlur3(gray, width, height);

  const { mag, gx, gy } = sobel(gray, width, height);
  const edgeThresh = target.houghEdgeThresh ?? 48;
  /** @type {{ x: number, y: number, gx: number, gy: number }[]} */
  const edges = [];
  for (let y = y0 + 1; y < y1 - 1; y++) {
    for (let x = x0 + 1; x < x1 - 1; x++) {
      const m = mag[y * width + x];
      if (m >= edgeThresh) edges.push({ x, y, gx: gx[y * width + x], gy: gy[y * width + x] });
    }
  }

  const minR = target.houghMinRadius ?? 8;
  const maxR = target.houghMaxRadius ?? Math.min(36, Math.round(Math.min(width, height) * 0.35));
  const minDist = target.houghMinDist ?? 28;
  const accThresh = target.houghAccumulator ?? 14;
  const minVotesScale = target.houghMinVotesScale ?? 0.55;
  const minInterior = target.houghMinInterior ?? 0.18;
  const maxResults = target.houghMaxResults ?? 3;

  const candidates = houghGradient(edges, width, height, minR, maxR, accThresh);
  candidates.sort((a, b) => b.votes - a.votes);

  /** @type {import('./detection.js').Detection[]} */
  const out = [];
  for (const c of candidates) {
    if (c.cy < y0 + minR || c.cy > y1 - minR) continue;
    if (c.cx < x0 + minR || c.cx > x1 - minR) continue;

    const minVotes = accThresh + c.r * minVotesScale;
    if (c.votes < minVotes) continue;

    const interior = circleInteriorScore(data, width, height, c.cx, c.cy, c.r, target);
    if (interior < minInterior) continue;

    let tooClose = false;
    for (const kept of out) {
      if (Math.hypot(c.cx - kept.cx, c.cy - kept.cy) < minDist) {
        tooClose = true;
        break;
      }
    }
    if (tooClose) continue;

    const area = Math.PI * c.r * c.r;
    const minArea = target.houghMinArea ?? target.minArea ?? 40;
    if (area < minArea) continue;

    out.push(toDetection(c, target, area, interior));
    if (out.length >= maxResults) break;
  }

  out.sort((a, b) => (b.votes ?? 0) - (a.votes ?? 0));
  return out;
}

/** @param {{ cx: number, cy: number, r: number, votes: number }} c @param {import('./config.js').ColorTarget} target @param {number} area @param {number} interior */
function toDetection(c, target, area, interior) {
  const d = Math.ceil(c.r * 2);
  return {
    category: "element",
    name: target.name,
    label: target.label,
    color: target.color,
    shape: "circle",
    cx: c.cx,
    cy: c.cy,
    area,
    x: c.cx - c.r,
    y: c.cy - c.r,
    w: d,
    h: d,
    radius: c.r,
    circularity: 1,
    aspectRatio: 1,
    votes: c.votes,
    interior,
    detector: "hough",
  };
}

/** Fraction of interior pixels that look white/off-white (tolerates holes). */
function circleInteriorScore(data, width, height, cx, cy, r, target) {
  const minBright = target.houghInteriorBright ?? 95;
  const maxSpread = target.houghInteriorSpread ?? 55;
  let inside = 0;
  let bright = 0;
  const r2 = r * r;
  const ri = Math.floor(r);
  const icx = Math.round(cx);
  const icy = Math.round(cy);

  for (let dy = -ri; dy <= ri; dy++) {
    for (let dx = -ri; dx <= ri; dx++) {
      if (dx * dx + dy * dy > r2) continue;
      const x = icx + dx;
      const y = icy + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      inside += 1;
      const o = (y * width + x) * 4;
      const rpx = data[o];
      const gpx = data[o + 1];
      const bpx = data[o + 2];
      const max = Math.max(rpx, gpx, bpx);
      const min = Math.min(rpx, gpx, bpx);
      if (max >= minBright && max - min <= maxSpread) bright += 1;
    }
  }
  return inside > 0 ? bright / inside : 0;
}

function boxBlur3(src, width, height) {
  const tmp = src.slice();
  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      let s = 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          s += tmp[(y + dy) * width + (x + dx)];
        }
      }
      src[y * width + x] = s / 9;
    }
  }
}

function sobel(gray, width, height) {
  const mag = new Float32Array(width * height);
  const gx = new Float32Array(width * height);
  const gy = new Float32Array(width * height);
  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      const idx = y * width + x;
      const gxp =
        -gray[idx - width - 1] + gray[idx - width + 1] +
        -2 * gray[idx - 1] + 2 * gray[idx + 1] +
        -gray[idx + width - 1] + gray[idx + width + 1];
      const gyp =
        -gray[idx - width - 1] - 2 * gray[idx - width] - gray[idx - width + 1] +
        gray[idx + width - 1] + 2 * gray[idx + width] + gray[idx + width + 1];
      gx[idx] = gxp;
      gy[idx] = gyp;
      mag[idx] = Math.hypot(gxp, gyp);
    }
  }
  return { mag, gx, gy };
}

/**
 * @param {{ x: number, y: number, gx: number, gy: number }[]} edges
 */
function houghGradient(edges, width, height, minR, maxR, accThresh) {
  const accW = width;
  const accH = height;
  const rSpan = maxR - minR + 1;
  const acc = new Int32Array(accW * accH * rSpan);

  for (const e of edges) {
    const angle = Math.atan2(e.gy, e.gx);
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    const cosA2 = Math.cos(angle + Math.PI);
    const sinA2 = Math.sin(angle + Math.PI);

    for (let r = minR; r <= maxR; r++) {
      const ri = r - minR;
      let cx = Math.round(e.x + r * cosA);
      let cy = Math.round(e.y + r * sinA);
      if (cx >= 0 && cx < accW && cy >= 0 && cy < accH) {
        acc[(cy * accW + cx) * rSpan + ri] += 1;
      }
      cx = Math.round(e.x + r * cosA2);
      cy = Math.round(e.y + r * sinA2);
      if (cx >= 0 && cx < accW && cy >= 0 && cy < accH) {
        acc[(cy * accW + cx) * rSpan + ri] += 1;
      }
    }
  }

  /** @type {{ cx: number, cy: number, r: number, votes: number }[]} */
  const peaks = [];
  for (let cy = 2; cy < accH - 2; cy++) {
    for (let cx = 2; cx < accW - 2; cx++) {
      for (let ri = 0; ri < rSpan; ri++) {
        const votes = acc[(cy * accW + cx) * rSpan + ri];
        if (votes < accThresh) continue;
        let isMax = true;
        for (let dy = -2; dy <= 2 && isMax; dy++) {
          for (let dx = -2; dx <= 2; dx++) {
            if (dx === 0 && dy === 0) continue;
            for (let dr = -1; dr <= 1; dr++) {
              const nr = ri + dr;
              if (nr < 0 || nr >= rSpan) continue;
              if (acc[((cy + dy) * accW + (cx + dx)) * rSpan + nr] > votes) {
                isMax = false;
                break;
              }
            }
          }
        }
        if (isMax) peaks.push({ cx, cy, r: minR + ri, votes });
      }
    }
  }
  return peaks;
}

/** Edge map preview for debug overlay. */
export function houghEdgePreview(imageData, target, roi) {
  const { width, height, data } = imageData;
  const gray = new Float32Array(width * height);
  for (let i = 0; i < width * height; i++) {
    const o = i * 4;
    gray[i] = 0.299 * data[o] + 0.587 * data[o + 1] + 0.114 * data[o + 2];
  }
  boxBlur3(gray, width, height);
  const { mag } = sobel(gray, width, height);
  const thresh = target.houghEdgeThresh ?? 48;
  const mask = new Uint8Array(width * height);
  const x0 = Math.max(0, roi.x);
  const y0 = Math.max(0, roi.y);
  const x1 = Math.min(width, roi.x + roi.w);
  const y1 = Math.min(height, roi.y + roi.h);
  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) {
      if (mag[y * width + x] >= thresh) mask[y * width + x] = 1;
    }
  }
  return mask;
}
