/**
 * Fit source rect inside dest while preserving aspect ratio (letterbox/pillarbox).
 * @returns {{ x: number, y: number, w: number, h: number, scale: number }}
 */
export function letterboxRect(srcW, srcH, destW, destH) {
  if (srcW <= 0 || srcH <= 0 || destW <= 0 || destH <= 0) {
    return { x: 0, y: 0, w: destW, h: destH, scale: 1 };
  }
  const scale = Math.min(destW / srcW, destH / srcH);
  const w = srcW * scale;
  const h = srcH * scale;
  return {
    x: (destW - w) / 2,
    y: (destH - h) / 2,
    w,
    h,
    scale,
  };
}

/**
 * Draw video/canvas 1:1 into a buffer sized to the source (no stretch).
 * @param {CanvasRenderingContext2D} ctx
 * @param {CanvasImageSource} src
 */
export function drawImageNative(ctx, src) {
  const sw = "videoWidth" in src ? src.videoWidth : src.width;
  const sh = "videoHeight" in src ? src.videoHeight : src.height;
  if (ctx.canvas.width !== sw || ctx.canvas.height !== sh) {
    ctx.canvas.width = sw;
    ctx.canvas.height = sh;
  }
  ctx.drawImage(src, 0, 0);
  return { width: sw, height: sh };
}

/**
 * @param {HTMLElement} el
 * @param {number} w
 * @param {number} h
 */
export function setElementAspectRatio(el, w, h) {
  if (w > 0 && h > 0) {
    el.style.aspectRatio = `${w} / ${h}`;
  }
}

/** Luminance grayscale — R=G=B so HSV element + Hough still work; plates should use color frame. */
export function toGrayscaleImageData(source) {
  const { width, height, data } = source;
  const out = new ImageData(width, height);
  for (let i = 0; i < width * height; i++) {
    const si = i * 4;
    const g = Math.round(0.299 * data[si] + 0.587 * data[si + 1] + 0.114 * data[si + 2]);
    const o = si;
    out.data[o] = g;
    out.data[o + 1] = g;
    out.data[o + 2] = g;
    out.data[o + 3] = 255;
  }
  return out;
}
