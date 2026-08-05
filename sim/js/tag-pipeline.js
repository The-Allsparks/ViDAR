/**
 * Top-half tag scout + gated decode (mirrors Java VidarTagScout / VidarAdaptiveTagProcessor).
 */

/** @typedef {'LEFT'|'MIDDLE'|'RIGHT'} HorizontalBand */

/** @typedef {{ cx: number, cy: number, widthPx: number, band: HorizontalBand }} TagScoutResult */

/** @typedef {{ tagId: number, captureTimeMs: number, centerX: number, centerY: number, band: HorizontalBand, decimation: number, decodePixels: number, fieldX: number, fieldY: number, fieldHeadingDeg: number, odomX: number, odomY: number, odomHeadingDeg: number }} TagObservation */

/** @param {import('./config.js').TagConfig} cfg */
export function bandForCx(cx, frameCols, cfg) {
  const norm = cx / Math.max(1, frameCols);
  if (norm < (cfg.bandLeftMax ?? 0.33)) return "LEFT";
  if (norm > (cfg.bandRightMin ?? 0.67)) return "RIGHT";
  return "MIDDLE";
}

/** @param {number} frameCols @param {number} frameRows @param {HorizontalBand} band */
export function tagDecodeCrop(frameCols, frameRows, band) {
  const topH = Math.max(1, Math.floor(frameRows / 2));
  const cropW = Math.max(1, Math.floor(frameCols / 2));
  let x = 0;
  if (band === "RIGHT") x = frameCols - cropW;
  else if (band === "MIDDLE") x = Math.floor((frameCols - cropW) / 2);
  return { x, y: 0, w: cropW, h: topH };
}

/** @param {number} cx @param {number} cy @param {number} frameCols @param {number} frameRows @param {HorizontalBand} band */
export function isInTagDecodeRegion(cx, cy, frameCols, frameRows, band) {
  if (cy >= frameRows / 2) return false;
  const r = tagDecodeCrop(frameCols, frameRows, band);
  return cx >= r.x && cx < r.x + r.w && cy >= r.y && cy < r.y + r.h;
}

/**
 * @param {ImageData} fullFrame process-resolution color image
 * @param {import('./config.js').TagConfig} cfg
 */
export function runTagScout(fullFrame, cfg) {
  const { width, height, data } = fullFrame;
  const topH = Math.floor(height / 2);
  const scoutW = cfg.scoutWidth ?? 320;
  const scoutH = Math.max(8, Math.round(topH * scoutW / width));

  let best = null;
  let bestScore = -1;

  for (let sy = 2; sy < scoutH - 2; sy += 2) {
    for (let sx = 2; sx < scoutW - 2; sx += 2) {
      const fx = Math.floor(sx * width / scoutW);
      const fy = Math.floor(sy * topH / scoutH);
      const i = (fy * width + fx) * 4;
      const r = data[i];
      const g = data[i + 1];
      const b = data[i + 2];
      const bright = Math.max(r, g, b);
      const dark = Math.min(r, g, b);
      if (bright < 160 || bright - dark < 40) continue;

      const wEst = estimateBlobWidth(data, width, fx, fy);
      if (wEst < (cfg.scoutMinWidthPx ?? 8)) continue;

      const cx = fx;
      const cy = fy;
      const band = bandForCx(cx, width, cfg);
      const score = wEst * wEst;
      if (score > bestScore) {
        bestScore = score;
        best = { cx, cy, widthPx: wEst, band };
      }
    }
  }
  return best;
}

function estimateBlobWidth(data, width, cx, cy) {
  let left = cx;
  let right = cx;
  const row = cy * width;
  const i0 = (row + cx) * 4;
  const ref = Math.max(data[i0], data[i0 + 1], data[i0 + 2]);

  while (left > 0) {
    const i = (row + left - 1) * 4;
    const m = Math.max(data[i], data[i + 1], data[i + 2]);
    if (Math.abs(m - ref) > 50) break;
    left -= 1;
  }
  while (right < width - 1) {
    const i = (row + right + 1) * 4;
    const m = Math.max(data[i], data[i + 1], data[i + 2]);
    if (Math.abs(m - ref) > 50) break;
    right += 1;
  }
  return Math.max(1, right - left);
}

/** @param {number} scoutWidth @param {number} scoutFrameWidth @param {number} fullWidth @param {import('./config.js').TagConfig} cfg */
export function chooseDecimation(scoutWidth, scoutFrameWidth, fullWidth, cfg) {
  const scale = fullWidth / Math.max(1, scoutFrameWidth);
  const fullW = scoutWidth * scale;
  if (fullW >= 80) return cfg.decimationMax ?? 3;
  return cfg.decimationMin ?? 2;
}

export function worthDecode(scoutWidth, scoutFrameWidth, fullWidth, cfg) {
  const scale = fullWidth / Math.max(1, scoutFrameWidth);
  return scoutWidth * scale >= (cfg.decodeMinTagWidthPx ?? 28);
}

/**
 * Decode only inside the cropped ROI (mirrors Java VidarTagCropDecoder).
 * @param {ImageData} fullFrame
 * @param {{ x: number, y: number, w: number, h: number }} region
 * @param {import('./config.js').TagConfig} cfg
 * @param {number} decimation
 */
export function decodeTagOnCrop(fullFrame, region, cfg, decimation) {
  const { width, height, data } = fullFrame;
  let workW = region.w;
  let workH = region.h;
  if (decimation > 1) {
    workW = Math.max(32, Math.round(region.w / decimation));
    workH = Math.max(24, Math.round(region.h / decimation));
  }

  let best = null;
  let bestScore = -1;

  for (let wy = 2; wy < workH - 2; wy += 2) {
    for (let wx = 2; wx < workW - 2; wx += 2) {
      const fx = region.x + Math.floor(wx * region.w / workW);
      const fy = region.y + Math.floor(wy * region.h / workH);
      if (fy >= height / 2) continue;

      const i = (fy * width + fx) * 4;
      const bright = Math.max(data[i], data[i + 1], data[i + 2]);
      const dark = Math.min(data[i], data[i + 1], data[i + 2]);
      if (bright < 170 || bright - dark < 50) continue;

      const wEst = estimateBlobWidth(data, width, fx, fy);
      if (wEst < (cfg.decodeMinTagWidthPx ?? 28) * 0.6) continue;

      const score = wEst * wEst;
      if (score > bestScore) {
        bestScore = score;
        best = { cx: fx, cy: fy, widthPx: wEst };
      }
    }
  }

  if (!best) return null;

  const rangeIn = (cfg.tagSizeIn ?? 4) * (width * 0.55) / Math.max(1, best.widthPx);
  return {
    tagId: cfg.desiredTagId >= 0 ? cfg.desiredTagId : 11,
    centerX: best.cx,
    centerY: best.cy,
    widthPx: best.widthPx,
    rangeIn,
    decodePixels: workW * workH,
  };
}

/** @typedef {{ lastDecodeMs: number, latest: TagObservation | null, lastScout: TagScoutResult | null, lastRegion: { x: number, y: number, w: number, h: number } | null, driverRequested: boolean, odom: { x: number, y: number, h: number } }} TagPipelineState */

/** @returns {TagPipelineState} */
export function createTagState() {
  return {
    lastDecodeMs: 0,
    latest: null,
    lastScout: null,
    lastRegion: null,
    driverRequested: false,
    odom: { x: 0, y: 0, h: 0 },
  };
}

/**
 * @param {TagPipelineState} state
 * @param {ImageData} processFrame
 * @param {import('./config.js').TagConfig} cfg
 * @param {number} nowMs
 * @param {{ expectedBearingDeg?: number, cameraBearingDeg?: number }} gate
 */
export function updateTagPipeline(state, processFrame, cfg, nowMs, gate) {
  if (!cfg?.enabled) return state;

  const scout = runTagScout(processFrame, cfg);
  state.lastScout = scout;

  const interval = cfg.minSampleIntervalMs ?? 2000;
  const due = nowMs - state.lastDecodeMs >= interval;
  const driver = state.driverRequested;
  state.driverRequested = false;

  let allowed = driver;
  if (!allowed && scout) {
    if (gate.expectedBearingDeg == null || cfg.poseGateDeg <= 0) {
      allowed = true;
    } else {
      const centerErr = scout.cx - processFrame.width / 2;
      const bearingErr = (centerErr / processFrame.width) * 70;
      const robotToTag = normalizeDeg((gate.expectedBearingDeg ?? 0) - (gate.cameraBearingDeg ?? 0));
      allowed = Math.abs(normalizeDeg(robotToTag - bearingErr)) <= (cfg.poseGateDeg ?? 30);
    }
  }

  if (!scout || !due || !allowed) return state;
  if (!worthDecode(scout.widthPx, cfg.scoutWidth ?? 320, processFrame.width, cfg)) return state;

  const dec = chooseDecimation(
    scout.widthPx, cfg.scoutWidth ?? 320, processFrame.width, cfg,
  );
  const region = tagDecodeCrop(processFrame.width, processFrame.height, scout.band);
  state.lastRegion = region;
  state.lastDecodeMs = nowMs;

  const decoded = decodeTagOnCrop(processFrame, region, cfg, dec);
  if (!decoded) return state;

  const bearingRad = Math.atan2(
    decoded.centerX - processFrame.width / 2,
    processFrame.width * 0.7,
  );
  const camBearing = (gate.cameraBearingDeg ?? 0) * Math.PI / 180;
  const fieldHeading = camBearing + bearingRad;

  state.latest = {
    tagId: decoded.tagId,
    captureTimeMs: nowMs,
    centerX: decoded.centerX,
    centerY: decoded.centerY,
    band: scout.band,
    decimation: dec,
    decodePixels: decoded.decodePixels,
    fieldX: Math.cos(fieldHeading) * decoded.rangeIn,
    fieldY: Math.sin(fieldHeading) * decoded.rangeIn,
    fieldHeadingDeg: (fieldHeading * 180) / Math.PI,
    odomX: state.odom.x,
    odomY: state.odom.y,
    odomHeadingDeg: state.odom.h,
  };
  return state;
}

/** @param {TagObservation | null} tag @param {{ x: number, y: number, h: number }} odomNow */
export function backdateFieldPose(tag, odomNow) {
  if (!tag) return null;
  return {
    x: tag.fieldX + (odomNow.x - tag.odomX),
    y: tag.fieldY + (odomNow.y - tag.odomY),
    h: tag.fieldHeadingDeg + (odomNow.h - tag.odomHeadingDeg),
  };
}

function normalizeDeg(d) {
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}
