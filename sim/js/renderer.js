import { scaleRect, processToCapture } from "./config.js";
import { letterboxRect } from "./canvas-util.js";
import { drawCalibrationDiagram, drawSamplePixelRays } from "./calibration-viz.js";

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {HTMLCanvasElement} sourceCanvas
 * @param {import('./detection.js').Detection[]} detections
 * @param {{ x: number, y: number, w: number, h: number }} roiProcess
 * @param {import('./config.js').VidarTuning} tuning
 * @param {{ overlay: string, viewMode: string, showProcessPip: boolean, showMask: boolean, showCrop: boolean, showCalibration?: boolean, grayscaleProcess?: boolean, processCanvas: HTMLCanvasElement | null, maskCanvas: HTMLCanvasElement | null, tagRegion?: { x: number, y: number, w: number, h: number } | null }} opts
 */
export function renderFrame(ctx, sourceCanvas, detections, roiProcess, tuning, opts) {
  const displayW = ctx.canvas.width;
  const displayH = ctx.canvas.height;
  const detectionView = opts.viewMode === "detection";
  const crop = tuning.crop ?? { x: 0, y: 0, w: tuning.captureWidth, h: tuning.captureHeight };
  const procW = tuning.processWidth ?? displayW;
  const procH = tuning.processHeight ?? displayH;
  const capW = sourceCanvas.width;
  const capH = sourceCanvas.height;

  /** @type {{ x: number, y: number, w: number, h: number, scale: number }} */
  let view;
  if (detectionView && opts.processCanvas) {
    view = letterboxRect(procW, procH, displayW, displayH);
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, displayW, displayH);
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(opts.processCanvas, view.x, view.y, view.w, view.h);
    ctx.restore();
  } else {
    view = letterboxRect(capW, capH, displayW, displayH);
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, displayW, displayH);
    ctx.save();
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(sourceCanvas, view.x, view.y, view.w, view.h);
    ctx.restore();
  }

  const sxCap = view.scale;
  const syCap = view.scale;
  const oxCap = view.x;
  const oyCap = view.y;
  const sxProc = view.scale;
  const syProc = view.scale;
  const oxProc = view.x;
  const oyProc = view.y;

  if (!detectionView && opts.showCrop !== false) {
    ctx.strokeStyle = "#7dffcf";
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 6]);
    ctx.strokeRect(
      oxCap + crop.x * sxCap,
      oyCap + crop.y * syCap,
      crop.w * sxCap,
      crop.h * syCap,
    );
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(0,0,0,0.35)";
    const top = oyCap + crop.y * syCap;
    if (top > 0) ctx.fillRect(0, 0, displayW, top);
    const bottom = oyCap + (crop.y + crop.h) * syCap;
    if (bottom < displayH) ctx.fillRect(0, bottom, displayW, displayH - bottom);
    const left = oxCap;
    if (left > 0) ctx.fillRect(0, 0, left, displayH);
    const right = oxCap + capW * sxCap;
    if (right < displayW) ctx.fillRect(right, 0, displayW - right, displayH);
  }

  if (opts.showMask && opts.maskCanvas) {
    ctx.save();
    ctx.globalAlpha = 0.45;
    ctx.imageSmoothingEnabled = false;
    if (detectionView) {
      ctx.drawImage(opts.maskCanvas, oxProc, oyProc, procW * sxProc, procH * syProc);
    } else {
      ctx.drawImage(
        opts.maskCanvas,
        0, 0, procW, procH,
        oxCap + crop.x * sxCap, oyCap + crop.y * syCap, crop.w * sxCap, crop.h * syCap,
      );
    }
    ctx.restore();
  }

  if (detectionView) {
    ctx.fillStyle = "rgba(0,0,0,0.55)";
    ctx.fillRect(displayW - 188, 8, 180, 22);
    ctx.fillStyle = "#8fd3ff";
    ctx.font = "600 12px Segoe UI, sans-serif";
    const grayTag = opts.grayscaleProcess ? " GRAY" : "";
    ctx.fillText(`DETECT ${procW}×${procH}${grayTag}`, displayW - 182, 23);
  }

  const roiDisplay = detectionView
    ? {
        x: oxProc + roiProcess.x * sxProc,
        y: oyProc + roiProcess.y * syProc,
        w: roiProcess.w * sxProc,
        h: roiProcess.h * syProc,
      }
    : {
        x: oxCap + (roiProcess.x / procW) * crop.w * sxCap,
        y: oyCap + (roiProcess.y / procH) * crop.h * syCap,
        w: (roiProcess.w / procW) * crop.w * sxCap,
        h: (roiProcess.h / procH) * crop.h * syCap,
      };

  if (opts.overlay !== "none") {
    const cross = detectionView
      ? { x: oxProc + (procW / 2) * sxProc, y: oyProc + (procH / 2) * syProc }
      : (() => {
          const c = processToCapture(procW / 2, procH / 2, crop, procW, procH);
          return { x: oxCap + c.x * sxCap, y: oyCap + c.y * syCap };
        })();
    drawCrosshair(ctx, cross.x, cross.y);
  }

  if ((opts.overlay === "all" || opts.overlay === "roi") && tuning.useCenterRoi) {
    ctx.strokeStyle = "#ffd166";
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    ctx.strokeRect(roiDisplay.x, roiDisplay.y, roiDisplay.w, roiDisplay.h);
    ctx.setLineDash([]);
  }

  if (opts.overlay === "all" || opts.overlay === "roi") {
    const avoidScale = sxProc;
    const center = detectionView
      ? { x: oxProc + (procW / 2) * sxProc, y: oyProc + (procH / 2) * syProc }
      : (() => {
          const c = processToCapture(procW / 2, procH / 2, crop, procW, procH);
          return { x: oxCap + c.x * sxCap, y: oyCap + c.y * syCap };
        })();
    ctx.strokeStyle = "rgba(255, 209, 102, 0.35)";
    ctx.beginPath();
    ctx.arc(center.x, center.y, tuning.avoidCenterRadius * avoidScale, 0, Math.PI * 2);
    ctx.stroke();
  }

  if (opts.overlay === "all" || opts.overlay === "roi") {
    drawHorizon(ctx, tuning, detectionView, oxProc, oyProc, sxProc, procH, oxCap, oyCap, sxCap, crop, procW);
    drawTagRegion(ctx, opts.tagRegion, detectionView, oxProc, oyProc, sxProc, syProc, oxCap, oyCap, sxCap, syCap, crop, procW, procH);
  }

  if (opts.overlay === "all" || opts.overlay === "boxes") {
    for (const det of detections) {
      let cx;
      let cy;
      let bx;
      let by;
      let bw;
      let bh;
      let radius;

      if (detectionView) {
        cx = oxProc + det.cx * sxProc;
        cy = oyProc + det.cy * syProc;
        bx = oxProc + det.x * sxProc;
        by = oyProc + det.y * syProc;
        bw = det.w * sxProc;
        bh = det.h * syProc;
        radius = det.radius ? det.radius * sxProc : 0;
      } else {
        const c = processToCapture(det.cx, det.cy, crop, procW, procH);
        cx = oxCap + c.x * sxCap;
        cy = oyCap + c.y * syCap;
        const tl = processToCapture(det.x, det.y, crop, procW, procH);
        const br = processToCapture(det.x + det.w, det.y + det.h, crop, procW, procH);
        bx = oxCap + tl.x * sxCap;
        by = oyCap + tl.y * syCap;
        bw = (br.x - tl.x) * sxCap;
        bh = (br.y - tl.y) * syCap;
        radius = det.radius ? det.radius * (crop.w / procW) * sxCap : 0;
      }

      ctx.save();
      const temporal = /** @type {'valid'|'potential'|'coasting'|undefined} */ (det.temporal);
      if (temporal === "potential") {
        ctx.globalAlpha = 0.5;
        ctx.setLineDash([6, 5]);
      } else if (temporal === "coasting") {
        ctx.globalAlpha = 0.72;
        ctx.setLineDash([4, 4]);
      }

      ctx.strokeStyle = det.color;
      ctx.lineWidth = detectionView ? 2 : 3;

      if (det.shape === "circle" && det.radius) {
        ctx.beginPath();
        ctx.arc(cx, cy, radius, 0, Math.PI * 2);
        ctx.stroke();
      } else {
        ctx.strokeRect(bx, by, bw, bh);
      }

      ctx.setLineDash([]);
      ctx.fillStyle = det.color;
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fill();

      ctx.font = "600 13px Segoe UI, sans-serif";
      ctx.fillStyle = det.color;
      const extra = det.circularity != null ? ` circ=${det.circularity.toFixed(2)}` : "";
      const tag =
        temporal === "potential" ? " ?" :
        temporal === "coasting" ? " hold" : "";
      const rangeTag = det.range != null ? ` ${det.range.toFixed(0)}in` : "";
      const confTag = det.confidence != null ? ` ${(det.confidence * 100).toFixed(0)}%` : "";
      const idTag = det.elementId ? ` id=${det.elementId}` : "";
      const rankTag = det.occurrenceRank != null && det.occurrenceRank >= 0
        ? `#${det.occurrenceRank}`
        : "";
      const trackTag = det.trackId != null && det.trackId >= 0 ? ` T${det.trackId}` : "";
      ctx.fillText(
        `${det.label}${idTag}${rankTag}${trackTag}${tag} (${Math.round(det.area)}px${extra})${rangeTag}${confTag}`,
        bx + 4,
        by - 6,
      );
      ctx.restore();
    }
  }

  if (opts.spatialTracks?.length && (opts.overlay === "all" || opts.overlay === "boxes")) {
    drawSpatialTracks(
      ctx,
      opts.spatialTracks,
      detectionView,
      oxProc,
      oyProc,
      sxProc,
      syProc,
      oxCap,
      oyCap,
      sxCap,
      syCap,
      crop,
      procW,
      procH,
    );
  }

  if (!detectionView && opts.showProcessPip && opts.processCanvas) {
    const pw = 160;
    const ph = Math.max(40, Math.round(pw * (procH / procW)));
    const pad = 10;
    ctx.fillStyle = "rgba(0,0,0,0.65)";
    ctx.fillRect(pad, pad, pw + 8, ph + 24);
    ctx.drawImage(opts.processCanvas, pad + 4, pad + 20, pw, ph);
    ctx.fillStyle = "#8fa3bf";
    ctx.font = "11px sans-serif";
    ctx.fillText(`Process ${procW}×${procH}${opts.grayscaleProcess ? " gray" : ""}`, pad + 4, pad + 14);
  }

  if (opts.showCalibration) {
    drawCalibrationDiagram(ctx, displayW - 150, 8, 40, tuning);
    if (!detectionView) {
      drawSamplePixelRays(ctx, oxCap, oyCap, sxCap, syCap, tuning);
    }
  }
}

function drawCrosshair(ctx, cx, cy) {
  ctx.strokeStyle = "rgba(255,255,255,0.45)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx - 14, cy);
  ctx.lineTo(cx + 14, cy);
  ctx.moveTo(cx, cy - 14);
  ctx.lineTo(cx, cy + 14);
  ctx.stroke();
}

function drawHorizon(ctx, tuning, detectionView, oxProc, oyProc, sxProc, procH, oxCap, oyCap, sxCap, crop, procW) {
  const geometry = tuning.geometry;
  const idx = geometry?.activeCameraIndex ?? 0;
  const profile = geometry?.cameras?.[idx];
  if (!profile) return;

  const yProc = profile.horizonRowPx;
  ctx.save();
  ctx.strokeStyle = "rgba(140, 200, 255, 0.55)";
  ctx.setLineDash([6, 8]);
  ctx.lineWidth = 1;

  if (detectionView) {
    const y = oyProc + yProc * sxProc;
    ctx.beginPath();
    ctx.moveTo(oxProc, y);
    ctx.lineTo(oxProc + (tuning.processWidth ?? procW) * sxProc, y);
    ctx.stroke();
  } else {
    const c = processToCapture(0, yProc, crop, procW, procH);
    const c2 = processToCapture(procW, yProc, crop, procW, procH);
    ctx.beginPath();
    ctx.moveTo(oxCap + c.x * sxCap, oyCap + c.y * sxCap);
    ctx.lineTo(oxCap + c2.x * sxCap, oyCap + c2.y * sxCap);
    ctx.stroke();
  }
  ctx.restore();
}

function drawTagRegion(ctx, region, detectionView, oxProc, oyProc, sxProc, syProc, oxCap, oyCap, sxCap, syCap, crop, procW, procH) {
  if (!region) return;
  ctx.save();
  ctx.strokeStyle = "rgba(180, 140, 255, 0.75)";
  ctx.setLineDash([5, 5]);
  ctx.lineWidth = 2;
  if (detectionView) {
    ctx.strokeRect(
      oxProc + region.x * sxProc,
      oyProc + region.y * syProc,
      region.w * sxProc,
      region.h * syProc,
    );
  } else {
    const tl = processToCapture(region.x, region.y, crop, procW, procH);
    const br = processToCapture(region.x + region.w, region.y + region.h, crop, procW, procH);
    ctx.strokeRect(
      oxCap + tl.x * sxCap,
      oyCap + tl.y * syCap,
      (br.x - tl.x) * sxCap,
      (br.y - tl.y) * syCap,
    );
  }
  ctx.restore();
}

/** @param {import('./detection.js').ReturnType<typeof import('./detection.js').bestByCategory>} best @param {import('./config.js').VidarTuning} tuning */
export function describeLogic(best, tuning) {
  const frameW = tuning.processWidth ?? 1;
  const cx = frameW / 2;
  const geometry = tuning.geometry;
  const pickupStop = geometry?.pickupStop ?? geometry?.pickupStop ?? 14;
  const minConf = geometry?.minElementConfidence ?? 0.35;

  if (best.robot) {
    const dx = best.robot.cx - cx;
    const dy = best.robot.cy - (tuning.processHeight ?? 1) / 2;
    const dist = Math.hypot(dx, dy);
    if (dist < tuning.avoidCenterRadius) {
      return {
        state: "BLOCKED — robot in avoid zone",
        elementError: null,
        robotDist: dist,
        elementRange: best.element?.range ?? null,
        dSize: best.element?.dSize ?? null,
        dFloor: best.element?.dFloor ?? null,
        dGround: best.element?.dGround ?? null,
        primaryRangeSource: best.element?.primaryRangeSource ?? null,
        elementConfidence: best.element?.confidence ?? null,
        robotXY: formatRobotXY(best.element),
      };
    }
  }

  if (best.element) {
    const err = best.element.cx - cx;
    const range = best.element.range;
    const conf = best.element.confidence ?? 0;
    const robotDist = best.robot
      ? Math.hypot(best.robot.cx - cx, best.robot.cy - (tuning.processHeight ?? 1) / 2)
      : null;

    if (conf < minConf) {
      return {
        state: "LOW CONF — size/floor mismatch",
        elementError: err,
        robotDist,
        elementRange: range ?? null,
        dSize: best.element.dSize ?? null,
        dFloor: best.element.dFloor ?? null,
        dGround: best.element.dGround ?? null,
        primaryRangeSource: best.element.primaryRangeSource ?? null,
        elementConfidence: conf,
        robotXY: formatRobotXY(best.element),
      };
    }

    if (range != null && range <= pickupStop) {
      return {
        state: "AT PICKUP RANGE — stop",
        elementError: err,
        robotDist,
        elementRange: range,
        dSize: best.element.dSize ?? null,
        dFloor: best.element.dFloor ?? null,
        dGround: best.element.dGround ?? null,
        primaryRangeSource: best.element.primaryRangeSource ?? null,
        elementConfidence: conf,
        robotXY: formatRobotXY(best.element),
      };
    }

    if (Math.abs(err) < 25) {
      return {
        state: range != null ? `DRIVE — ${range.toFixed(0)} in` : "DRIVE — element centered",
        elementError: err,
        robotDist,
        elementRange: range ?? null,
        dSize: best.element.dSize ?? null,
        dFloor: best.element.dFloor ?? null,
        dGround: best.element.dGround ?? null,
        primaryRangeSource: best.element.primaryRangeSource ?? null,
        elementConfidence: conf,
        robotXY: formatRobotXY(best.element),
      };
    }

    return {
      state: "TURN — seeking element",
      elementError: err,
      robotDist,
      elementRange: range ?? null,
      dSize: best.element.dSize ?? null,
      dFloor: best.element.dFloor ?? null,
      dGround: best.element.dGround ?? null,
      primaryRangeSource: best.element.primaryRangeSource ?? null,
      elementConfidence: conf,
      robotXY: formatRobotXY(best.element),
    };
  }

  return {
    state: "SEARCH — slow spin",
    elementError: null,
    robotDist: best.robot
      ? Math.hypot(best.robot.cx - cx, best.robot.cy - (tuning.processHeight ?? 1) / 2)
      : null,
    elementRange: null,
    dSize: null,
    dFloor: null,
    dGround: null,
    primaryRangeSource: null,
    elementConfidence: null,
    robotXY: null,
  };
}

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {import('./spatial-tracks.js').SpatialTrack[]} tracks
 */
function drawSpatialTracks(
  ctx,
  tracks,
  detectionView,
  oxProc,
  oyProc,
  sxProc,
  syProc,
  oxCap,
  oyCap,
  sxCap,
  syCap,
  crop,
  procW,
  procH,
) {
  for (const track of tracks) {
    if (track.source !== "REMEMBERED") continue;

    let cx;
    let cy;
    if (detectionView) {
      cx = oxProc + track.cx * sxProc;
      cy = oyProc + track.cy * syProc;
    } else {
      const c = processToCapture(track.cx, track.cy, crop, procW, procH);
      cx = oxCap + c.x * sxCap;
      cy = oyCap + c.y * syCap;
    }

    ctx.save();
    ctx.strokeStyle = track.kind === "FOE" ? "#ff6b6b" : "#c8ff80";
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, 14, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle;
    ctx.font = "600 11px Segoe UI, sans-serif";
    const id = track.elementId ? ` ${track.elementId}` : "";
    ctx.fillText(`T${track.trackId}${id}`, cx + 10, cy - 8);
    ctx.restore();
  }
}

/** @param {import('./detection.js').Detection | null | undefined} element */
function formatRobotXY(element) {
  if (!element || element.robotX == null || element.robotY == null) return null;
  return { x: element.robotX, y: element.robotY };
}
