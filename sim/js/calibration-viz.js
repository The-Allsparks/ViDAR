/**
 * Calibration axis visualization — robot frame, camera optical frame, sample rays.
 * X red, Y green, Z blue (conventional RGB axes).
 *
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} x origin x on display canvas
 * @param {number} y origin y
 * @param {number} scale pixels per inch-ish unit for diagram
 * @param {import('./config.js').VidarTuning} tuning
 */
export function drawCalibrationDiagram(ctx, x, y, scale, tuning) {
  const geom = tuning.geometry;
  if (!geom?.cameras?.length) return;
  const idx = geom.activeCameraIndex ?? 0;
  const cam = geom.cameras[idx] ?? geom.cameras[0];
  const w = 140;
  const h = 140;
  ctx.save();
  ctx.fillStyle = "rgba(0,0,0,0.55)";
  ctx.fillRect(x, y, w, h);
  ctx.strokeStyle = "#888";
  ctx.strokeRect(x, y, w, h);
  ctx.font = "10px system-ui,sans-serif";
  ctx.fillStyle = "#ccc";
  ctx.fillText(`cal: ${cam.name} ${cam.bearingDeg}°`, x + 4, y + 12);

  const ox = x + w / 2;
  const oy = y + h - 18;
  drawAxisTriad(ctx, ox, oy, scale * 0.35, 0, "robot");

  const camOx = ox + scale * 0.15;
  const camOy = oy - scale * 0.25;
  const bearingRad = (cam.bearingDeg * Math.PI) / 180;
  drawAxisTriad(ctx, camOx, camOy, scale * 0.22, bearingRad, "cam");

  ctx.strokeStyle = "#ffcc00";
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.moveTo(camOx, camOy);
  ctx.lineTo(camOx + Math.cos(bearingRad) * scale * 0.5, camOy - Math.sin(bearingRad) * scale * 0.5);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#ffcc00";
  ctx.fillText("fwd", camOx + 4, camOy - scale * 0.55);

  if (Number.isFinite(cam.principalPointX) && Number.isFinite(cam.principalPointY)) {
    ctx.fillStyle = "#fff";
    ctx.fillText(`cx,cy ${cam.principalPointX ?? 320},${cam.principalPointY ?? 240}`, x + 4, y + h - 4);
  }

  ctx.restore();
}

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} ox
 * @param {number} oy
 * @param {number} len
 * @param {number} yawRad
 * @param {string} label
 */
function drawAxisTriad(ctx, ox, oy, len, yawRad, label) {
  const cos = Math.cos(yawRad);
  const sin = Math.sin(yawRad);
  /** @type {[string, number, number, string][]} */
  const axes = [
    ["X", cos, -sin, "#e74c3c"],
    ["Y", sin, cos, "#2ecc71"],
    ["Z", 0, 0, "#3498db"],
  ];
  for (const [name, dx, dy, color] of axes) {
    if (name === "Z") continue;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ox + dx * len, oy + dy * len);
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.fillText(name, ox + dx * len + 2, oy + dy * len + 3);
  }
  ctx.fillStyle = "#aaa";
  ctx.font = "9px system-ui";
  ctx.fillText(label, ox - 10, oy + 10);
}

/**
 * Draw representative pixel rays on the process view (full-frame coords).
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} oxCap
 * @param {number} oyCap
 * @param {number} sx
 * @param {number} sy
 * @param {import('./config.js').VidarTuning} tuning
 */
export function drawSamplePixelRays(ctx, oxCap, oyCap, sx, sy, tuning) {
  const geom = tuning.geometry;
  if (!geom?.cameras?.length) return;
  const idx = geom.activeCameraIndex ?? 0;
  const cam = geom.cameras[idx] ?? geom.cameras[0];
  const cx = cam.principalPointX ?? 320;
  const cy = cam.principalPointY ?? 240;
  const samples = [
    [cx, cy],
    [cx - 80, cy],
    [cx + 80, cy],
    [cx, cy + 60],
  ];
  ctx.save();
  ctx.strokeStyle = "rgba(52, 152, 219, 0.85)";
  ctx.lineWidth = 1;
  for (const [px, py] of samples) {
    const dx = oxCap + px * sx;
    const dy = oyCap + py * sy;
    ctx.beginPath();
    ctx.arc(dx, dy, 3, 0, Math.PI * 2);
    ctx.fillStyle = "#3498db";
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(dx, dy);
    ctx.lineTo(dx + (px - cx) * 0.15, dy + (py - cy) * 0.15);
    ctx.stroke();
  }
  ctx.restore();
}
