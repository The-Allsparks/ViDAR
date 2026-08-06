/**
 * Lightweight motion-track preview — mirrors Java predict/gate/associate at robot-frame level.
 */

/** @typedef {'ELEMENT'|'FOE'} TrackKind */
/** @typedef {'LIVE'|'REMEMBERED'} TrackSource */

/**
 * @typedef {Object} SpatialTrack
 * @property {number} trackId
 * @property {TrackKind} kind
 * @property {string} elementId
 * @property {number} robotX
 * @property {number} robotY
 * @property {number} cx
 * @property {number} cy
 * @property {number} range
 * @property {number} confidence
 * @property {TrackSource} source
 * @property {number} missCount
 * @property {number} lastUpdateMs
 */

const GATE_RADIUS_IN = 12;
const GATE_RADIUS_FOE_IN = 18;
const MAX_MISS = 8;
const MIN_CONF = 0.35;

/** @returns {{ tracks: SpatialTrack[], nextTrackId: number, lastOdom: { x: number, y: number, h: number } | null }} */
export function createSpatialTrackState() {
  return { tracks: [], nextTrackId: 1, lastOdom: null };
}

/**
 * @param {ReturnType<typeof createSpatialTrackState>} state
 * @param {import('./detection.js').Detection[]} detections
 * @param {number} nowMs
 * @param {{ x: number, y: number, h: number } | null | undefined} odom
 * @param {{ enabled?: boolean }} [opts]
 */
export function updateSpatialTracks(state, detections, nowMs, odom, opts = {}) {
  if (!opts.enabled) {
    state.tracks = [];
    return state;
  }

  const dets = detectionsToTracks(detections);
  const fieldDelta = fieldDeltaFromOdom(state.lastOdom, odom);
  state.lastOdom = odom ? { x: odom.x, y: odom.y, h: odom.h } : state.lastOdom;

  /** @type {SpatialTrack[]} */
  const predicted = state.tracks.map((t) => predictTrack(t, fieldDelta, nowMs));

  if (dets.length === 0) {
    state.tracks = predicted
      .map((t) => coastTrack(t, nowMs))
      .filter((t) => t.missCount <= MAX_MISS);
    return state;
  }

  /** @type {{ ti: number, di: number, dist: number }[]} */
  const candidates = [];
  for (let ti = 0; ti < predicted.length; ti++) {
    const track = predicted[ti];
    for (let di = 0; di < dets.length; di++) {
      const det = dets[di];
      if (track.kind !== det.kind) continue;
      if (track.elementId && det.elementId && track.elementId !== det.elementId) continue;
      const dist = Math.hypot(track.robotX - det.robotX, track.robotY - det.robotY);
      const gate = track.kind === "FOE" ? GATE_RADIUS_FOE_IN : GATE_RADIUS_IN;
      if (dist <= gate) candidates.push({ ti, di, dist });
    }
  }
  candidates.sort((a, b) => a.dist - b.dist);

  const trackUsed = new Array(predicted.length).fill(false);
  const detUsed = new Array(dets.length).fill(false);
  /** @type {SpatialTrack[]} */
  const updated = [];

  for (const match of candidates) {
    if (trackUsed[match.ti] || detUsed[match.di]) continue;
    updated.push(mergeTrack(predicted[match.ti], dets[match.di], nowMs));
    trackUsed[match.ti] = true;
    detUsed[match.di] = true;
  }

  for (let ti = 0; ti < predicted.length; ti++) {
    if (!trackUsed[ti]) updated.push(coastTrack(predicted[ti], nowMs));
  }

  for (let di = 0; di < dets.length; di++) {
    if (detUsed[di]) continue;
    const det = dets[di];
    if (det.confidence < MIN_CONF) continue;
    updated.push({
      ...det,
      trackId: state.nextTrackId++,
      source: "LIVE",
      missCount: 0,
      lastUpdateMs: nowMs,
    });
  }

  state.tracks = updated.filter((t) => t.missCount <= MAX_MISS);
  return state;
}

/** @param {import('./detection.js').Detection[]} detections */
function detectionsToTracks(detections) {
  /** @type {Omit<SpatialTrack, 'trackId'|'source'|'missCount'|'lastUpdateMs'>[]} */
  const out = [];
  for (const d of detections) {
    if (d.robotX == null || d.robotY == null || d.range == null) continue;
    const conf = d.confidence ?? 1;
    if (conf < MIN_CONF) continue;
    if (d.category === "element") {
      out.push({
        kind: "ELEMENT",
        elementId: d.elementId ?? d.name ?? "",
        robotX: d.robotX,
        robotY: d.robotY,
        cx: d.cx,
        cy: d.cy,
        range: d.range,
        confidence: conf,
      });
    } else if (d.category === "robot") {
      out.push({
        kind: "FOE",
        elementId: "",
        robotX: d.robotX,
        robotY: d.robotY,
        cx: d.cx,
        cy: d.cy,
        range: d.range,
        confidence: conf,
      });
    }
  }
  return out;
}

/** @param {{ x: number, y: number, h: number } | null} prev @param {{ x: number, y: number, h: number } | null | undefined} next */
function fieldDeltaFromOdom(prev, next) {
  if (!prev || !next) return { dx: 0, dy: 0, dh: 0 };
  return { dx: next.x - prev.x, dy: next.y - prev.y, dh: next.h - prev.h };
}

/** @param {SpatialTrack} track @param {{ dx: number, dy: number, dh: number }} delta @param {number} nowMs */
function predictTrack(track, delta, nowMs) {
  if (!delta.dx && !delta.dy && !delta.dh) return { ...track };
  const rad = (-delta.dh * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  const tx = track.robotX - delta.dx;
  const ty = track.robotY - delta.dy;
  return {
    ...track,
    robotX: tx * cos - ty * sin,
    robotY: tx * sin + ty * cos,
    source: "REMEMBERED",
    lastUpdateMs: track.lastUpdateMs ?? nowMs,
  };
}

/** @param {SpatialTrack} track @param {number} nowMs */
function coastTrack(track, nowMs) {
  return {
    ...track,
    source: "REMEMBERED",
    missCount: (track.missCount ?? 0) + 1,
    lastUpdateMs: track.lastUpdateMs ?? nowMs,
  };
}

/**
 * @param {SpatialTrack} track
 * @param {Omit<SpatialTrack, 'trackId'|'source'|'missCount'|'lastUpdateMs'>} det
 * @param {number} nowMs
 */
function mergeTrack(track, det, nowMs) {
  const alpha = 0.7;
  return {
    ...track,
    robotX: alpha * det.robotX + (1 - alpha) * track.robotX,
    robotY: alpha * det.robotY + (1 - alpha) * track.robotY,
    cx: det.cx,
    cy: det.cy,
    range: det.range,
    confidence: det.confidence,
    elementId: det.elementId || track.elementId,
    source: "LIVE",
    missCount: 0,
    lastUpdateMs: nowMs,
  };
}

/** @param {SpatialTrack[]} tracks */
export function assignOccurrenceRanks(tracks) {
  /** @type {Map<string, SpatialTrack[]>} */
  const byId = new Map();
  for (const t of tracks) {
    if (t.kind !== "ELEMENT" || !t.elementId) continue;
    const list = byId.get(t.elementId) ?? [];
    list.push(t);
    byId.set(t.elementId, list);
  }
  /** @type {Map<number, number>} */
  const ranks = new Map();
  for (const list of byId.values()) {
    list.sort((a, b) => a.range - b.range);
    list.forEach((t, i) => ranks.set(t.trackId, i));
  }
  return ranks;
}
