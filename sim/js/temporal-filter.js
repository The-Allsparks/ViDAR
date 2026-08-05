/**
 * Temporal consensus filter — require 3 of last 5 frames to confirm or drop a detection.
 */

/** @typedef {import('./detection.js').Detection & { temporal?: 'valid' | 'potential' | 'coasting' }} StableDetection */

/**
 * @typedef {{ id: number, category: string, name: string, detector: string | null, history: boolean[], last: import('./detection.js').Detection | null, confirmed: boolean }} Track
 */

/** @typedef {{ tracks: Track[], nextId: number }} TemporalFilterState */

export const TEMPORAL_WINDOW = 5;
export const TEMPORAL_REQUIRED = 3;

/** @returns {TemporalFilterState} */
export function createTemporalFilterState() {
  return { tracks: [], nextId: 1 };
}

/**
 * @param {TemporalFilterState} state
 * @param {import('./detection.js').Detection[]} raw
 * @param {{ enabled: boolean, windowSize?: number, required?: number, matchDistance?: number }} opts
 * @returns {{ stable: StableDetection[], raw: import('./detection.js').Detection[], state: TemporalFilterState }}
 */
export function applyTemporalFilter(state, raw, opts) {
  if (!opts.enabled) {
    return {
      stable: raw.map((d) => ({ ...d, temporal: /** @type {const} */ ("valid") })),
      raw,
      state: createTemporalFilterState(),
    };
  }

  const windowSize = opts.windowSize ?? TEMPORAL_WINDOW;
  const required = opts.required ?? TEMPORAL_REQUIRED;
  const matchDistance = opts.matchDistance ?? 48;

  /** @type {Track[]} */
  const tracks = state.tracks.map((t) => ({ ...t, history: [...t.history] }));
  let nextId = state.nextId;
  const matchedTrackIds = new Set();

  for (const det of raw) {
    const track = findBestTrack(tracks, det, matchDistance, matchedTrackIds);
    if (track) {
      matchedTrackIds.add(track.id);
      pushHistory(track, true, windowSize);
      track.last = det;
      track.name = det.name;
      track.detector = det.detector ?? null;
    } else {
      const id = nextId++;
      tracks.push({
        id,
        category: det.category,
        name: det.name,
        detector: det.detector ?? null,
        history: [true],
        last: det,
        confirmed: false,
      });
      matchedTrackIds.add(id);
    }
  }

  for (const track of tracks) {
    if (!matchedTrackIds.has(track.id)) {
      pushHistory(track, false, windowSize);
    }
    track.confirmed = countPresent(track.history) >= required;
  }

  /** @type {StableDetection[]} */
  const stable = [];
  for (const track of tracks) {
    if (!track.last) continue;
    const presentNow = track.history[track.history.length - 1] === true;
    const confirmed = track.confirmed;

    if (!confirmed && !presentNow) continue;

    let temporal = /** @type {'valid' | 'potential' | 'coasting'} */ ("potential");
    if (confirmed && presentNow) temporal = "valid";
    else if (confirmed && !presentNow) temporal = "coasting";
    else temporal = "potential";

    stable.push({ ...track.last, temporal });
  }

  const pruned = tracks.filter(
    (t) => t.confirmed || (t.history.length > 0 && t.history[t.history.length - 1]),
  );

  return {
    stable,
    raw,
    state: { tracks: pruned, nextId },
  };
}

/** @param {Track[]} tracks @param {import('./detection.js').Detection} det @param {Set<number>} used */
function findBestTrack(tracks, det, maxDist, used) {
  let best = null;
  let bestD = maxDist;
  for (const track of tracks) {
    if (used.has(track.id)) continue;
    if (track.category !== det.category) continue;
    if (track.name !== det.name) continue;
    if ((track.detector ?? null) !== (det.detector ?? null)) continue;
    if (!track.last) continue;
    const d = Math.hypot(det.cx - track.last.cx, det.cy - track.last.cy);
    if (d < bestD) {
      bestD = d;
      best = track;
    }
  }
  return best;
}

/** @param {Track} track @param {boolean} present @param {number} windowSize */
function pushHistory(track, present, windowSize) {
  track.history.push(present);
  while (track.history.length > windowSize) track.history.shift();
}

/** @param {boolean[]} history */
function countPresent(history) {
  let n = 0;
  for (const v of history) if (v) n += 1;
  return n;
}

/** Logic + primary overlays use confirmed detections only (valid or coasting). */
export function confirmedDetections(stable) {
  return stable.filter((d) => d.temporal === "valid" || d.temporal === "coasting");
}
