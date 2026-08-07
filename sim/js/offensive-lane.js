/** @typedef {'LEFT'|'CENTER'|'RIGHT'} OffensiveLane */

/**
 * @param {import('./spatial-tracks.js').SpatialTrack[]} foeTracks
 * @param {{ maxRangeIn?: number, coneHalfDeg?: number }} [opts]
 */
export function analyzeOffensiveLane(foeTracks, opts = {}) {
  const maxRangeIn = opts.maxRangeIn ?? 48;
  const coneHalfDeg = opts.coneHalfDeg ?? 35;
  const third = coneHalfDeg / 3;

  let left = 0;
  let center = 0;
  let right = 0;

  for (const foe of foeTracks) {
    if (foe.range > maxRangeIn) continue;
    const bearing = (Math.atan2(foe.robotY, foe.robotX) * 180) / Math.PI;
    if (Math.abs(bearing) > coneHalfDeg) continue;
    if (bearing < -third) left += 1;
    else if (bearing > third) right += 1;
    else center += 1;
  }

  const min = Math.min(left, center, right);
  /** @type {OffensiveLane} */
  let recommended = "CENTER";
  if (center === min) recommended = "CENTER";
  else if (left === min) recommended = "LEFT";
  else recommended = "RIGHT";

  return { left, center, right, recommended, maxRangeIn, coneHalfDeg };
}
