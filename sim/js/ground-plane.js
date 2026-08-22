/**
 * Ground-plane ray intersection (parity with src/vidar/transforms.py).
 * Requires mount + intrinsics on the camera profile; otherwise callers skip ground fusion.
 */

/** @param {number[]} a @param {number[]} b */
function mat3Mul(a, b) {
  const out = new Array(9);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      out[r * 3 + c] =
        a[r * 3 + 0] * b[0 * 3 + c] +
        a[r * 3 + 1] * b[1 * 3 + c] +
        a[r * 3 + 2] * b[2 * 3 + c];
    }
  }
  return out;
}

/** @param {number} rad */
function rotateX(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}

/** @param {number} rad */
function rotateY(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}

/** @param {number} rad */
function rotateZ(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

function opticalToRobotBase() {
  return [0, 0, 1, -1, 0, 0, 0, -1, 0];
}

/** Vitelli axes: R = Rz(yaw) * Ry(pitch) * Rx(roll). Config pitchDeg negated before Ry. */
function fromRollPitchYawDeg(rollDeg, pitchDeg, yawDeg) {
  const rRoll = rotateX((rollDeg * Math.PI) / 180);
  const rPitch = rotateY((pitchDeg * Math.PI) / 180);
  const rYaw = rotateZ((yawDeg * Math.PI) / 180);
  return mat3Mul(mat3Mul(rYaw, rPitch), rRoll);
}

/** @param {number[]} m @param {{ x: number, y: number, z: number }} v */
function rotateVec(m, v) {
  return {
    x: m[0] * v.x + m[1] * v.y + m[2] * v.z,
    y: m[3] * v.x + m[4] * v.y + m[5] * v.z,
    z: m[6] * v.x + m[7] * v.y + m[8] * v.z,
  };
}

/** @param {{ x: number, y: number, z: number }} v */
function normalize(v) {
  const len = Math.hypot(v.x, v.y, v.z);
  if (len <= 1e-9) return { x: 0, y: 0, z: 1 };
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

/** @param {import('./config.js').CameraProfile} profile */
export function hasGroundPlaneExtrinsics(profile) {
  return (
    profile != null &&
    Number.isFinite(profile.mountX) &&
    Number.isFinite(profile.mountY) &&
    Number.isFinite(profile.mountZ) &&
    Number.isFinite(profile.focalLengthPx)
  );
}

/**
 * Slant range from camera mount to ground-plane intersection at {@code planeZ}.
 * @param {number} cx pixel x (full frame)
 * @param {number} cy pixel y
 * @param {import('./config.js').CameraProfile} profile
 * @param {number} [planeZ=0] floor height in robot frame
 * @returns {number | null} slant range inches, or null when extrinsics missing / ray invalid
 */
export function distanceFromGroundPlane(cx, cy, profile, planeZ = 0) {
  if (!hasGroundPlaneExtrinsics(profile)) {
    return null;
  }

  const fx = profile.focalLengthPx;
  const fy = profile.focalLengthYPx ?? fx;
  const cx0 = profile.principalPointX ?? 320;
  const cy0 = profile.principalPointY ?? 240;
  const u = (cx - cx0) / fx;
  const v = (cy - cy0) / fy;
  let dir = normalize({ x: u, y: v, z: 1 });

  const mountRot = fromRollPitchYawDeg(
    profile.mountRollDeg ?? 0,
    -(profile.mountPitchDeg ?? 0),
    (profile.bearingDeg ?? 0) + (profile.mountYawDeg ?? 0),
  );
  const rot = mat3Mul(mountRot, opticalToRobotBase());
  dir = rotateVec(rot, dir);

  const origin = {
    x: profile.mountX,
    y: profile.mountY,
    z: profile.mountZ,
  };

  if (!Number.isFinite(dir.z) || Math.abs(dir.z) < 1e-6) {
    return null;
  }
  if (origin.z <= planeZ && dir.z >= 0) {
    return null;
  }
  const t = (planeZ - origin.z) / dir.z;
  if (t <= 0) {
    return null;
  }
  return t;
}
