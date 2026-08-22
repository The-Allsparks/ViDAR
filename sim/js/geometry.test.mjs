/**
 * Range fusion parity cases — mirrors java-pure RangeFusionTest / tests/test_vidar_core.py.
 * Run: node sim/js/geometry.test.mjs
 */
import {
  buildFloorEstimate,
  buildGroundPlaneEstimate,
  buildSizeEstimate,
  fuseRangeEstimates,
  fuseRangeWeighted,
} from "./geometry.js";

function assert(cond, msg) {
  if (!cond) throw new Error(msg);
}

function approx(a, b, eps = 1e-9) {
  assert(Math.abs(a - b) <= eps, `expected ${b}, got ${a}`);
}

// geometryDisagreementKeepsGroundDistance — 23 vs 42 → fused 42
{
  const size = buildSizeEstimate(23, 20, 0.9, false, false);
  const floor = buildFloorEstimate(23, 60, 0.8, false);
  const ground = buildGroundPlaneEstimate(42, 200, 0.8, false);
  const result = fuseRangeEstimates([size, floor, ground]);
  assert(result.distance === 42, "geometry disagree must keep ground distance");
  assert(result.confidence < 0.5, "confidence should drop on disagree");
  assert(result.primarySource === "GROUND_PLANE", "primary source ground");
}

// threeWayFusionWhenGeometryAgrees — fused 24.5
{
  const size = buildSizeEstimate(24, 14, 0.9, false, false);
  const floor = buildFloorEstimate(25, 200, 0.8, false);
  const ground = buildGroundPlaneEstimate(24.5, 200, 0.8, false);
  const result = fuseRangeEstimates([size, floor, ground]);
  approx(result.distance, 24.5);
  assert(result.confidence > 0.7, "agreeing heuristics raise confidence");
}

// heuristicsOnlyStillWeighted
{
  const size = buildSizeEstimate(36, 20, 0.9, false, false);
  const floor = buildFloorEstimate(38, 60, 0.8, false);
  const result = fuseRangeEstimates([size, floor]);
  assert(result.distance > 34 && result.distance < 40, "heuristic weighted average");
}

// legacy rollback path
{
  const legacy = fuseRangeWeighted(36, 38, { legacyAverageOnly: true });
  assert(legacy.distance > 34 && legacy.distance < 40, "legacy average");
  const modern = fuseRangeWeighted(23, 23, { dGround: 42, radiusPx: 20, nearHorizon: false });
  assert(modern.distance === 42, "modern path with explicit dGround");
}

console.log("geometry.test.mjs: all parity checks passed");
