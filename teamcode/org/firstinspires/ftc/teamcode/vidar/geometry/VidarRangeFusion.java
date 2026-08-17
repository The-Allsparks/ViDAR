package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeEstimate;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;

/**
 * Range fusion: projective ground-plane geometry is authoritative when valid;
 * size / floor LUT / plate-width are cross-checks and fallbacks.
 */
public final class VidarRangeFusion {

    private VidarRangeFusion() {}

    public static VidarRangeEstimate buildGroundPlaneEstimate(
            double dGround,
            double cyPx,
            double horizonConfidence,
            boolean nearHorizon) {
        if (Double.isNaN(dGround) || dGround <= 0) {
            return VidarRangeEstimate.rejected(
                    VidarRangeEstimate.Source.GROUND_PLANE, "invalid_geometry");
        }
        if (nearHorizon) {
            return VidarRangeEstimate.rejected(
                    VidarRangeEstimate.Source.GROUND_PLANE, "near_horizon");
        }
        double baseUncertainty = dGround * 0.10 / Math.max(0.25, horizonConfidence);
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.GROUND_PLANE, dGround, weight, baseUncertainty);
    }

    public static VidarRangeEstimate buildSizeEstimate(
            double dSize,
            double radiusPx,
            double circleFitQuality,
            boolean partialOcclusion,
            boolean touchesBoundary) {
        if (Double.isNaN(dSize) || dSize <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.SIZE, "invalid_distance");
        }
        if (radiusPx < VidarConfig.HOUGH_MIN_RADIUS) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.SIZE, "radius_too_small");
        }
        double baseUncertainty = dSize * 0.08 / Math.max(0.3, circleFitQuality);
        if (partialOcclusion) {
            baseUncertainty *= 1.8;
        }
        if (touchesBoundary) {
            baseUncertainty *= 2.0;
        }
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.SIZE, dSize, weight, baseUncertainty);
    }

    public static VidarRangeEstimate buildFloorEstimate(
            double dFloor,
            double cyPx,
            double horizonConfidence,
            boolean nearHorizon) {
        if (Double.isNaN(dFloor) || dFloor <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.FLOOR, "invalid_lut");
        }
        if (nearHorizon) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.FLOOR, "near_horizon");
        }
        double baseUncertainty = dFloor * 0.12 / Math.max(0.2, horizonConfidence);
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.FLOOR, dFloor, weight, baseUncertainty);
    }

    public static VidarRangeEstimate buildPlateWidthEstimate(
            double dWidth,
            double pixelWidth,
            double rectangularity,
            double whiteRatio,
            boolean partialVisibility,
            boolean touchesRoiBoundary,
            double rotationPenalty) {
        if (Double.isNaN(dWidth) || dWidth <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.PLATE_WIDTH, "invalid_width");
        }
        if (pixelWidth < 20) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.PLATE_WIDTH, "width_too_small");
        }
        double baseUncertainty = dWidth * 0.10;
        baseUncertainty *= (2.0 - Math.min(1.0, rectangularity));
        baseUncertainty *= (1.5 - Math.min(0.5, whiteRatio));
        if (partialVisibility) {
            baseUncertainty *= 1.6;
        }
        if (touchesRoiBoundary) {
            baseUncertainty *= 1.4;
        }
        baseUncertainty *= (1.0 + rotationPenalty);
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.PLATE_WIDTH, dWidth, weight, baseUncertainty);
    }

    public static VidarRangeResult fuseRangeWeighted(VidarRangeEstimate... estimates) {
        return fuseRangeWeighted(VidarConfig.MAX_RANGE_MISMATCH_RATIO, estimates);
    }

    /**
     * When {@link VidarRangeEstimate.Source#GROUND_PLANE} is valid, its distance is authoritative.
     * Heuristics that disagree lower confidence instead of pulling the fused range toward a midpoint.
     * Without a valid ground-plane estimate, falls back to inverse-variance weighting among heuristics.
     */
    public static VidarRangeResult fuseRangeWeighted(
            double maxRangeMismatchRatio, VidarRangeEstimate... estimates) {
        VidarRangeEstimate[] valid = new VidarRangeEstimate[4];
        VidarRangeEstimate firstAny = null;
        VidarRangeEstimate secondAny = null;
        int validCount = 0;
        int anyCount = 0;

        for (VidarRangeEstimate est : estimates) {
            if (est == null) {
                continue;
            }
            if (anyCount == 0) {
                firstAny = est;
                anyCount = 1;
            } else if (anyCount == 1) {
                secondAny = est;
                anyCount = 2;
            }
            if (!est.isValid()) {
                continue;
            }
            if (validCount < valid.length) {
                valid[validCount++] = est;
            }
        }

        if (validCount == 0) {
            if (anyCount == 0) {
                return VidarRangeResult.invalid();
            }
            if (anyCount == 1) {
                return new VidarRangeResult(Double.NaN, Double.NaN, 0, firstAny, null, 1);
            }
            return new VidarRangeResult(Double.NaN, Double.NaN, 0, firstAny, secondAny, 2);
        }

        VidarRangeEstimate ground = null;
        VidarRangeEstimate[] heuristics = new VidarRangeEstimate[4];
        int heuristicCount = 0;
        for (int i = 0; i < validCount; i++) {
            VidarRangeEstimate est = valid[i];
            if (est.source == VidarRangeEstimate.Source.GROUND_PLANE) {
                ground = est;
            } else if (heuristicCount < heuristics.length) {
                heuristics[heuristicCount++] = est;
            }
        }

        if (ground != null) {
            return fuseGeometryPrimary(ground, heuristics, heuristicCount, maxRangeMismatchRatio);
        }
        return fuseHeuristicsWeighted(valid, validCount, maxRangeMismatchRatio);
    }

    private static VidarRangeResult fuseGeometryPrimary(
            VidarRangeEstimate ground,
            VidarRangeEstimate[] heuristics,
            int heuristicCount,
            double maxRangeMismatchRatio) {
        double maxDiffVsGround = 0;
        int agreeCount = 0;
        int disagreeCount = 0;
        VidarRangeEstimate bestHeuristic = null;
        for (int i = 0; i < heuristicCount; i++) {
            VidarRangeEstimate h = heuristics[i];
            double denom = Math.max(ground.distance, h.distance);
            double rel = denom > 0 ? Math.abs(ground.distance - h.distance) / denom : 0;
            maxDiffVsGround = Math.max(maxDiffVsGround, rel);
            if (rel <= maxRangeMismatchRatio) {
                agreeCount++;
            } else {
                disagreeCount++;
            }
            if (bestHeuristic == null || h.weight > bestHeuristic.weight) {
                bestHeuristic = h;
            }
        }

        double confidence;
        double uncertainty = ground.uncertainty;
        if (heuristicCount == 0) {
            confidence = Math.min(0.75, Math.max(0.35, 0.55 * Math.min(1.0, ground.weight)));
        } else if (disagreeCount > 0 && maxDiffVsGround > maxRangeMismatchRatio) {
            // Keep geometry distance; do not average toward bad heuristics.
            confidence = Math.max(0.15, 0.7 * (1.0 - maxDiffVsGround));
            uncertainty = ground.uncertainty * (1.0 + maxDiffVsGround);
        } else {
            // Cross-checks agree — geometry remains the range; confidence rises with support.
            confidence = Math.min(1.0, 0.65 + 0.12 * agreeCount);
        }

        return new VidarRangeResult(
                ground.distance,
                uncertainty,
                confidence,
                ground,
                bestHeuristic,
                1 + heuristicCount);
    }

    private static VidarRangeResult fuseHeuristicsWeighted(
            VidarRangeEstimate[] valid,
            int validCount,
            double maxRangeMismatchRatio) {
        double weightSum = 0;
        double weightedDist = 0;
        double varianceSum = 0;
        for (int i = 0; i < validCount; i++) {
            VidarRangeEstimate est = valid[i];
            weightSum += est.weight;
            weightedDist += est.weight * est.distance;
            varianceSum += est.weight * est.uncertainty * est.uncertainty;
        }

        if (weightSum <= 0) {
            return VidarRangeResult.invalid();
        }
        double fused = weightedDist / weightSum;
        double uncertainty = Math.sqrt(varianceSum / weightSum);

        double maxPairDiff = 0;
        for (int i = 0; i < validCount; i++) {
            for (int j = i + 1; j < validCount; j++) {
                double a = valid[i].distance;
                double b = valid[j].distance;
                double denom = Math.max(a, b);
                if (denom > 0) {
                    maxPairDiff = Math.max(maxPairDiff, Math.abs(a - b) / denom);
                }
            }
        }
        double disagreementPenalty = 1.0;
        if (validCount > 1 && maxPairDiff > maxRangeMismatchRatio) {
            disagreementPenalty = Math.max(0.2, 1.0 - maxPairDiff);
        }
        double confidence = Math.min(1.0, (weightSum / validCount) * disagreementPenalty);

        VidarRangeEstimate top0 = valid[0];
        VidarRangeEstimate top1 = validCount > 1 ? valid[1] : null;
        for (int i = 1; i < validCount; i++) {
            if (valid[i].weight > top0.weight) {
                top1 = top0;
                top0 = valid[i];
            } else if (top1 == null || valid[i].weight > top1.weight) {
                top1 = valid[i];
            }
        }
        return new VidarRangeResult(fused, uncertainty, confidence, top0, top1, validCount);
    }

    public static double profileHorizonConfidence(int horizonRowPx) {
        return Math.max(0.3, 1.0 - horizonRowPx / 120.0);
    }
}
