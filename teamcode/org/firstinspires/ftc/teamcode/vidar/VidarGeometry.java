package org.firstinspires.ftc.teamcode.vidar;

import java.util.ArrayList;
import java.util.List;

/**
 * Known-size monocular range, floor-plane cross-check, and uncertainty-weighted fusion.
 */
public final class VidarGeometry {

    private VidarGeometry() {}

    public static double distanceFromSizeInches(double diameterIn, double focalPx, double radiusPx) {
        if (radiusPx <= 0 || focalPx <= 0 || diameterIn <= 0) {
            return Double.NaN;
        }
        return (diameterIn * focalPx) / (2.0 * radiusPx);
    }

    public static double distanceFromWidthInches(double physicalWidthIn, double focalPx, double pixelWidth) {
        if (pixelWidth <= 0 || focalPx <= 0 || physicalWidthIn <= 0) {
            return Double.NaN;
        }
        return (physicalWidthIn * focalPx) / pixelWidth;
    }

    /** Interpolate floor slant range from ball center row (process-frame coordinates). */
    public static double distanceFromFloorInches(double cyPx, VidarCameraProfile profile) {
        if (profile.floorCyPx == null || profile.floorDistIn == null
                || profile.floorCyPx.length == 0
                || profile.floorCyPx.length != profile.floorDistIn.length) {
            return Double.NaN;
        }

        double[] xs = profile.floorCyPx;
        double[] ys = profile.floorDistIn;

        if (cyPx <= xs[0]) {
            return ys[0];
        }
        int last = xs.length - 1;
        if (cyPx >= xs[last]) {
            return ys[last];
        }

        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            if (cyPx >= x0 && cyPx <= x1) {
                double t = (cyPx - x0) / (x1 - x0);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return Double.NaN;
    }

    /** @deprecated Use {@link #fuseRangeWeighted} for uncertainty-weighted fusion. */
    @Deprecated
    public static double fuseRangeInches(double dSizeIn, double dFloorIn) {
        VidarRangeResult result = fuseRangeWeighted(
                buildBallSizeEstimate(dSizeIn, 20, 1.0, false, false),
                buildFloorEstimate(dFloorIn, 12, profileHorizonConfidence(12), false));
        return result.isValid() ? result.distanceIn : Double.NaN;
    }

    /** @deprecated Use {@link #fuseRangeWeighted}. */
    @Deprecated
    public static double rangeConfidence(double dSizeIn, double dFloorIn) {
        VidarRangeResult result = fuseRangeWeighted(
                buildBallSizeEstimate(dSizeIn, 20, 1.0, false, false),
                buildFloorEstimate(dFloorIn, 12, profileHorizonConfidence(12), false));
        return result.confidence;
    }

    public static VidarRangeEstimate buildBallSizeEstimate(
            double dSizeIn,
            double radiusPx,
            double circleFitQuality,
            boolean partialOcclusion,
            boolean touchesBoundary) {
        if (Double.isNaN(dSizeIn) || dSizeIn <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.SIZE, "invalid_distance");
        }
        if (radiusPx < VidarConfig.HOUGH_MIN_RADIUS) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.SIZE, "radius_too_small");
        }
        double baseUncertainty = dSizeIn * 0.08 / Math.max(0.3, circleFitQuality);
        if (partialOcclusion) {
            baseUncertainty *= 1.8;
        }
        if (touchesBoundary) {
            baseUncertainty *= 2.0;
        }
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.SIZE, dSizeIn, weight, baseUncertainty);
    }

    public static VidarRangeEstimate buildFloorEstimate(
            double dFloorIn,
            double cyPx,
            double horizonConfidence,
            boolean nearHorizon) {
        if (Double.isNaN(dFloorIn) || dFloorIn <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.FLOOR, "invalid_lut");
        }
        if (nearHorizon) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.FLOOR, "near_horizon");
        }
        double baseUncertainty = dFloorIn * 0.12 / Math.max(0.2, horizonConfidence);
        double weight = 1.0 / (baseUncertainty * baseUncertainty);
        return new VidarRangeEstimate(
                VidarRangeEstimate.Source.FLOOR, dFloorIn, weight, baseUncertainty);
    }

    public static VidarRangeEstimate buildPlateWidthEstimate(
            double dWidthIn,
            double pixelWidth,
            double rectangularity,
            double whiteRatio,
            boolean partialVisibility,
            boolean touchesRoiBoundary,
            double rotationPenalty) {
        if (Double.isNaN(dWidthIn) || dWidthIn <= 0) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.PLATE_WIDTH, "invalid_width");
        }
        if (pixelWidth < 20) {
            return VidarRangeEstimate.rejected(VidarRangeEstimate.Source.PLATE_WIDTH, "width_too_small");
        }
        double baseUncertainty = dWidthIn * 0.10;
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
                VidarRangeEstimate.Source.PLATE_WIDTH, dWidthIn, weight, baseUncertainty);
    }

    public static VidarRangeResult fuseRangeWeighted(VidarRangeEstimate... estimates) {
        List<VidarRangeEstimate> valid = new ArrayList<>();
        for (VidarRangeEstimate est : estimates) {
            if (est != null && est.isValid()) {
                valid.add(est);
            }
        }
        if (valid.isEmpty()) {
            List<VidarRangeEstimate> all = new ArrayList<>();
            for (VidarRangeEstimate est : estimates) {
                if (est != null) {
                    all.add(est);
                }
            }
            return new VidarRangeResult(Double.NaN, Double.NaN, 0, all);
        }

        double weightSum = 0;
        double weightedDist = 0;
        double varianceSum = 0;
        for (VidarRangeEstimate est : valid) {
            weightSum += est.weight;
            weightedDist += est.weight * est.distanceIn;
            varianceSum += est.weight * est.uncertaintyIn * est.uncertaintyIn;
        }
        if (weightSum <= 0) {
            return VidarRangeResult.invalid();
        }
        double fused = weightedDist / weightSum;
        double uncertainty = Math.sqrt(varianceSum / weightSum);

        double disagreementPenalty = 1.0;
        if (valid.size() >= 2) {
            double maxDiff = 0;
            for (VidarRangeEstimate a : valid) {
                for (VidarRangeEstimate b : valid) {
                    if (a == b) continue;
                    double denom = Math.max(a.distanceIn, b.distanceIn);
                    if (denom > 0) {
                        maxDiff = Math.max(maxDiff, Math.abs(a.distanceIn - b.distanceIn) / denom);
                    }
                }
            }
            if (maxDiff > VidarConfig.MAX_RANGE_MISMATCH_RATIO) {
                disagreementPenalty = Math.max(0.2, 1.0 - maxDiff);
            }
        }
        double confidence = Math.min(1.0, (weightSum / valid.size()) * disagreementPenalty);
        return new VidarRangeResult(fused, uncertainty, confidence, valid);
    }

    private static double profileHorizonConfidence(int horizonRowPx) {
        return Math.max(0.3, 1.0 - horizonRowPx / 120.0);
    }

    /** Robot-frame floor coords from bearing + range (inches). +X forward, +Y left. */
    public static double robotXInches(double rangeIn, double bearingDeg) {
        return robotXInches(rangeIn, bearingDeg, null);
    }

    public static double robotYInches(double rangeIn, double bearingDeg) {
        return robotYInches(rangeIn, bearingDeg, null);
    }

    public static double robotXInches(double rangeIn, double bearingDeg, VidarCameraProfile profile) {
        if (Double.isNaN(rangeIn)) {
            return Double.NaN;
        }
        double mountX = profile == null ? 0 : profile.mountXIn;
        double rad = Math.toRadians(bearingDeg);
        return mountX + rangeIn * Math.cos(rad);
    }

    public static double robotYInches(double rangeIn, double bearingDeg, VidarCameraProfile profile) {
        if (Double.isNaN(rangeIn)) {
            return Double.NaN;
        }
        double mountY = profile == null ? 0 : profile.mountYIn;
        double rad = Math.toRadians(bearingDeg);
        return mountY + rangeIn * Math.sin(rad);
    }

    public static VidarBallObservation fuseBallObservation(
            double cx,
            double cy,
            double radiusPx,
            double areaPx,
            double aspectRatio,
            double circularity,
            double fillRatio,
            double interiorScore,
            VidarBallDetectorType detectorType,
            VidarCameraProfile profile,
            String cameraName,
            long captureTimeNanos,
            boolean touchesBoundary,
            boolean partialOcclusion,
            double circleFitQuality) {
        double dSize = distanceFromSizeInches(
                VidarConfig.BALL_DIAMETER_IN, profile.focalLengthPx, radiusPx);
        double dFloor = distanceFromFloorInches(cy, profile);
        boolean nearHorizon = cy <= profile.horizonRowPx + 8;

        VidarRangeEstimate sizeEst = buildBallSizeEstimate(
                dSize, radiusPx, circleFitQuality, partialOcclusion, touchesBoundary);
        VidarRangeEstimate floorEst = buildFloorEstimate(
                dFloor, cy, profileHorizonConfidence(profile.horizonRowPx), nearHorizon);
        VidarRangeResult rangeResult = fuseRangeWeighted(sizeEst, floorEst);

        double confidence = composeBallConfidence(
                interiorScore, circularity, fillRatio, circleFitQuality,
                touchesBoundary, rangeResult, areaPx);

        double robotX = robotXInches(rangeResult.distanceIn, profile.bearingDeg, profile);
        double robotY = robotYInches(rangeResult.distanceIn, profile.bearingDeg, profile);

        return new VidarBallObservation(
                cameraName,
                captureTimeNanos,
                cx, cy,
                radiusPx * 2, radiusPx * 2,
                cx, cy, radiusPx,
                areaPx, aspectRatio, circularity, fillRatio, interiorScore,
                detectorType, confidence,
                rangeResult.distanceIn, rangeResult.uncertaintyIn,
                rangeResult.sourceDistance(VidarRangeEstimate.Source.SIZE),
                rangeResult.sourceDistance(VidarRangeEstimate.Source.FLOOR),
                rangeResult,
                robotX, robotY);
    }

    /** @deprecated Use {@link #fuseBallObservation}. Kept for legacy Hough path. */
    @Deprecated
    public static VidarBallObservation fuseObservation(
            double cx,
            double cy,
            double radiusPx,
            int houghVotes,
            VidarCameraProfile profile,
            String cameraName) {
        double area = Math.PI * radiusPx * radiusPx;
        return fuseBallObservation(
                cx, cy, radiusPx, area, 1.0, 1.0, 0.85, 0.5,
                VidarBallDetectorType.LEGACY_HOUGH, profile, cameraName, 0,
                false, false, Math.min(1.0, houghVotes / 20.0));
    }

    public static double composeBallConfidence(
            double interiorScore,
            double circularity,
            double fillRatio,
            double circleFitQuality,
            boolean touchesBoundary,
            VidarRangeResult rangeResult,
            double areaPx) {
        double maskScore = Math.min(1.0, interiorScore);
        double shapeScore = 0.35 * circularity + 0.25 * fillRatio + 0.40 * circleFitQuality;
        double sizeScore = Math.min(1.0, areaPx / (VidarConfig.MIN_BALL_AREA_PX * 3.0));
        double rangeScore = rangeResult.isValid() ? rangeResult.confidence : 0.3;
        double boundaryPenalty = touchesBoundary ? 0.7 : 1.0;
        return Math.max(0, Math.min(1.0,
                boundaryPenalty * (0.20 * maskScore + 0.30 * shapeScore + 0.15 * sizeScore + 0.35 * rangeScore)));
    }

    public static double composePlateConfidence(
            double whiteRatio,
            double area,
            double rectangularity,
            double aspectRatio,
            VidarRangeResult rangeResult,
            double viewingAnglePenalty,
            double partialPenalty) {
        double whiteScore = Math.min(1.0, whiteRatio / Math.max(0.01, VidarConfig.PLATE_MIN_WHITE_RATIO));
        double areaScore = Math.min(1.0, area / (VidarConfig.PLATE_MIN_AREA_PX * 4.0));
        double rectScore = Math.min(1.0, rectangularity);
        double aspectScore = aspectRatio >= VidarConfig.PLATE_MIN_ASPECT
                && aspectRatio <= VidarConfig.PLATE_MAX_ASPECT ? 1.0 : 0.5;
        double rangeScore = rangeResult.isValid() ? rangeResult.confidence : 0.35;
        double penalty = viewingAnglePenalty * partialPenalty;
        return Math.max(0, Math.min(1.0, penalty * (
                0.25 * whiteScore + 0.20 * areaScore + 0.15 * rectScore
                        + 0.10 * aspectScore + 0.30 * rangeScore)));
    }
}
