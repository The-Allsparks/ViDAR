package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.config.VidarElementSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarPlateSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarGroundPlane;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarTransformRegistry;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarVec3;

import java.util.ArrayList;
import java.util.List;

/**
 * Known-size monocular range, floor-plane cross-check, and uncertainty-weighted fusion.
 */
public final class VidarGeometry {

    private VidarGeometry() {}

    public static double distanceFromSize(double diameter, double focalPx, double radiusPx) {
        if (radiusPx <= 0 || focalPx <= 0 || diameter <= 0) {
            return Double.NaN;
        }
        return (diameter * focalPx) / (2.0 * radiusPx);
    }

    public static double distanceFromWidth(double physicalWidthIn, double focalPx, double pixelWidth) {
        if (pixelWidth <= 0 || focalPx <= 0 || physicalWidthIn <= 0) {
            return Double.NaN;
        }
        return (physicalWidthIn * focalPx) / pixelWidth;
    }

    /** Interpolate floor slant range from element center row (process-frame coordinates). */
    public static double distanceFromFloor(double cyPx, VidarCameraProfile profile) {
        if (profile.floorCyPx == null || profile.floorDist == null
                || profile.floorCyPx.length == 0
                || profile.floorCyPx.length != profile.floorDist.length) {
            return Double.NaN;
        }

        double[] xs = profile.floorCyPx;
        double[] ys = profile.floorDist;

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

    /**
     * Slant range along a pixel ray to a horizontal plane at {@code targetHeightZ} in robot frame.
     * For floor game pieces use {@code targetHeightZ = elementDiameter / 2} (ball center height).
     */
    public static double distanceFromGroundPlane(
            double cx, double cy, VidarCameraProfile profile, double targetHeightZ) {
        VidarGroundPlane.Intersection hit =
                intersectGroundPlaneAtHeight(cx, cy, profile, targetHeightZ);
        return hit.valid ? hit.slantRange : Double.NaN;
    }

    public static VidarGroundPlane.Intersection intersectGroundPlaneAtHeight(
            double cx, double cy, VidarCameraProfile profile, double targetHeightZ) {
        if (profile == null) {
            return VidarGroundPlane.Intersection.rejected("no_profile");
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms == null) {
            return VidarGroundPlane.Intersection.rejected("no_transform");
        }
        VidarVec3 origin = transforms.robotTCamera.translation;
        VidarVec3 dir = transforms.robotTCamera.transformDirection(
                transforms.intrinsics.pixelToRay(cx, cy));
        return VidarGroundPlane.intersectAtPlane(origin, dir, targetHeightZ, Double.NaN);
    }

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
        // Mount/intrinsic uncertainty — tighter when geometry agrees with calibrated mount.
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

    public static VidarRangeResult fuseRangeWeighted(
            double maxRangeMismatchRatio, VidarRangeEstimate... estimates) {
        VidarRangeEstimate[] valid = new VidarRangeEstimate[3];
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

    private static double profileHorizonConfidence(int horizonRowPx) {
        return Math.max(0.3, 1.0 - horizonRowPx / 120.0);
    }

    /** Robot-frame floor coords from bearing + range (inches). +X forward, +Y left. */
    public static double robotX(double range, double bearingDeg) {
        return robotX(range, bearingDeg, null);
    }

    public static double robotY(double range, double bearingDeg) {
        return robotY(range, bearingDeg, null);
    }

    /**
     * Unit ray direction in robot frame for a full-frame pixel.
     * Robot frame: +X forward, +Y left, +Z up. Camera pitch negative = looking down.
     *
     * <p>Uses cached {@code robot_T_cameraOptical} from {@link VidarTransformRegistry}.
     */
    public static double[] rayDirectionRobotFrame(double cx, double cy, VidarCameraProfile profile) {
        if (profile == null) {
            return new double[] { Double.NaN, Double.NaN, Double.NaN };
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms == null) {
            return legacyRayDirectionRobotFrame(cx, cy, profile);
        }
        VidarVec3 ray = transforms.robotTCamera.transformDirection(
                transforms.intrinsics.pixelToRay(cx, cy));
        return new double[] { ray.x, ray.y, ray.z };
    }

    /** Legacy inline implementation — kept for regression tests. */
    static double[] legacyRayDirectionRobotFrame(double cx, double cy, VidarCameraProfile profile) {
        double u = (cx - profile.principalPointX) / profile.focalLengthPx;
        double v = (cy - profile.principalPointY) / profile.focalLengthYPx;
        double[] cam = normalize3(u, v, 1.0);

        // Camera optical frame: x right, y down, z forward -> robot at bearing 0: z->+X, x->-Y, y->-Z
        double rx = cam[2];
        double ry = -cam[0];
        double rz = -cam[1];
        double[] base = new double[] { rx, ry, rz };

        double bearingRad = Math.toRadians(profile.bearingDeg + profile.mountYawDeg);
        double[] panned = rotateZ(base, bearingRad);
        double[] pitched = rotateX(panned, Math.toRadians(profile.mountPitchDeg));
        double[] rolled = rotateZ(pitched, Math.toRadians(profile.mountRollDeg));
        return normalize3(rolled[0], rolled[1], rolled[2]);
    }

    /**
     * Floor contact point in robot frame from slant range along the pixel ray.
     * Returns {@code [robotX, robotY, robotZ]} (floor targets have z near 0).
     *
     * <p>Primary path uses {@code robot_T_cameraOptical}; falls back to legacy ray math on failure.
     */
    public static double[] floorPointInRobot(
            double cx, double cy, double slantRange, VidarCameraProfile profile) {
        if (profile == null || Double.isNaN(slantRange) || slantRange <= 0) {
            return new double[] { Double.NaN, Double.NaN, Double.NaN };
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms != null) {
            VidarGroundPlane.Intersection hit = VidarGroundPlane.floorPointFromSlantRange(
                    transforms, cx, cy, slantRange, Double.NaN);
            if (hit.valid) {
                return new double[] { hit.robotX, hit.robotY, 0.0 };
            }
        }
        double[] dir = legacyRayDirectionRobotFrame(cx, cy, profile);
        return new double[] {
                profile.mountX + slantRange * dir[0],
                profile.mountY + slantRange * dir[1],
                profile.mountZ + slantRange * dir[2]
        };
    }

    /**
     * Ground-plane intersection from pixel ray only (no slant range).
     * Assumes target contacts z = 0 floor.
     */
    public static VidarGroundPlane.Intersection intersectGroundPlane(
            double cx, double cy, VidarCameraProfile profile) {
        if (profile == null) {
            return VidarGroundPlane.Intersection.rejected("no_profile");
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms == null) {
            return VidarGroundPlane.Intersection.rejected("no_transform");
        }
        VidarVec3 origin = transforms.robotTCamera.translation;
        VidarVec3 dir = transforms.robotTCamera.transformDirection(
                transforms.intrinsics.pixelToRay(cx, cy));
        return VidarGroundPlane.intersect(origin, dir, Double.NaN);
    }

    public static double robotX(double range, double bearingDeg, VidarCameraProfile profile) {
        if (Double.isNaN(range)) {
            return Double.NaN;
        }
        if (profile == null) {
            double rad = Math.toRadians(bearingDeg);
            return range * Math.cos(rad);
        }
        return floorPointInRobot(
                profile.principalPointX, profile.principalPointY, range, profile)[0];
    }

    public static double robotY(double range, double bearingDeg, VidarCameraProfile profile) {
        if (Double.isNaN(range)) {
            return Double.NaN;
        }
        if (profile == null) {
            double rad = Math.toRadians(bearingDeg);
            return range * Math.sin(rad);
        }
        return floorPointInRobot(
                profile.principalPointX, profile.principalPointY, range, profile)[1];
    }

    private static double[] normalize3(double x, double y, double z) {
        double len = Math.sqrt(x * x + y * y + z * z);
        if (len <= 1e-9) {
            return new double[] { 0, 0, 1 };
        }
        return new double[] { x / len, y / len, z / len };
    }

    private static double[] rotateX(double[] v, double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new double[] {
                v[0],
                v[1] * c - v[2] * s,
                v[1] * s + v[2] * c
        };
    }

    private static double[] rotateZ(double[] v, double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new double[] {
                v[0] * c - v[1] * s,
                v[0] * s + v[1] * c,
                v[2]
        };
    }

    public static VidarElementObservation fuseElementObservation(
            double cx,
            double cy,
            double radiusPx,
            double areaPx,
            double aspectRatio,
            double circularity,
            double fillRatio,
            double interiorScore,
            VidarElementDetectorType detectorType,
            VidarCameraProfile profile,
            String cameraName,
            long captureTimeNanos,
            boolean touchesBoundary,
            boolean partialOcclusion,
            double circleFitQuality) {
        return fuseElementObservation(
                cx, cy, radiusPx, areaPx, aspectRatio, circularity, fillRatio, interiorScore,
                detectorType, profile, cameraName, captureTimeNanos,
                touchesBoundary, partialOcclusion, circleFitQuality,
                null, null);
    }

    public static VidarElementObservation fuseElementObservation(
            double cx,
            double cy,
            double radiusPx,
            double areaPx,
            double aspectRatio,
            double circularity,
            double fillRatio,
            double interiorScore,
            VidarElementDetectorType detectorType,
            VidarCameraProfile profile,
            String cameraName,
            long captureTimeNanos,
            boolean touchesBoundary,
            boolean partialOcclusion,
            double circleFitQuality,
            VidarElementSpec element,
            VidarSeasonConfig season) {
        VidarElementSpec activeElement = element != null ? element : org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason().primaryElement();
        VidarSeasonConfig activeSeason = season != null ? season : org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason();
        double dSize = distanceFromSize(
                activeElement.diameter, profile.focalLengthPx, radiusPx);
        double dFloor = distanceFromFloor(cy, profile);
        double ballCenterZ = activeElement.diameter * 0.5;
        double dGround = distanceFromGroundPlane(cx, cy, profile, ballCenterZ);
        boolean nearHorizon = cy <= profile.horizonRowPx + 8;
        double horizonConf = profileHorizonConfidence(profile.horizonRowPx);

        VidarRangeEstimate sizeEst = buildSizeEstimate(
                dSize, radiusPx, circleFitQuality, partialOcclusion, touchesBoundary);
        VidarRangeEstimate floorEst = buildFloorEstimate(
                dFloor, cy, horizonConf, nearHorizon);
        VidarRangeEstimate groundEst = buildGroundPlaneEstimate(
                dGround, cy, horizonConf, nearHorizon);
        VidarRangeResult rangeResult = fuseRangeWeighted(
                activeSeason.maxRangeMismatchRatio, sizeEst, floorEst, groundEst);

        double confidence = composeElementConfidence(
                interiorScore, circularity, fillRatio, circleFitQuality,
                touchesBoundary, rangeResult, areaPx, activeElement);

        double[] robotPoint = floorPointInRobot(cx, cy, rangeResult.distance, profile);
        double robotX = robotPoint[0];
        double robotY = robotPoint[1];

        return new VidarElementObservation(
                cameraName,
                captureTimeNanos,
                cx, cy,
                radiusPx * 2, radiusPx * 2,
                cx, cy, radiusPx,
                areaPx, aspectRatio, circularity, fillRatio, interiorScore,
                detectorType, confidence,
                rangeResult.distance, rangeResult.uncertainty,
                dSize, dFloor,
                rangeResult,
                robotX, robotY);
    }

    public static double composeElementConfidence(
            double interiorScore,
            double circularity,
            double fillRatio,
            double circleFitQuality,
            boolean touchesBoundary,
            VidarRangeResult rangeResult,
            double areaPx) {
        return composeElementConfidence(
                interiorScore, circularity, fillRatio, circleFitQuality,
                touchesBoundary, rangeResult, areaPx,
                org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason().primaryElement());
    }

    public static double composeElementConfidence(
            double interiorScore,
            double circularity,
            double fillRatio,
            double circleFitQuality,
            boolean touchesBoundary,
            VidarRangeResult rangeResult,
            double areaPx,
            VidarElementSpec element) {
        double maskScore = Math.min(1.0, interiorScore);
        double shapeScore = 0.35 * circularity + 0.25 * fillRatio + 0.40 * circleFitQuality;
        double sizeScore = Math.min(1.0, areaPx / (element.minAreaPx * 3.0));
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
        return composePlateConfidence(
                whiteRatio, area, rectangularity, aspectRatio, rangeResult,
                viewingAnglePenalty, partialPenalty,
                org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason().plateSpec(VidarAlliance.RED));
    }

    public static double composePlateConfidence(
            double whiteRatio,
            double area,
            double rectangularity,
            double aspectRatio,
            VidarRangeResult rangeResult,
            double viewingAnglePenalty,
            double partialPenalty,
            VidarPlateSpec plate) {
        double whiteScore = Math.min(1.0, whiteRatio / Math.max(0.01, plate.minWhiteRatio));
        double areaScore = Math.min(1.0, area / (plate.minAreaPx * 4.0));
        double rectScore = Math.min(1.0, rectangularity);
        double aspectScore = aspectRatio >= plate.minAspect
                && aspectRatio <= plate.maxAspect ? 1.0 : 0.5;
        double rangeScore = rangeResult.isValid() ? rangeResult.confidence : 0.35;
        double penalty = viewingAnglePenalty * partialPenalty;
        return Math.max(0, Math.min(1.0, penalty * (
                0.25 * whiteScore + 0.20 * areaScore + 0.15 * rectScore
                        + 0.10 * aspectScore + 0.30 * rangeScore)));
    }
}
