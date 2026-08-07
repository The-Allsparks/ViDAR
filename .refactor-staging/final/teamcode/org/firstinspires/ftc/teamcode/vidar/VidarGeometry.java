package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeEstimate;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.config.VidarElementSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarPlateSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarGroundPlane;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarRangeFusion;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarTransformRegistry;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarVec3;

/**
 * Known-size monocular range, floor-plane cross-check, and uncertainty-weighted fusion.
 */
public final class VidarGeometry {

    private static final double[] INVALID_POINT = {
            Double.NaN, Double.NaN, Double.NaN
    };

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

    public static double distanceFromGroundPlane(
            double cx, double cy, VidarCameraProfile profile, double targetHeightZ) {
        VidarGroundPlane.Intersection hit = intersectGroundPlaneAtHeight(cx, cy, profile, targetHeightZ);
        return hit.valid ? hit.slantRange : Double.NaN;
    }

    public static VidarGroundPlane.Intersection intersectGroundPlaneAtHeight(
            double cx, double cy, VidarCameraProfile profile, double targetHeightZ) {
        return intersectGroundPlaneInternal(cx, cy, profile, targetHeightZ);
    }

    public static VidarRangeEstimate buildGroundPlaneEstimate(
            double dGround, double cyPx, double horizonConfidence, boolean nearHorizon) {
        return VidarRangeFusion.buildGroundPlaneEstimate(dGround, cyPx, horizonConfidence, nearHorizon);
    }

    public static VidarRangeEstimate buildSizeEstimate(
            double dSize, double radiusPx, double circleFitQuality,
            boolean partialOcclusion, boolean touchesBoundary) {
        return VidarRangeFusion.buildSizeEstimate(
                dSize, radiusPx, circleFitQuality, partialOcclusion, touchesBoundary);
    }

    public static VidarRangeEstimate buildFloorEstimate(
            double dFloor, double cyPx, double horizonConfidence, boolean nearHorizon) {
        return VidarRangeFusion.buildFloorEstimate(dFloor, cyPx, horizonConfidence, nearHorizon);
    }

    public static VidarRangeEstimate buildPlateWidthEstimate(
            double dWidth, double pixelWidth, double rectangularity, double whiteRatio,
            boolean partialVisibility, boolean touchesRoiBoundary, double rotationPenalty) {
        return VidarRangeFusion.buildPlateWidthEstimate(
                dWidth, pixelWidth, rectangularity, whiteRatio,
                partialVisibility, touchesRoiBoundary, rotationPenalty);
    }

    public static VidarRangeResult fuseRangeWeighted(VidarRangeEstimate... estimates) {
        return VidarRangeFusion.fuseRangeWeighted(estimates);
    }

    public static VidarRangeResult fuseRangeWeighted(
            double maxRangeMismatchRatio, VidarRangeEstimate... estimates) {
        return VidarRangeFusion.fuseRangeWeighted(maxRangeMismatchRatio, estimates);
    }

    public static double robotX(double range, double bearingDeg) {
        return robotX(range, bearingDeg, null);
    }

    public static double robotY(double range, double bearingDeg) {
        return robotY(range, bearingDeg, null);
    }

    /**
     * Unit ray direction in robot frame for a full-frame pixel via {@link VidarTransformRegistry}.
     */
    public static double[] rayDirectionRobotFrame(double cx, double cy, VidarCameraProfile profile) {
        if (profile == null) {
            return INVALID_POINT.clone();
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms == null) {
            return INVALID_POINT.clone();
        }
        VidarVec3 ray = transforms.robotTCamera.transformDirection(
                transforms.intrinsics.pixelToRay(cx, cy));
        return new double[] { ray.x, ray.y, ray.z };
    }

    /** Floor contact point in robot frame from slant range along the pixel ray. */
    public static double[] floorPointInRobot(
            double cx, double cy, double slantRange, VidarCameraProfile profile) {
        if (profile == null || Double.isNaN(slantRange) || slantRange <= 0) {
            return INVALID_POINT.clone();
        }
        VidarTransformRegistry.CameraTransforms transforms =
                VidarTransformRegistry.buildForProfile(profile);
        if (transforms == null) {
            return INVALID_POINT.clone();
        }
        VidarGroundPlane.Intersection hit = VidarGroundPlane.floorPointFromSlantRange(
                transforms, cx, cy, slantRange, Double.NaN);
        if (!hit.valid) {
            return INVALID_POINT.clone();
        }
        return new double[] { hit.robotX, hit.robotY, 0.0 };
    }

    /** Ground-plane intersection at z = 0 from pixel ray only. */
    public static VidarGroundPlane.Intersection intersectGroundPlane(
            double cx, double cy, VidarCameraProfile profile) {
        return intersectGroundPlaneAtHeight(cx, cy, profile, 0.0);
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

    private static VidarGroundPlane.Intersection intersectGroundPlaneInternal(
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
        if (targetHeightZ == 0.0) {
            return VidarGroundPlane.intersect(origin, dir, Double.NaN);
        }
        return VidarGroundPlane.intersectAtPlane(origin, dir, targetHeightZ, Double.NaN);
    }

    public static VidarElementObservation fuseElementObservation(
            double cx, double cy, double radiusPx, double areaPx,
            double aspectRatio, double circularity, double fillRatio, double interiorScore,
            VidarElementDetectorType detectorType, VidarCameraProfile profile,
            String cameraName, long captureTimeNanos,
            boolean touchesBoundary, boolean partialOcclusion, double circleFitQuality) {
        return fuseElementObservation(
                cx, cy, radiusPx, areaPx, aspectRatio, circularity, fillRatio, interiorScore,
                detectorType, profile, cameraName, captureTimeNanos,
                touchesBoundary, partialOcclusion, circleFitQuality, null, null);
    }

    public static VidarElementObservation fuseElementObservation(
            double cx, double cy, double radiusPx, double areaPx,
            double aspectRatio, double circularity, double fillRatio, double interiorScore,
            VidarElementDetectorType detectorType, VidarCameraProfile profile,
            String cameraName, long captureTimeNanos,
            boolean touchesBoundary, boolean partialOcclusion, double circleFitQuality,
            VidarElementSpec element, VidarSeasonConfig season) {
        VidarElementSpec activeElement = element != null ? element
                : org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason().primaryElement();
        VidarSeasonConfig activeSeason = season != null ? season
                : org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason();
        double dSize = distanceFromSize(activeElement.diameter, profile.focalLengthPx, radiusPx);
        double dFloor = distanceFromFloor(cy, profile);
        double ballCenterZ = activeElement.diameter * 0.5;
        double dGround = distanceFromGroundPlane(cx, cy, profile, ballCenterZ);
        boolean nearHorizon = cy <= profile.horizonRowPx + 8;
        double horizonConf = VidarRangeFusion.profileHorizonConfidence(profile.horizonRowPx);

        VidarRangeEstimate sizeEst = VidarRangeFusion.buildSizeEstimate(
                dSize, radiusPx, circleFitQuality, partialOcclusion, touchesBoundary);
        VidarRangeEstimate floorEst = VidarRangeFusion.buildFloorEstimate(
                dFloor, cy, horizonConf, nearHorizon);
        VidarRangeEstimate groundEst = VidarRangeFusion.buildGroundPlaneEstimate(
                dGround, cy, horizonConf, nearHorizon);
        VidarRangeResult rangeResult = VidarRangeFusion.fuseRangeWeighted(
                activeSeason.maxRangeMismatchRatio, sizeEst, floorEst, groundEst);

        double confidence = composeElementConfidence(
                interiorScore, circularity, fillRatio, circleFitQuality,
                touchesBoundary, rangeResult, areaPx, activeElement);

        double[] robotPoint = floorPointInRobot(cx, cy, rangeResult.distance, profile);
        String elementId = activeElement.id == null ? "" : activeElement.id;

        return new VidarElementObservation(
                elementId, cameraName, captureTimeNanos,
                cx, cy, radiusPx * 2, radiusPx * 2, cx, cy, radiusPx,
                areaPx, aspectRatio, circularity, fillRatio, interiorScore,
                detectorType, confidence,
                rangeResult.distance, rangeResult.uncertainty,
                dSize, dFloor, dGround, rangeResult,
                robotPoint[0], robotPoint[1], 0);
    }

    public static VidarRangeResult fusePlateRange(
            double cx, double cy, double cyForFloor, double dWidth, double pixelWidth,
            double rectangularity, double whiteRatio, boolean partialVisibility,
            boolean touchesRoiBoundary, double rotationPenalty, boolean nearHorizon,
            double horizonConfidence, VidarCameraProfile profile, double maxRangeMismatchRatio) {
        double dFloor = distanceFromFloor(cyForFloor, profile);
        double dGround = distanceFromGroundPlane(cx, cy, profile, 0.0);
        VidarRangeEstimate widthEst = VidarRangeFusion.buildPlateWidthEstimate(
                dWidth, pixelWidth, rectangularity, whiteRatio,
                partialVisibility, touchesRoiBoundary, rotationPenalty);
        VidarRangeEstimate floorEst = VidarRangeFusion.buildFloorEstimate(
                dFloor, cyForFloor, horizonConfidence, nearHorizon);
        VidarRangeEstimate groundEst = VidarRangeFusion.buildGroundPlaneEstimate(
                dGround, cyForFloor, horizonConfidence, nearHorizon);
        return VidarRangeFusion.fuseRangeWeighted(maxRangeMismatchRatio, widthEst, floorEst, groundEst);
    }

    public static double composeElementConfidence(
            double interiorScore, double circularity, double fillRatio, double circleFitQuality,
            boolean touchesBoundary, VidarRangeResult rangeResult, double areaPx) {
        return composeElementConfidence(
                interiorScore, circularity, fillRatio, circleFitQuality,
                touchesBoundary, rangeResult, areaPx,
                org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason().primaryElement());
    }

    public static double composeElementConfidence(
            double interiorScore, double circularity, double fillRatio, double circleFitQuality,
            boolean touchesBoundary, VidarRangeResult rangeResult, double areaPx,
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
            double whiteRatio, double area, double rectangularity, double aspectRatio,
            VidarRangeResult rangeResult, double viewingAnglePenalty, double partialPenalty) {
        return composePlateConfidence(
                whiteRatio, area, rectangularity, aspectRatio, rangeResult,
                viewingAnglePenalty, partialPenalty,
                org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader.defaultSeason()
                        .plateSpec(VidarAlliance.RED));
    }

    public static double composePlateConfidence(
            double whiteRatio, double area, double rectangularity, double aspectRatio,
            VidarRangeResult rangeResult, double viewingAnglePenalty, double partialPenalty,
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
