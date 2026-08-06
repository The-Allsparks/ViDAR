package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarCoordinateFrames;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarOdomHistory;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarRangeResult;
import org.firstinspires.ftc.teamcode.vidar.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarTagScoutObservation;

/**
 * Spatial metadata helpers for observations — image vs robot vs field frames.
 */
public final class VidarObservationSpatial {

    public static final class SpatialInfo {
        public final String cameraName;
        public final VidarFrameId sourceFrame;
        public final long captureTimeNanos;
        public final double observationAgeMs;
        public final VidarSpatialDepthKind depthKind;
        public final double positionalUncertainty;
        public final double confidence;

        SpatialInfo(
                String cameraName,
                VidarFrameId sourceFrame,
                long captureTimeNanos,
                double observationAgeMs,
                VidarSpatialDepthKind depthKind,
                double positionalUncertainty,
                double confidence) {
            this.cameraName = cameraName;
            this.sourceFrame = sourceFrame;
            this.captureTimeNanos = captureTimeNanos;
            this.observationAgeMs = observationAgeMs;
            this.depthKind = depthKind;
            this.positionalUncertainty = positionalUncertainty;
            this.confidence = confidence;
        }
    }

    private VidarObservationSpatial() {}

    public static SpatialInfo fromElement(VidarElementObservation obs) {
        if (obs == null) {
            return null;
        }
        VidarSpatialDepthKind depth = depthKindFromRange(obs.rangeResult);
        return new SpatialInfo(
                obs.cameraName,
                VidarFrameId.ROBOT,
                obs.captureTimeNanos,
                VidarCoordinateFrames.observationAgeMs(obs.captureTimeNanos),
                depth,
                obs.rangeUncertainty,
                obs.confidence);
    }

    public static SpatialInfo fromPlate(VidarPlateObservation plate) {
        if (plate == null) {
            return null;
        }
        VidarSpatialDepthKind depth = depthKindFromRange(plate.rangeResult);
        return new SpatialInfo(
                plate.cameraName,
                VidarFrameId.ROBOT,
                plate.captureTimeNanos,
                VidarCoordinateFrames.observationAgeMs(plate.captureTimeNanos),
                depth,
                plate.rangeUncertainty,
                plate.confidence);
    }

    public static SpatialInfo fromTag(VidarTagObservation tag, String cameraName) {
        if (tag == null) {
            return null;
        }
        return new SpatialInfo(
                cameraName,
                VidarFrameId.FIELD,
                tag.captureTimeNanos,
                VidarCoordinateFrames.observationAgeMs(tag.captureTimeNanos),
                VidarSpatialDepthKind.MEASURED,
                Double.NaN,
                tag.decodePixels > 0 ? Math.min(1.0, tag.decodePixels / 500.0) : 0.5);
    }

    public static SpatialInfo fromTagScout(VidarTagScoutObservation scout) {
        if (scout == null) {
            return null;
        }
        return new SpatialInfo(
                scout.cameraName,
                VidarFrameId.ROBOT,
                scout.captureTimeNanos,
                VidarCoordinateFrames.observationAgeMs(scout.captureTimeNanos),
                VidarSpatialDepthKind.BEARING_ONLY,
                Double.NaN,
                scout.scoutConfidence);
    }

    private static VidarSpatialDepthKind depthKindFromRange(VidarRangeResult range) {
        if (range == null || !range.isValid()) {
            return VidarSpatialDepthKind.UNAVAILABLE;
        }
        if (range.source0 != null
                && range.source0.source == org.firstinspires.ftc.teamcode.vidar.VidarRangeEstimate.Source.PLATE_WIDTH) {
            return VidarSpatialDepthKind.INFERRED;
        }
        return VidarSpatialDepthKind.INFERRED;
    }
}
