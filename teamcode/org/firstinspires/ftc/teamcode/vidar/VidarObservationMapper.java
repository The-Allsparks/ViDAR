package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.world.VidarTrackDetection;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;

/**
 * Maps rich detections into downstream spatial DTOs.
 */
public final class VidarObservationMapper {

    private VidarObservationMapper() {}

    public static ElementFields fromElement(VidarElementObservation obs, String elementId, int occurrenceRank) {
        if (obs == null) {
            return null;
        }
        String id = elementId == null || elementId.isEmpty() ? obs.elementId : elementId;
        return new ElementFields(
                id,
                occurrenceRank,
                obs.robotX,
                obs.robotY,
                obs.range,
                obs.confidence,
                obs.cameraName,
                obs.captureTimeNanos);
    }

    public static VidarSpatialPoint toSpatialPoint(ElementFields fields) {
        if (fields == null) {
            return null;
        }
        return new VidarSpatialPoint(
                VidarSpatialPoint.Kind.ELEMENT,
                VidarSpatialPoint.Source.LIVE,
                -1,
                fields.elementId,
                fields.occurrenceRank,
                fields.robotX,
                fields.robotY,
                0,
                0,
                fields.range,
                fields.confidence,
                fields.cameraName,
                fields.captureTimeNanos);
    }

    public static VidarTrackDetection toTrackDetection(ElementFields fields) {
        if (fields == null || Double.isNaN(fields.robotX) || Double.isNaN(fields.robotY)) {
            return null;
        }
        return new VidarTrackDetection(
                VidarWorldModel.Kind.ELEMENT,
                fields.elementId,
                fields.occurrenceRank,
                fields.robotX,
                fields.robotY,
                fields.range,
                fields.confidence,
                fields.cameraName,
                fields.captureTimeNanos);
    }

    public static final class ElementFields {
        public final String elementId;
        public final int occurrenceRank;
        public final double robotX;
        public final double robotY;
        public final double range;
        public final double confidence;
        public final String cameraName;
        public final long captureTimeNanos;

        ElementFields(
                String elementId,
                int occurrenceRank,
                double robotX,
                double robotY,
                double range,
                double confidence,
                String cameraName,
                long captureTimeNanos) {
            this.elementId = elementId;
            this.occurrenceRank = occurrenceRank;
            this.robotX = robotX;
            this.robotY = robotY;
            this.range = range;
            this.confidence = confidence;
            this.cameraName = cameraName;
            this.captureTimeNanos = captureTimeNanos;
        }
    }
}
