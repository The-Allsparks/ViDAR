package org.firstinspires.ftc.teamcode.vidar;

/**
 * Normalized spatial detection for one vision cycle — input to {@link VidarTrackAssociator}.
 */
public final class VidarTrackDetection {

    public final VidarWorldModel.Kind kind;
    public final String elementId;
    public final int occurrenceRank;
    public final double robotX;
    public final double robotY;
    public final double range;
    public final double confidence;
    public final String cameraName;
    public final long captureTimeNanos;

    public VidarTrackDetection(
            VidarWorldModel.Kind kind,
            String elementId,
            int occurrenceRank,
            double robotX,
            double robotY,
            double range,
            double confidence,
            String cameraName,
            long captureTimeNanos) {
        this.kind = kind;
        this.elementId = elementId == null ? "" : elementId;
        this.occurrenceRank = occurrenceRank;
        this.robotX = robotX;
        this.robotY = robotY;
        this.range = range;
        this.confidence = confidence;
        this.cameraName = cameraName == null ? "" : cameraName;
        this.captureTimeNanos = captureTimeNanos;
    }

    public static VidarTrackDetection fromElement(VidarElementObservation obs) {
        return fromElement(obs, obs == null ? "" : obs.elementId, -1);
    }

    public static VidarTrackDetection fromElement(
            VidarElementObservation obs, String elementId, int occurrenceRank) {
        if (obs == null || Double.isNaN(obs.robotX) || Double.isNaN(obs.robotY)) {
            return null;
        }
        String id = elementId == null || elementId.isEmpty() ? obs.elementId : elementId;
        return new VidarTrackDetection(
                VidarWorldModel.Kind.ELEMENT,
                id,
                occurrenceRank,
                obs.robotX,
                obs.robotY,
                obs.range,
                obs.confidence,
                obs.cameraName,
                obs.captureTimeNanos);
    }

    public static VidarTrackDetection fromPlate(
            VidarPlateObservation plate, VidarWorldModel.Kind kind) {
        if (plate == null || Double.isNaN(plate.robotX) || Double.isNaN(plate.robotY)) {
            return null;
        }
        return new VidarTrackDetection(
                kind,
                "",
                -1,
                plate.robotX,
                plate.robotY,
                plate.range,
                plate.confidence,
                plate.cameraName,
                plate.captureTimeNanos);
    }

    public double distance() {
        return Math.hypot(robotX, robotY);
    }

    public VidarTrackDetection withOccurrenceRank(int occurrenceRank) {
        return new VidarTrackDetection(
                kind,
                elementId,
                occurrenceRank,
                robotX,
                robotY,
                range,
                confidence,
                cameraName,
                captureTimeNanos);
    }
}
