package org.firstinspires.ftc.teamcode.vidar;

/**
 * One game piece or plate in the <b>current</b> robot frame after latency compensation.
 */
public final class VidarCorrectedPoint {

    public enum Kind {
        ELEMENT,
        PLATE,
        PLATE_FOE,
        PLATE_ALLY
    }

    public final Kind kind;
    public final double robotX;
    public final double robotY;
    public final double confidence;
    public final double range;
    public final long captureTimeNanos;
    public final double ageMs;
    public final String cameraName;
    public final VidarElementObservation elementSource;
    public final VidarPlateObservation plateSource;

    private VidarCorrectedPoint(
            Kind kind,
            double robotX,
            double robotY,
            double confidence,
            double range,
            long captureTimeNanos,
            double ageMs,
            String cameraName,
            VidarElementObservation elementSource,
            VidarPlateObservation plateSource) {
        this.kind = kind;
        this.robotX = robotX;
        this.robotY = robotY;
        this.confidence = confidence;
        this.range = range;
        this.captureTimeNanos = captureTimeNanos;
        this.ageMs = ageMs;
        this.cameraName = cameraName;
        this.elementSource = elementSource;
        this.plateSource = plateSource;
    }

    static VidarCorrectedPoint element(
            VidarElementObservation obs,
            double robotX,
            double robotY,
            long queryTimeNanos) {
        if (obs == null || Double.isNaN(robotX) || Double.isNaN(robotY)) {
            return null;
        }
        return new VidarCorrectedPoint(
                Kind.ELEMENT,
                robotX,
                robotY,
                obs.confidence,
                obs.range,
                obs.captureTimeNanos,
                (queryTimeNanos - obs.captureTimeNanos) / 1_000_000.0,
                obs.cameraName,
                obs,
                null);
    }

    static VidarCorrectedPoint plate(VidarPlateObservation plate, Kind kind,
                                   double robotX, double robotY, long queryTimeNanos) {
        if (plate == null || Double.isNaN(robotX) || Double.isNaN(robotY)) {
            return null;
        }
        return new VidarCorrectedPoint(
                kind,
                robotX,
                robotY,
                plate.confidence,
                plate.range,
                plate.captureTimeNanos,
                (queryTimeNanos - plate.captureTimeNanos) / 1_000_000.0,
                plate.cameraName,
                null,
                plate);
    }

    public double bearingDeg() {
        return Math.toDegrees(Math.atan2(robotY, robotX));
    }

    public double distance() {
        return Math.hypot(robotX, robotY);
    }
}
