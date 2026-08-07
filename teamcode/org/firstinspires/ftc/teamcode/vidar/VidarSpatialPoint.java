package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.world.VidarSpatialTrack;
/**
 * Robot-frame spatial sample (+X forward, +Y left) in the active distance unit.
 *
 * <p>Produced by {@link VidarSpatial} from live detections or {@link VidarSpatialTrack} memory.
 */
public final class VidarSpatialPoint {

    public enum Source {
        /** Camera sees the target this cycle. */
        LIVE,
        /** {@link VidarSpatialTrack} — target may be occluded or off-screen. */
        REMEMBERED
    }

    public enum Kind {
        ELEMENT,
        FOE,
        ALLY
    }

    public final Kind kind;
    public final Source source;
    public final int trackId;
    /** Season element id (e.g. {@code artifact_purple}); empty for plates and unknown tracks. */
    public final String elementId;
    /** Fused rank 0 = closest/easiest; {@code -1} when not ranked. */
    public final int occurrenceRank;
    public final double robotX;
    public final double robotY;
    public final double velFieldXInPerSec;
    public final double velFieldYInPerSec;
    public final double range;
    public final double confidence;
    public final String cameraName;
    public final long captureTimeNanos;

    public VidarSpatialPoint(
            Kind kind,
            Source source,
            int trackId,
            String elementId,
            int occurrenceRank,
            double robotX,
            double robotY,
            double velFieldXInPerSec,
            double velFieldYInPerSec,
            double range,
            double confidence,
            String cameraName,
            long captureTimeNanos) {
        this.kind = kind;
        this.source = source;
        this.trackId = trackId;
        this.elementId = elementId == null ? "" : elementId;
        this.occurrenceRank = occurrenceRank;
        this.robotX = robotX;
        this.robotY = robotY;
        this.velFieldXInPerSec = velFieldXInPerSec;
        this.velFieldYInPerSec = velFieldYInPerSec;
        this.range = range;
        this.confidence = confidence;
        this.cameraName = cameraName == null ? "" : cameraName;
        this.captureTimeNanos = captureTimeNanos;
    }

    public double bearingDeg() {
        return Math.toDegrees(Math.atan2(robotY, robotX));
    }

    public double distance() {
        return Math.hypot(robotX, robotY);
    }

    public double speedFieldInPerSec() {
        return Math.hypot(velFieldXInPerSec, velFieldYInPerSec);
    }

    public boolean isValid() {
        return !Double.isNaN(robotX) && !Double.isNaN(robotY)
                && confidence > 0 && distance() > 0;
    }

    public static VidarSpatialPoint fromElement(VidarElementObservation obs) {
        return fromElement(obs, obs == null ? "" : obs.elementId, -1);
    }

    public static VidarSpatialPoint fromElement(
            VidarElementObservation obs, String elementId, int occurrenceRank) {
        if (obs == null) {
            return null;
        }
        String id = elementId == null || elementId.isEmpty() ? obs.elementId : elementId;
        return new VidarSpatialPoint(
                Kind.ELEMENT,
                Source.LIVE,
                -1,
                id,
                occurrenceRank,
                obs.robotX,
                obs.robotY,
                0,
                0,
                obs.range,
                obs.confidence,
                obs.cameraName,
                obs.captureTimeNanos);
    }

    public VidarSpatialPoint withOccurrenceRank(int occurrenceRank) {
        return new VidarSpatialPoint(
                kind,
                source,
                trackId,
                elementId,
                occurrenceRank,
                robotX,
                robotY,
                velFieldXInPerSec,
                velFieldYInPerSec,
                range,
                confidence,
                cameraName,
                captureTimeNanos);
    }

    public static VidarSpatialPoint fromPlate(VidarPlateObservation plate, Kind kind) {
        if (plate == null) {
            return null;
        }
        return new VidarSpatialPoint(
                kind,
                Source.LIVE,
                -1,
                "",
                -1,
                plate.robotX,
                plate.robotY,
                0,
                0,
                plate.range,
                plate.confidence,
                plate.cameraName,
                plate.captureTimeNanos);
    }

    public static VidarSpatialPoint fromTrack(VidarSpatialTrack track) {
        if (track == null) {
            return null;
        }
        Kind kind;
        switch (track.kind) {
            case FOE:
                kind = Kind.FOE;
                break;
            case ALLY:
                kind = Kind.ALLY;
                break;
            case ELEMENT:
            default:
                kind = Kind.ELEMENT;
                break;
        }
        return new VidarSpatialPoint(
                kind,
                Source.REMEMBERED,
                track.trackId,
                track.elementId,
                -1,
                track.robotX,
                track.robotY,
                track.velFieldXInPerSec,
                track.velFieldYInPerSec,
                track.range,
                track.confidence,
                track.cameraName,
                track.captureTimeNanos);
    }
}
