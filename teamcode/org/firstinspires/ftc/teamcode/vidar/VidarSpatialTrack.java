package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Persistent spatial track with field-frame velocity and predict/update helpers.
 */
public final class VidarSpatialTrack {

    public enum MotionClass {
        UNKNOWN,
        STATIC,
        MOVING
    }

    public final int trackId;
    public final VidarWorldModel.Kind kind;
    public final String elementId;
    public final double robotX;
    public final double robotY;
    public final Double fieldXIn;
    public final Double fieldYIn;
    public final double velFieldXInPerSec;
    public final double velFieldYInPerSec;
    public final double range;
    public final double confidence;
    public final String cameraName;
    public final long lastSeenNanos;
    public final long lastUpdateNanos;
    public final long captureTimeNanos;
    public final int missCount;
    public final int staticStableFrames;
    public final MotionClass motionClass;

    VidarSpatialTrack(
            int trackId,
            VidarWorldModel.Kind kind,
            String elementId,
            double robotX,
            double robotY,
            Double fieldXIn,
            Double fieldYIn,
            double velFieldXInPerSec,
            double velFieldYInPerSec,
            double range,
            double confidence,
            String cameraName,
            long lastSeenNanos,
            long lastUpdateNanos,
            long captureTimeNanos,
            int missCount,
            int staticStableFrames,
            MotionClass motionClass) {
        this.trackId = trackId;
        this.kind = kind;
        this.elementId = elementId == null ? "" : elementId;
        this.robotX = robotX;
        this.robotY = robotY;
        this.fieldXIn = fieldXIn;
        this.fieldYIn = fieldYIn;
        this.velFieldXInPerSec = velFieldXInPerSec;
        this.velFieldYInPerSec = velFieldYInPerSec;
        this.range = range;
        this.confidence = confidence;
        this.cameraName = cameraName == null ? "" : cameraName;
        this.lastSeenNanos = lastSeenNanos;
        this.lastUpdateNanos = lastUpdateNanos;
        this.captureTimeNanos = captureTimeNanos;
        this.missCount = missCount;
        this.staticStableFrames = staticStableFrames;
        this.motionClass = motionClass == null ? MotionClass.UNKNOWN : motionClass;
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

    /** Predict track to {@code nowNanos} and reproject into the current robot frame. */
    VidarSpatialTrack predict(long nowNanos, Pose2D fieldPose) {
        double dtSec = dtSeconds(lastUpdateNanos, nowNanos);
        if (fieldXIn != null && fieldYIn != null && fieldPose != null) {
            double predFieldX = fieldXIn;
            double predFieldY = fieldYIn;
            if (motionClass == MotionClass.MOVING) {
                predFieldX = fieldXIn + velFieldXInPerSec * dtSec;
                predFieldY = fieldYIn + velFieldYInPerSec * dtSec;
            }
            double[] robot = VidarCoordinateFrames.fieldToRobot(predFieldX, predFieldY, fieldPose);
            return copyWith(robot[0], robot[1], predFieldX, predFieldY,
                    velFieldXInPerSec, velFieldYInPerSec, confidence * 0.99,
                    lastSeenNanos, lastUpdateNanos, missCount, staticStableFrames, motionClass);
        }
        return copyWith(robotX, robotY, fieldXIn, fieldYIn,
                velFieldXInPerSec, velFieldYInPerSec, confidence * 0.99,
                lastSeenNanos, lastUpdateNanos, missCount, staticStableFrames, motionClass);
    }

    /** Gate distance in robot frame to a detection (uses predicted robot position). */
    double gateDistanceTo(VidarTrackDetection detection) {
        return Math.hypot(robotX - detection.robotX, robotY - detection.robotY);
    }

    static VidarSpatialTrack birth(
            int trackId,
            VidarTrackDetection detection,
            long nowNanos,
            Pose2D fieldPose) {
        Double fieldX = null;
        Double fieldY = null;
        if (fieldPose != null) {
            double[] field = VidarCoordinateFrames.robotToField(
                    detection.robotX, detection.robotY, fieldPose);
            if (!Double.isNaN(field[0]) && !Double.isNaN(field[1])) {
                fieldX = field[0];
                fieldY = field[1];
            }
        }
        MotionClass motion = MotionClass.UNKNOWN;
        return new VidarSpatialTrack(
                trackId,
                detection.kind,
                detection.elementId,
                detection.robotX,
                detection.robotY,
                fieldX,
                fieldY,
                0,
                0,
                detection.range,
                detection.confidence,
                detection.cameraName,
                nowNanos,
                nowNanos,
                detection.captureTimeNanos,
                0,
                0,
                motion);
    }

    VidarSpatialTrack updateFromDetection(
            VidarTrackDetection detection,
            long nowNanos,
            Pose2D fieldPose,
            double dtSec) {
        Double newFieldX = fieldXIn;
        Double newFieldY = fieldYIn;
        if (fieldPose != null) {
            double[] field = VidarCoordinateFrames.robotToField(
                    detection.robotX, detection.robotY, fieldPose);
            if (!Double.isNaN(field[0]) && !Double.isNaN(field[1])) {
                newFieldX = field[0];
                newFieldY = field[1];
            }
        }

        double alpha = VidarConfig.WORLD_TRACK_POS_ALPHA;
        double mergedRobotX = alpha * detection.robotX + (1.0 - alpha) * robotX;
        double mergedRobotY = alpha * detection.robotY + (1.0 - alpha) * robotY;
        if (newFieldX != null && fieldXIn != null && newFieldY != null && fieldYIn != null) {
            newFieldX = alpha * newFieldX + (1.0 - alpha) * fieldXIn;
            newFieldY = alpha * newFieldY + (1.0 - alpha) * fieldYIn;
        }

        double newVelX = velFieldXInPerSec;
        double newVelY = velFieldYInPerSec;
        if (newFieldX != null && fieldXIn != null && newFieldY != null && fieldYIn != null
                && dtSec >= VidarConfig.WORLD_TRACK_MIN_DT_SEC
                && dtSec <= VidarConfig.WORLD_TRACK_MAX_DT_SEC) {
            double rawVx = (newFieldX - fieldXIn) / dtSec;
            double rawVy = (newFieldY - fieldYIn) / dtSec;
            double jump = Math.hypot(newFieldX - fieldXIn, newFieldY - fieldYIn);
            if (jump <= gateRadius() * 2.0) {
                double velAlpha = VidarConfig.WORLD_TRACK_VEL_ALPHA;
                newVelX = velAlpha * rawVx + (1.0 - velAlpha) * velFieldXInPerSec;
                newVelY = velAlpha * rawVy + (1.0 - velAlpha) * velFieldYInPerSec;
            }
        }

        int newStaticFrames = staticStableFrames;
        MotionClass newMotion = motionClass;
        if (kind == VidarWorldModel.Kind.ELEMENT) {
            if (Math.hypot(newVelX, newVelY) < VidarConfig.WORLD_TRACK_STATIC_SPEED_IN_PER_SEC) {
                newStaticFrames++;
            } else {
                newStaticFrames = 0;
            }
            if (newStaticFrames >= VidarConfig.WORLD_TRACK_STATIC_FRAMES) {
                newMotion = MotionClass.STATIC;
                newVelX = 0;
                newVelY = 0;
            }
        } else if (Math.hypot(newVelX, newVelY) >= VidarConfig.WORLD_TRACK_MOVING_SPEED_IN_PER_SEC) {
            newMotion = MotionClass.MOVING;
        }

        double mergedConf = Math.max(confidence, detection.confidence);
        double mergedRange = Double.isNaN(detection.range) ? range : detection.range;
        return new VidarSpatialTrack(
                trackId, kind, elementId.isEmpty() ? detection.elementId : elementId,
                mergedRobotX, mergedRobotY, newFieldX, newFieldY,
                newVelX, newVelY, mergedRange, mergedConf, detection.cameraName,
                nowNanos, nowNanos, detection.captureTimeNanos,
                0, newStaticFrames, newMotion);
    }

    VidarSpatialTrack coast(long nowNanos, Pose2D fieldPose) {
        VidarSpatialTrack predicted = predict(nowNanos, fieldPose);
        return predicted.copyWith(predicted.robotX, predicted.robotY, predicted.fieldXIn, predicted.fieldYIn,
                predicted.velFieldXInPerSec, predicted.velFieldYInPerSec,
                predicted.confidence * 0.96, predicted.lastSeenNanos, predicted.lastUpdateNanos,
                missCount + 1, staticStableFrames, motionClass);
    }

    double gateRadius() {
        switch (kind) {
            case FOE:
            case ALLY:
                return VidarConfig.WORLD_TRACK_GATE_RADIUS_FOE_IN;
            case ELEMENT:
            default:
                return VidarConfig.WORLD_TRACK_GATE_RADIUS_IN;
        }
    }

    private VidarSpatialTrack copyWith(
            double newRobotX,
            double newRobotY,
            Double newFieldX,
            Double newFieldY,
            double newVelX,
            double newVelY,
            double newConfidence,
            long newLastSeen,
            long newLastUpdate,
            int newMissCount,
            int newStaticFrames,
            MotionClass newMotion) {
        return new VidarSpatialTrack(
                trackId, kind, elementId, newRobotX, newRobotY, newFieldX, newFieldY,
                newVelX, newVelY, range, newConfidence, cameraName,
                newLastSeen, newLastUpdate, captureTimeNanos,
                newMissCount, newStaticFrames, newMotion);
    }

    static double dtSeconds(long fromNanos, long toNanos) {
        if (fromNanos <= 0 || toNanos <= fromNanos) {
            return 0;
        }
        return (toNanos - fromNanos) / 1_000_000_000.0;
    }
}
