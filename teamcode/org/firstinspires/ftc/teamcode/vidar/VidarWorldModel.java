package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

/**
 * Short-term spatial memory with motion-corrected robot-relative tracks.
 * Stores field coordinates when pose is available; otherwise applies odom delta transform.
 */
public class VidarWorldModel {

    public enum Kind {
        BALL,
        ALLY,
        FOE
    }

    public static final class Track {
        public final Kind kind;
        public final double robotXIn;
        public final double robotYIn;
        public final Double fieldXIn;
        public final Double fieldYIn;
        public final double rangeIn;
        public final double confidence;
        public final String cameraName;
        public final long lastSeenNanos;
        public final long captureTimeNanos;
        public final Pose2D captureRobotPose;

        Track(Kind kind, double robotXIn, double robotYIn, Double fieldXIn, Double fieldYIn,
              double rangeIn, double confidence, String cameraName,
              long lastSeenNanos, long captureTimeNanos, Pose2D captureRobotPose) {
            this.kind = kind;
            this.robotXIn = robotXIn;
            this.robotYIn = robotYIn;
            this.fieldXIn = fieldXIn;
            this.fieldYIn = fieldYIn;
            this.rangeIn = rangeIn;
            this.confidence = confidence;
            this.cameraName = cameraName;
            this.lastSeenNanos = lastSeenNanos;
            this.captureTimeNanos = captureTimeNanos;
            this.captureRobotPose = captureRobotPose;
        }

        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(robotYIn, robotXIn));
        }

        public double distanceIn() {
            return Math.hypot(robotXIn, robotYIn);
        }

        Track withRobotPosition(double x, double y, double confidence, long nowNanos) {
            return new Track(kind, x, y, fieldXIn, fieldYIn, rangeIn,
                    confidence, cameraName, nowNanos, captureTimeNanos, captureRobotPose);
        }

        Track withFieldPosition(double fieldX, double fieldY, double robotX, double robotY,
                                double confidence, long nowNanos) {
            return new Track(kind, robotX, robotY, fieldX, fieldY, rangeIn,
                    confidence, cameraName, nowNanos, captureTimeNanos, captureRobotPose);
        }
    }

    private final List<Track> tracks = new ArrayList<>();
    private Pose2D lastRobotPose;
    private Pose2D lastFieldPose;
    private final Supplier<Pose2D> odomSupplier;
    private final Supplier<Pose2D> fieldPoseSupplier;

    public VidarWorldModel() {
        this(null, null);
    }

    public VidarWorldModel(Supplier<Pose2D> odomSupplier, Supplier<Pose2D> fieldPoseSupplier) {
        this.odomSupplier = odomSupplier;
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    public void update(VidarMultiVision vision, long nowNanos) {
        applyMotionCorrection(nowNanos);
        decay(nowNanos);

        if (vision == null) {
            return;
        }

        VidarBallObservation ball = vision.getBestElement();
        if (ball != null && ball.confidence >= VidarConfig.MIN_BALL_CONFIDENCE) {
            upsert(Kind.BALL, ball.robotXIn, ball.robotYIn, ball.rangeIn,
                    ball.confidence, ball.cameraName, ball.captureTimeNanos, nowNanos);
        }

        VidarPlateObservation foe = vision.getBestFoe();
        if (foe != null && foe.confidence >= VidarConfig.MIN_PLATE_CONFIDENCE) {
            upsert(Kind.FOE, foe.robotXIn, foe.robotYIn, foe.rangeIn,
                    foe.confidence, foe.cameraName, foe.captureTimeNanos, nowNanos);
        }

        VidarPlateObservation ally = vision.getBestAlly();
        if (ally != null && ally.confidence >= VidarConfig.MIN_PLATE_CONFIDENCE) {
            upsert(Kind.ALLY, ally.robotXIn, ally.robotYIn, ally.rangeIn,
                    ally.confidence, ally.cameraName, ally.captureTimeNanos, nowNanos);
        }
    }

    private void applyMotionCorrection(long nowNanos) {
        Pose2D robotNow = odomSupplier == null ? null : odomSupplier.get();
        Pose2D fieldNow = fieldPoseSupplier == null ? null : fieldPoseSupplier.get();

        if (robotNow != null && lastRobotPose != null) {
            VidarMotionTransform delta = VidarMotionTransform.fromOdomDelta(
                    lastRobotPose.getX(DistanceUnit.INCH),
                    lastRobotPose.getY(DistanceUnit.INCH),
                    lastRobotPose.getHeading(AngleUnit.DEGREES),
                    robotNow.getX(DistanceUnit.INCH),
                    robotNow.getY(DistanceUnit.INCH),
                    robotNow.getHeading(AngleUnit.DEGREES));

            List<Track> updated = new ArrayList<>();
            for (Track track : tracks) {
                if (fieldNow != null && track.fieldXIn != null && track.fieldYIn != null) {
                    double[] robot = fieldToRobot(track.fieldXIn, track.fieldYIn, fieldNow);
                    updated.add(track.withRobotPosition(robot[0], robot[1],
                            track.confidence * 0.98, nowNanos));
                } else {
                    double[] pt = delta.transformPoint(track.robotXIn, track.robotYIn);
                    updated.add(track.withRobotPosition(pt[0], pt[1],
                            track.confidence * 0.97, nowNanos));
                }
            }
            tracks.clear();
            tracks.addAll(updated);
        }

        lastRobotPose = robotNow;
        lastFieldPose = fieldNow;
    }

    private static double[] fieldToRobot(double fieldX, double fieldY, Pose2D robotFieldPose) {
        double rx = robotFieldPose.getX(DistanceUnit.INCH);
        double ry = robotFieldPose.getY(DistanceUnit.INCH);
        double heading = Math.toRadians(robotFieldPose.getHeading(AngleUnit.DEGREES));
        double dx = fieldX - rx;
        double dy = fieldY - ry;
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        return new double[] {
                dx * cos - dy * sin,
                dx * sin + dy * cos
        };
    }

    private void upsert(Kind kind, double x, double y, double rangeIn,
                        double confidence, String cameraName, long captureNanos, long nowNanos) {
        if (Double.isNaN(x) || Double.isNaN(y)) {
            return;
        }

        for (Track track : tracks) {
            if (track.kind != kind) {
                continue;
            }
            if (captureNanos > 0 && track.captureTimeNanos > captureNanos) {
                return;
            }
            double dist = Math.hypot(track.robotXIn - x, track.robotYIn - y);
            if (dist <= VidarConfig.WORLD_MERGE_RADIUS_IN) {
                tracks.remove(track);
                double mergedConf = Math.max(track.confidence, confidence);
                double mergedX = 0.5 * (track.robotXIn + x);
                double mergedY = 0.5 * (track.robotYIn + y);
                double mergedRange = Double.isNaN(rangeIn) ? track.rangeIn : rangeIn;
                Double fieldX = track.fieldXIn;
                Double fieldY = track.fieldYIn;
                if (lastFieldPose != null) {
                    fieldX = lastFieldPose.getX(DistanceUnit.INCH) + mergedX;
                    fieldY = lastFieldPose.getY(DistanceUnit.INCH) + mergedY;
                }
                tracks.add(new Track(kind, mergedX, mergedY, fieldX, fieldY, mergedRange,
                        mergedConf, cameraName, nowNanos, captureNanos, lastRobotPose));
                return;
            }
        }

        Double fieldX = null;
        Double fieldY = null;
        if (lastFieldPose != null) {
            fieldX = lastFieldPose.getX(DistanceUnit.INCH) + x;
            fieldY = lastFieldPose.getY(DistanceUnit.INCH) + y;
        }
        tracks.add(new Track(kind, x, y, fieldX, fieldY, rangeIn, confidence,
                cameraName, nowNanos, captureNanos, lastRobotPose));
    }

    public void decay(long nowNanos) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            Track track = it.next();
            double ageSec = (nowNanos - track.lastSeenNanos) / 1_000_000_000.0;
            if (ageSec > ttlSeconds(track.kind)) {
                it.remove();
            }
        }
    }

    private static double ttlSeconds(Kind kind) {
        switch (kind) {
            case BALL:
                return VidarConfig.WORLD_BALL_TTL_SEC;
            case FOE:
                return VidarConfig.WORLD_FOE_TTL_SEC;
            case ALLY:
            default:
                return VidarConfig.WORLD_ALLY_TTL_SEC;
        }
    }

    public List<Track> getTracks() {
        return new ArrayList<>(tracks);
    }

    public List<Track> getTracks(Kind kind) {
        List<Track> out = new ArrayList<>();
        for (Track track : tracks) {
            if (track.kind == kind) {
                out.add(track);
            }
        }
        return out;
    }

    public Track nearestBall() {
        return nearest(Kind.BALL);
    }

    public Track nearestFoe() {
        return nearest(Kind.FOE);
    }

    private Track nearest(Kind kind) {
        Track best = null;
        double bestDist = Double.MAX_VALUE;
        for (Track track : tracks) {
            if (track.kind != kind) {
                continue;
            }
            double dist = track.distanceIn();
            if (dist < bestDist) {
                bestDist = dist;
                best = track;
            }
        }
        return best;
    }

    public boolean intakeBlocked() {
        for (Track track : tracks) {
            if (track.kind != Kind.FOE) {
                continue;
            }
            if (track.distanceIn() > VidarConfig.WORLD_BLOCK_RANGE_IN) {
                continue;
            }
            if (Math.abs(track.bearingDeg()) <= VidarConfig.WORLD_BLOCK_CONE_DEG) {
                return true;
            }
        }
        return false;
    }

    public int trackCount() {
        return tracks.size();
    }
}
