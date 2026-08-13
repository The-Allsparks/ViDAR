package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.fusion.VidarFusionEngine;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Ring buffer of odometry samples keyed by {@link System#nanoTime()}, for latency-compensated
 * vision correction (same role as timestamps in WPILib {@code addVisionMeasurement}).
 *
 * <p>Record once per OpMode loop — before or after {@link VidarFusionEngine#update()}:
 * <pre>{@code
 * odomHistory.record(odom.getPose());
 * VidarObservationFrame raw = vision.update();
 * VidarCorrectedFrame now = raw.toRobotNow(odomHistory, odom.getPose());
 * }</pre>
 */
public final class VidarOdomHistory {

    private static final class Sample {
        final long timeNanos;
        final Pose2D pose;

        Sample(long timeNanos, Pose2D pose) {
            this.timeNanos = timeNanos;
            this.pose = pose;
        }
    }

    private final Sample[] ring;
    private int size;
    private int head;

    public VidarOdomHistory() {
        this(128);
    }

    public VidarOdomHistory(int capacity) {
        ring = new Sample[Math.max(8, capacity)];
    }

    /** Append current odometry with {@link System#nanoTime()}. */
    public void record(Pose2D odom) {
        record(System.nanoTime(), odom);
    }

    public synchronized void record(long timeNanos, Pose2D odom) {
        if (odom == null || timeNanos <= 0) {
            return;
        }
        if (size > 0 && timeNanos <= ring[(head + size - 1) % ring.length].timeNanos) {
            return;
        }
        if (size == ring.length) {
            head = (head + 1) % ring.length;
            size--;
        }
        int index = (head + size) % ring.length;
        ring[index] = new Sample(timeNanos, odom);
        size++;
    }

    /**
     * Interpolate odometry at {@code captureTimeNanos}. Uses bracketing samples when possible;
     * otherwise nearest available sample.
     */
    public synchronized Pose2D at(long captureTimeNanos) {
        if (size == 0 || captureTimeNanos <= 0) {
            return null;
        }
        Sample oldest = ring[head];
        Sample newest = ring[(head + size - 1) % ring.length];
        if (captureTimeNanos <= oldest.timeNanos) {
            return oldest.pose;
        }
        if (captureTimeNanos >= newest.timeNanos) {
            return newest.pose;
        }
        Sample before = oldest;
        Sample after = newest;
        for (int i = 0; i < size - 1; i++) {
            Sample a = ring[(head + i) % ring.length];
            Sample b = ring[(head + i + 1) % ring.length];
            if (a.timeNanos <= captureTimeNanos && captureTimeNanos <= b.timeNanos) {
                before = a;
                after = b;
                break;
            }
        }
        double t = (double) (captureTimeNanos - before.timeNanos)
                / (double) (after.timeNanos - before.timeNanos);
        return interpolate(before.pose, after.pose, t);
    }

    public synchronized int size() {
        return size;
    }

    public synchronized void clear() {
        size = 0;
        head = 0;
    }

    private static Pose2D interpolate(Pose2D a, Pose2D b, double t) {
        t = Math.max(0, Math.min(1, t));
        double x = a.getX(DistanceUnit.INCH)
                + t * (b.getX(DistanceUnit.INCH) - a.getX(DistanceUnit.INCH));
        double y = a.getY(DistanceUnit.INCH)
                + t * (b.getY(DistanceUnit.INCH) - a.getY(DistanceUnit.INCH));
        double ha = a.getHeading(AngleUnit.DEGREES);
        double hb = b.getHeading(AngleUnit.DEGREES);
        double dh = hb - ha;
        while (dh > 180) dh -= 360;
        while (dh < -180) dh += 360;
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, ha + t * dh);
    }
}
