package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Timestamped field pose suitable for fusion (WPILib {@code addVisionMeasurement} style).
 */
public final class VidarVisionMeasurement {

    public final Pose2D fieldPose;
    public final long timestampNanos;
    /** Higher = more trust (e.g. from decode pixel count). */
    public final double trust;
    public final int tagId;

    public VidarVisionMeasurement(Pose2D fieldPose, long timestampNanos, double trust, int tagId) {
        this.fieldPose = fieldPose;
        this.timestampNanos = timestampNanos;
        this.trust = trust;
        this.tagId = tagId;
    }
}
