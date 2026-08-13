package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Minimal stub for JVM unit tests. */
public final class AprilTagMetadata {
    public final int id;
    public final String name;
    public final double size;
    public final DistanceUnit unit;

    public AprilTagMetadata(int id, String name, double size, DistanceUnit unit) {
        this.id = id;
        this.name = name;
        this.size = size;
        this.unit = unit;
    }
}
