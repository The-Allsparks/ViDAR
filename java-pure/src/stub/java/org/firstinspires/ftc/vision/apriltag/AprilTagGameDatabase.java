package org.firstinspires.ftc.vision.apriltag;

/** Minimal stub for JVM unit tests. */
public final class AprilTagGameDatabase {

    private AprilTagGameDatabase() {}

    public static AprilTagLibrary getCurrentGameTagLibrary() {
        return new AprilTagLibrary.Builder().build();
    }
}
