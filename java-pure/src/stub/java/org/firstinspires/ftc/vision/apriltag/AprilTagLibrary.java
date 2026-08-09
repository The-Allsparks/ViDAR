package org.firstinspires.ftc.vision.apriltag;

/** Minimal stub for JVM unit tests. */
public final class AprilTagLibrary {

    private AprilTagLibrary() {}

    public static final class Builder {
        public Builder addTag(AprilTagMetadata metadata) {
            return this;
        }

        public AprilTagLibrary build() {
            return new AprilTagLibrary();
        }
    }
}
