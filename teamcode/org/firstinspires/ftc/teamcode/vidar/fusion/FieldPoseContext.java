package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarVisionFusion;

import java.util.function.Supplier;

/**
 * Single owner for field-pose suppliers used by vision fusion and world-model tracks.
 */
public final class FieldPoseContext {

    private Supplier<Pose2D> odomSupplier;
    private Supplier<Pose2D> externalFieldPoseSupplier;
    private VidarVisionFusion vision;

    public FieldPoseContext(Supplier<Pose2D> odomSupplier) {
        this.odomSupplier = odomSupplier;
    }

    public void bindVision(VidarVisionFusion vision) {
        this.vision = vision;
    }

    /** Rebind odom when a new OpMode recreates {@code VidarSpatial} against a live runtime. */
    public void setOdomSupplier(Supplier<Pose2D> supplier) {
        this.odomSupplier = supplier;
    }

    public void setExternalFieldPoseSupplier(Supplier<Pose2D> supplier) {
        this.externalFieldPoseSupplier = supplier;
    }

    public Supplier<Pose2D> worldTrackFieldPoseSupplier() {
        return this::fieldPoseForWorldTracks;
    }

    public Pose2D fieldPoseForWorldTracks() {
        if (externalFieldPoseSupplier != null) {
            Pose2D external = externalFieldPoseSupplier.get();
            if (external != null) {
                return external;
            }
        }
        return vision == null ? null : vision.getFieldPoseForMotionTracking();
    }

    public Pose2D fieldPoseForSnapshot() {
        if (externalFieldPoseSupplier != null) {
            Pose2D external = externalFieldPoseSupplier.get();
            if (external != null) {
                return external;
            }
        }
        return vision == null ? null : vision.getFusedFieldPose();
    }

    public Supplier<Pose2D> odomSupplier() {
        return odomSupplier;
    }
}
