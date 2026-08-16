package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;

/**
 * Narrow read-only fusion surface for snapshots and world-model updates.
 */
public interface VidarVisionFusion {

    VidarRankedElementFrame getRankedElements();

    VidarPlateObservation getBestAlly();

    VidarPlateObservation getBestFoe();

    Pose2D getFusedFieldPose();

    Pose2D getFieldPoseForMotionTracking();

    /** Gated tag fix re-propagated to current odom; null until a pose gate accepts a fix. */
    Pose2D getGatedTagCorrectedFieldPoseNow();

    /** {@link System#nanoTime()} of last gate-accepted correction (0 = none). */
    long lastTagCorrectionNanos();
}
