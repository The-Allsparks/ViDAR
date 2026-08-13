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
}
