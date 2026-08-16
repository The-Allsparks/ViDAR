package org.firstinspires.ftc.teamcode.vidar.integration;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarMotionCorrection;

/**
 * Sparse ViDAR → Pedro pose correction with latency compensation.
 *
 * <p><b>Why not {@code spatial.fieldPose()} alone?</b> When Pedro is wired via
 * {@code setFieldPoseSupplier}, {@code fieldPose()} is the live Pedro pose — not the fused tag
 * anchor. Gating novelty on that value re-fires {@code setPose} as the robot drives.
 *
 * <p><b>Correct chain:</b>
 * <ol>
 *   <li>Tag stores {@code fieldPoseAtCapture}</li>
 *   <li>Fusion accepts a gated fix → {@code lastTagCorrectionNanos()} advances;
 *       odom-at-fuse is stamped once</li>
 *   <li>At setPose time: {@code tagCorrectedFieldPoseNow()} re-propagates fuse → current odom</li>
 * </ol>
 *
 * <pre>{@code
 * tracker.poll(spatial.lastTagCorrectionNanos(), spatial.tagCorrectedFieldPoseNow())
 * }</pre>
 */
public final class VidarPedroCorrectionTracker {

    private long lastAppliedCorrectionNanos;

    public VidarPedroCorrectionTracker() {
        this.lastAppliedCorrectionNanos = 0L;
    }

    /**
     * Preferred entry: event-id gating (no pose epsilon, no second cooldown).
     *
     * @param correctionNanos {@code spatial.lastTagCorrectionNanos()} — 0 means no fix yet
     * @param poseCorrectedToNow {@code spatial.tagCorrectedFieldPoseNow()}
     * @return Pedro components for {@code follower.setPose}, or {@code null} to skip
     */
    public VidarPedroPose poll(long correctionNanos, Pose2D poseCorrectedToNow) {
        if (correctionNanos <= 0L || poseCorrectedToNow == null) {
            return null;
        }
        if (correctionNanos == lastAppliedCorrectionNanos) {
            return null;
        }
        lastAppliedCorrectionNanos = correctionNanos;
        return VidarPedroPoseBridge.fromPose2D(poseCorrectedToNow);
    }

    /**
     * Manual backdate when you hold capture-time field pose + odom samples yourself.
     */
    public static Pose2D backdateToNow(
            Pose2D fieldPoseAtCapture,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        return VidarMotionCorrection.robotFieldPoseNow(fieldPoseAtCapture, odomAtCapture, odomNow);
    }

    public void reset() {
        lastAppliedCorrectionNanos = 0L;
    }

    public long lastAppliedCorrectionNanos() {
        return lastAppliedCorrectionNanos;
    }
}
