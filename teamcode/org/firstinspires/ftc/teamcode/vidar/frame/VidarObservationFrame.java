package org.firstinspires.ftc.teamcode.vidar.frame;

import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutResult;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarMultiVision;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarMotionCorrection;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarOdomHistory;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Immutable snapshot of all fused ViDAR outputs from one {@link VidarMultiVision#update()} cycle.
 *
 * <p>Every observation carries VisionPortal {@code frameCaptureNanos} as {@code captureTimeNanos}.
 * Prefer batch correction:
 * {@link #toRobotNow(VidarOdomHistory, Pose2D)} — low-level per-point helpers remain in
 * {@link VidarMotionCorrection}.
 */
public final class VidarObservationFrame {

    public final long updateTimeNanos;
    public final VidarRankedElementFrame rankedElements;
    public final VidarElementObservation bestElement;
    public final VidarPlateObservation bestPlate;
    public final VidarPlateObservation bestFoe;
    public final VidarPlateObservation bestAlly;
    public final VidarTagObservation bestTag;
    public final VidarTagScoutObservation bestScout;
    public final VidarTagScoutResult bestScoutResult;

    /** Per-camera ranked elements (may differ in count/cap). Index matches robot config. */
    public final VidarRankedElementFrame[] rankedByCamera;
    /** Latest decoded tag per camera, or null. */
    public final VidarTagObservation[] tagsByCamera;

    public VidarObservationFrame(
            long updateTimeNanos,
            VidarRankedElementFrame rankedElements,
            VidarElementObservation bestElement,
            VidarPlateObservation bestPlate,
            VidarPlateObservation bestFoe,
            VidarPlateObservation bestAlly,
            VidarTagObservation bestTag,
            VidarTagScoutObservation bestScout,
            VidarTagScoutResult bestScoutResult,
            VidarRankedElementFrame[] rankedByCamera,
            VidarTagObservation[] tagsByCamera) {
        this.updateTimeNanos = updateTimeNanos;
        this.rankedElements = rankedElements != null
                ? rankedElements
                : VidarRankedElementFrame.empty("fused", VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        this.bestElement = bestElement;
        this.bestPlate = bestPlate;
        this.bestFoe = bestFoe;
        this.bestAlly = bestAlly;
        this.bestTag = bestTag;
        this.bestScout = bestScout;
        this.bestScoutResult = bestScoutResult;
        this.rankedByCamera = rankedByCamera != null ? rankedByCamera : new VidarRankedElementFrame[0];
        this.tagsByCamera = tagsByCamera != null ? tagsByCamera : new VidarTagObservation[0];
    }

    public static VidarObservationFrame empty() {
        return new VidarObservationFrame(
                0,
                VidarRankedElementFrame.empty("fused", VidarConfig.FUSION_MAX_RANKED_ELEMENTS),
                null, null, null, null, null, null, null,
                new VidarRankedElementFrame[0],
                new VidarTagObservation[0]);
    }

    public int cameraCount() {
        return Math.max(rankedByCamera.length, tagsByCamera.length);
    }

    public VidarRankedElementFrame rankedForCamera(int index) {
        if (index < 0 || index >= rankedByCamera.length) {
            return null;
        }
        return rankedByCamera[index];
    }

    public VidarTagObservation tagForCamera(int index) {
        if (index < 0 || index >= tagsByCamera.length) {
            return null;
        }
        return tagsByCamera[index];
    }

    /**
     * Latency-compensate every observation to the current robot frame (one call per update cycle).
     */
    public VidarCorrectedFrame toRobotNow(VidarOdomHistory history, Pose2D odomNow) {
        return VidarCorrectedFrame.from(this, history, odomNow);
    }
}
