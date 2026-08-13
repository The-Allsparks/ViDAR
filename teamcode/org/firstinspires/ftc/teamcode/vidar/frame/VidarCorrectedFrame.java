package org.firstinspires.ftc.teamcode.vidar.frame;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarRobotPose2D;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarMotionCorrection;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarOdomHistory;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementDensityMap;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarVisionMeasurement;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * All ViDAR observations extrapolated to the current robot frame in one pass.
 *
 * <p>This mirrors WPILib latency compensation: record odom each loop, then call
 * {@link VidarObservationFrame#toRobotNow(VidarOdomHistory, Pose2D)} once — no per-element
 * {@code odomHistory.at()} at the call site.
 *
 * <pre>{@code
 * odomHistory.record(odom.getPose());
 * VidarCorrectedFrame now = vision.update().toRobotNow(odomHistory, odom.getPose());
 * VidarElementDensityMap.Peak target = now.peak();
 * if (now.tagMeasurement() != null) {
 *     poseEstimator.addVisionMeasurement(now.tagMeasurement());
 * }
 * }</pre>
 */
public final class VidarCorrectedFrame {

    public final long queryTimeNanos;
    public final Pose2D odomNow;
    public final VidarObservationFrame source;

    /** All unique corrected game elements (fused + per-camera, merged). */
    public final List<VidarCorrectedPoint> elements;
    public final List<VidarCorrectedPoint> plates;

    /** Best decoded tag field pose at {@code odomNow}, or null. */
    public final Pose2D tagFieldNow;
    /** WPILib-style measurement for pose fusion, or null. */
    public final VidarVisionMeasurement tagMeasurement;

    public final VidarElementDensityMap densityMap;
    public final VidarElementDensityMap.Peak peak;

    private VidarCorrectedFrame(
            long queryTimeNanos,
            Pose2D odomNow,
            VidarObservationFrame source,
            List<VidarCorrectedPoint> elements,
            List<VidarCorrectedPoint> plates,
            Pose2D tagFieldNow,
            VidarVisionMeasurement tagMeasurement,
            VidarElementDensityMap densityMap,
            VidarElementDensityMap.Peak peak) {
        this.queryTimeNanos = queryTimeNanos;
        this.odomNow = odomNow;
        this.source = source;
        this.elements = Collections.unmodifiableList(elements);
        this.plates = Collections.unmodifiableList(plates);
        this.tagFieldNow = tagFieldNow;
        this.tagMeasurement = tagMeasurement;
        this.densityMap = densityMap;
        this.peak = peak;
    }

    /**
     * Batch-correct every observation in {@code frame} to robot-now using {@code history}.
     */
    public static VidarCorrectedFrame from(
            VidarObservationFrame frame,
            VidarOdomHistory history,
            Pose2D odomNow) {
        long queryTime = System.nanoTime();
        if (frame == null) {
            frame = VidarObservationFrame.empty();
        }
        if (history == null) {
            history = new VidarOdomHistory();
        }

        List<VidarCorrectedPoint> elements = new ArrayList<>();
        collectElements(frame.rankedElements, history, odomNow, queryTime, elements);
        for (VidarRankedElementFrame ranked : frame.rankedByCamera) {
            collectElements(ranked, history, odomNow, queryTime, elements);
        }
        mergeNearbyElements(elements);

        List<VidarCorrectedPoint> plates = new ArrayList<>(3);
        addPlate(plates, frame.bestPlate, VidarCorrectedPoint.Kind.PLATE, history, odomNow, queryTime);
        addPlate(plates, frame.bestFoe, VidarCorrectedPoint.Kind.PLATE_FOE, history, odomNow, queryTime);
        addPlate(plates, frame.bestAlly, VidarCorrectedPoint.Kind.PLATE_ALLY, history, odomNow, queryTime);

        Pose2D tagFieldNow = null;
        VidarVisionMeasurement tagMeasurement = null;
        if (frame.bestTag != null) {
            Pose2D odomAtCapture = history.at(frame.bestTag.captureTimeNanos);
            tagFieldNow = VidarMotionCorrection.tagFieldNow(frame.bestTag, odomAtCapture, odomNow);
            if (tagFieldNow != null) {
                tagMeasurement = new VidarVisionMeasurement(
                        tagFieldNow,
                        frame.bestTag.captureTimeNanos,
                        tagTrust(frame.bestTag),
                        frame.bestTag.tagId);
            }
        }

        VidarElementDensityMap density = VidarElementDensityMap.defaultRobotGrid();
        for (VidarCorrectedPoint element : elements) {
            density.splat(element);
        }
        VidarElementDensityMap.Peak peak = density.peak();

        return new VidarCorrectedFrame(
                queryTime,
                odomNow,
                frame,
                elements,
                plates,
                tagFieldNow,
                tagMeasurement,
                density,
                peak);
    }

    /** Highest-density element cluster in robot frame, or null. */
    public VidarElementDensityMap.Peak peak() {
        return peak;
    }

    /** Nearest corrected element by range, or null. */
    public VidarCorrectedPoint nearestElement() {
        VidarCorrectedPoint best = null;
        double bestRange = Double.MAX_VALUE;
        for (VidarCorrectedPoint p : elements) {
            double r = p.distance();
            if (r < bestRange) {
                bestRange = r;
                best = p;
            }
        }
        return best;
    }

    /** Prefer density peak; fall back to nearest element. */
    public VidarElementDensityMap.Peak autoTarget() {
        return peak != null ? peak : elementAsPeak(nearestElement());
    }

    private static VidarElementDensityMap.Peak elementAsPeak(VidarCorrectedPoint p) {
        if (p == null) {
            return null;
        }
        return new VidarElementDensityMap.Peak(
                p.robotX, p.robotY, p.confidence, -1, -1);
    }

    private static void collectElements(
            VidarRankedElementFrame ranked,
            VidarOdomHistory history,
            Pose2D odomNow,
            long queryTime,
            List<VidarCorrectedPoint> out) {
        if (ranked == null) {
            return;
        }
        for (int i = 0; i < ranked.count(); i++) {
            VidarElementObservation obs = ranked.at(i);
            if (obs == null) {
                continue;
            }
            Pose2D odomAtCapture = history.at(obs.captureTimeNanos);
            double[] robotNow = VidarMotionCorrection.elementRobotNow(obs, odomAtCapture, odomNow);
            VidarCorrectedPoint point = VidarCorrectedPoint.element(
                    obs, robotNow[0], robotNow[1], queryTime);
            if (point != null) {
                out.add(point);
            }
        }
    }

    private static void addPlate(
            List<VidarCorrectedPoint> out,
            VidarPlateObservation plate,
            VidarCorrectedPoint.Kind kind,
            VidarOdomHistory history,
            Pose2D odomNow,
            long queryTime) {
        if (plate == null) {
            return;
        }
        Pose2D odomAtCapture = history.at(plate.captureTimeNanos);
        double[] robotNow = VidarMotionCorrection.plateRobotNow(plate, odomAtCapture, odomNow);
        VidarCorrectedPoint point = VidarCorrectedPoint.plate(plate, kind, robotNow[0], robotNow[1], queryTime);
        if (point != null) {
            out.add(point);
        }
    }

    /** Collapse multi-camera duplicates in corrected robot space. */
    private static void mergeNearbyElements(List<VidarCorrectedPoint> points) {
        if (points.size() < 2) {
            return;
        }
        double mergeR = VidarConfig.WORLD_MERGE_RADIUS_IN;
        for (int i = 0; i < points.size(); i++) {
            VidarCorrectedPoint a = points.get(i);
            for (int j = points.size() - 1; j > i; j--) {
                VidarCorrectedPoint b = points.get(j);
                if (VidarRobotPose2D.withinRadius(a.robotX, a.robotY, b.robotX, b.robotY, mergeR)) {
                    if (a.confidence >= b.confidence) {
                        points.remove(j);
                    } else {
                        points.set(i, b);
                        points.remove(j);
                        a = b;
                    }
                }
            }
        }
    }

    private static double tagTrust(VidarTagObservation tag) {
        if (tag == null || tag.decodePixels <= 0) {
            return 0;
        }
        int refPixels = VidarTagConfig.MIN_DECISION_MARGIN > 0
                ? VidarTagConfig.MIN_DECISION_MARGIN
                : 800;
        return Math.min(1.0, tag.decodePixels / (double) refPixels);
    }
}
