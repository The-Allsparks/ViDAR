package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

/**
 * 1–4 cameras at 640×480 with tic-toc processing and fused localization.
 */
public class VidarMultiVision {

    private final VidarVision[] cameras;
    private final int cameraCount;
    private final Supplier<VidarAlliance> ourAlliance;
    private final Supplier<Pose2D> odomSupplier;
    private final VidarLocalizationFusion localization = new VidarLocalizationFusion();
    private final VidarTemporalFilter temporalFilter = new VidarTemporalFilter();
    private final VidarResourceBudget resourceBudget = new VidarResourceBudget();

    private Pose2D lastOdomSample;
    private long lastOdomNanos;
    private double travelHeadingDeg;
    private double speedInPerSec;

    private VidarBallObservation bestBall;
    private VidarPlateObservation bestPlate;
    private VidarPlateObservation bestFoe;
    private VidarPlateObservation bestAlly;
    private VidarTagObservation latestTag;
    private VidarTagScoutObservation latestScoutObservation;
    private VidarTagScoutResult lastTagScout;
    private Pose2D fusedFieldPose;

    public VidarMultiVision(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        this(hardwareMap, null, () -> VidarConfig.DEFAULT_ALLIANCE);
    }

    public VidarMultiVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier) {
        this(hardwareMap, odomSupplier, () -> VidarConfig.DEFAULT_ALLIANCE);
    }

    public VidarMultiVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> ourAlliance) {
        this.ourAlliance = ourAlliance == null ? () -> VidarConfig.DEFAULT_ALLIANCE : ourAlliance;
        this.odomSupplier = odomSupplier;
        cameraCount = VidarConfig.activeCameraCount();
        VidarDecodeArbiter.reset();
        cameras = new VidarVision[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            try {
                VidarCameraMount mount = VidarConfig.cameraMount(i);
                cameras[i] = new VidarVision(
                        hardwareMap,
                        mount.webcamName,
                        mount.profile,
                        odomSupplier,
                        mount.webcamName,
                        localization::fieldPosePrior);
            } catch (RuntimeException ex) {
                cameras[i] = null;
            }
        }
    }

    public void setFieldPosePrior(Pose2D prior) {
        localization.setFieldPosePrior(prior);
    }

    public void update() {
        long loopStart = System.nanoTime();
        sampleOdomMotion();

        int connected = 0;
        for (VidarVision camera : cameras) {
            if (camera == null) {
                continue;
            }
            connected++;
            try {
                camera.applyDirectionTier(travelHeadingDeg, speedInPerSec);
                camera.update();
                camera.metrics().recordLoopCpu((System.nanoTime() - loopStart) / 1_000_000.0);
            } catch (RuntimeException ex) {
                camera.metrics().setHealth(VidarMetrics.CameraHealth.FAILED);
                camera.metrics().setLastError(ex.getMessage());
            }
        }

        resourceBudget.update(collectMetrics(), connected);

        bestBall = null;
        bestPlate = null;
        bestFoe = null;
        bestAlly = null;
        latestTag = null;
        latestScoutObservation = null;
        lastTagScout = null;

        double bestBallScore = -1;
        double bestPlateScore = -1;
        double bestFoeScore = -1;
        double bestAllyScore = -1;
        long newestTagTime = Long.MIN_VALUE;
        int bestDecodePixels = -1;
        double bestScoutScore = -1;

        for (VidarVision camera : cameras) {
            if (camera == null || camera.isFailed()) {
                continue;
            }

            VidarBallObservation ball = camera.getBestElement();
            if (ball != null) {
                ball = temporalFilter.filterBall(ball);
                if (ball != null) {
                    double score = ballScore(ball);
                    if (score > bestBallScore) {
                        bestBallScore = score;
                        bestBall = ball;
                    }
                }
            }

            VidarPlateObservation plate = camera.getBestPlate();
            if (plate != null) {
                plate = temporalFilter.filterPlate(plate);
                if (plate != null) {
                    double score = plateScore(plate);
                    VidarAlliance ours = ourAlliance.get();
                    if (score > bestPlateScore) {
                        bestPlateScore = score;
                        bestPlate = plate;
                    }
                    if (plate.isFoe(ours) && score > bestFoeScore) {
                        bestFoeScore = score;
                        bestFoe = plate;
                    }
                    if (plate.isAlly(ours) && score > bestAllyScore) {
                        bestAllyScore = score;
                        bestAlly = plate;
                    }
                }
            }

            VidarTagObservation tag = camera.getLatestTag();
            if (tag != null) {
                if (tag.decodePixels > bestDecodePixels
                        || (tag.decodePixels == bestDecodePixels && tag.captureTimeNanos > newestTagTime)) {
                    bestDecodePixels = tag.decodePixels;
                    newestTagTime = tag.captureTimeNanos;
                    latestTag = tag;
                }
            }

            VidarTagScoutObservation scout = camera.getLatestScoutObservation();
            if (scout != null) {
                double score = scout.scoutConfidence * scout.apparentWidthPx;
                if (score > bestScoutScore) {
                    bestScoutScore = score;
                    latestScoutObservation = scout;
                }
            }

            VidarTagScoutResult scoutResult = camera.getLastTagScout();
            if (scoutResult != null && (lastTagScout == null || scoutResult.widthPx > lastTagScout.widthPx)) {
                lastTagScout = scoutResult;
            }
        }

        Pose2D odomNow = odomSupplier == null ? null : odomSupplier.get();
        fusedFieldPose = localization.fusedFieldPoseNow(latestTag, latestScoutObservation, odomNow);
    }

    private VidarMetrics[] collectMetrics() {
        VidarMetrics[] out = new VidarMetrics[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            out[i] = cameras[i] == null ? null : cameras[i].metrics();
        }
        return out;
    }

    private void sampleOdomMotion() {
        if (odomSupplier == null) {
            speedInPerSec = 0;
            return;
        }
        Pose2D now = odomSupplier.get();
        long t = System.nanoTime();
        if (lastOdomSample != null && lastOdomNanos > 0) {
            double dt = (t - lastOdomNanos) / 1e9;
            if (dt > 0.02) {
                double dx = now.getX(DistanceUnit.INCH) - lastOdomSample.getX(DistanceUnit.INCH);
                double dy = now.getY(DistanceUnit.INCH) - lastOdomSample.getY(DistanceUnit.INCH);
                speedInPerSec = Math.hypot(dx, dy) / dt;
                if (speedInPerSec > 1.0) {
                    travelHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
                }
            }
        }
        lastOdomSample = now;
        lastOdomNanos = t;
    }

    private static double ballScore(VidarBallObservation ball) {
        if (ball.confidence < VidarConfig.MIN_BALL_CONFIDENCE) {
            return -1;
        }
        double rangeWeight = Double.isNaN(ball.rangeIn) ? 0.5 : 1.0 / Math.max(6.0, ball.rangeIn);
        return ball.confidence * ball.radiusPx * ball.radiusPx * rangeWeight;
    }

    private static double plateScore(VidarPlateObservation plate) {
        double rangeWeight = Double.isNaN(plate.rangeIn) ? 0.5 : 1.0 / Math.max(8.0, plate.rangeIn);
        return plate.confidence * plate.whiteRatio * rangeWeight;
    }

    public int getCameraCount() {
        return cameraCount;
    }

    public VidarVision camera(int index) {
        if (index < 0 || index >= cameraCount) {
            return null;
        }
        return cameras[index];
    }

    public VidarBallObservation getBestElement() {
        return bestBall;
    }

    public VidarPlateObservation getBestPlate() {
        return bestPlate;
    }

    public VidarPlateObservation getBestFoe() {
        return bestFoe;
    }

    public VidarPlateObservation getBestAlly() {
        return bestAlly;
    }

    public VidarTagScoutResult getLastTagScout() {
        return lastTagScout;
    }

    public VidarTagObservation getLatestTag() {
        return latestTag;
    }

    public VidarTagScoutObservation getLatestScoutObservation() {
        return latestScoutObservation;
    }

    /** @deprecated Scouts no longer localize. */
    @Deprecated
    public VidarScoutLandmarkObservation getLatestScoutLandmark() {
        return null;
    }

    public Pose2D getFusedFieldPose() {
        return fusedFieldPose;
    }

    public Pose2D getBackdatedFieldPose(Pose2D odomNow) {
        if (latestTag != null) {
            for (VidarVision camera : cameras) {
                if (camera == null) continue;
                VidarTagObservation tag = camera.getLatestTag();
                if (tag == latestTag) {
                    return camera.getBackdatedFieldPose(odomNow);
                }
            }
            return VidarPoseBackdate.fieldPoseNow(
                    latestTag,
                    odomNow,
                    DistanceUnit.INCH,
                    AngleUnit.DEGREES);
        }
        return fusedFieldPose;
    }

    public double travelSpeedInPerSec() {
        return speedInPerSec;
    }

    public VidarResourceBudget resourceBudget() {
        return resourceBudget;
    }

    public void close() {
        for (VidarVision camera : cameras) {
            if (camera != null) {
                camera.close();
            }
        }
    }
}
