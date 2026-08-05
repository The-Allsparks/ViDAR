package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

/**
 * One camera: tic-toc ball / plate / tag at 640×480 with shared scheduler per portal.
 */
public class VidarVision {

    private final VidarProcessScheduler scheduler;
    private final VidarBallProcessor ballProcessor;
    private final VidarAdaptiveTagProcessor tagProcessor;
    private final VidarPlateProcessor plateProcessor;
    private final VidarCameraScheduler cameraScheduler;
    private final VidarMetrics metrics;
    private final org.firstinspires.ftc.vision.VisionPortal portal;
    private final VidarCameraProfile profile;
    private final String cameraName;
    private boolean failed;

    private VidarBallObservation bestElement;
    private VidarPlateObservation bestPlate;

    public VidarVision(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        this(hardwareMap, VidarConfig.CAMERA_NAME, VidarConfig.cameraProfile(), null, VidarConfig.CAMERA_NAME, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName) {
        this(hardwareMap, cameraName, VidarConfig.cameraProfile(), null, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile) {
        this(hardwareMap, cameraName, profile, null, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier) {
        this(hardwareMap, cameraName, profile, odomSupplier, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier) {
        this.profile = profile;
        this.cameraName = portalLabel;
        metrics = new VidarMetrics(portalLabel);

        scheduler = new VidarProcessScheduler();
        cameraScheduler = new VidarCameraScheduler();

        ballProcessor = new VidarBallProcessor(profile, portalLabel, scheduler,
                VidarConfig.BALL_DETECTOR_TYPE, metrics);
        tagProcessor = new VidarAdaptiveTagProcessor(
                scheduler, profile, portalLabel, odomSupplier, fieldPosePriorSupplier, metrics);
        plateProcessor = new VidarPlateProcessor(profile, portalLabel, scheduler);

        org.firstinspires.ftc.vision.VisionPortal.Builder builder =
                new org.firstinspires.ftc.vision.VisionPortal.Builder()
                        .addProcessor(ballProcessor)
                        .setCameraResolution(VidarConfig.portalCameraResolution())
                        .setStreamFormat(VidarConfig.portalStreamFormat())
                        .enableLiveView(VidarConfig.LIVE_VIEW_ENABLED)
                        .setCamera(hardwareMap.get(
                                org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                                cameraName));

        if (VidarTagConfig.ENABLED) {
            builder.addProcessor(tagProcessor);
        }
        builder.addProcessor(plateProcessor);

        portal = builder.build();
        VidarTagGate.setCameraBearingDeg(profile.bearingDeg);
        metrics.setHealth(VidarMetrics.CameraHealth.STREAMING);
    }

    public void update() {
        try {
            bestElement = ballProcessor.getBestBall();
            bestPlate = plateProcessor.getBestPlate();
            metrics.setPortalFps(portal.getFps());
            if (!failed) {
                metrics.setHealth(VidarMetrics.CameraHealth.HEALTHY);
            }
        } catch (RuntimeException ex) {
            failed = true;
            metrics.setHealth(VidarMetrics.CameraHealth.FAILED);
            metrics.setLastError(ex.getMessage());
        }
    }

    public void applyDirectionTier(double travelHeadingDeg, double speedInPerSec) {
        VidarCameraScheduler.State state =
                cameraScheduler.tierForCamera(profile.bearingDeg, travelHeadingDeg, speedInPerSec);
        if (state == VidarCameraScheduler.State.IDLE && speedInPerSec < VidarConfig.DIRECTION_MIN_SPEED_IN_PER_SEC) {
            state = VidarCameraScheduler.State.PRIMARY;
        }
        cameraScheduler.apply(portal, ballProcessor, plateProcessor, tagProcessor, state, metrics);
    }

    public VidarBallObservation getBestElement() {
        return bestElement;
    }

    public VidarPlateObservation getBestPlate() {
        return bestPlate;
    }

    /** @deprecated Use {@link #getBestPlate()} — plates replace crude color blobs. */
    @Deprecated
    public org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.Blob getBestRobot() {
        return null;
    }

    public VidarTagScoutResult getLastTagScout() {
        return tagProcessor.getLastScout();
    }

    public VidarTagObservation getLatestTag() {
        return tagProcessor.getLatestTag();
    }

    public VidarTagScoutObservation getLatestScoutObservation() {
        return tagProcessor.getLatestScoutObservation();
    }

    /** @deprecated Scouts no longer localize. */
    @Deprecated
    public VidarScoutLandmarkObservation getLatestScoutLandmark() {
        return null;
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Pose2D getBackdatedFieldPose(
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D odomNow) {
        return VidarPoseBackdate.fieldPoseNow(
                getLatestTag(),
                odomNow,
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
    }

    /** @deprecated Scouts no longer localize. */
    @Deprecated
    public org.firstinspires.ftc.robotcore.external.navigation.Pose2D getBackdatedScoutLandmarkPose(
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D odomNow) {
        return null;
    }

    public VidarCameraProfile getProfile() {
        return profile;
    }

    public String getCameraName() {
        return cameraName;
    }

    public org.firstinspires.ftc.vision.VisionPortal getPortal() {
        return portal;
    }

    public VidarCameraScheduler.State directionState() {
        return cameraScheduler.currentState();
    }

    /** @deprecated Use {@link #directionState()}. */
    @Deprecated
    public VidarCameraScheduler.Tier directionTier() {
        return cameraScheduler.currentTier();
    }

    public VidarMetrics metrics() {
        return metrics;
    }

    public VidarBallRejectionStats ballRejectionStats() {
        return ballProcessor.getRejectionStats();
    }

    public boolean isFailed() {
        return failed;
    }

    public float portalFps() {
        return portal.getFps();
    }

    public void close() {
        portal.close();
    }
}
