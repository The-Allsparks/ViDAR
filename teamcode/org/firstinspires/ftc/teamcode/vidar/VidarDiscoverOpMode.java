package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.detect.VidarBlobUtil;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarAllianceSelector;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagGate;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Lesson 1 — elements, plates, adaptive AprilTags, multi-camera fusion, world model.
 *
 * <p>INIT: hold {@code Y} = RED alliance, {@code B} = BLUE (overrides color sensor).
 * Color sensor on own ROBOT SIGN is used when no button is held.
 * After start: {@code Back} toggles alliance if enabled in {@link VidarConfig}.
 *
 * <p>Gamepad: {@code A} requests a tag sample.
 */
@TeleOp(name = "ViDAR: Discover", group = "ViDAR")
public class VidarDiscoverOpMode extends VidarSpatialOpModeBase {

    private boolean lastA;

    @Override
    public void runOpMode() {
        VidarAllianceSelector alliance = new VidarAllianceSelector(hardwareMap);
        Pose2D[] odomHolder = {new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)};
        VidarSpatial spatial = VidarSpatial.createWithBundledDefaults(
                hardwareMap, () -> odomHolder[0], alliance::get);
        spatial.setFieldPosePrior(odomHolder[0]);

        telemetry.addLine("ViDAR Discover — " + spatial.cameraCount() + " cam(s) @ 640×480 tic-toc");
        telemetry.addLine("INIT: Y=RED B=BLUE (or color sensor on own sign)");
        telemetry.addLine("Run: Back toggles alliance · A = tag sample");
        telemetry.update();

        pollAllianceInit(alliance, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            alliance.pollRuntime(gamepad1);

            if (gamepad1.a && !lastA) {
                VidarTagGate.requestSample();
            }
            lastA = gamepad1.a;

            spatial.update();

            VidarObservationFrame frame = spatial.lastFrame();
            VidarElementObservation element = frame == null ? null : frame.bestElement;
            VidarPlateObservation plate = frame == null ? null : frame.bestPlate;
            VidarTagScoutObservation scout = frame == null ? null : frame.bestScoutHit;
            VidarTagObservation tag = frame == null ? null : frame.bestTag;
            Pose2D fieldNow = spatial.fieldPose();
            VidarAlliance ours = alliance.get();

            telemetry.addData("Cameras", spatial.diagnostics().connectedCameras
                    + "/" + spatial.diagnostics().cameraCount);
            telemetry.addData("Config", spatial.diagnostics().configSource);
            if (spatial.diagnostics().observationWorkerTotalFailures > 0) {
                telemetry.addData("Worker", "failures="
                        + spatial.diagnostics().observationWorkerTotalFailures
                        + " consecutive="
                        + spatial.diagnostics().observationWorkerConsecutiveFailures
                        + (spatial.diagnostics().observationWorkerLastError.isEmpty()
                                ? "" : ": " + spatial.diagnostics().observationWorkerLastError));
            }
            if (spatial.diagnostics().observationTickSamples > 0) {
                telemetry.addData("Tick ms", String.format(
                        "p50=%.2f p95=%.2f max=%.2f n=%d",
                        spatial.diagnostics().observationTickP50Ms,
                        spatial.diagnostics().observationTickP95Ms,
                        spatial.diagnostics().observationTickMaxMs,
                        spatial.diagnostics().observationTickSamples));
            }
            telemetry.addData("FPS cam1", spatial.cameraCount() > 0
                    && spatial.runtime().camera(0) != null
                    ? String.format("%.1f", spatial.runtime().camera(0).portalFps()) : "—");
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Element", VidarBlobUtil.formatElement(element));
            telemetry.addData("Element detail", VidarBlobUtil.formatElementDetail(element));
            telemetry.addData("Spatial live", VidarBlobUtil.formatSpatialPoint(spatial.bestElement()));
            telemetry.addData("Spatial remembered", VidarBlobUtil.formatSpatialPoint(spatial.nearestElement()));
            if (spatial.runtime().fusionEngine() != null
                    && spatial.runtime().fusionEngine().calibrationDiagnostics() != null) {
                telemetry.addData("Calibration", VidarBlobUtil.formatCalibrationDiagnostics(
                        spatial.runtime().fusionEngine().calibrationDiagnostics().toTelemetryMap()));
            }
            telemetry.addData("Plate", VidarBlobUtil.formatPlate(plate, ours));
            telemetry.addData("Plate detail", VidarBlobUtil.formatPlateDetail(plate));
            telemetry.addData("Foe", VidarBlobUtil.formatPlate(
                    frame == null ? null : frame.bestFoe, ours));
            telemetry.addData("World tracks", spatial.trackCount());
            telemetry.addData("Intake blocked", spatial.intakeBlocked());
            telemetry.addData("Tag scout", scout == null ? "none"
                    : String.format("(%.0f,%.0f) w=%.0f %s", scout.cx, scout.cy, scout.apparentWidthPx, scout.band));
            telemetry.addData("Tag fix", VidarBlobUtil.formatTag(tag));
            telemetry.addData("Scout obs", VidarBlobUtil.formatScoutObservation(
                    frame == null ? null : frame.bestScoutObservation));
            if (spatial.runtime().camera(0) != null) {
                telemetry.addData("Element reject",
                        spatial.runtime().camera(0).elementRejectionStats().summary());
                telemetry.addData("Cam state", spatial.runtime().camera(0).directionState().name());
            }
            telemetry.addData("Tag @capture", VidarBlobUtil.formatTagPose(tag));
            telemetry.addData("Field fused", formatFieldPose(fieldNow));
            for (String warning : spatial.diagnostics().warnings) {
                if (warning.startsWith("Observation worker failures=")) {
                    continue;
                }
                telemetry.addLine("⚠ " + warning);
            }
            telemetry.update();
        }

        spatial.close();
    }
}
