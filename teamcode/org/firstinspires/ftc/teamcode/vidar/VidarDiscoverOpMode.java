package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.detect.VidarBlobUtil;
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
        VidarSpatial spatial = VidarSpatial.create(hardwareMap, () -> odomHolder[0], alliance::get);
        spatial.setFieldPosePrior(odomHolder[0]);
        VidarMultiVision vision = spatial.vision();

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

            VidarElementObservation element = vision.getBestElement();
            VidarPlateObservation plate = vision.getBestPlate();
            VidarTagScoutObservation scout = vision.getLastTagScout();
            VidarTagObservation tag = vision.getLatestTag();
            Pose2D fieldNow = spatial.fieldPose();
            VidarAlliance ours = alliance.get();

            telemetry.addData("Cameras", spatial.cameraCount());
            telemetry.addData("FPS cam1", spatial.cameraCount() > 0
                    ? String.format("%.1f", vision.camera(0).portalFps()) : "—");
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Element", VidarBlobUtil.formatElement(element));
            telemetry.addData("Element detail", VidarBlobUtil.formatElementDetail(element));
            telemetry.addData("Spatial live", VidarBlobUtil.formatSpatialPoint(spatial.bestElement()));
            telemetry.addData("Spatial remembered", VidarBlobUtil.formatSpatialPoint(spatial.nearestElement()));
            telemetry.addData("Calibration", VidarBlobUtil.formatCalibrationDiagnostics(
                    vision.calibrationDiagnostics().toTelemetryMap()));
            telemetry.addData("Plate", VidarBlobUtil.formatPlate(plate, ours));
            telemetry.addData("Plate detail", VidarBlobUtil.formatPlateDetail(plate));
            telemetry.addData("Foe", VidarBlobUtil.formatPlate(vision.getBestFoe(), ours));
            telemetry.addData("World tracks", spatial.trackCount());
            telemetry.addData("Intake blocked", spatial.intakeBlocked());
            telemetry.addData("Tag scout", scout == null ? "none"
                    : String.format("(%.0f,%.0f) w=%.0f %s", scout.cx, scout.cy, scout.apparentWidthPx, scout.band));
            telemetry.addData("Tag fix", VidarBlobUtil.formatTag(tag));
            telemetry.addData("Scout obs", VidarBlobUtil.formatScoutObservation(vision.getLatestScoutObservation()));
            if (vision.camera(0) != null) {
                telemetry.addData("Element reject", vision.camera(0).elementRejectionStats().summary());
                telemetry.addData("Cam state", vision.camera(0).directionState().name());
            }
            telemetry.addData("Tag @capture", VidarBlobUtil.formatTagPose(tag));
            telemetry.addData("Field fused", formatFieldPose(fieldNow));
            telemetry.update();
        }

        spatial.close();
    }
}
