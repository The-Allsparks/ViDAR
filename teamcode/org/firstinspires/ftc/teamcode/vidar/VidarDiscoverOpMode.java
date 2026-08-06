package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class VidarDiscoverOpMode extends LinearOpMode {

    private boolean lastA;

    @Override
    public void runOpMode() {
        VidarAllianceSelector alliance = new VidarAllianceSelector(hardwareMap);
        Pose2D[] odomHolder = {new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)};
        VidarMultiVision vision = new VidarMultiVision(hardwareMap, () -> odomHolder[0], alliance::get);
        VidarWorldModel world = new VidarWorldModel(() -> odomHolder[0], vision::getFusedFieldPose);
        vision.setFieldPosePrior(odomHolder[0]);

        telemetry.addLine("ViDAR Discover — " + vision.getCameraCount() + " cam(s) @ 640×480 tic-toc");
        telemetry.addLine("INIT: Y=RED B=BLUE (or color sensor on own sign)");
        telemetry.addLine("Run: Back toggles alliance · A = tag sample");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            alliance.pollInit(gamepad1);
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Color sensor", alliance.hasColorSensor() ? "configured" : "none");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            alliance.pollRuntime(gamepad1);

            if (gamepad1.a && !lastA) {
                VidarTagGate.requestSample();
            }
            lastA = gamepad1.a;

            long now = System.nanoTime();
            vision.update();
            world.update(vision, now);

            VidarElementObservation element = vision.getBestElement();
            VidarPlateObservation plate = vision.getBestPlate();
            VidarTagScoutResult scout = vision.getLastTagScout();
            VidarTagObservation tag = vision.getLatestTag();
            Pose2D fieldNow = vision.getFusedFieldPose();
            if (fieldNow == null) {
                fieldNow = vision.getBackdatedFieldPose(odomHolder[0]);
            }
            VidarAlliance ours = alliance.get();

            telemetry.addData("Cameras", vision.getCameraCount());
            telemetry.addData("FPS cam1", vision.getCameraCount() > 0
                    ? String.format("%.1f", vision.camera(0).portalFps()) : "—");
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Element", VidarBlobUtil.formatElement(element));
            telemetry.addData("Element detail", VidarBlobUtil.formatElementDetail(element));
            telemetry.addData("Plate", VidarBlobUtil.formatPlate(plate, ours));
            telemetry.addData("Plate detail", VidarBlobUtil.formatPlateDetail(plate));
            telemetry.addData("Foe", VidarBlobUtil.formatPlate(vision.getBestFoe(), ours));
            telemetry.addData("World tracks", world.trackCount());
            telemetry.addData("Nearest element", VidarBlobUtil.formatWorldTrack(world.nearestElement()));
            telemetry.addData("Nearest foe", VidarBlobUtil.formatWorldTrack(world.nearestFoe()));
            telemetry.addData("Intake blocked", world.intakeBlocked());
            telemetry.addData("Tag scout", scout == null ? "none"
                    : String.format("(%.0f,%.0f) w=%.0f %s", scout.cx, scout.cy, scout.widthPx, scout.band));
            telemetry.addData("Tag fix", VidarBlobUtil.formatTag(tag));
            telemetry.addData("Scout obs", VidarBlobUtil.formatScoutObservation(vision.getLatestScoutObservation()));
            if (vision.camera(0) != null) {
                telemetry.addData("Element reject", vision.camera(0).elementRejectionStats().summary());
                telemetry.addData("Cam state", vision.camera(0).directionState().name());
            }
            telemetry.addData("Tag @capture", VidarBlobUtil.formatTagPose(tag));
            telemetry.addData("Field fused", fieldNow == null ? "—"
                    : String.format("(%.1f, %.1f) %.0f°",
                    fieldNow.getX(DistanceUnit.INCH),
                    fieldNow.getY(DistanceUnit.INCH),
                    fieldNow.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }

        vision.close();
    }
}
