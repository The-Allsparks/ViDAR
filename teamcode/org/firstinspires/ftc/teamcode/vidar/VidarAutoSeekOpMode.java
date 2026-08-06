package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Lesson 3 — seek nearest remembered element; stop for foes blocking intake.
 *
 * <p>INIT: color sensor on own sign auto-sets alliance; hold Y/B to override.
 */
@Autonomous(name = "ViDAR: Auto Seek", group = "ViDAR")
public class VidarAutoSeekOpMode extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private VidarMultiVision vision;
    private VidarWorldModel world;
    private VidarAllianceSelector alliance;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, VidarConfig.LEFT_DRIVE);
        rightDrive = hardwareMap.get(DcMotor.class, VidarConfig.RIGHT_DRIVE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        alliance = new VidarAllianceSelector(hardwareMap);
        vision = new VidarMultiVision(hardwareMap, null, alliance::get);
        world = new VidarWorldModel();

        telemetry.addLine("ViDAR Auto Seek — alliance from sensor or Y/B");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            alliance.pollInit(gamepad1);
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.update();
        }

        waitForStart();

        double frameWidth = VidarConfig.portalCameraResolution().getWidth();
        VidarAlliance ours = alliance.get();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            vision.update();
            world.update(vision, now);
            ours = alliance.get();

            VidarElementObservation element = vision.getBestElement();
            VidarWorldModel.Track remembered = world.nearestElement();

            double turn = 0;
            double drive = 0;

            if (world.intakeBlocked()) {
                drive = 0;
                turn = VidarConfig.AVOID_TURN_POWER;
                telemetry.addData("State", "BLOCKED by remembered foe");
            } else if (element != null
                    && element.confidence >= VidarConfig.MIN_ELEMENT_CONFIDENCE
                    && !Double.isNaN(element.range)) {
                double error = VidarBlobUtil.errorFromCenter(element, frameWidth);
                turn = clamp(error / (frameWidth / 2.0) * VidarConfig.SEEK_TURN_GAIN, -1, 1);

                if (element.range <= VidarConfig.PICKUP_STOP) {
                    drive = 0;
                    telemetry.addData("State", "AT PICKUP RANGE");
                } else if (Math.abs(error) < VidarConfig.SEEK_ALIGNED_PIXELS) {
                    drive = VidarBlobUtil.rangeDrivePower(element);
                    telemetry.addData("State", "DRIVING (range-scaled)");
                } else {
                    telemetry.addData("State", "TURNING toward element");
                }
            } else if (remembered != null) {
                turn = clamp(remembered.bearingDeg() / 45.0 * VidarConfig.SEEK_TURN_GAIN, -1, 1);
                telemetry.addData("State", "TURNING toward remembered element");
            } else if (element != null) {
                double error = VidarBlobUtil.errorFromCenter(element, frameWidth);
                turn = clamp(error / (frameWidth / 2.0) * VidarConfig.SEEK_TURN_GAIN, -1, 1);
                telemetry.addData("State", "TURNING (low confidence)");
            } else {
                telemetry.addData("State", "SEARCHING (slow spin)");
                turn = VidarConfig.SEARCH_TURN_POWER;
            }

            leftDrive.setPower(drive + turn);
            rightDrive.setPower(drive - turn);

            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Element", VidarBlobUtil.formatElement(element));
            telemetry.addData("Remembered", VidarBlobUtil.formatWorldTrack(remembered));
            telemetry.addData("Foe", VidarBlobUtil.formatPlate(vision.getBestFoe(), ours));
            telemetry.addData("Drive/Turn", "%.2f / %.2f", drive, turn);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        vision.close();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
