package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Lesson 2 — drive with gamepad; nudge away from foe plates using world memory.
 *
 * <p>INIT: Y=RED, B=BLUE on gamepad1 (or color sensor on own sign).
 */
@TeleOp(name = "ViDAR: TeleOp", group = "ViDAR")
public class VidarTeleOp extends LinearOpMode {

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

        telemetry.addLine("ViDAR TeleOp — INIT: Y=RED B=BLUE");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            alliance.pollInit(gamepad1);
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            alliance.pollRuntime(gamepad1);

            long now = System.nanoTime();
            vision.update();
            world.update(vision, now);

            double drive = -gamepad1.left_stick_y * VidarConfig.DRIVE_SPEED;
            double turn = gamepad1.right_stick_x * VidarConfig.DRIVE_SPEED;

            turn += avoidanceTurn(vision.getBestFoe());

            leftDrive.setPower(drive + turn);
            rightDrive.setPower(drive - turn);

            VidarAlliance ours = alliance.get();
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Ball", VidarBlobUtil.formatBall(vision.getBestElement()));
            telemetry.addData("Foe", VidarBlobUtil.formatPlate(vision.getBestFoe(), ours));
            telemetry.addData("World foes", world.getTracks(VidarWorldModel.Kind.FOE).size());
            telemetry.addData("Drive", "%.2f  Turn: %.2f", drive, turn);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        vision.close();
    }

    private double avoidanceTurn(VidarPlateObservation foe) {
        if (foe == null) {
            return 0;
        }

        double frameWidth = VidarConfig.portalCameraResolution().getWidth();
        double error = VidarBlobUtil.errorFromCenter(foe, frameWidth);
        double dist = Math.abs(error);
        if (dist > VidarConfig.AVOID_CENTER_RADIUS) {
            return 0;
        }

        double direction = error > 0 ? -1 : 1;
        double strength = 1.0 - (dist / VidarConfig.AVOID_CENTER_RADIUS);
        return direction * VidarConfig.AVOID_TURN_POWER * strength;
    }
}
