package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarAllianceSelector;

/**
 * Shared INIT alliance polling and field-pose telemetry for spatial OpModes.
 */
public abstract class VidarSpatialOpModeBase extends LinearOpMode {

    protected void pollAllianceInit(VidarAllianceSelector alliance, Gamepad gamepad) {
        while (!isStarted() && !isStopRequested()) {
            alliance.pollInit(gamepad);
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.update();
        }
    }

    protected static String formatFieldPose(Pose2D field) {
        if (field == null) {
            return "—";
        }
        return String.format(
                "(%.1f, %.1f) %.0f°",
                field.getX(DistanceUnit.INCH),
                field.getY(DistanceUnit.INCH),
                field.getHeading(AngleUnit.DEGREES));
    }
}
