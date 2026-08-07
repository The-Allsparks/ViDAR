package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/**
 * Lesson 2 — spatial snapshot telemetry (no motor output).
 *
 * <p>ViDAR reports pose and the three spatial groups: elements, allies, foes.
 *
 * <p>INIT: Y=RED, B=BLUE on gamepad1 (or color sensor on own sign).
 */
@TeleOp(name = "ViDAR: Spatial", group = "ViDAR")
public class VidarTeleOp extends LinearOpMode {

    private VidarSpatial spatial;
    private VidarAllianceSelector alliance;

    @Override
    public void runOpMode() {
        alliance = new VidarAllianceSelector(hardwareMap);
        spatial = VidarSpatial.create(hardwareMap, null, alliance::get);

        telemetry.addLine("ViDAR Spatial — elements / allies / foes (no motors)");
        telemetry.addLine("Motion tracks: off (no odom supplier)");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            alliance.pollInit(gamepad1);
            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            alliance.pollRuntime(gamepad1);
            spatial.update();

            List<VidarSpatialPoint> elements = spatial.elements();
            List<VidarSpatialPoint> allies = spatial.allies();
            List<VidarSpatialPoint> foes = spatial.foes();
            Pose2D field = spatial.fieldPose();

            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Motion tracks", spatial.isMotionTrackingActive());
            telemetry.addData("Field pose", field == null ? "—"
                    : String.format("(%.1f, %.1f) %.0f°",
                    field.getX(DistanceUnit.INCH),
                    field.getY(DistanceUnit.INCH),
                    field.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Elements", elements.size());
            telemetry.addData("Nearest element", VidarBlobUtil.formatSpatialPoint(
                    elements.isEmpty() ? null : elements.get(0)));
            telemetry.addData("Nearest ally", VidarBlobUtil.formatSpatialPoint(
                    allies.isEmpty() ? null : allies.get(0)));
            telemetry.addData("Nearest foe", VidarBlobUtil.formatSpatialPoint(
                    foes.isEmpty() ? null : foes.get(0)));
            telemetry.addData("Intake blocked", spatial.intakeBlocked());
            telemetry.update();
        }

        spatial.close();
    }
}
