package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.detect.VidarBlobUtil;
import org.firstinspires.ftc.teamcode.vidar.model.VidarOffensiveLaneAnalysis;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarAllianceSelector;
import org.firstinspires.ftc.teamcode.vidar.world.VidarSpatialTrack;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/**
 * Lesson 3 — full elements / allies / foes map (no motor output).
 *
 * <p>Wire odom at {@link VidarSpatial#create} to enable motion-corrected tracks.
 *
 * <p>INIT: color sensor on own sign auto-sets alliance; hold Y/B to override.
 */
@TeleOp(name = "ViDAR: Spatial Map", group = "ViDAR")
public class VidarAutoSeekOpMode extends VidarSpatialOpModeBase {

    private static final int TELEMETRY_CAP = 4;

    private VidarSpatial spatial;
    private VidarAllianceSelector alliance;

    @Override
    public void runOpMode() {
        alliance = new VidarAllianceSelector(hardwareMap);
        spatial = VidarSpatial.createWithBundledDefaults(hardwareMap, null, alliance::get);

        telemetry.addLine("ViDAR Spatial Map — three groups (no motors)");
        telemetry.addLine("Calibrate robot.json — docs/CALIBRATION_CHECKLIST.md");
        if (!spatial.diagnostics().warnings.isEmpty()) {
            telemetry.addLine(spatial.diagnostics().warnings.get(0));
        }
        telemetry.update();

        pollAllianceInit(alliance, gamepad1);

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
            telemetry.addData("Field pose", formatFieldPose(field));
            telemetry.addData("Elements", VidarBlobUtil.formatSpatialPointList(elements, TELEMETRY_CAP));
            telemetry.addData("Allies", VidarBlobUtil.formatSpatialPointList(allies, TELEMETRY_CAP));
            telemetry.addData("Foes", VidarBlobUtil.formatSpatialPointList(foes, TELEMETRY_CAP));
            telemetry.addData("Intake blocked", spatial.intakeBlocked());
            VidarOffensiveLaneAnalysis lane = spatial.offensiveLaneAnalysis();
            telemetry.addData("Offensive lane",
                    String.format("%s (L%d C%d R%d)",
                            lane.recommended,
                            lane.leftCount,
                            lane.centerCount,
                            lane.rightCount));
            telemetry.addData("Tracks", spatial.trackCount());
            if (spatial.isMotionTrackingActive() && spatial.trackCount() > 0) {
                List<VidarSpatialTrack> elementTracks =
                        spatial.runtime().world().getTracks(VidarWorldModel.Kind.ELEMENT);
                if (!elementTracks.isEmpty()) {
                    telemetry.addData("Sample track",
                            VidarBlobUtil.formatWorldTrack(elementTracks.get(0)));
                }
            }
            telemetry.update();
        }

        spatial.close();
    }
}
