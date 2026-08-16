package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroCorrectionTracker;
import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroPose;
import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroPoseBridge;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarAllianceSelector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Compilable sample: wire ViDAR to a pathing follower without a Pedro Maven dependency.
 *
 * <p>Uses an in-memory pose stand-in for {@code Follower.getPose()/setPose()}. In a Pedro project,
 * replace {@link #tickFollower} / {@link #applyFollowerPose} with your real {@code Follower}
 * (see {@code docs/PEDRO_INTEGRATION.md}).
 *
 * <p>Loop contract:
 * <ol>
 *   <li>Update pathing / odom (Pedro {@code follower.update()})</li>
 *   <li>{@code spatial.update()} - pin snapshot</li>
 *   <li>Read {@code elements()}/{@code foes()} for assists</li>
 *   <li>Sparse tag correction via {@link VidarPedroCorrectionTracker} (event-id gate)</li>
 * </ol>
 */
@TeleOp(name = "ViDAR: Pedro Bridge Sample", group = "ViDAR")
public class VidarPedroBridgeSampleOpMode extends VidarSpatialOpModeBase {

    private final AtomicReference<VidarPedroPose> followerPose =
            new AtomicReference<>(new VidarPedroPose(0, 0, 0));
    private final VidarPedroCorrectionTracker corrections = new VidarPedroCorrectionTracker();

    private VidarSpatial spatial;
    private VidarAllianceSelector alliance;

    @Override
    public void runOpMode() {
        alliance = new VidarAllianceSelector(hardwareMap);

        spatial = VidarSpatial.createWithBundledDefaults(
                hardwareMap,
                VidarPedroPoseBridge.asPose2DSupplier(followerPose::get),
                alliance::get);
        // Continuous field pose for world tracks — does NOT replace fusedFieldPose().
        spatial.setFieldPoseSupplier(VidarPedroPoseBridge.asPose2DSupplier(followerPose::get));

        telemetry.addLine("ViDAR <-> Pedro bridge sample (in-memory follower stand-in)");
        telemetry.addLine("Replace stand-in with Follower - see docs/PEDRO_INTEGRATION.md");
        telemetry.update();

        pollAllianceInit(alliance, gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            alliance.pollRuntime(gamepad1);

            tickFollower();
            spatial.update();

            VidarSpatialPoint nearest = spatial.nearestElement();
            boolean blocked = spatial.intakeBlocked();

            long correctionId = spatial.lastTagCorrectionNanos();
            Pose2D fusedAnchor = spatial.fusedFieldPose();
            Pose2D correctedNow = spatial.tagCorrectedFieldPoseNow();
            VidarPedroPose correction = corrections.poll(correctionId, correctedNow);
            if (correction != null) {
                applyFollowerPose(correction);
            }

            telemetry.addData("Alliance", alliance.formatStatus());
            telemetry.addData("Follower pose", followerPose.get());
            telemetry.addData("Fused anchor", formatFieldPose(fusedAnchor));
            telemetry.addData("Corrected now", formatFieldPose(correctedNow));
            telemetry.addData("Fuse→now Δ", deltaInches(fusedAnchor, correctedNow));
            telemetry.addData("Correction id", correctionId);
            telemetry.addData("Correction applied", correction != null ? correction : "-");
            telemetry.addData("Nearest element", nearest == null ? "-" : nearest);
            telemetry.addData("Intake blocked", blocked);
            telemetry.update();
        }

        spatial.close();
    }

    /**
     * Stand-in for {@code follower.update()} — drifts slowly so Driver Station can show fused
     * anchor vs corrected-now diverge between sparse tag fixes.
     */
    private void tickFollower() {
        // Pedro: follower.update();
        VidarPedroPose cur = followerPose.get();
        followerPose.set(new VidarPedroPose(cur.x + 0.05, cur.y, cur.headingRad));
    }

    private void applyFollowerPose(VidarPedroPose pose) {
        // Pedro: follower.setPose(new Pose(pose.x, pose.y, pose.headingRad));
        followerPose.set(pose);
    }

    private static String deltaInches(Pose2D fused, Pose2D corrected) {
        if (fused == null || corrected == null) {
            return "-";
        }
        double dx = corrected.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)
                - fused.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double dy = corrected.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)
                - fused.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double dh = corrected.getHeading(AngleUnit.DEGREES) - fused.getHeading(AngleUnit.DEGREES);
        return String.format("dx=%.2f dy=%.2f dh=%.1f", dx, dy, dh);
    }
}
