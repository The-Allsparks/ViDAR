package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.VidarTeamConfig;
import org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.fusion.FieldPoseContext;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarGlobalVisionWorker;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagDecodeWorker;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * Instance-scoped ViDAR orchestrator — owns vision, world model, decode budget, and background workers.
 */
public final class VidarSession {

    private final VidarMultiVision vision;
    private final VidarWorldModel world;
    private final FieldPoseContext fieldPoseContext;
    private final TagDecodeBudget tagDecodeBudget;
    private final VidarTagDecodeWorker tagDecodeWorker;
    private final VidarDiagnostics.ConfigSource configSource;

    private VidarSession(
            VidarMultiVision vision,
            VidarWorldModel world,
            FieldPoseContext fieldPoseContext,
            TagDecodeBudget tagDecodeBudget,
            VidarTagDecodeWorker tagDecodeWorker,
            VidarDiagnostics.ConfigSource configSource) {
        this.vision = vision;
        this.world = world;
        this.fieldPoseContext = fieldPoseContext;
        this.tagDecodeBudget = tagDecodeBudget;
        this.tagDecodeWorker = tagDecodeWorker;
        this.configSource = configSource;
        fieldPoseContext.bindVision(vision);
        world.setFieldPoseSupplier(fieldPoseContext.worldTrackFieldPoseSupplier());
    }

    public static VidarSession create(HardwareMap hardwareMap) {
        return create(hardwareMap, null, null);
    }

    public static VidarSession create(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        try {
            return create(
                    hardwareMap,
                    VidarTeamConfig.loadRobot(hardwareMap),
                    VidarTeamConfig.loadSeason(hardwareMap),
                    odomSupplier,
                    allianceSupplier,
                    VidarDiagnostics.ConfigSource.TEAM_ASSETS);
        } catch (IOException e) {
            throw new VidarConfigException(
                    "Missing ViDAR config assets. Copy season.json and robot.json to "
                            + "TeamCode/src/main/assets/vidar/ or call createWithBundledDefaults().",
                    e);
        }
    }

    public static VidarSession createWithBundledDefaults(HardwareMap hardwareMap) {
        return createWithBundledDefaults(hardwareMap, null, null);
    }

    public static VidarSession createWithBundledDefaults(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return create(
                hardwareMap,
                VidarTeamConfig.defaultRobot(),
                VidarTeamConfig.defaultSeason(),
                odomSupplier,
                allianceSupplier,
                VidarDiagnostics.ConfigSource.BUNDLED_DEFAULTS);
    }

    public static VidarSession create(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return create(
                hardwareMap,
                robot,
                season,
                odomSupplier,
                allianceSupplier,
                VidarDiagnostics.ConfigSource.TEAM_ASSETS);
    }

    private static VidarSession create(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier,
            VidarDiagnostics.ConfigSource configSource) {
        FieldPoseContext fieldPoseContext = new FieldPoseContext(odomSupplier);
        TagDecodeBudget budget = new TagDecodeBudget();
        VidarTagDecodeWorker worker = null;
        if (VidarTagConfig.ENABLED && VidarConfig.ASYNC_TAG_DECODE_ENABLED) {
            worker = new VidarTagDecodeWorker();
            worker.ensureStarted();
        }
        VidarMultiVision vision = new VidarMultiVision(
                hardwareMap, robot, season, odomSupplier, allianceSupplier, budget, worker);
        VidarWorldModel world = new VidarWorldModel(odomSupplier, null);
        return new VidarSession(vision, world, fieldPoseContext, budget, worker, configSource);
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> supplier) {
        fieldPoseContext.setExternalFieldPoseSupplier(supplier);
    }

    public void setFieldPosePrior(Pose2D prior) {
        vision.setFieldPosePrior(prior);
    }

    public void setMotionTrackingEnabled(boolean enabled) {
        world.setMotionTrackingEnabled(enabled);
    }

    public void update() {
        if (fieldPoseContext.odomSupplier() != null) {
            vision.recordOdom(fieldPoseContext.odomSupplier().get());
        }
        vision.update();
        world.update(vision, System.nanoTime());
    }

    public void close() {
        vision.close();
        if (tagDecodeWorker != null) {
            tagDecodeWorker.shutdownAndJoin();
        }
    }

    public VidarMultiVision vision() {
        return vision;
    }

    public VidarWorldModel world() {
        return world;
    }

    public FieldPoseContext fieldPoseContext() {
        return fieldPoseContext;
    }

    public TagDecodeBudget tagDecodeBudget() {
        return tagDecodeBudget;
    }

    public VidarTagDecodeWorker tagDecodeWorker() {
        return tagDecodeWorker;
    }

    public VidarDiagnostics.ConfigSource configSource() {
        return configSource;
    }

    public VidarGlobalVisionWorker globalVisionWorker() {
        return vision.globalVisionWorker();
    }
}
