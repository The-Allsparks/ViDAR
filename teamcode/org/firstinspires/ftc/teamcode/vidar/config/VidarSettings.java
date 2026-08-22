package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;

/**
 * Effective ViDAR tuning merged from season JSON, robot JSON, and hardware fallbacks.
 * Prefer loading team assets via {@link VidarConfigLoader} rather than {@link VidarConfig} constants.
 */
public final class VidarSettings {

    public final VidarRobotConfig robot;
    public final VidarSeasonConfig season;

    public final double minElementConfidence;
    public final double minPlateConfidence;
    public final double maxRangeMismatchRatio;
    public final int fusionMaxRankedElements;
    public final int defaultMaxRankedElements;
    public final double worldMergeRadiusIn;
    public final double worldTrackGateRadiusIn;
    public final double worldBlockRangeIn;
    public final double worldBlockConeDeg;
    public final boolean motionTrackingEnabled;
    public final boolean asyncTagDecodeEnabled;
    public final boolean globalVisionWorkerEnabled;

    public VidarSettings(VidarRobotConfig robot, VidarSeasonConfig season) {
        this.robot = robot != null ? robot : VidarConfigLoader.defaultRobot();
        this.season = season != null ? season : VidarConfigLoader.defaultSeason();
        this.minElementConfidence = this.season.minElementConfidence;
        this.minPlateConfidence = this.season.minPlateConfidence;
        this.maxRangeMismatchRatio = this.season.maxRangeMismatchRatio;
        this.fusionMaxRankedElements = this.season.fusionMaxRankedElements;
        this.defaultMaxRankedElements = this.season.defaultMaxRankedElements;
        this.worldMergeRadiusIn = this.season.world.mergeRadius;
        this.worldTrackGateRadiusIn = this.season.world.trackGateRadius;
        this.worldBlockRangeIn = this.season.world.blockRange;
        this.worldBlockConeDeg = this.season.world.blockConeDeg;
        this.motionTrackingEnabled = VidarConfig.WORLD_MOTION_TRACKING_ENABLED;
        this.asyncTagDecodeEnabled = VidarConfig.ASYNC_TAG_DECODE_ENABLED;
        this.globalVisionWorkerEnabled = VidarConfig.GLOBAL_VISION_WORKER_ENABLED;
    }

    public static VidarSettings bundledDefaults() {
        return new VidarSettings(VidarConfigLoader.defaultRobot(), VidarConfigLoader.defaultSeason());
    }
}
