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
        this.fusionMaxRankedElements = VidarConfig.FUSION_MAX_RANKED_ELEMENTS;
        this.defaultMaxRankedElements = VidarConfig.DEFAULT_MAX_RANKED_ELEMENTS;
        this.worldMergeRadiusIn = VidarConfig.WORLD_MERGE_RADIUS_IN;
        this.worldTrackGateRadiusIn = VidarConfig.WORLD_TRACK_GATE_RADIUS_IN;
        this.worldBlockRangeIn = VidarConfig.WORLD_BLOCK_RANGE_IN;
        this.worldBlockConeDeg = VidarConfig.WORLD_BLOCK_CONE_DEG;
        this.motionTrackingEnabled = VidarConfig.WORLD_MOTION_TRACKING_ENABLED;
        this.asyncTagDecodeEnabled = VidarConfig.ASYNC_TAG_DECODE_ENABLED;
        this.globalVisionWorkerEnabled = VidarConfig.GLOBAL_VISION_WORKER_ENABLED;
    }

    public static VidarSettings bundledDefaults() {
        return new VidarSettings(VidarConfigLoader.defaultRobot(), VidarConfigLoader.defaultSeason());
    }
}
