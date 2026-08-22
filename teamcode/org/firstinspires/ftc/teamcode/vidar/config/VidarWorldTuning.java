package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.json.JSONObject;

/**
 * World-model tuning from season JSON ({@code "world": { ... }}).
 * Distances use the season file's {@code distanceUnit}.
 */
public final class VidarWorldTuning {

    /** Library fallback when JSON omits {@code world.mergeRadius}. */
    public static final double LIBRARY_DEFAULT_MERGE_RADIUS = 8.0;

    public final double mergeRadius;
    public final double trackGateRadius;
    public final double trackGateRadiusFoe;
    public final double blockRange;
    public final double blockConeDeg;

    public VidarWorldTuning(
            double mergeRadius,
            double trackGateRadius,
            double trackGateRadiusFoe,
            double blockRange,
            double blockConeDeg) {
        this.mergeRadius = mergeRadius;
        this.trackGateRadius = trackGateRadius;
        this.trackGateRadiusFoe = trackGateRadiusFoe;
        this.blockRange = blockRange;
        this.blockConeDeg = blockConeDeg;
    }

    public static VidarWorldTuning libraryDefaults() {
        return new VidarWorldTuning(
                LIBRARY_DEFAULT_MERGE_RADIUS,
                VidarConfig.WORLD_TRACK_GATE_RADIUS_IN,
                VidarConfig.WORLD_TRACK_GATE_RADIUS_FOE_IN,
                VidarConfig.WORLD_BLOCK_RANGE_IN,
                VidarConfig.WORLD_BLOCK_CONE_DEG);
    }

    static VidarWorldTuning parse(JSONObject world) {
        VidarWorldTuning defaults = libraryDefaults();
        if (world == null) {
            return defaults;
        }
        return new VidarWorldTuning(
                world.optDouble("mergeRadius", defaults.mergeRadius),
                world.optDouble("trackGateRadius", defaults.trackGateRadius),
                world.optDouble("trackGateRadiusFoe", defaults.trackGateRadiusFoe),
                world.optDouble("blockRange", defaults.blockRange),
                world.optDouble("blockConeDeg", defaults.blockConeDeg));
    }
}
