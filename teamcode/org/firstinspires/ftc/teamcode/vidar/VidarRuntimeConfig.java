package org.firstinspires.ftc.teamcode.vidar;

/**
 * Runtime-tunable ViDAR parameters (safe to adjust from an OpMode loop or gamepad tuner).
 */
public final class VidarRuntimeConfig {

    private volatile int defaultMaxRankedElements = VidarConfig.DEFAULT_MAX_RANKED_ELEMENTS;
    private volatile int fusionMaxRankedElements = VidarConfig.FUSION_MAX_RANKED_ELEMENTS;

    private volatile int temporalConfirmFrames = VidarConfig.TEMPORAL_CONFIRM_FRAMES;
    private volatile double temporalStrongConfidence = VidarConfig.TEMPORAL_STRONG_CONFIDENCE;
    private volatile double temporalMaxJump = VidarConfig.TEMPORAL_MAX_JUMP;

    public int defaultMaxRankedElements() {
        return defaultMaxRankedElements;
    }

    public void setDefaultMaxRankedElements(int n) {
        defaultMaxRankedElements = clamp(n, 1, VidarConfig.MAX_RANKED_ELEMENTS_CAP);
    }

    public int fusionMaxRankedElements() {
        return fusionMaxRankedElements;
    }

    public void setFusionMaxRankedElements(int n) {
        fusionMaxRankedElements = clamp(n, 1, VidarConfig.MAX_RANKED_ELEMENTS_CAP);
    }

    public int temporalConfirmFrames() {
        return temporalConfirmFrames;
    }

    public void setTemporalConfirmFrames(int frames) {
        temporalConfirmFrames = Math.max(1, frames);
    }

    public double temporalStrongConfidence() {
        return temporalStrongConfidence;
    }

    public void setTemporalStrongConfidence(double confidence) {
        temporalStrongConfidence = Math.max(0, Math.min(1, confidence));
    }

    public double temporalMaxJump() {
        return temporalMaxJump;
    }

    public void setTemporalMaxJump(double value) {
        temporalMaxJump = Math.max(1, value);
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
