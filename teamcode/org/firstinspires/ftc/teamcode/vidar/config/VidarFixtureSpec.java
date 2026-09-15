package org.firstinspires.ftc.teamcode.vidar.config;

/**
 * One known field structure from season JSON {@code fixtures[]}.
 *
 * <p>Lives in {@code vidar.config} so a later {@code vidar.fixture} package can depend on
 * config without config importing detect, fusion, world, tag, or schedule.
 */
public final class VidarFixtureSpec {

    public final String id;
    public final String label;
    public final VidarFixtureLocalizationMode localization;
    public final double xIn;
    public final double yIn;
    public final double zIn;
    public final double yawDeg;
    public final double pitchDeg;
    public final double rollDeg;
    /** Detector ids resolved later; empty means state-only / no detector yet. */
    public final String[] detectors;
    /** Tag identities for {@link VidarFixtureLocalizationMode#APRIL_TAG}; empty otherwise. */
    public final int[] tagIds;

    public VidarFixtureSpec(
            String id,
            String label,
            VidarFixtureLocalizationMode localization,
            double xIn,
            double yIn,
            double zIn,
            double yawDeg,
            double pitchDeg,
            double rollDeg,
            String[] detectors,
            int[] tagIds) {
        if (id == null || id.trim().isEmpty()) {
            throw new IllegalArgumentException("Fixture id is required");
        }
        if (localization == null) {
            throw new IllegalArgumentException("Fixture localization is required: " + id);
        }
        this.id = id;
        this.label = (label == null || label.isEmpty()) ? id : label;
        this.localization = localization;
        this.xIn = xIn;
        this.yIn = yIn;
        this.zIn = zIn;
        this.yawDeg = yawDeg;
        this.pitchDeg = pitchDeg;
        this.rollDeg = rollDeg;
        this.detectors = detectors == null ? new String[0] : detectors.clone();
        this.tagIds = tagIds == null ? new int[0] : tagIds.clone();
    }

    public boolean hasFieldPosition() {
        return !Double.isNaN(xIn) && !Double.isNaN(yIn) && !Double.isNaN(zIn);
    }
}
