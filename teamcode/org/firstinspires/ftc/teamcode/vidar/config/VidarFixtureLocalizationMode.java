package org.firstinspires.ftc.teamcode.vidar.config;

/**
 * How a season-JSON fixture is localized. Detector implementations are later issues;
 * this enum is config-only.
 */
public enum VidarFixtureLocalizationMode {
    /** Tagged fixture; optional {@code tagIds} identify the AprilTags. */
    APRIL_TAG,
    /** Known field pose; project an ROI from robot pose at capture time. */
    STATIC_FIELD,
    /** Untagged visual landmark; geometry lands in a later issue. */
    VISUAL;

    /**
     * Parse a season-JSON {@code localization} string.
     * Unknown values fail loudly (do not default to tag).
     */
    public static VidarFixtureLocalizationMode fromJson(String raw) {
        if (raw == null || raw.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "Fixture localization is required (april_tag, static_field, or visual)");
        }
        String key = raw.trim().toLowerCase();
        switch (key) {
            case "april_tag":
                return APRIL_TAG;
            case "static_field":
                return STATIC_FIELD;
            case "visual":
                return VISUAL;
            default:
                throw new IllegalArgumentException(
                        "Unknown fixture localization \"" + raw
                                + "\" (expected april_tag, static_field, or visual)");
        }
    }
}
