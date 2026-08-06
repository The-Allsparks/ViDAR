package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

/**
 * Builds an FTC {@link AprilTagLibrary} from season JSON tag definitions.
 */
public final class VidarAprilTagLibraryFactory {

    private VidarAprilTagLibraryFactory() {}

    public static AprilTagLibrary build(VidarSeasonConfig season) {
        if (season == null || season.aprilTags.length == 0) {
            return sdkCurrentGameLibrary();
        }
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (VidarAprilTagSpec spec : season.aprilTags) {
            builder.addTag(spec.toMetadata());
        }
        return builder.build();
    }

    private static AprilTagLibrary sdkCurrentGameLibrary() {
        try {
            return AprilTagGameDatabase.getCurrentGameTagLibrary();
        } catch (RuntimeException ex) {
            return new AprilTagLibrary.Builder().build();
        }
    }
}
