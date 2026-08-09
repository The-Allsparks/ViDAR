package org.firstinspires.ftc.teamcode.vidar;

/**
 * Thrown when ViDAR cannot load required team configuration assets.
 *
 * <p>Place {@code season.json} and {@code robot.json} under {@code TeamCode/src/main/assets/vidar/},
 * or call {@link VidarSpatial#createWithBundledDefaults} for bundled fallbacks.
 */
public final class VidarConfigException extends RuntimeException {

    public VidarConfigException(String message, Throwable cause) {
        super(message, cause);
    }

    public VidarConfigException(String message) {
        super(message);
    }
}
