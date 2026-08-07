package org.firstinspires.ftc.teamcode.vidar.config;

/**
 * Robot body size for robot-frame output (+X forward, +Y left, +Z up from floor at center).
 */
public final class VidarRobotDimensions {

    /** Forward-back outer size (inches). */
    public final double length;
    /** Left-right outer size (inches). */
    public final double width;
    /** Floor to highest point (inches). */
    public final double height;

    public VidarRobotDimensions(double length, double width, double height) {
        this.length = length;
        this.width = width;
        this.height = height;
    }

    /** Half-extents from robot center on the floor. */
    public double halfLength() {
        return length * 0.5;
    }

    public double halfWidth() {
        return width * 0.5;
    }
}
