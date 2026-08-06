package org.firstinspires.ftc.teamcode.vidar.config;

/**
 * Playing-field dimensions for season tag maps (+X right, +Y forward from field center).
 */
public final class VidarFieldSpec {

    public final double length;
    public final double width;

    public VidarFieldSpec(double length, double width) {
        this.length = length;
        this.width = width;
    }

    public double halfLength() {
        return length * 0.5;
    }

    public double halfWidth() {
        return width * 0.5;
    }
}
