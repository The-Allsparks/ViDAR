package org.firstinspires.ftc.teamcode.vidar;

/**
 * FTC alliance color for friend/foe plate classification.
 */
public enum VidarAlliance {
    RED,
    BLUE,
    UNKNOWN;

    public boolean isFoe(VidarAlliance ourAlliance) {
        if (ourAlliance == null || ourAlliance == UNKNOWN || this == UNKNOWN) {
            return false;
        }
        return this != ourAlliance;
    }

    public boolean isAlly(VidarAlliance ourAlliance) {
        if (ourAlliance == null || ourAlliance == UNKNOWN || this == UNKNOWN) {
            return false;
        }
        return this == ourAlliance;
    }
}
