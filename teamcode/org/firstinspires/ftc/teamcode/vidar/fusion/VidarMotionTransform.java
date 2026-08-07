package org.firstinspires.ftc.teamcode.vidar.fusion;

/**
 * Robot-frame motion transform for correcting tracks between observation updates.
 */
public final class VidarMotionTransform {

    public final double deltaXIn;
    public final double deltaYIn;
    public final double deltaHeadingDeg;

    public VidarMotionTransform(double deltaXIn, double deltaYIn, double deltaHeadingDeg) {
        this.deltaXIn = deltaXIn;
        this.deltaYIn = deltaYIn;
        this.deltaHeadingDeg = deltaHeadingDeg;
    }

    public static VidarMotionTransform identity() {
        return new VidarMotionTransform(0, 0, 0);
    }

    /**
     * Transform a point from the previous robot frame into the current robot frame.
     * Applies rotation then translation (odom delta in field frame expressed in robot frame).
     */
    public double[] transformPoint(double prevRobotX, double prevRobotY) {
        double rad = Math.toRadians(-deltaHeadingDeg);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        double rotatedX = prevRobotX * cos - prevRobotY * sin;
        double rotatedY = prevRobotX * sin + prevRobotY * cos;
        return new double[] {
                rotatedX - deltaXIn,
                rotatedY - deltaYIn
        };
    }

    public static VidarMotionTransform fromOdomDelta(
            double prevX, double prevY, double prevHeadingDeg,
            double currX, double currY, double currHeadingDeg) {
        double dx = currX - prevX;
        double dy = currY - prevY;
        double dHeading = normalizeDeg(currHeadingDeg - prevHeadingDeg);
        double rad = Math.toRadians(-prevHeadingDeg);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        double robotDx = dx * cos - dy * sin;
        double robotDy = dx * sin + dy * cos;
        return new VidarMotionTransform(robotDx, robotDy, dHeading);
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
