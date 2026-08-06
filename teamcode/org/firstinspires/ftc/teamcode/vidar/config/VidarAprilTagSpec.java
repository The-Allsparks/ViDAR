package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

/**
 * One AprilTag on the field in FTC world coordinates (inches, degrees).
 * Origin at field center; +X right, +Y forward, +Z up.
 */
public final class VidarAprilTagSpec {

    public final int id;
    public final String name;
    /** Black square outer size (inches). */
    public final double size;
    public final double xIn;
    public final double yIn;
    public final double zIn;
    public final double yawDeg;
    public final double pitchDeg;
    public final double rollDeg;
    /** When false, detections may be used for motif/targeting but not localization fusion. */
    public final boolean localization;

    public VidarAprilTagSpec(
            int id,
            String name,
            double size,
            double xIn,
            double yIn,
            double zIn,
            double yawDeg,
            double pitchDeg,
            double rollDeg,
            boolean localization) {
        this.id = id;
        this.name = name == null ? "tag_" + id : name;
        this.size = size;
        this.xIn = xIn;
        this.yIn = yIn;
        this.zIn = zIn;
        this.yawDeg = yawDeg;
        this.pitchDeg = pitchDeg;
        this.rollDeg = rollDeg;
        this.localization = localization;
    }

    public boolean hasFieldPosition() {
        return !Double.isNaN(xIn) && !Double.isNaN(yIn) && !Double.isNaN(zIn);
    }

    /** Bearing from a robot field pose to this tag (degrees, robot-relative forward = 0). */
    public double bearingFromFieldPose(double robotX, double robotY, double robotHeadingDeg) {
        if (!hasFieldPosition()) {
            return Double.NaN;
        }
        double fieldBearing = Math.toDegrees(Math.atan2(yIn - robotY, xIn - robotX));
        return normalizeDeg(fieldBearing - robotHeadingDeg);
    }

    public AprilTagMetadata toMetadata() {
        AprilTagMetadata meta = new AprilTagMetadata(id, name, size, DistanceUnit.INCH);
        if (hasFieldPosition()) {
            meta.fieldPosition = new Position(DistanceUnit.INCH, (float) xIn, (float) yIn, (float) zIn);
            meta.fieldOrientation = Rotation.getQuaternion(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES,
                    (float) rollDeg,
                    (float) pitchDeg,
                    (float) yawDeg);
        }
        return meta;
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
