package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * 3D rotation stored as an orthonormal 3×3 matrix (row-major, no heap allocs after construction).
 *
 * <p>ViDAR uses <b>intrinsic</b> roll-pitch-yaw about fixed body axes in this multiply order:
 * {@code R = Rz(yaw) * Rx(pitch) * Rz(roll)} applied as {@code v_out = R * v_in}.
 *
 * <p>Camera mount rotations compose on top of a fixed optical-to-robot-base mapping; see
 * {@link VidarTransformRegistry}.
 */
public final class VidarRotation3D {

    /** Row-major 3×3 rotation matrix. */
    public final double[] m;

    private VidarRotation3D(double[] matrix) {
        m = matrix;
    }

    public static VidarRotation3D identity() {
        return new VidarRotation3D(new double[] {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
        });
    }

    /**
     * Intrinsic roll (about +Z), pitch (about +X), yaw (about +Z) in degrees.
     * Multiply order: {@code Rz(yaw) * Rx(pitch) * Rz(roll)}.
     */
    public static VidarRotation3D fromRollPitchYawDeg(double rollDeg, double pitchDeg, double yawDeg) {
        VidarRotation3D rRoll = rotateZ(Math.toRadians(rollDeg));
        VidarRotation3D rPitch = rotateX(Math.toRadians(pitchDeg));
        VidarRotation3D rYaw = rotateZ(Math.toRadians(yawDeg));
        return rYaw.times(rPitch).times(rRoll);
    }

    public static VidarRotation3D rotateX(double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new VidarRotation3D(new double[] {
                1, 0, 0,
                0, c, -s,
                0, s, c
        });
    }

    public static VidarRotation3D rotateZ(double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new VidarRotation3D(new double[] {
                c, -s, 0,
                s, c, 0,
                0, 0, 1
        });
    }

    /** Fixed mapping from camera optical (+X right, +Y down, +Z forward) to robot axes at bearing 0. */
    public static VidarRotation3D opticalToRobotBase() {
        return new VidarRotation3D(new double[] {
                0, 0, 1,
                -1, 0, 0,
                0, -1, 0
        });
    }

    public VidarRotation3D times(VidarRotation3D other) {
        double[] out = new double[9];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                out[r * 3 + c] =
                        m[r * 3 + 0] * other.m[0 * 3 + c]
                                + m[r * 3 + 1] * other.m[1 * 3 + c]
                                + m[r * 3 + 2] * other.m[2 * 3 + c];
            }
        }
        return new VidarRotation3D(out);
    }

    public VidarRotation3D inverse() {
        // Orthonormal rotation: inverse = transpose
        return new VidarRotation3D(new double[] {
                m[0], m[3], m[6],
                m[1], m[4], m[7],
                m[2], m[5], m[8]
        });
    }

    public VidarVec3 rotate(VidarVec3 v) {
        return new VidarVec3(
                m[0] * v.x + m[1] * v.y + m[2] * v.z,
                m[3] * v.x + m[4] * v.y + m[5] * v.z,
                m[6] * v.x + m[7] * v.y + m[8] * v.z);
    }

    public double[] rotateArray(double x, double y, double z) {
        VidarVec3 out = rotate(new VidarVec3(x, y, z));
        return new double[] { out.x, out.y, out.z };
    }

    public boolean isFinite() {
        for (double v : m) {
            if (!Double.isFinite(v)) {
                return false;
            }
        }
        return true;
    }
}
