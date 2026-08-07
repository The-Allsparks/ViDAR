package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Immutable 3-vector for translations, points, and directions.
 */
public final class VidarVec3 {

    public final double x;
    public final double y;
    public final double z;

    public VidarVec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static VidarVec3 zero() {
        return new VidarVec3(0, 0, 0);
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public VidarVec3 normalized() {
        double len = length();
        if (len <= 1e-9) {
            return new VidarVec3(0, 0, 1);
        }
        return new VidarVec3(x / len, y / len, z / len);
    }

    public VidarVec3 plus(VidarVec3 other) {
        return new VidarVec3(x + other.x, y + other.y, z + other.z);
    }

    public VidarVec3 minus(VidarVec3 other) {
        return new VidarVec3(x - other.x, y - other.y, z - other.z);
    }

    public VidarVec3 scaled(double s) {
        return new VidarVec3(x * s, y * s, z * s);
    }

    public double dot(VidarVec3 other) {
        return x * other.x + y * other.y + z * other.z;
    }

    @Override
    public String toString() {
        return String.format("(%.4f, %.4f, %.4f)", x, y, z);
    }
}
