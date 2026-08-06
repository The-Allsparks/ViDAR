package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Rigid 3D transform ({@code destination_T_source}): maps points and directions from
 * {@code source} into {@code destination}.
 *
 * <p>Point: {@code p_dest = R * p_source + t}. Direction (ray): {@code d_dest = R * d_source}
 * without translation.
 */
public final class VidarTransform3D {

    public final VidarFrameId sourceFrame;
    public final VidarFrameId destFrame;
    public final String sourceLabel;
    public final String destLabel;
    public final VidarRotation3D rotation;
    public final VidarVec3 translation;

    public VidarTransform3D(
            VidarFrameId destFrame,
            VidarFrameId sourceFrame,
            VidarRotation3D rotation,
            VidarVec3 translation) {
        this(destFrame, sourceFrame, null, null, rotation, translation);
    }

    public VidarTransform3D(
            VidarFrameId destFrame,
            VidarFrameId sourceFrame,
            String destLabel,
            String sourceLabel,
            VidarRotation3D rotation,
            VidarVec3 translation) {
        this.destFrame = destFrame;
        this.sourceFrame = sourceFrame;
        this.destLabel = destLabel;
        this.sourceLabel = sourceLabel;
        this.rotation = rotation == null ? VidarRotation3D.identity() : rotation;
        this.translation = translation == null ? VidarVec3.zero() : translation;
    }

    public static VidarTransform3D identity(VidarFrameId frame) {
        return new VidarTransform3D(frame, frame, VidarRotation3D.identity(), VidarVec3.zero());
    }

    /** Inverts {@code dest_T_source} to {@code source_T_dest}. */
    public VidarTransform3D inverse() {
        VidarRotation3D invR = rotation.inverse();
        VidarVec3 invT = invR.rotate(translation.scaled(-1));
        return new VidarTransform3D(
                sourceFrame, destFrame, sourceLabel, destLabel, invR, invT);
    }

    /**
     * Chains transforms: {@code (A_T_C) = (A_T_B) * (B_T_C)}.
     * {@code other} must have {@code other.destFrame == this.sourceFrame} logically.
     */
    public VidarTransform3D compose(VidarTransform3D other) {
        VidarRotation3D r = rotation.times(other.rotation);
        VidarVec3 t = rotation.rotate(other.translation).plus(translation);
        return new VidarTransform3D(
                destFrame,
                other.sourceFrame,
                destLabel,
                other.sourceLabel,
                r,
                t);
    }

    public VidarVec3 transformPoint(VidarVec3 pointInSource) {
        return rotation.rotate(pointInSource).plus(translation);
    }

    public VidarVec3 transformDirection(VidarVec3 directionInSource) {
        return rotation.rotate(directionInSource).normalized();
    }

    public double[] transformPointArray(double x, double y, double z) {
        VidarVec3 out = transformPoint(new VidarVec3(x, y, z));
        return new double[] { out.x, out.y, out.z };
    }

    public double[] transformDirectionArray(double x, double y, double z) {
        VidarVec3 out = transformDirection(new VidarVec3(x, y, z));
        return new double[] { out.x, out.y, out.z };
    }

    public boolean isFinite() {
        return rotation.isFinite() && translation.isFinite();
    }

    public String notationName() {
        String dest = destLabel != null ? destLabel : destFrame.name();
        String src = sourceLabel != null ? sourceLabel : sourceFrame.name();
        return dest + "_T_" + src;
    }
}
