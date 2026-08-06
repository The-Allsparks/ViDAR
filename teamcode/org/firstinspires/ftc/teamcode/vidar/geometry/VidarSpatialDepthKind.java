package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * How depth was obtained for a spatial observation — avoids fabricating 3D points from bearings.
 */
public enum VidarSpatialDepthKind {
    /** Slant range or triangulation with explicit distance. */
    MEASURED,
    /** Range inferred from size, floor LUT, or ground-plane intersection. */
    INFERRED,
    /** Bearing or pixel ray only — no trustworthy depth. */
    BEARING_ONLY,
    /** Depth unavailable or rejected. */
    UNAVAILABLE
}
