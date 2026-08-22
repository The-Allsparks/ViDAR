package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;



/**

 * Per-season game-piece definitions (elements, plates, fusion thresholds) and field AprilTag map.

 * Teams load a JSON season file and pass it when creating {@link org.firstinspires.ftc.teamcode.vidar.VidarSpatial}.

 */

public final class VidarSeasonConfig {



    public final String seasonId;

    public final String seasonName;

    public final VidarFieldSpec field;

    public final VidarElementSpec[] elements;

    public final VidarPlateSpec[] plates;

    public final VidarAprilTagSpec[] aprilTags;

    /** Default black-square size when a tag entry omits {@code size}. */

    public final double defaultTagSize;

    public final double minElementConfidence;

    public final double minPlateConfidence;

    public final double maxRangeMismatchRatio;

    public final int fusionMaxRankedElements;

    public final int defaultMaxRankedElements;

    public final VidarWorldTuning world;

    /** Unit for all distance fields in this season file and runtime observations (default inches). */
    public final org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit distanceUnit;

    private final AprilTagLibrary aprilTagLibrary;



    public VidarSeasonConfig(

            String seasonId,

            String seasonName,

            VidarFieldSpec field,

            VidarElementSpec[] elements,

            VidarPlateSpec[] plates,

            VidarAprilTagSpec[] aprilTags,

            double defaultTagSize,

            double minElementConfidence,

            double minPlateConfidence,

            double maxRangeMismatchRatio,

            int fusionMaxRankedElements,

            int defaultMaxRankedElements,

            VidarWorldTuning world,

            org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit distanceUnit) {

        this.seasonId = seasonId;

        this.seasonName = seasonName;

        this.field = field;

        this.elements = elements == null ? new VidarElementSpec[0] : elements;

        this.plates = plates == null ? new VidarPlateSpec[0] : plates;

        this.aprilTags = aprilTags == null ? new VidarAprilTagSpec[0] : aprilTags;

        this.defaultTagSize = defaultTagSize;

        this.minElementConfidence = minElementConfidence;

        this.minPlateConfidence = minPlateConfidence;

        this.maxRangeMismatchRatio = maxRangeMismatchRatio;

        this.fusionMaxRankedElements = fusionMaxRankedElements;

        this.defaultMaxRankedElements = defaultMaxRankedElements;

        this.world = world != null ? world : VidarWorldTuning.libraryDefaults();

        this.distanceUnit = distanceUnit == null
                ? org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit.IN
                : distanceUnit;

        this.aprilTagLibrary = VidarAprilTagLibraryFactory.build(this);

    }



    public AprilTagLibrary aprilTagLibrary() {

        return aprilTagLibrary;

    }



    /** Primary element — first entry in {@link #elements}. */

    public VidarElementSpec primaryElement() {

        if (elements.length == 0) {

            throw new IllegalStateException("Season config has no elements: " + seasonId);

        }

        return elements[0];

    }



    public VidarPlateSpec plateSpec(VidarAlliance alliance) {

        for (VidarPlateSpec spec : plates) {

            if (spec.alliance == alliance) {

                return spec;

            }

        }

        throw new IllegalArgumentException("No plate spec for alliance " + alliance);

    }



    public VidarAprilTagSpec tagById(int id) {

        for (VidarAprilTagSpec spec : aprilTags) {

            if (spec.id == id) {

                return spec;

            }

        }

        return null;

    }



    public boolean useTagForLocalization(int tagId) {

        VidarAprilTagSpec spec = tagById(tagId);

        if (spec != null) {

            return spec.localization;

        }

        return aprilTags.length == 0;

    }



    /**

     * Robot-relative bearing (degrees) to the nearest localization tag from a field pose prior.

     * Returns NaN when no prior or no localization tags with field positions.

     */

    public double nearestLocalizationTagBearing(Pose2D fieldPose) {

        if (fieldPose == null) {

            return Double.NaN;

        }

        double rx = fieldPose.getX(DistanceUnit.INCH);

        double ry = fieldPose.getY(DistanceUnit.INCH);

        double rh = fieldPose.getHeading(AngleUnit.DEGREES);



        VidarAprilTagSpec best = null;

        double bestDist = Double.MAX_VALUE;

        for (VidarAprilTagSpec spec : aprilTags) {

            if (!spec.localization || !spec.hasFieldPosition()) {

                continue;

            }

            double dist = Math.hypot(spec.xIn - rx, spec.yIn - ry);

            if (dist < bestDist) {

                bestDist = dist;

                best = spec;

            }

        }

        return best == null ? Double.NaN : best.bearingFromFieldPose(rx, ry, rh);

    }



    /** Field bearing (degrees) from robot position to the nearest localization tag. */

    public double nearestLocalizationTagFieldBearing(Pose2D fieldPose) {

        if (fieldPose == null) {

            return Double.NaN;

        }

        double rx = fieldPose.getX(DistanceUnit.INCH);

        double ry = fieldPose.getY(DistanceUnit.INCH);



        VidarAprilTagSpec best = null;

        double bestDist = Double.MAX_VALUE;

        for (VidarAprilTagSpec spec : aprilTags) {

            if (!spec.localization || !spec.hasFieldPosition()) {

                continue;

            }

            double dist = Math.hypot(spec.xIn - rx, spec.yIn - ry);

            if (dist < bestDist) {

                bestDist = dist;

                best = spec;

            }

        }

        return best == null

                ? Double.NaN

                : Math.toDegrees(Math.atan2(best.yIn - ry, best.xIn - rx));

    }

}


