package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.config.VidarElementSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarHsvRange;
import org.firstinspires.ftc.teamcode.vidar.config.VidarPlateSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * Unified color-contour detection target for game elements and alliance plates.
 */
public final class VidarContourTarget {

    public enum Kind {
        GAME,
        PLATE
    }

    public final String id;
    public final Kind kind;
    public final VidarElementShape shape;
    public final VidarAlliance alliance;
    public final VidarHsvRange hsv;
    public final VidarHsvRange hsvWrap;
    public final VidarElementDetectorType detector;
    public final double rangingSizeIn;
    public final double minAreaPx;
    public final double maxAreaPx;
    public final double minWidthPx;
    public final double maxWidthPx;
    public final double minHeightPx;
    public final double maxHeightPx;
    public final double minCircularity;
    public final double minRectangularity;
    public final double minAspect;
    public final double maxAspect;
    /** Bounding-box aspect limit for circle/blob contours. */
    public final double maxBoundingAspect;
    public final double minFillRatio;
    public final double minInteriorScore;
    public final int interiorBright;
    public final int interiorSpread;
    public final int holeDarkMax;
    public final int morphErodePasses;
    public final int morphDilatePasses;
    public final int morphOpenPasses;
    public final int morphClosePasses;
    public final boolean morphEllipseKernel;
    public final double houghDp;
    public final double houghMinDist;
    public final double houghParam1;
    public final double houghParam2;
    public final int houghMinRadius;
    public final int houghMaxRadius;
    public final double houghMinInterior;
    public final double minAreaHoughPx;
    public final double minWhiteRatio;
    public final int whiteSampleGrid;
    public final int whiteBrightMin;
    public final int whiteSpreadMax;

    private VidarContourTarget(
            String id,
            Kind kind,
            VidarElementShape shape,
            VidarAlliance alliance,
            VidarHsvRange hsv,
            VidarHsvRange hsvWrap,
            VidarElementDetectorType detector,
            double rangingSizeIn,
            double minAreaPx,
            double maxAreaPx,
            double minWidthPx,
            double maxWidthPx,
            double minHeightPx,
            double maxHeightPx,
            double minCircularity,
            double minRectangularity,
            double minAspect,
            double maxAspect,
            double maxBoundingAspect,
            double minFillRatio,
            double minInteriorScore,
            int interiorBright,
            int interiorSpread,
            int holeDarkMax,
            int morphErodePasses,
            int morphDilatePasses,
            int morphOpenPasses,
            int morphClosePasses,
            boolean morphEllipseKernel,
            double houghDp,
            double houghMinDist,
            double houghParam1,
            double houghParam2,
            int houghMinRadius,
            int houghMaxRadius,
            double houghMinInterior,
            double minAreaHoughPx,
            double minWhiteRatio,
            int whiteSampleGrid,
            int whiteBrightMin,
            int whiteSpreadMax) {
        this.id = id;
        this.kind = kind;
        this.shape = shape;
        this.alliance = alliance;
        this.hsv = hsv;
        this.hsvWrap = hsvWrap;
        this.detector = detector;
        this.rangingSizeIn = rangingSizeIn;
        this.minAreaPx = minAreaPx;
        this.maxAreaPx = maxAreaPx;
        this.minWidthPx = minWidthPx;
        this.maxWidthPx = maxWidthPx;
        this.minHeightPx = minHeightPx;
        this.maxHeightPx = maxHeightPx;
        this.minCircularity = minCircularity;
        this.minRectangularity = minRectangularity;
        this.minAspect = minAspect;
        this.maxAspect = maxAspect;
        this.maxBoundingAspect = maxBoundingAspect;
        this.minFillRatio = minFillRatio;
        this.minInteriorScore = minInteriorScore;
        this.interiorBright = interiorBright;
        this.interiorSpread = interiorSpread;
        this.holeDarkMax = holeDarkMax;
        this.morphErodePasses = morphErodePasses;
        this.morphDilatePasses = morphDilatePasses;
        this.morphOpenPasses = morphOpenPasses;
        this.morphClosePasses = morphClosePasses;
        this.morphEllipseKernel = morphEllipseKernel;
        this.houghDp = houghDp;
        this.houghMinDist = houghMinDist;
        this.houghParam1 = houghParam1;
        this.houghParam2 = houghParam2;
        this.houghMinRadius = houghMinRadius;
        this.houghMaxRadius = houghMaxRadius;
        this.houghMinInterior = houghMinInterior;
        this.minAreaHoughPx = minAreaHoughPx;
        this.minWhiteRatio = minWhiteRatio;
        this.whiteSampleGrid = whiteSampleGrid;
        this.whiteBrightMin = whiteBrightMin;
        this.whiteSpreadMax = whiteSpreadMax;
    }

    public static VidarContourTarget fromElement(VidarElementSpec spec) {
        VidarElementShape shape = spec.shape;
        double minRect = spec.minRectangularity;
        if (shape == VidarElementShape.RECT && minRect <= 0) {
            minRect = 0.40;
        }
        return new VidarContourTarget(
                spec.id,
                Kind.GAME,
                shape,
                null,
                spec.hsv,
                null,
                spec.detector,
                spec.diameter,
                spec.minAreaPx,
                spec.maxAreaPx,
                spec.minWidthPx,
                spec.maxWidthPx,
                spec.minHeightPx,
                spec.maxHeightPx,
                shape == VidarElementShape.CIRCLE ? spec.minCircularity : 0,
                minRect,
                spec.minAspect,
                spec.maxAspect,
                spec.maxAspectRatio,
                spec.minFillRatio,
                spec.minInteriorScore,
                spec.interiorBright,
                spec.interiorSpread,
                spec.holeDarkMax,
                spec.morphErodePasses,
                spec.morphDilatePasses,
                spec.morphOpenPasses,
                spec.morphClosePasses,
                true,
                spec.houghDp,
                spec.houghMinDist,
                spec.houghParam1,
                spec.houghParam2,
                spec.houghMinRadius,
                spec.houghMaxRadius,
                spec.houghMinInterior,
                spec.minAreaHoughPx,
                0,
                0,
                0,
                0);
    }

    public static VidarContourTarget fromPlate(VidarPlateSpec spec) {
        return new VidarContourTarget(
                spec.alliance.name().toLowerCase() + "_plate",
                Kind.PLATE,
                VidarElementShape.RECT,
                spec.alliance,
                spec.hsv,
                spec.hsvWrap,
                VidarElementDetectorType.COLOR_BLOB,
                spec.width,
                spec.minAreaPx,
                spec.maxAreaPx,
                4,
                500,
                4,
                500,
                0,
                spec.minRectangularity,
                spec.minAspect,
                spec.maxAspect,
                spec.maxAspect,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                false,
                0, 0, 0, 0, 0, 0, 0, 0,
                spec.minWhiteRatio,
                spec.whiteSampleGrid,
                spec.whiteBrightMin,
                spec.whiteSpreadMax);
    }

    public static List<VidarContourTarget> fromSeason(VidarSeasonConfig season) {
        List<VidarContourTarget> out = new ArrayList<>();
        for (VidarElementSpec element : season.elements) {
            out.add(fromElement(element));
        }
        for (VidarPlateSpec plate : season.plates) {
            out.add(fromPlate(plate));
        }
        return out;
    }
}
