package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.Rect;

/**
 * Per-camera ROI layout with coordinate transforms between ROI-local and full-frame space.
 */
public final class VidarFrameRegions {

    public enum HorizontalBand {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private VidarFrameRegions() {}

    public static VidarRoiRect ballRoi(VidarCameraProfile profile, int frameCols, int frameRows) {
        return profile.roiConfig.ballRoi(frameCols, frameRows).clamped(frameCols, frameRows);
    }

    public static VidarRoiRect plateRoi(VidarCameraProfile profile, int frameCols, int frameRows) {
        return profile.roiConfig.plateRoi(frameCols, frameRows).clamped(frameCols, frameRows);
    }

    public static VidarRoiRect tagRoi(VidarCameraProfile profile, int frameCols, int frameRows) {
        return profile.roiConfig.tagRoi(frameCols, frameRows).clamped(frameCols, frameRows);
    }

    /** @deprecated Use {@link #ballRoi(VidarCameraProfile, int, int)}. */
    @Deprecated
    public static Rect ballCrop(int frameCols, int frameRows) {
        VidarRoiRect roi = VidarCameraRoiConfig.DEFAULT.ballRoi(frameCols, frameRows);
        return roi.toOpenCvRect();
    }

    /** @deprecated Use {@link #tagRoi(VidarCameraProfile, int, int)}. */
    @Deprecated
    public static Rect tagTopHalf(int frameCols, int frameRows) {
        VidarRoiRect roi = VidarCameraRoiConfig.DEFAULT.tagRoi(frameCols, frameRows);
        return roi.toOpenCvRect();
    }

    public static HorizontalBand bandForCx(double cx, int frameCols) {
        double norm = cx / Math.max(1, frameCols);
        if (norm < VidarTagConfig.BAND_LEFT_MAX) {
            return HorizontalBand.LEFT;
        }
        if (norm > VidarTagConfig.BAND_RIGHT_MIN) {
            return HorizontalBand.RIGHT;
        }
        return HorizontalBand.MIDDLE;
    }

    public static Rect tagDecodeCrop(int frameCols, int frameRows, HorizontalBand band) {
        VidarRoiRect tag = VidarCameraRoiConfig.DEFAULT.tagRoi(frameCols, frameRows);
        int topH = tag.height;
        int w = frameCols;
        int x;
        int cropW = Math.max(1, w / 2);
        switch (band) {
            case LEFT:
                x = 0;
                break;
            case RIGHT:
                x = w - cropW;
                break;
            case MIDDLE:
            default:
                x = (w - cropW) / 2;
                break;
        }
        return new Rect(x, tag.y, cropW, topH);
    }

    public static Rect tagDecodeCrop(VidarCameraProfile profile, int frameCols, int frameRows, HorizontalBand band) {
        VidarRoiRect tag = tagRoi(profile, frameCols, frameRows);
        int w = frameCols;
        int cropW = Math.max(1, w / 2);
        int x;
        switch (band) {
            case LEFT:
                x = 0;
                break;
            case RIGHT:
                x = w - cropW;
                break;
            case MIDDLE:
            default:
                x = (w - cropW) / 2;
                break;
        }
        return new Rect(x, tag.y, cropW, tag.height);
    }

    public static boolean isInTagDecodeRegion(
            double cx,
            double cy,
            int frameCols,
            int frameRows,
            HorizontalBand band) {
        Rect region = tagDecodeCrop(frameCols, frameRows, band);
        return cx >= region.x
                && cx < region.x + region.width
                && cy >= region.y
                && cy < region.y + region.height;
    }
}
