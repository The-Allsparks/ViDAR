package org.firstinspires.ftc.teamcode.vidar.model;

import org.firstinspires.ftc.teamcode.vidar.VidarTagScoutResult;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
/**
 * Non-localizing AprilTag scout observation — guides decode scheduling only.
 * Must never alter absolute field pose.
 */
public final class VidarTagScoutObservation {

    public final double bearingDeg;
    public final double apparentWidthPx;
    public final double scoutConfidence;
    public final String cameraName;
    public final VidarFrameRegions.HorizontalBand band;
    public final double cx;
    public final double cy;
    public final long captureTimeNanos;

    public VidarTagScoutObservation(
            double bearingDeg,
            double apparentWidthPx,
            double scoutConfidence,
            String cameraName,
            VidarFrameRegions.HorizontalBand band,
            double cx,
            double cy,
            long captureTimeNanos) {
        this.bearingDeg = bearingDeg;
        this.apparentWidthPx = apparentWidthPx;
        this.scoutConfidence = scoutConfidence;
        this.cameraName = cameraName;
        this.band = band;
        this.cx = cx;
        this.cy = cy;
        this.captureTimeNanos = captureTimeNanos;
    }

    public static VidarTagScoutObservation fromScoutResult(
            VidarTagScoutResult scout,
            VidarCameraProfile profile,
            int frameCols,
            long captureTimeNanos,
            String cameraName) {
        if (scout == null) {
            return null;
        }
        double fullWidth = scout.widthPx * ((double) frameCols / VidarTagConfig.SCOUT_WIDTH);
        double centerErr = scout.cx - frameCols / 2.0;
        double bearingErr = centerErr / Math.max(1, frameCols)
                * VidarTagConfig.horizontalFovDeg(profile);
        double bearing = normalizeDeg(profile.bearingDeg + bearingErr);
        double confidence = Math.min(1.0, fullWidth / VidarTagConfig.DECODE_MIN_TAG_WIDTH_PX);
        return new VidarTagScoutObservation(
                bearing, fullWidth, confidence, cameraName, scout.band,
                scout.cx, scout.cy, captureTimeNanos);
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
