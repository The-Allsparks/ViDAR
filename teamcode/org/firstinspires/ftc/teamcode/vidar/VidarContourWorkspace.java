package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Reusable contour-detection scratch (curves, contour lists, pixel buffers).
 * Borrow from {@link VidarContourWorkspacePool} and return when the pass completes.
 */
final class VidarContourWorkspace {

    final MatOfPoint2f curve = new MatOfPoint2f();
    final MatOfPoint2f approx = new MatOfPoint2f();
    final Point centerPoint = new Point();
    final float[] radiusHolder = new float[1];
    final List<MatOfPoint> contours = new ArrayList<>();
    final List<VidarContourDetect.CircleHit> circleHits = new ArrayList<>();
    final List<VidarContourDetect.RectHit> rectHits = new ArrayList<>();
    byte[] rgbaBytes;

    byte[] ensureRgbaBytes(int length) {
        if (rgbaBytes == null || rgbaBytes.length < length) {
            rgbaBytes = new byte[length];
        }
        return rgbaBytes;
    }

    void releaseContours() {
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        contours.clear();
    }

    void resetPass() {
        releaseContours();
        circleHits.clear();
        rectHits.clear();
    }
}
