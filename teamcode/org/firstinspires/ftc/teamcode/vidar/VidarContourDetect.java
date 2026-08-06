package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Shared HSV mask, contour filtering, and shape fitting for {@link VidarContourProcessor}.
 */
final class VidarContourDetect {

    private static final int RGBA_CHANNELS = 4;

    static final class CircleHit {
        final double cx;
        final double cy;
        final double radius;
        final double area;
        final double aspectRatio;
        final double circularity;
        final double fillRatio;
        final double interiorScore;
        final double circleFitQuality;
        final boolean touchesBoundary;

        CircleHit(double cx, double cy, double radius, double area,
                  double aspectRatio, double circularity, double fillRatio,
                  double interiorScore, double circleFitQuality, boolean touchesBoundary) {
            this.cx = cx;
            this.cy = cy;
            this.radius = radius;
            this.area = area;
            this.aspectRatio = aspectRatio;
            this.circularity = circularity;
            this.fillRatio = fillRatio;
            this.interiorScore = interiorScore;
            this.circleFitQuality = circleFitQuality;
            this.touchesBoundary = touchesBoundary;
        }
    }

    static final class RectHit {
        final RotatedRect box;
        final double contourArea;
        final double rectangularity;
        final double aspect;
        final double whiteRatio;
        final boolean touchesBoundary;

        RectHit(RotatedRect box, double contourArea, double rectangularity,
                double aspect, double whiteRatio, boolean touchesBoundary) {
            this.box = box;
            this.contourArea = contourArea;
            this.rectangularity = rectangularity;
            this.aspect = aspect;
            this.whiteRatio = whiteRatio;
            this.touchesBoundary = touchesBoundary;
        }
    }

    private VidarContourDetect() {}

    static void buildMask(Mat hsv, Mat mask, VidarContourTarget target, Mat wrapScratch) {
        Scalar low = new Scalar(target.hsv.hMin, target.hsv.sMin, target.hsv.vMin);
        Scalar high = new Scalar(target.hsv.hMax, target.hsv.sMax, target.hsv.vMax);
        Imgproc.inRange(hsv, low, high, mask);
        if (target.hsvWrap != null && wrapScratch != null) {
            Imgproc.inRange(hsv,
                    new Scalar(target.hsvWrap.hMin, target.hsvWrap.sMin, target.hsvWrap.vMin),
                    new Scalar(target.hsvWrap.hMax, target.hsvWrap.sMax, target.hsvWrap.vMax),
                    wrapScratch);
            org.opencv.core.Core.bitwise_or(mask, wrapScratch, mask);
        }
    }

    static void applyMorphology(Mat mask, Mat kernel, VidarContourTarget target) {
        if (target.morphErodePasses > 0) {
            for (int i = 0; i < target.morphErodePasses; i++) {
                Imgproc.erode(mask, mask, kernel);
            }
        }
        if (target.morphDilatePasses > 0) {
            for (int i = 0; i < target.morphDilatePasses; i++) {
                Imgproc.dilate(mask, mask, kernel);
            }
        }
        if (target.morphOpenPasses > 0) {
            for (int i = 0; i < target.morphOpenPasses; i++) {
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            }
        }
        if (target.morphClosePasses > 0) {
            for (int i = 0; i < target.morphClosePasses; i++) {
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
            }
        }
    }

    static List<CircleHit> findCircleHits(
            Mat rgba,
            Mat mask,
            Mat hierarchy,
            VidarContourTarget target,
            VidarFramePipeline.ScaledRoi scaled,
            int frameW,
            int frameH,
            VidarCameraProfile profile,
            VidarContourWorkspace workspace) {
        List<CircleHit> out = workspace.circleHits;
        out.clear();
        workspace.releaseContours();
        List<MatOfPoint> contours = workspace.contours;
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f curve = workspace.curve;
        Point center = workspace.centerPoint;
        float[] radiusHolder = workspace.radiusHolder;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < target.minAreaPx || area > target.maxAreaPx) {
                contour.release();
                continue;
            }

            Rect bounds = Imgproc.boundingRect(contour);
            double bw = bounds.width * scaled.scale;
            double bh = bounds.height * scaled.scale;
            if (bw < target.minWidthPx || bh < target.minHeightPx
                    || bw > target.maxWidthPx || bh > target.maxHeightPx) {
                contour.release();
                continue;
            }

            double aspect = Math.max(bw, bh) / Math.max(1, Math.min(bw, bh));
            if (aspect > target.maxBoundingAspect) {
                contour.release();
                continue;
            }

            contour.convertTo(curve, CvType.CV_32FC2);
            double perimeter = Imgproc.arcLength(curve, true);
            double circularity = perimeter > 0 ? 4 * Math.PI * area / (perimeter * perimeter) : 0;
            if (target.minCircularity > 0 && circularity < target.minCircularity) {
                contour.release();
                continue;
            }

            Imgproc.minEnclosingCircle(curve, center, radiusHolder);
            double radius = radiusHolder[0];
            double fillRatio = target.minFillRatio > 0
                    ? area / Math.max(1, Math.PI * radius * radius) : 1.0;
            if (target.minFillRatio > 0 && fillRatio < target.minFillRatio) {
                contour.release();
                continue;
            }

            double interior = target.minInteriorScore > 0
                    ? interiorScore(rgba, center.x, center.y, radius, target, workspace) : 1.0;
            if (target.minInteriorScore > 0 && interior < target.minInteriorScore) {
                contour.release();
                continue;
            }

            double fullCx = scaled.toFullX(center.x);
            double fullCy = scaled.toFullY(center.y);
            double fullRadius = radius * scaled.scale;
            boolean touches = profile.roiConfig.elementRoi(frameW, frameH)
                    .touchesBoundary(frameW, frameH, fullCx, fullCy, 3);
            double fitQuality = Math.min(1.0, Math.max(0.35, circularity * fillRatio));
            out.add(new CircleHit(fullCx, fullCy, fullRadius,
                    area * scaled.scale * scaled.scale,
                    aspect, circularity, fillRatio, interior, fitQuality, touches));
            contour.release();
        }
        return out;
    }

    static List<RectHit> findRectHits(
            Mat rgba,
            Mat mask,
            Mat hierarchy,
            VidarContourTarget target,
            VidarFramePipeline.ScaledRoi scaled,
            int frameW,
            int frameH,
            VidarCameraProfile profile,
            VidarContourWorkspace workspace) {
        List<RectHit> out = workspace.rectHits;
        out.clear();
        workspace.releaseContours();
        List<MatOfPoint> contours = workspace.contours;
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f curve = workspace.curve;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < target.minAreaPx || area > target.maxAreaPx) {
                contour.release();
                continue;
            }

            contour.convertTo(curve, CvType.CV_32FC2);
            RotatedRect box = Imgproc.minAreaRect(curve);
            contour.release();

            double w = box.size.width;
            double h = box.size.height;
            if (w < 4 || h < 4) {
                continue;
            }

            double shortSide = Math.min(w, h);
            double longSide = Math.max(w, h);
            double aspect = longSide / shortSide;
            if (aspect < target.minAspect || aspect > target.maxAspect) {
                continue;
            }

            double rectangularity = area / (shortSide * longSide);
            if (rectangularity < target.minRectangularity) {
                continue;
            }

            double whiteRatio = target.minWhiteRatio > 0
                    ? whiteDigitRatio(rgba, box, target, workspace) : 1.0;
            if (target.minWhiteRatio > 0 && whiteRatio < target.minWhiteRatio) {
                continue;
            }

            double absCx = scaled.toFullX(box.center.x);
            double absCy = scaled.toFullY(box.center.y);
            boolean touches = profile.roiConfig.elementRoi(frameW, frameH)
                    .touchesBoundary(frameW, frameH, absCx, absCy, 4);
            out.add(new RectHit(box, area * scaled.scale * scaled.scale,
                    rectangularity, aspect, whiteRatio, touches));
        }
        return out;
    }

    /** Expects {@code gray} to already be blurred when local Hough is enabled. */
    static List<CircleHit> applyLocalHough(
            VidarFramePipeline.ScaledRoi scaled,
            List<CircleHit> hits,
            VidarContourTarget target,
            Mat gray,
            Mat circles) {
        if (hits.isEmpty()) {
            return hits;
        }
        List<CircleHit> validated = new java.util.ArrayList<>();
        for (CircleHit hit : hits) {
            double localCx = (hit.cx - scaled.sourceCrop.x) / scaled.scale;
            double localCy = (hit.cy - scaled.sourceCrop.y) / scaled.scale;
            double localR = hit.radius / scaled.scale;
            int pad = (int) Math.ceil(localR * 1.5);
            int x = Math.max(0, (int) localCx - pad);
            int y = Math.max(0, (int) localCy - pad);
            int w = Math.min(gray.cols() - x, pad * 2);
            int h = Math.min(gray.rows() - y, pad * 2);
            if (w < 8 || h < 8) {
                validated.add(hit);
                continue;
            }

            Mat patch = null;
            try {
                patch = gray.submat(y, y + h, x, x + w);
                Imgproc.HoughCircles(patch, circles, Imgproc.HOUGH_GRADIENT,
                        target.houghDp, Math.max(8, localR),
                        target.houghParam1, target.houghParam2,
                        Math.max(4, (int) (localR * 0.6)), Math.max(6, (int) (localR * 1.4)));
            } finally {
                if (patch != null) {
                    patch.release();
                }
            }

            if (!circles.empty()) {
                validated.add(new CircleHit(hit.cx, hit.cy, hit.radius, hit.area, hit.aspectRatio,
                        hit.circularity, hit.fillRatio, hit.interiorScore,
                        Math.min(1.0, hit.circleFitQuality + 0.15), hit.touchesBoundary));
            }
        }
        return validated.isEmpty() ? hits : validated;
    }

    private static double interiorScore(
            Mat rgba,
            double cx,
            double cy,
            double radius,
            VidarContourTarget target,
            VidarContourWorkspace workspace) {
        int ri = (int) Math.floor(radius);
        int icx = (int) Math.round(cx);
        int icy = (int) Math.round(cy);
        int x0 = Math.max(0, icx - ri);
        int y0 = Math.max(0, icy - ri);
        int x1 = Math.min(rgba.cols(), icx + ri + 1);
        int y1 = Math.min(rgba.rows(), icy + ri + 1);
        int patchW = x1 - x0;
        int patchH = y1 - y0;
        if (patchW <= 0 || patchH <= 0) {
            return 0;
        }

        Mat patch = null;
        try {
            patch = rgba.submat(y0, y1, x0, x1);
            int byteCount = patchW * patchH * RGBA_CHANNELS;
            byte[] buf = workspace.ensureRgbaBytes(byteCount);
            patch.get(0, 0, buf);

            double r2 = radius * radius;
            int inside = 0;
            int bright = 0;
            int darkHole = 0;

            for (int row = 0; row < patchH; row++) {
                int y = y0 + row;
                int ly = y - icy;
                for (int col = 0; col < patchW; col++) {
                    int lx = x0 + col - icx;
                    if (lx * lx + ly * ly > r2) {
                        continue;
                    }
                    int bi = (row * patchW + col) * RGBA_CHANNELS;
                    int r = buf[bi] & 0xFF;
                    int g = buf[bi + 1] & 0xFF;
                    int b = buf[bi + 2] & 0xFF;
                    inside++;
                    int max = Math.max(r, Math.max(g, b));
                    int min = Math.min(r, Math.min(g, b));
                    if (max >= target.interiorBright && max - min <= target.interiorSpread) {
                        bright++;
                    }
                    if (max < target.holeDarkMax) {
                        darkHole++;
                    }
                }
            }

            if (inside == 0) {
                return 0;
            }
            double brightRatio = (double) bright / inside;
            double holeBonus = darkHole > inside * 0.05 ? 0.15 : 0;
            return Math.min(1.0, brightRatio + holeBonus);
        } finally {
            if (patch != null) {
                patch.release();
            }
        }
    }

    private static double whiteDigitRatio(
            Mat rgba,
            RotatedRect box,
            VidarContourTarget target,
            VidarContourWorkspace workspace) {
        Point[] corners = new Point[4];
        box.points(corners);

        int grid = target.whiteSampleGrid;
        int samples = 0;
        int white = 0;

        for (int gy = 1; gy < grid; gy++) {
            for (int gx = 1; gx < grid; gx++) {
                double u = gx / (double) grid;
                double v = gy / (double) grid;
                double px = bilinearX(corners, u, v);
                double py = bilinearY(corners, u, v);
                int ix = (int) Math.round(px);
                int iy = (int) Math.round(py);
                if (ix < 0 || iy < 0 || ix >= rgba.cols() || iy >= rgba.rows()) {
                    continue;
                }

                byte[] pixel = workspace.ensureRgbaBytes(RGBA_CHANNELS);
                rgba.get(iy, ix, pixel);
                samples++;
                int r = pixel[0] & 0xFF;
                int g = pixel[1] & 0xFF;
                int b = pixel[2] & 0xFF;
                int max = Math.max(r, Math.max(g, b));
                int min = Math.min(r, Math.min(g, b));
                if (max >= target.whiteBrightMin && max - min <= target.whiteSpreadMax) {
                    white++;
                }
            }
        }
        return samples == 0 ? 0 : (double) white / samples;
    }

    private static double bilinearX(Point[] c, double u, double v) {
        return (1 - u) * (1 - v) * c[0].x + u * (1 - v) * c[1].x + u * v * c[2].x + (1 - u) * v * c[3].x;
    }

    private static double bilinearY(Point[] c, double u, double v) {
        return (1 - u) * (1 - v) * c[0].y + u * (1 - v) * c[1].y + u * v * c[2].y + (1 - u) * v * c[3].y;
    }
}
