package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Fast top-half scout — square-ish high-contrast quads for adaptive tag crop selection.
 */
public final class VidarTagScout {

    private VidarTagScout() {}

    public static VidarTagScoutResult run(Mat frameBgra) {
        if (frameBgra == null || frameBgra.empty()) {
            return null;
        }

        int cols = frameBgra.cols();
        int rows = frameBgra.rows();
        Rect top = VidarFrameRegions.tagTopHalf(cols, rows);

        Mat topRoi = null;
        Mat gray = null;
        Mat edges = null;

        try {
            topRoi = new Mat(frameBgra, top);
            int scoutW = VidarTagConfig.SCOUT_WIDTH;
            int scoutH = Math.max(8, top.height * scoutW / Math.max(1, top.width));
            Mat scout = new Mat();
            Imgproc.resize(topRoi, scout, new Size(scoutW, scoutH), 0, 0, Imgproc.INTER_AREA);

            gray = new Mat();
            Imgproc.cvtColor(scout, gray, Imgproc.COLOR_RGBA2GRAY);
            scout.release();
            Imgproc.GaussianBlur(gray, gray, new Size(3, 3), 0);
            edges = new Mat();
            Imgproc.Canny(gray, edges, 60, 160);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(
                    edges,
                    contours,
                    new Mat(),
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE);

            VidarTagScoutResult best = null;
            double bestScore = -1;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < 40) {
                    continue;
                }

                MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approx = new MatOfPoint2f();
                double peri = Imgproc.arcLength(curve, true);
                Imgproc.approxPolyDP(curve, approx, 0.06 * peri, true);
                curve.release();

                if (approx.total() < 4 || approx.total() > 6) {
                    approx.release();
                    continue;
                }

                Rect box = Imgproc.boundingRect(contour);
                double aspect = Math.max(box.width, box.height) / Math.max(1.0, Math.min(box.width, box.height));
                if (aspect > 1.6) {
                    approx.release();
                    continue;
                }

                double cxScout = box.x + box.width / 2.0;
                double cyScout = box.y + box.height / 2.0;
                double wScout = Math.max(box.width, box.height);

                double cxFull = top.x + cxScout * top.width / scoutW;
                double cyFull = top.y + cyScout * top.height / scoutH;
                double wFull = wScout * top.width / scoutW;

                if (wFull < VidarTagConfig.SCOUT_MIN_WIDTH_PX) {
                    approx.release();
                    continue;
                }

                VidarFrameRegions.HorizontalBand band =
                        VidarFrameRegions.bandForCx(cxFull, cols);
                double score = area * (1.0 / aspect);
                if (score > bestScore) {
                    bestScore = score;
                    best = new VidarTagScoutResult(cxFull, cyFull, wFull, band);
                }
                approx.release();
            }

            for (MatOfPoint c : contours) {
                c.release();
            }
            return best;
        } finally {
            if (topRoi != null) topRoi.release();
            if (gray != null) gray.release();
            if (edges != null) edges.release();
        }
    }
}
