package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.VidarTagScoutResult;
import org.firstinspires.ftc.teamcode.vidar.detect.VidarContourWorkspace;
import org.firstinspires.ftc.teamcode.vidar.detect.VidarContourWorkspacePool;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraRoiConfig;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRoiRect;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Reusable-buffer tag scout — edge/contour pass without per-frame Mat churn.
 */
public final class VidarTagScoutRunner {

    private final VidarContourWorkspacePool workspacePool = new VidarContourWorkspacePool();

    private Mat reusableScout;
    private Mat reusableGray;
    private Mat reusableEdges;
    private Mat reusableHierarchy;

    public VidarTagScoutResult run(Mat frameBgra) {
        return run(frameBgra, null);
    }

    public VidarTagScoutResult run(Mat frameBgra, VidarCameraProfile profile) {
        if (frameBgra == null || frameBgra.empty()) {
            return null;
        }

        VidarContourWorkspace workspace = workspacePool.borrow();
        try {
            int cols = frameBgra.cols();
            int rows = frameBgra.rows();
            VidarRoiRect tag = profile == null
                    ? VidarCameraRoiConfig.DEFAULT.tagRoi(cols, rows).clamped(cols, rows)
                    : VidarFrameRegions.tagRoi(profile, cols, rows);
            Rect tagRegion = new Rect(tag.x, tag.y, tag.width, tag.height);

            Mat topRoi = null;
            try {
                topRoi = new Mat(frameBgra, tagRegion);
                int scoutW = VidarTagConfig.SCOUT_WIDTH;
                int scoutH = Math.max(8, tagRegion.height * scoutW / Math.max(1, tagRegion.width));
                reusableScout = ensureRgba(reusableScout, scoutH, scoutW);
                Imgproc.resize(topRoi, reusableScout, new Size(scoutW, scoutH), 0, 0, Imgproc.INTER_AREA);

                reusableGray = ensureGray(reusableGray, scoutH, scoutW);
                Imgproc.cvtColor(reusableScout, reusableGray, Imgproc.COLOR_RGBA2GRAY);
                Imgproc.GaussianBlur(reusableGray, reusableGray, new Size(3, 3), 0);

                reusableEdges = ensureGray(reusableEdges, scoutH, scoutW);
                Imgproc.Canny(reusableGray, reusableEdges, 60, 160);

                if (reusableHierarchy == null) {
                    reusableHierarchy = new Mat();
                }

                workspace.releaseContours();
                List<MatOfPoint> contours = workspace.contours;
                Imgproc.findContours(
                        reusableEdges,
                        contours,
                        reusableHierarchy,
                        Imgproc.RETR_EXTERNAL,
                        Imgproc.CHAIN_APPROX_SIMPLE);

                MatOfPoint2f curve = workspace.curve;
                MatOfPoint2f approx = workspace.approx;

                VidarTagScoutResult best = null;
                double bestScore = -1;

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area < 40) {
                        contour.release();
                        continue;
                    }

                    contour.convertTo(curve, org.opencv.core.CvType.CV_32FC2);
                    double peri = Imgproc.arcLength(curve, true);
                    Imgproc.approxPolyDP(curve, approx, 0.06 * peri, true);

                    if (approx.total() < 4 || approx.total() > 6) {
                        contour.release();
                        continue;
                    }

                    Rect box = Imgproc.boundingRect(contour);
                    double aspect = Math.max(box.width, box.height)
                            / Math.max(1.0, Math.min(box.width, box.height));
                    if (aspect > 1.6) {
                        contour.release();
                        continue;
                    }

                    double cxScout = box.x + box.width / 2.0;
                    double cyScout = box.y + box.height / 2.0;
                    double wScout = Math.max(box.width, box.height);

                    double cxFull = tagRegion.x + cxScout * tagRegion.width / scoutW;
                    double cyFull = tagRegion.y + cyScout * tagRegion.height / scoutH;
                    double wFull = wScout * tagRegion.width / scoutW;

                    if (wFull < VidarTagConfig.SCOUT_MIN_WIDTH_PX) {
                        contour.release();
                        continue;
                    }

                    VidarFrameRegions.HorizontalBand band = VidarFrameRegions.bandForCx(cxFull, cols);
                    double score = area * (1.0 / aspect);
                    if (score > bestScore) {
                        bestScore = score;
                        best = new VidarTagScoutResult(cxFull, cyFull, wFull, band);
                    }
                    contour.release();
                }
                return best;
            } finally {
                if (topRoi != null) {
                    topRoi.release();
                }
            }
        } finally {
            workspacePool.release(workspace);
        }
    }

    public void release() {
        if (reusableScout != null) reusableScout.release();
        if (reusableGray != null) reusableGray.release();
        if (reusableEdges != null) reusableEdges.release();
        if (reusableHierarchy != null) reusableHierarchy.release();
        reusableScout = reusableGray = reusableEdges = reusableHierarchy = null;
    }

    private static Mat ensureRgba(Mat mat, int rows, int cols) {
        if (mat == null || mat.empty() || mat.rows() != rows || mat.cols() != cols) {
            if (mat != null) {
                mat.release();
            }
            return new Mat(rows, cols, org.opencv.core.CvType.CV_8UC4);
        }
        return mat;
    }

    private static Mat ensureGray(Mat mat, int rows, int cols) {
        if (mat == null || mat.empty() || mat.rows() != rows || mat.cols() != cols) {
            if (mat != null) {
                mat.release();
            }
            return new Mat(rows, cols, org.opencv.core.CvType.CV_8UC1);
        }
        return mat;
    }
}

