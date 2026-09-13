package org.opencv.imgproc;

import org.opencv.core.Mat;
import org.opencv.core.Size;

/** Minimal stub for JVM unit tests (no OpenCV native libs). */
public final class Imgproc {
    public static final int INTER_AREA = 3;

    private Imgproc() {}

    public static void resize(Mat src, Mat dst, Size dsize, double fx, double fy, int interpolation) {
    }
}
