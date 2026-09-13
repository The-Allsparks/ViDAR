package org.opencv.core;

/** Minimal stub for JVM unit tests (no OpenCV native libs). */
public class Mat {
    public Mat() {
    }

    public Mat(Mat m, Rect roi) {
    }

    public boolean empty() {
        return true;
    }

    public int rows() {
        return 0;
    }

    public int cols() {
        return 0;
    }

    public int type() {
        return 0;
    }

    public void create(int rows, int cols, int type) {
    }

    public void release() {
    }
}
