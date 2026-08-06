package org.firstinspires.ftc.teamcode.vidar;

/**
 * Game-element detection pipeline selection. Default: color blob with optional local Hough validation.
 */
public enum VidarElementDetectorType {
    /** HSV/YCrCb segmentation + geometric filtering (primary, lowest CPU). */
    COLOR_BLOB,
    /** Color blob candidates validated with local Hough inside ambiguous regions. */
    COLOR_BLOB_WITH_LOCAL_HOUGH
}
