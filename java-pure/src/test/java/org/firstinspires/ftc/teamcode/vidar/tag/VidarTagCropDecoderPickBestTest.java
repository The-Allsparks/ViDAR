package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.junit.jupiter.api.Test;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VidarTagCropDecoderPickBestTest {

    private static final Rect CROP = new Rect(40, 20, 320, 240);

    @Test
    void pickBestAcceptsClusterNameAndOrigin() {
        AprilTagPoseFtc origin = ftcPose(12.0, 24.0, -8.0);
        AprilTagDetection cluster = cluster("Hive Low CELL", origin, new Pose3D());

        AprilTagDetection best = VidarTagCropDecoder.pickBest(
                Collections.singletonList(cluster), null, CROP, 320, 240);
        assertNotNull(best, "cluster detection must not be dropped");

        VidarTagCropDecoder.DecodeResult result =
                VidarTagCropDecoder.toDecodeResult(best, CROP, 320, 240, 1);
        assertTrue(result.cluster);
        assertEquals("Hive Low CELL", result.clusterName);
        assertEquals(VidarTagCropDecoder.CLUSTER_TAG_ID, result.tagId);
        assertEquals(12.0, result.fieldPose.getX(DistanceUnit.INCH), 1e-9);
        assertEquals(24.0, result.fieldPose.getY(DistanceUnit.INCH), 1e-9);
        assertEquals(-8.0, result.fieldPose.getHeading(AngleUnit.DEGREES), 1e-9);
    }

    @Test
    void pickBestReturnsSingleTagId() {
        AprilTagDetection single = single(24, new Point(80, 60), ftcPose(3, 10, 0));

        AprilTagDetection best = VidarTagCropDecoder.pickBest(
                Collections.singletonList(single), null, CROP, 320, 240);
        VidarTagCropDecoder.DecodeResult result =
                VidarTagCropDecoder.toDecodeResult(best, CROP, 320, 240, 2);

        assertFalse(result.cluster);
        assertNull(result.clusterName);
        assertEquals(24, result.tagId);
        assertEquals(40 + 80, result.centerX, 1e-9);
        assertEquals(20 + 60, result.centerY, 1e-9);
    }

    @Test
    void pickBestPrefersClusterOverSingle() {
        List<AprilTagDetection> detections = Arrays.asList(
                single(11, new Point(10, 10), ftcPose(1, 1, 0)),
                cluster("Hive Low CELL", ftcPose(9, 18, 5), null),
                single(12, new Point(200, 100), ftcPose(2, 2, 0)));

        AprilTagDetection best = VidarTagCropDecoder.pickBest(detections, null, CROP, 320, 240);
        VidarTagCropDecoder.DecodeResult result =
                VidarTagCropDecoder.toDecodeResult(best, CROP, 320, 240, 1);

        assertTrue(result.cluster);
        assertEquals("Hive Low CELL", result.clusterName);
        assertEquals(9.0, result.fieldPose.getX(DistanceUnit.INCH), 1e-9);
    }

    @Test
    void pickBestUsesScoutDistanceForSinglesOnly() {
        VidarTagScoutObservation scout = new VidarTagScoutObservation(
                0, 40, 0.9, "front", VidarFrameRegions.HorizontalBand.MIDDLE, 240, 80, 1L);
        AprilTagDetection near = single(3, new Point(200, 60), ftcPose(1, 1, 0));
        AprilTagDetection far = single(4, new Point(10, 10), ftcPose(2, 2, 0));

        AprilTagDetection best = VidarTagCropDecoder.pickBest(
                Arrays.asList(far, near), scout, CROP, 320, 240);
        VidarTagCropDecoder.DecodeResult result =
                VidarTagCropDecoder.toDecodeResult(best, CROP, 320, 240, 1);
        assertEquals(3, result.tagId);
    }

    @Test
    void emptyDetectionsReturnNull() {
        assertNull(VidarTagCropDecoder.pickBest(Collections.emptyList(), null, CROP, 320, 240));
        assertNull(VidarTagCropDecoder.toDecodeResult(null, CROP, 320, 240, 1));
    }

    @Test
    void pickBestSkipsUnknownDetectionSubtype() {
        AprilTagDetection unknown = new AprilTagDetection(
                ftcPose(1, 1, 0), new AprilTagPoseRaw(), null, 0L, DistanceUnit.INCH) {};
        AprilTagDetection tagged = single(7, new Point(80, 60), ftcPose(3, 10, 0));

        AprilTagDetection best = VidarTagCropDecoder.pickBest(
                Arrays.asList(unknown, tagged), null, CROP, 320, 240);
        VidarTagCropDecoder.DecodeResult result =
                VidarTagCropDecoder.toDecodeResult(best, CROP, 320, 240, 1);
        assertEquals(7, result.tagId);
        assertNull(VidarTagCropDecoder.toDecodeResult(unknown, CROP, 320, 240, 1));
    }

    private static AprilTagSingleDetection single(int id, Point center, AprilTagPoseFtc pose) {
        return new AprilTagSingleDetection(
                id, 0, 1f, center, new Point[0], null, pose, new AprilTagPoseRaw(),
                null, 0L, DistanceUnit.INCH);
    }

    private static AprilTagClusterDetection cluster(
            String name, AprilTagPoseFtc pose, Pose3D robotPose) {
        return new AprilTagClusterDetection(
                100,
                new AprilTagClusterMetadata(name, name),
                DistanceUnit.INCH,
                pose,
                new AprilTagPoseRaw(),
                robotPose,
                0L);
    }

    private static AprilTagPoseFtc ftcPose(double x, double y, double yaw) {
        return new AprilTagPoseFtc(x, y, 0, yaw, 0, 0, Math.hypot(x, y), 0, 0);
    }
}
