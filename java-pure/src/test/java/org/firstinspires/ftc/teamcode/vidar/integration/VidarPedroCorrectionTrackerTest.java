package org.firstinspires.ftc.teamcode.vidar.integration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class VidarPedroCorrectionTrackerTest {

    @Test
    void firstEventIdIsApplied() {
        VidarPedroCorrectionTracker tracker = new VidarPedroCorrectionTracker();
        Pose2D now = new Pose2D(DistanceUnit.INCH, 11, 20, AngleUnit.DEGREES, 0);
        VidarPedroPose applied = tracker.poll(100L, now);
        assertNotNull(applied);
        assertEquals(11.0, applied.x, 1e-9);
        assertEquals(100L, tracker.lastAppliedCorrectionNanos());
    }

    @Test
    void sameEventIdIsNotReappliedEvenIfPoseMoves() {
        VidarPedroCorrectionTracker tracker = new VidarPedroCorrectionTracker();
        Pose2D now1 = new Pose2D(DistanceUnit.INCH, 10, 20, AngleUnit.DEGREES, 0);
        Pose2D now2 = new Pose2D(DistanceUnit.INCH, 12, 20, AngleUnit.DEGREES, 0);
        assertNotNull(tracker.poll(100L, now1));
        assertNull(tracker.poll(100L, now2));
    }

    @Test
    void newEventIdAppliesUpdatedPose() {
        VidarPedroCorrectionTracker tracker = new VidarPedroCorrectionTracker();
        Pose2D a = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D bNow = new Pose2D(DistanceUnit.INCH, 5.5, 0, AngleUnit.DEGREES, 0);
        assertNotNull(tracker.poll(100L, a));
        VidarPedroPose next = tracker.poll(200L, bNow);
        assertNotNull(next);
        assertEquals(5.5, next.x, 1e-9);
        assertEquals(200L, tracker.lastAppliedCorrectionNanos());
    }

    @Test
    void zeroOrNullSkipped() {
        VidarPedroCorrectionTracker tracker = new VidarPedroCorrectionTracker();
        Pose2D pose = new Pose2D(DistanceUnit.INCH, 1, 0, AngleUnit.DEGREES, 0);
        assertNull(tracker.poll(0L, pose));
        assertNull(tracker.poll(50L, null));
    }

    @Test
    void pedroLivePoseMustNotBeUsedAsEventId() {
        // Regression: gating on changing Pose2D (Pedro fieldPose) would re-fire every move.
        // Event-id API ignores pose motion without a new correctionNanos.
        VidarPedroCorrectionTracker tracker = new VidarPedroCorrectionTracker();
        Pose2D pedroMoving = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        assertNotNull(tracker.poll(1L, pedroMoving));
        for (int i = 1; i <= 10; i++) {
            Pose2D later = new Pose2D(DistanceUnit.INCH, i, 0, AngleUnit.DEGREES, 0);
            assertNull(tracker.poll(1L, later), "drive motion must not re-inject setPose");
        }
    }

    @Test
    void backdateToNowAddsOdomDelta() {
        Pose2D atCapture = new Pose2D(DistanceUnit.INCH, 10, 0, AngleUnit.DEGREES, 0);
        Pose2D odomThen = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D odomNow = new Pose2D(DistanceUnit.INCH, 3, 1, AngleUnit.DEGREES, 10);
        Pose2D corrected = VidarPedroCorrectionTracker.backdateToNow(atCapture, odomThen, odomNow);
        assertEquals(13.0, corrected.getX(DistanceUnit.INCH), 1e-9);
        assertEquals(1.0, corrected.getY(DistanceUnit.INCH), 1e-9);
        assertEquals(10.0, corrected.getHeading(AngleUnit.DEGREES), 1e-9);
    }
}
