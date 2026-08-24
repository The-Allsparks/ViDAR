package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VidarOpModeCameraSessionTest {

    @Test
    void autoThenTeleOpUsesNewGeneration() {
        VidarOpModeCameraSession session = new VidarOpModeCameraSession();

        assertEquals(1, session.attach());
        assertTrue(session.isAttached());

        assertTrue(session.detach());
        assertFalse(session.isAttached());
        assertEquals(1, session.releaseCount());

        assertEquals(2, session.attach());
        assertTrue(session.isAttached());
        assertEquals(1, session.releaseCount());
    }

    @Test
    void secondDetachIsNoOp() {
        VidarOpModeCameraSession session = new VidarOpModeCameraSession();
        session.attach();

        assertTrue(session.detach());
        assertFalse(session.detach());
        assertEquals(1, session.releaseCount());
        assertEquals(1, session.generation());
    }

    @Test
    void attachWhileAttachedReleasesPreviousSessionFirst() {
        VidarOpModeCameraSession session = new VidarOpModeCameraSession();
        session.attach();

        assertEquals(2, session.attach());
        assertTrue(session.isAttached());
        assertEquals(1, session.releaseCount());
    }

    @Test
    void shutdownResetClearsGeneration() {
        VidarOpModeCameraSession session = new VidarOpModeCameraSession();
        session.attach();
        session.reset();

        assertFalse(session.isAttached());
        assertEquals(0, session.generation());
        assertEquals(1, session.releaseCount());

        assertEquals(1, session.attach());
    }

    @Test
    void semverIsDottedTriple() {
        assertTrue(VidarVersion.SEMVER.matches("[0-9]+\\.[0-9]+\\.[0-9]+"));
    }
}
