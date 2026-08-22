package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VidarTagGateStateTest {

    @Test
    void sequentialInstancesDoNotLeakDriverRequest() {
        VidarTagGateState first = new VidarTagGateState();
        first.requestSample();
        assertTrue(first.consumeDriverRequest());

        VidarTagGateState second = new VidarTagGateState();
        assertFalse(second.consumeDriverRequest(),
                "new gate state must not inherit a prior driverRequested flag");
    }

    @Test
    void resetClearsPendingDriverRequest() {
        VidarTagGateState gate = new VidarTagGateState();
        gate.requestSample();
        gate.reset();
        assertFalse(gate.consumeDriverRequest());
    }

    @Test
    void shouldSampleHonorsDriverRequestOnce() {
        VidarTagGateState gate = new VidarTagGateState();
        gate.setAutoEnabled(false);
        gate.requestSample();
        VidarTagScoutObservation scout = scout(10);
        assertTrue(gate.shouldSample(scout, 640));
        assertFalse(gate.shouldSample(scout, 640));
    }

    private static VidarTagScoutObservation scout(double widthPx) {
        return new VidarTagScoutObservation(
                0, widthPx, 0.5, "cam", VidarFrameRegions.HorizontalBand.MIDDLE, 320, 240, 1L);
    }
}
