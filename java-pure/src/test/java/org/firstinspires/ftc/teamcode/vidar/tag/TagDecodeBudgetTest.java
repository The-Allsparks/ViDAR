package org.firstinspires.ftc.teamcode.vidar.tag;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TagDecodeBudgetTest {

    @Test
    void tryAcquireRespectsInterval() {
        TagDecodeBudget budget = new TagDecodeBudget();
        long t0 = 1_000_000_000L;
        assertTrue(budget.tryAcquire(t0, "front"));
        assertEquals("front", budget.lastDecodeCamera());
        assertFalse(budget.tryAcquire(t0 + 500_000_000L, "front"));
        assertTrue(budget.tryAcquire(t0 + VidarTagConfig.DECODE_INTERVAL_MS * 1_000_000L, "rear"));
        assertEquals("rear", budget.lastDecodeCamera());
    }

    @Test
    void resetClearsState() {
        TagDecodeBudget budget = new TagDecodeBudget();
        budget.tryAcquire(5_000_000_000L, "cam0");
        budget.reset();
        assertEquals("", budget.lastDecodeCamera());
        assertTrue(budget.tryAcquire(5_000_000_000L, "cam0"));
    }
}
