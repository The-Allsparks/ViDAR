package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;

/**
 * Minimal stub for {@code java-pure} fusion tests — not used on the Control Hub.
 */
public class VidarVision {

    private final VidarRankedElementFrame ranked;
    private boolean failed;
    private boolean excludedFromRotation;

    public VidarVision(VidarRankedElementFrame ranked) {
        this.ranked = ranked != null ? ranked : VidarRankedElementFrame.empty("stub", 8);
    }

    public boolean isFailed() {
        return failed;
    }

    public void setFailed(boolean failed) {
        this.failed = failed;
    }

    public boolean isExcludedFromRotation() {
        return excludedFromRotation;
    }

    public void setExcludedFromRotation(boolean excludedFromRotation) {
        this.excludedFromRotation = excludedFromRotation;
    }

    public VidarRankedElementFrame getRankedElements() {
        return ranked;
    }
}
