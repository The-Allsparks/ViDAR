package org.firstinspires.ftc.teamcode.vidar.detect;

import java.util.ArrayDeque;

/** Small pool so nested passes can borrow independent contour workspaces. */
public final class VidarContourWorkspacePool {

    private static final int MAX_POOLED = 4;

    private final ArrayDeque<VidarContourWorkspace> available = new ArrayDeque<>();

    public VidarContourWorkspace borrow() {
        synchronized (available) {
            if (!available.isEmpty()) {
                return available.pop();
            }
        }
        return new VidarContourWorkspace();
    }

    public void release(VidarContourWorkspace workspace) {
        if (workspace == null) {
            return;
        }
        workspace.resetPass();
        synchronized (available) {
            if (available.size() < MAX_POOLED) {
                available.push(workspace);
            }
        }
    }
}
