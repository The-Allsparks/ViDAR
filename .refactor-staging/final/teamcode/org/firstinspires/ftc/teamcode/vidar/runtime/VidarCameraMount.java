package org.firstinspires.ftc.teamcode.vidar.runtime;

/**
 * One physical camera: robot configuration name + calibrated mount profile.
 */
public final class VidarCameraMount {

    public final String webcamName;
    public final VidarCameraProfile profile;

    public VidarCameraMount(String webcamName, VidarCameraProfile profile) {
        this.webcamName = webcamName;
        this.profile = profile;
    }
}
