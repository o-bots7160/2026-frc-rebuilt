package frc.robot.subsystems.vision.config;

import java.util.function.Supplier;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration for the driver camera subsystem.
 */
public class DriverCameraSubsystemConfig extends AbstractConfig {

    /**
     * The network name of the camera (e.g., "limelight").
     */
    public String cameraName;

    /**
     * Pipeline index to use for driver mode (typically 0 for a driver
     * camera pipeline).
     */
    public int pipelineIndex;

    /**
     * Returns the configured driver camera name.
     *
     * @return the driver camera network name
     */
    public Supplier<String> getCameraName() {
        return () -> readTunableString("cameraName", cameraName);
    }

    /**
     * Returns the pipeline index to use for driver mode.
     *
     * @return the driver pipeline index
     */
    public Supplier<Integer> getPipelineIndex() {
        return () -> (int) readTunableNumber("pipelineIndex", pipelineIndex);
    }
}
