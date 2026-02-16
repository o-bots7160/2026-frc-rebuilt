package frc.robot.subsystems.drivercameravision.config;

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
     * Startup stream mode for Limelight driver feed selection.
     * <p>
     * 1 = Limelight onboard camera, 2 = external USB camera, 0 = both (unused in this project).
     * </p>
     */
    public int    defaultStream;

    /**
     * Pipeline index to use for driver mode (typically 0 for a driver camera pipeline).
     */
    public int    pipelineIndex;

    /**
     * Returns the configured driver camera name.
     *
     * @return the driver camera network name
     */
    public Supplier<String> getCameraName() {
        return () -> readTunableString("cameraName", cameraName);
    }

    /**
     * Returns the default stream to use at startup in the case there are multiple available.
     *
     * @return the stream to show at startup
     */
    public Supplier<Integer> getDefaultStream() {
        return () -> (int) readTunableNumber("defaultStream", defaultStream);
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
