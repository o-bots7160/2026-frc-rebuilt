package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.vision.config.DriverCameraSubsystemConfig;

/**
 * Subsystem that manages a camera in driver mode for driver camera streaming.
 *
 * <p>This subsystem configures the camera to operate as a driver camera,
 * disabling vision processing and LEDs to provide a clean video stream for
 * the driver station.</p>
 * 
 * <p>This is currently Limelight specific.  If we want to have the option
 * of using the PhotonVision rig for a drive camera we'll need to adjust.</p>
 */
public class DriverCameraSubsystem extends AbstractSubsystem<DriverCameraSubsystemConfig> {

    private static final int CAM_MODE_DRIVER = 1;
    private static final int LED_MODE_FORCE_OFF = 1;
    private static final int STREAM_MODE_STANDARD = 0;

    private final NetworkTable table;

    private boolean initialized = false;

    /**
     * Creates a new DriverCameraSubsystem.
     *
     * @param config configuration for the driver camera
     */
    public DriverCameraSubsystem(DriverCameraSubsystemConfig config) {
        super(config);

        String cameraName = config.getCameraName().get();
        this.table = NetworkTableInstance.getDefault().getTable(cameraName);
        log.info("DriverCameraSubsystem initialized for camera: " + cameraName);
    }

    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            return;
        }

        if (!initialized) {
            configureDriverMode();
            initialized = true;
        }
    }

    /**
     * Configures the camera for driver mode operation.
     */
    private void configureDriverMode() {
        table.getEntry("pipeline").setNumber(config.getPipelineIndex().get());
        table.getEntry("camMode").setNumber(CAM_MODE_DRIVER);
        table.getEntry("ledMode").setNumber(LED_MODE_FORCE_OFF);
        table.getEntry("stream").setNumber(STREAM_MODE_STANDARD);

        log.info("drive camera configured for driver mode");
    }
}
