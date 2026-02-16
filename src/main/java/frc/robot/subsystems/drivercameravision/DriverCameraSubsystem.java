package frc.robot.subsystems.drivercameravision;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.drivercameravision.config.DriverCameraSubsystemConfig;

/**
 * Subsystem that manages a camera in driver mode for driver camera streaming.
 * <p>
 * This subsystem configures the camera to operate as a driver camera, disabling vision processing and LEDs to provide a clean video stream for the
 * driver station.
 * </p>
 * <p>
 * This is currently Limelight specific. If we want to have the option of using the PhotonVision rig for a drive camera we'll need to adjust.
 * </p>
 */
public class DriverCameraSubsystem extends AbstractSubsystem<DriverCameraSubsystemConfig> {

    /**
     * Supported driver camera stream modes.
     */
    public enum StreamMode {
        /* there is a 0 mode that is side-by-side but we're not supporting it */
        LIMELIGHT_ONBOARD(1),
        USB_CAMERA(2);

        /**
         * Maybe be used in config or NT at runtime and value could come in as typo.
         */
        public static Optional<StreamMode> from(int networkTableValue) {
            for (StreamMode streamMode : values()) {
                if (streamMode.networkTableValue == networkTableValue) {
                    return Optional.of(streamMode);
                }
            }
            return Optional.empty();
        }

        public final int networkTableValue;

        StreamMode(int networkTableValue) {
            this.networkTableValue = networkTableValue;
        }

        public int getNetworkTableValue() {
            return networkTableValue;
        }

        public StreamMode toggle() {
            return this == LIMELIGHT_ONBOARD ? USB_CAMERA : LIMELIGHT_ONBOARD;
        }
    }

    /** The API documentation specifies that LED mode 1 forces targeting LEDs off. */
    private static final int    LED_MODE_FORCE_OFF         = 1;

    /** The dashboard topic to be available at competition runtime will be in SmartDashboard table. */
    private static final String SMART_DASHBOARD_TABLE      = "SmartDashboard";

    /** Dashboard topic to toggle between onboard and USB camera streams. */
    private static final String USE_USB_STREAM_TOPIC       = "DriverCamera/useUsbCameraStream";

    /** The topic used in limelight table to set the pipeline index. */
    private static final String LIMELIGHT_PIPELINE_TOPIC   = "pipeline";

    /** The topic used in limelight table to set the LED mode. */
    private static final String LIMELIGHT_LED_MODE_TOPIC   = "ledMode";

    /** The topic used in limelight table to set the camera stream it should send. */
    private static final String LIMELIGHT_STREAM_TOPIC     = "stream";

    /** The table used to communicate to Limelight API. */
    private final NetworkTable      cameraApiTable;

    /** The entry in the dashboard table to toggle which camera is desired. */
    private final NetworkTableEntry useUsbStreamEntry;

    /** Tracks currently configured stream mode to enable simple toggling. */
    private StreamMode              activeStreamMode;

    /** Tracks whether the startup config has been pushed to the hardware. */
    private boolean                 initialized            = false;

    /**
     * Creates a new DriverCameraSubsystem.
     *
     * @param config configuration for the driver camera
     */
    public DriverCameraSubsystem(DriverCameraSubsystemConfig config) {
        super(config);

        String cameraName = config.getCameraName().get();
        this.cameraApiTable    = NetworkTableInstance.getDefault()
                .getTable(cameraName);
        this.useUsbStreamEntry = NetworkTableInstance.getDefault()
                .getTable(SMART_DASHBOARD_TABLE)
                .getEntry(USE_USB_STREAM_TOPIC);
        this.activeStreamMode = parseConfiguredDefaultStreamMode();

        useUsbStreamEntry.setBoolean(activeStreamMode == StreamMode.USB_CAMERA);

        // put a command into the dashboard to toggle camera stream
        log.dashboard("ToggleStreamCommand", createToggleStreamCommand());

        log.info("DriverCameraSubsystem initialized for camera: " + cameraName);
    }

    /**
     * Creates a one-shot command that toggles the active driver stream.
     *
     * @return command for dashboard or button bindings
     */
    public Command createToggleStreamCommand() {
        return Commands.runOnce(this::toggleStreamMode, this);
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

        StreamMode requestedMode = useUsbStreamEntry.getBoolean(activeStreamMode == StreamMode.USB_CAMERA)
                ? StreamMode.USB_CAMERA
                : StreamMode.LIMELIGHT_ONBOARD;
        if (requestedMode != activeStreamMode) {
            setStreamMode(requestedMode);
            log.recordVerboseOutput("activeStreamMode", activeStreamMode.getNetworkTableValue());
            log.recordVerboseOutput("activeUsbCameraStream", activeStreamMode == StreamMode.USB_CAMERA);
        }
    }

    /**
     * Sets the active Limelight driver stream mode.
     *
     * @param streamMode desired stream mode
     */
    public void setStreamMode(StreamMode streamMode) {
        if (isSubsystemDisabled()) {
            logDisabled("setStreamMode");
            return;
        }

        if (initialized && streamMode == activeStreamMode) {
            return;
        }

        cameraApiTable.getEntry(LIMELIGHT_STREAM_TOPIC).setNumber(streamMode.getNetworkTableValue());
        activeStreamMode = streamMode;
        log.info("driver camera stream set to " + streamMode + " (" + streamMode.getNetworkTableValue() + ")");
    }

    /**
     * Toggles between Limelight onboard camera stream and external USB camera stream.
     */
    public void toggleStreamMode() {
        if (isSubsystemDisabled()) {
            logDisabled("toggleStreamMode");
            return;
        }

        StreamMode toggledMode = activeStreamMode.toggle();
        useUsbStreamEntry.setBoolean(toggledMode == StreamMode.USB_CAMERA);
        setStreamMode(toggledMode);
    }

    /**
     * Configures the camera for driver mode operation.
     */
    private void configureDriverMode() {
        cameraApiTable.getEntry(LIMELIGHT_PIPELINE_TOPIC).setNumber(config.getPipelineIndex().get());
        cameraApiTable.getEntry(LIMELIGHT_LED_MODE_TOPIC).setNumber(LED_MODE_FORCE_OFF);
        setStreamMode(activeStreamMode);

        log.info("drive camera configured for driver mode");
    }

    private StreamMode parseConfiguredDefaultStreamMode() {
        int configuredValue = config.getDefaultStream().get();
        return StreamMode.from(configuredValue)
                .orElseGet(() -> {
                    log.warning("invalid defaultStream value " + configuredValue + "; expected 1 or 2. Falling back to stream 1.");
                    return StreamMode.LIMELIGHT_ONBOARD;
                });
    }
}
