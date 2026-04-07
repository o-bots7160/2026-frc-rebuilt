package frc.robot.shared.bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.logging.Logger;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.commands.GameplayStateCommandFactory;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Base class for controller-to-command binding strategies. Houses shared infrastructure — controllers, command factories, configuration, and
 * controller health monitoring — so concrete subclasses only define how buttons map to commands.
 * <p>
 * Uses WPILib's {@link CommandXboxController} which is compatible with Logitech F310 controllers when set to XInput mode (back switch on X).
 * </p>
 */
public abstract class AbstractTriggerBindings {

    /**
     * Default USB port for the driver controller.
     */
    protected static final int                       DEFAULT_DRIVE_CONTROLLER_PORT    = 0;

    /**
     * Default USB port for the operator controller.
     */
    protected static final int                       DEFAULT_OPERATOR_CONTROLLER_PORT = 1;

    /** Number of consecutive zero-input cycles before a stale-input warning fires (~1 second at 50 Hz). */
    private static final int                         STALE_INPUT_CYCLE_THRESHOLD      = 50;

    /**
     * Driver gamepad used for manual driving.
     */
    protected final CommandXboxController            driverController;

    /**
     * Operator gamepad used for mechanism control (turret, shooter, etc.).
     */
    protected final CommandXboxController            operatorController;

    /**
     * Factory that creates drive base commands tied to the driver inputs.
     */
    protected final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    /**
     * Configuration for per-axis response curves and speed tiers.
     */
    protected final TriggerBindingsConfig            triggerBindingsConfig;

    /**
     * Factory that creates turret commands tied to driver buttons.
     */
    protected final TurretSubsystemCommandFactory    turretCommandFactory;

    /**
     * Factory that creates shooter commands tied to driver buttons.
     */
    protected final ShooterSubsystemCommandFactory   shooterCommandFactory;

    /**
     * Factory that creates indexer commands tied to driver buttons.
     */
    protected final IndexerSubsystemCommandFactory   indexerCommandFactory;

    /**
     * Factory that creates feeder commands tied to driver buttons.
     */
    protected final FeederSubsystemCommandFactory    feederCommandFactory;

    /**
     * Factory that creates intake commands tied to driver buttons.
     */
    protected final IntakeSubsystemCommandFactory    intakeCommandFactory;

    /**
     * Factory that creates harvester commands tied to driver buttons.
     */
    protected final HarvesterSubsystemCommandFactory harvesterCommandFactory;

    /**
     * Factory that composes gameplay state transition commands for operator bindings.
     */
    protected final GameplayStateCommandFactory      gameplayStateCommandFactory;

    /** Logger for controller health telemetry. */
    private final Logger                             log                              = Logger.getInstance("TriggerBindings");

    /** USB port number for the driver controller, stored for health checks. */
    private final int                                driverControllerPort;

    /** USB port number for the operator controller, stored for health checks. */
    private final int                                operatorControllerPort;

    /** True when the driver controller was connected on the previous cycle. */
    private boolean                                  driverConnectedLastCycle         = true;

    /** True when the operator controller was connected on the previous cycle. */
    private boolean                                  operatorConnectedLastCycle       = true;

    /**
     * Number of consecutive teleop cycles where all driver controller axes have been exactly zero. Used to detect a stale USB data pipe where the
     * controller appears connected in the DS but is not actually sending input data.
     */
    private int                                      driverZeroCycleCount             = 0;

    /**
     * Number of consecutive teleop cycles where all operator controller axes have been exactly zero.
     */
    private int                                      operatorZeroCycleCount           = 0;

    /** True once the stale-input warning has been reported for the driver controller this enable cycle. */
    private boolean                                  driverStaleWarningFired          = false;

    /** True once the stale-input warning has been reported for the operator controller this enable cycle. */
    private boolean                                  operatorStaleWarningFired        = false;

    /**
     * Creates trigger bindings with the default driver controller port.
     *
     * @param driveBaseCommandFactory     factory for creating drive base commands
     * @param triggerBindingsConfig       configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory        factory for creating turret commands
     * @param shooterCommandFactory       factory for creating shooter commands
     * @param indexerCommandFactory       factory for creating indexer commands
     * @param feederCommandFactory        factory for creating feeder commands
     * @param intakeCommandFactory        factory for creating intake commands
     * @param harvesterCommandFactory     factory for creating harvester commands
     * @param gameplayStateCommandFactory factory for creating gameplay state transition commands
     */
    protected AbstractTriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            GameplayStateCommandFactory gameplayStateCommandFactory) {
        this(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                indexerCommandFactory,
                feederCommandFactory,
                intakeCommandFactory,
                harvesterCommandFactory,
                gameplayStateCommandFactory,
                DEFAULT_DRIVE_CONTROLLER_PORT,
                DEFAULT_OPERATOR_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using explicit controller ports.
     *
     * @param driveBaseCommandFactory     factory for creating drive base commands
     * @param triggerBindingsConfig       configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory        factory for creating turret commands
     * @param shooterCommandFactory       factory for creating shooter commands
     * @param indexerCommandFactory       factory for creating indexer commands
     * @param feederCommandFactory        factory for creating feeder commands
     * @param intakeCommandFactory        factory for creating intake commands
     * @param harvesterCommandFactory     factory for creating harvester commands
     * @param gameplayStateCommandFactory factory for creating gameplay state transition commands
     * @param driverControllerPort        USB port for the driver controller
     * @param operatorControllerPort      USB port for the operator controller
     */
    protected AbstractTriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            GameplayStateCommandFactory gameplayStateCommandFactory,
            int driverControllerPort,
            int operatorControllerPort) {
        this.driveBaseCommandFactory     = driveBaseCommandFactory;
        this.triggerBindingsConfig       = triggerBindingsConfig;
        this.turretCommandFactory        = turretCommandFactory;
        this.shooterCommandFactory       = shooterCommandFactory;
        this.indexerCommandFactory       = indexerCommandFactory;
        this.feederCommandFactory        = feederCommandFactory;
        this.intakeCommandFactory        = intakeCommandFactory;
        this.harvesterCommandFactory     = harvesterCommandFactory;
        this.gameplayStateCommandFactory = gameplayStateCommandFactory;
        this.driverControllerPort        = driverControllerPort;
        this.operatorControllerPort      = operatorControllerPort;
        this.driverController            = new CommandXboxController(driverControllerPort);
        this.operatorController          = new CommandXboxController(operatorControllerPort);

        assignDriverControls();
        assignOperatorControls();
    }

    /**
     * Checks controller connectivity and input health, logging warnings and telemetry.
     * <p>
     * Call this once per robot cycle (e.g., from a default command supplier or {@code robotPeriodic}). It performs two checks per controller:
     * </p>
     * <p>
     * 1. Connection check via {@link DriverStation#isJoystickConnected(int)}. A warning fires once when a controller transitions from connected to
     * disconnected. The latch resets when the controller reconnects.
     * </p>
     * <p>
     * 2. Stale-input detection during teleop. If a controller reports as connected but all axes have been exactly zero for
     * {@value #STALE_INPUT_CYCLE_THRESHOLD} consecutive teleop cycles, a warning fires once. This catches the failure mode where the Windows USB data
     * pipe goes to sleep during autonomous and fails to wake for teleop (the DS shows the device as present because the USB descriptor is cached, but
     * actual input data is stale). The stale warning resets when the robot leaves teleop.
     * </p>
     */
    public void checkControllerHealth() {
        // Skip health checks in simulation — controllers are rarely connected and
        // the stale-input detector would fire false warnings every teleop enable.
        if (RobotEnvironment.isSimulation()) {
            return;
        }

        boolean driverConnected   = DriverStation.isJoystickConnected(driverControllerPort);
        boolean operatorConnected = DriverStation.isJoystickConnected(operatorControllerPort);

        // Connection transition warnings (fire once per disconnect).
        if (driverConnectedLastCycle && !driverConnected) {
            RobotEnvironment.reportWarning("Driver controller (port " + driverControllerPort + ") disconnected.", false);
        }
        if (operatorConnectedLastCycle && !operatorConnected) {
            RobotEnvironment.reportWarning("Operator controller (port " + operatorControllerPort + ") disconnected.", false);
        }
        driverConnectedLastCycle   = driverConnected;
        operatorConnectedLastCycle = operatorConnected;

        // Stale-input detection (teleop only).
        if (RobotEnvironment.isTeleop() && !RobotEnvironment.isDisabled()) {
            checkStaleInput(driverController, driverConnected, true);
            checkStaleInput(operatorController, operatorConnected, false);
        } else {
            // Reset stale counters and warning latches outside teleop.
            driverZeroCycleCount      = 0;
            operatorZeroCycleCount    = 0;
            driverStaleWarningFired   = false;
            operatorStaleWarningFired = false;
        }

        // AdvantageKit telemetry.
        boolean driverInputActive   = hasAnyInput(driverController);
        boolean operatorInputActive = hasAnyInput(operatorController);
        log.recordOutput("DriverConnected", driverConnected);
        log.recordOutput("OperatorConnected", operatorConnected);
        log.recordOutput("DriverInputActive", driverInputActive);
        log.recordOutput("OperatorInputActive", operatorInputActive);
    }

    /**
     * Wires driver controller buttons and sticks to commands. Called once during construction.
     */
    protected abstract void assignDriverControls();

    /**
     * Wires operator controller buttons and triggers to commands. Called once during construction.
     */
    protected abstract void assignOperatorControls();

    /**
     * Applies deadband, a response curve, and speed scaling to a raw joystick axis value.
     * <p>
     * The processing pipeline is: deadband, then {@code sign(input) * |input|^exponent}, then multiply by speedScale, then clamp to [-1, 1]. Applying
     * the deadband first eliminates stick noise before the power function can amplify it. The response curve reshapes how quickly the output ramps
     * up: an exponent of 1.0 is linear, 2.0 (quadratic) reduces sensitivity near center while keeping full range at the edges. The speed scale
     * represents a fraction of maximum robot speed (e.g., 0.8 for normal, 1.0 for sprint, 0.4 for slow).
     * </p>
     *
     * @param rawValue   raw joystick axis value in the range [-1, 1]
     * @param exponent   response curve exponent (1.0 = linear, 2.0 = quadratic, 0.5 = square root)
     * @param speedScale fraction of maximum speed for the active trigger tier (e.g., 0.4, 0.8, or 1.0)
     * @return shaped and scaled value clamped to the range [-1, 1]
     */
    protected double applyResponseCurve(double rawValue, double exponent, double speedScale) {
        double deadbanded = MathUtil.applyDeadband(rawValue, triggerBindingsConfig.getJoystickDeadband());
        double shaped     = Math.signum(deadbanded) * Math.pow(Math.abs(deadbanded), exponent);
        return Math.max(-1.0, Math.min(1.0, shaped * speedScale));
    }

    /**
     * Selects the active translation speed scale based on trigger state.
     * <p>
     * Each scale is an absolute fraction of the robot's configured maximum linear speed. The right trigger (sprint) takes priority when both triggers
     * are pressed.
     * </p>
     *
     * @return speed scale for the current trigger state (fraction of max speed)
     */
    protected double computeTranslationSpeedScale() {
        double deadband    = triggerBindingsConfig.getTriggerDeadband();

        // Read all three scales every cycle so their tunable entries always
        // exist in NetworkTables, even when a trigger is not pressed.
        double sprintScale = triggerBindingsConfig.getSprintSpeedScale();
        double slowScale   = triggerBindingsConfig.getSlowSpeedScale();
        double normalScale = triggerBindingsConfig.getNormalSpeedScale();

        // Sprint takes priority over slow.
        if (driverController.getRightTriggerAxis() > deadband) {
            return sprintScale;
        }

        if (driverController.getLeftTriggerAxis() > deadband) {
            return slowScale;
        }

        return normalScale;
    }

    /**
     * Tracks consecutive all-zero cycles for one controller and fires a warning if the threshold is exceeded.
     *
     * @param controller the controller to check
     * @param connected  whether this controller reports as connected via the DS
     * @param isDriver   true for the driver controller, false for the operator
     */
    private void checkStaleInput(CommandXboxController controller, boolean connected, boolean isDriver) {
        if (!connected) {
            return;
        }

        boolean hasInput = hasAnyInput(controller);

        if (hasInput) {
            if (isDriver) {
                driverZeroCycleCount    = 0;
                driverStaleWarningFired = false;
            } else {
                operatorZeroCycleCount    = 0;
                operatorStaleWarningFired = false;
            }
            return;
        }

        if (isDriver) {
            driverZeroCycleCount++;
            if (driverZeroCycleCount >= STALE_INPUT_CYCLE_THRESHOLD && !driverStaleWarningFired) {
                RobotEnvironment.reportWarning(
                        "Driver controller (port " + driverControllerPort
                                + ") is connected but has sent no input for ~1 second. "
                                + "USB data pipe may be frozen — try pressing buttons or reboot the DS laptop.",
                        false);
                driverStaleWarningFired = true;
            }
        } else {
            operatorZeroCycleCount++;
            if (operatorZeroCycleCount >= STALE_INPUT_CYCLE_THRESHOLD && !operatorStaleWarningFired) {
                RobotEnvironment.reportWarning(
                        "Operator controller (port " + operatorControllerPort
                                + ") is connected but has sent no input for ~1 second. "
                                + "USB data pipe may be frozen — try pressing buttons or reboot the DS laptop.",
                        false);
                operatorStaleWarningFired = true;
            }
        }
    }

    /**
     * Returns true if any axis on the controller has a non-zero value.
     *
     * @param controller the controller to check
     * @return true if at least one axis is non-zero
     */
    private boolean hasAnyInput(CommandXboxController controller) {
        return controller.getLeftX() != 0.0
                || controller.getLeftY() != 0.0
                || controller.getRightX() != 0.0
                || controller.getRightY() != 0.0
                || controller.getLeftTriggerAxis() != 0.0
                || controller.getRightTriggerAxis() != 0.0;
    }
}
