package frc.robot.shared.bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Maps the driver controller to robot commands so RobotContainer stays lean. Currently wires the drive controller to the default manual drive
 * command.
 * <p>
 * Uses WPILib's {@link CommandXboxController} which is compatible with Logitech F310 controllers when set to XInput mode (back switch on X).
 * </p>
 */
public class TriggerBindings {

    /**
     * Default USB port for the driver controller.
     */
    private static final int                       DEFAULT_DRIVE_CONTROLLER_PORT     = 0;

    /**
     * Default USB port for the operator controller.
     */
    private static final int                       DEFAULT_OPERATOR_CONTROLLER_PORT  = 1;

    /**
     * Turret test angle for the A button in degrees (counterclockwise positive).
     */
    private static final double                    TURRET_TEST_ANGLE_LEFT_DEGREES    = -100.0;

    /**
     * Turret test angle for the B button in degrees (counterclockwise positive).
     */
    private static final double                    TURRET_TEST_ANGLE_RIGHT_DEGREES   = 100.0;

    /**
     * Delay before system identification begins in seconds.
     */
    private static final double                    SYSID_DELAY_SECONDS               = 1.0;

    /**
     * Timeout for the quasistatic (slow ramp) portion of system identification in seconds.
     */
    private static final double                    SYSID_QUASISTATIC_TIMEOUT_SECONDS = 8.0;

    /**
     * Timeout for the dynamic (step voltage) portion of system identification in seconds.
     */
    private static final double                    SYSID_DYNAMIC_TIMEOUT_SECONDS     = 3.0;

    /**
     * Driver gamepad used for manual driving.
     */
    private final CommandXboxController            driverController;

    // TODO: Wire operator bindings once more subsystems and commands are available.
    /**
     * Operator gamepad used for mechanism control (turret, shooter, etc.).
     */
    @SuppressWarnings("unused")
    private final CommandXboxController            operatorController;

    /**
     * Factory that creates drive base commands tied to the driver inputs.
     */
    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    /**
     * Configuration for per-axis stick sensitivity.
     */
    private final TriggerBindingsConfig            triggerBindingsConfig;

    /**
     * Factory that creates turret commands tied to driver buttons.
     */
    private final TurretSubsystemCommandFactory    turretCommandFactory;

    /**
     * Factory that creates shooter commands tied to operator buttons.
     */
    private final ShooterSubsystemCommandFactory   shooterCommandFactory;

    /**
     * Creates trigger bindings with the default driver controller port.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param triggerBindingsConfig   configuration for per-axis stick sensitivity
     * @param turretCommandFactory    factory for creating turret commands
     * @param shooterCommandFactory   factory for creating shooter commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory) {
        this(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                DEFAULT_DRIVE_CONTROLLER_PORT,
                DEFAULT_OPERATOR_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using explicit controller ports.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param triggerBindingsConfig   configuration for per-axis stick sensitivity
     * @param turretCommandFactory    factory for creating turret commands
     * @param shooterCommandFactory   factory for creating shooter commands
     * @param driverControllerPort    USB port for the driver controller
     * @param operatorControllerPort  USB port for the operator controller
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            int driverControllerPort,
            int operatorControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.triggerBindingsConfig   = triggerBindingsConfig;
        this.turretCommandFactory    = turretCommandFactory;
        this.shooterCommandFactory   = shooterCommandFactory;
        this.driverController        = new CommandXboxController(driverControllerPort);
        this.operatorController      = new CommandXboxController(operatorControllerPort);

        configureDriveControllerBindings();
        configureTurretBindings();
        configureShooterBindings();
    }

    private void configureDriveControllerBindings() {
        // Map sticks to field-relative driving using the drive base command factory.
        // Left stick: translation (forward/back on Y, left/right on X).
        // Right stick X: rotation rate (omega).
        // Triggers: scale translation speed for slow/fast control.
        // Per-axis sensitivity multipliers are read from the tunable config each cycle.
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> MathUtil.clamp(
                        driverController.getLeftY()
                                * triggerBindingsConfig.getLeftStickYSensitivity()
                                * computeDriveThrottleScale(),
                        -1.0,
                        1.0),
                () -> MathUtil.clamp(
                        driverController.getLeftX()
                                * triggerBindingsConfig.getLeftStickXSensitivity()
                                * computeDriveThrottleScale(),
                        -1.0,
                        1.0),
                () -> MathUtil.clamp(
                        driverController.getRightX()
                                * triggerBindingsConfig.getRightStickXSensitivity(),
                        -1.0,
                        1.0));
    }

    /**
     * Computes a throttle multiplier based on trigger state.
     * <p>
     * If the right trigger is pressed beyond the deadband, the configured speed-up factor is returned (e.g., 1.5 for +50 percent speed). Otherwise,
     * if the left trigger is pressed beyond the deadband, the configured slow-down factor is returned (e.g., 0.5 for -50 percent speed). When neither
     * trigger is pressed, 1.0 is returned (normal speed). The right trigger (speed-up) takes priority when both are pressed.
     * </p>
     *
     * @return throttle multiplier applied to translation inputs
     */
    private double computeDriveThrottleScale() {
        double deadband = triggerBindingsConfig.getTriggerDeadband();

        // Speed-up takes priority over slow-down.
        if (driverController.getRightTriggerAxis() > deadband) {
            return triggerBindingsConfig.getSpeedUpTriggerFactor();
        }

        if (driverController.getLeftTriggerAxis() > deadband) {
            return triggerBindingsConfig.getSlowDownTriggerFactor();
        }

        return 1.0;
    }

    private void configureShooterBindings() {
        // Hold right bumper to spin the shooter up to a fixed test RPM.
        operatorController.rightBumper().whileTrue(
                shooterCommandFactory.createSpinUpCommand(3000.0));

        operatorController.y().whileTrue(
                shooterCommandFactory.createSysIdFullSweepCommand(
                        SYSID_DELAY_SECONDS,
                        SYSID_QUASISTATIC_TIMEOUT_SECONDS,
                        SYSID_DYNAMIC_TIMEOUT_SECONDS));
    }

    private void configureTurretBindings() {
        // Hold A/B to spin the turret to the configured angles (for testing and alignment).
        driverController.a().whileTrue(
                turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_LEFT_DEGREES));
        driverController.b().whileTrue(
                turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_RIGHT_DEGREES));

        driverController.x().whileTrue(
                turretCommandFactory.createSysIdFullSweepCommand(
                        SYSID_DELAY_SECONDS,
                        SYSID_QUASISTATIC_TIMEOUT_SECONDS,
                        SYSID_DYNAMIC_TIMEOUT_SECONDS));
    }

}
