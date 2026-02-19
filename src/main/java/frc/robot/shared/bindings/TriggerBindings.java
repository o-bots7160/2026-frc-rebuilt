package frc.robot.shared.bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
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
    private static final double                    SYSID_DELAY_SECONDS               = 20.0;

    /**
     * Timeout for the quasistatic (slow ramp) portion of system identification in seconds.
     */
    private static final double                    SYSID_QUASISTATIC_TIMEOUT_SECONDS = 10.0;

    /**
     * Timeout for the dynamic (step voltage) portion of system identification in seconds.
     */
    private static final double                    SYSID_DYNAMIC_TIMEOUT_SECONDS     = 10.0;

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
     * Configuration for per-axis response curves and speed tiers.
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
     * Factory that creates indexer commands tied to operator buttons.
     */
    private final IndexerSubsystemCommandFactory   indexerCommandFactory;

    /**
     * Factory that creates climber commands tied to operator buttons.
     */
    @SuppressWarnings("unused")
    private final ClimberSubsystemCommandFactory    climberCommandFactory;

    /**
     * Creates trigger bindings with the default driver controller port.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param triggerBindingsConfig   configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory    factory for creating turret commands
     * @param shooterCommandFactory   factory for creating shooter commands
     * @param indexerCommandFactory   factory for creating indexer commands
     * @param climberCommandFactory   factory for creating climber commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            ClimberSubsystemCommandFactory climberCommandFactory) {
        this(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                indexerCommandFactory,
                climberCommandFactory,
                DEFAULT_DRIVE_CONTROLLER_PORT,
                DEFAULT_OPERATOR_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using explicit controller ports.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param triggerBindingsConfig   configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory    factory for creating turret commands
     * @param shooterCommandFactory   factory for creating shooter commands
     * @param indexerCommandFactory   factory for creating indexer commands
     * @param climberCommandFactory   factory for creating climber commands
     * @param driverControllerPort    USB port for the driver controller
     * @param operatorControllerPort  USB port for the operator controller
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            ClimberSubsystemCommandFactory climberCommandFactory,
            int driverControllerPort,
            int operatorControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.triggerBindingsConfig   = triggerBindingsConfig;
        this.turretCommandFactory    = turretCommandFactory;
        this.shooterCommandFactory   = shooterCommandFactory;
        this.indexerCommandFactory   = indexerCommandFactory;
        this.climberCommandFactory   = climberCommandFactory;
        this.driverController        = new CommandXboxController(driverControllerPort);
        this.operatorController      = new CommandXboxController(operatorControllerPort);

        configureDriveControllerBindings();
        configureTurretBindings();
        configureShooterBindings();
        configureIndexerBindings();
        configureClimberBindings();
    }

    private void configureDriveControllerBindings() {
        // Map sticks to field-relative driving using the drive base command factory.
        // Left stick: translation (forward/back on Y, left/right on X).
        // Right stick X: rotation rate (omega).
        // Triggers: select a speed tier (slow / normal / sprint).
        // Response curve exponents and speed scales are read from tunable config each cycle.
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> applyResponseCurve(
                        driverController.getLeftY(),
                        triggerBindingsConfig.getLeftStickYResponseExponent(),
                        computeTranslationSpeedScale()),
                () -> applyResponseCurve(
                        driverController.getLeftX(),
                        triggerBindingsConfig.getLeftStickXResponseExponent(),
                        computeTranslationSpeedScale()),
                () -> applyResponseCurve(
                        driverController.getRightX(),
                        triggerBindingsConfig.getRightStickXResponseExponent(),
                        1.0));
    }

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
    private double applyResponseCurve(double rawValue, double exponent, double speedScale) {
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
    private double computeTranslationSpeedScale() {
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

    private void configureShooterBindings() {
        // Hold right bumper to spin the shooter at the tunable test RPM.
        // Uses continuous spin so slider changes take effect immediately.
        driverController.rightBumper().whileTrue(
                shooterCommandFactory.createContinuousSpinCommand(
                        triggerBindingsConfig::getShooterTestSpeedRpm));

        // driverController.y().whileTrue(
        //         shooterCommandFactory.createSysIdFullSweepCommand(
        //                 SYSID_DELAY_SECONDS,
        //                 SYSID_QUASISTATIC_TIMEOUT_SECONDS,
        //                 SYSID_DYNAMIC_TIMEOUT_SECONDS));
    }

    private void configureTurretBindings() {
        // Hold A/B to spin the turret to the configured angles (for testing and alignment).
        // driverController.a().whileTrue(
        //         turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_LEFT_DEGREES));
        // driverController.b().whileTrue(
        //         turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_RIGHT_DEGREES));

        // driverController.x().whileTrue(
        //         turretCommandFactory.createSysIdFullSweepCommand(
        //                 SYSID_DELAY_SECONDS,
        //                 SYSID_QUASISTATIC_TIMEOUT_SECONDS,
        //                 SYSID_DYNAMIC_TIMEOUT_SECONDS));
    }

    private void configureClimberBindings() {
        // TODO: wire climber commands when motor hardware is selected
    }

    private void configureIndexerBindings() {
        // Hold left bumper to feed Fuel into the shooter at the configured feed RPM.
        driverController.leftBumper().whileTrue(
                indexerCommandFactory.createFeedAndHoldCommand());

        // Hold D-pad down to run the unjam cycle (alternating forward/reverse pulses).
        driverController.povDown().whileTrue(
                indexerCommandFactory.createUnjamCommand());

        // Hold D-pad left to reverse the indexer and back Fuel toward the hopper.
        driverController.povLeft().whileTrue(
                indexerCommandFactory.createReverseCommand());
    }

}
