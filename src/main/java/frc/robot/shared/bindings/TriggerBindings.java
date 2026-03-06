package frc.robot.shared.bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Maps the driver controller to robot commands so RobotContainer stays lean. Wires the drive controller to the default manual drive command and
 * provides subsystem test bindings via a dashboard-selectable chooser.
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

    // Chooser option constants for subsystem test selection.
    private static final String                    TEST_SUBSYSTEM_SHOOTER            = "Shooter";

    private static final String                    TEST_SUBSYSTEM_INDEXER            = "Indexer";

    private static final String                    TEST_SUBSYSTEM_FEEDER             = "Feeder";

    private static final String                    TEST_SUBSYSTEM_INTAKE             = "Intake";

    private static final String                    TEST_SUBSYSTEM_TURRET             = "Turret";

    private static final String                    TEST_SUBSYSTEM_HARVESTER          = "Harvester";

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
     * Factory that creates shooter commands tied to driver buttons.
     */
    private final ShooterSubsystemCommandFactory   shooterCommandFactory;

    /**
     * Factory that creates indexer commands tied to driver buttons.
     */
    private final IndexerSubsystemCommandFactory   indexerCommandFactory;

    /**
     * Factory that creates climber commands tied to driver buttons.
     */
    @SuppressWarnings("unused")
    private final ClimberSubsystemCommandFactory   climberCommandFactory;

    /**
     * Factory that creates feeder commands tied to driver buttons.
     */
    private final FeederSubsystemCommandFactory    feederCommandFactory;

    /**
     * Factory that creates intake commands tied to driver buttons.
     */
    private final IntakeSubsystemCommandFactory    intakeCommandFactory;

    /**
     * Factory that creates harvester commands tied to driver buttons.
     */
    private final HarvesterSubsystemCommandFactory harvesterCommandFactory;

    /**
     * Dashboard chooser that selects which subsystem the A/B/X test buttons control.
     */
    private final SendableChooser<String>          testSubsystemChooser;

    /**
     * Creates trigger bindings with the default driver controller port.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param triggerBindingsConfig   configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory    factory for creating turret commands
     * @param shooterCommandFactory   factory for creating shooter commands
     * @param indexerCommandFactory   factory for creating indexer commands
     * @param climberCommandFactory   factory for creating climber commands
     * @param feederCommandFactory    factory for creating feeder commands
     * @param intakeCommandFactory    factory for creating intake commands
     * @param harvesterCommandFactory factory for creating harvester commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            ClimberSubsystemCommandFactory climberCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory) {
        this(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                indexerCommandFactory,
                climberCommandFactory,
                feederCommandFactory,
                intakeCommandFactory,
                harvesterCommandFactory,
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
     * @param feederCommandFactory    factory for creating feeder commands
     * @param intakeCommandFactory    factory for creating intake commands
     * @param harvesterCommandFactory factory for creating harvester commands
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
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            int driverControllerPort,
            int operatorControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.triggerBindingsConfig   = triggerBindingsConfig;
        this.turretCommandFactory    = turretCommandFactory;
        this.shooterCommandFactory   = shooterCommandFactory;
        this.indexerCommandFactory   = indexerCommandFactory;
        this.climberCommandFactory   = climberCommandFactory;
        this.feederCommandFactory    = feederCommandFactory;
        this.intakeCommandFactory    = intakeCommandFactory;
        this.harvesterCommandFactory = harvesterCommandFactory;
        this.driverController        = new CommandXboxController(driverControllerPort);
        this.operatorController      = new CommandXboxController(operatorControllerPort);

        // Build the subsystem test chooser and publish it to SmartDashboard.
        testSubsystemChooser         = new SendableChooser<>();
        testSubsystemChooser.setDefaultOption(TEST_SUBSYSTEM_SHOOTER, TEST_SUBSYSTEM_SHOOTER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INDEXER, TEST_SUBSYSTEM_INDEXER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_FEEDER, TEST_SUBSYSTEM_FEEDER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INTAKE, TEST_SUBSYSTEM_INTAKE);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_TURRET, TEST_SUBSYSTEM_TURRET);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_HARVESTER, TEST_SUBSYSTEM_HARVESTER);
        SmartDashboard.putData("TriggerBindings/TestSubsystem", testSubsystemChooser);

        configureDriveControllerBindings();
        configureSubsystemTestBindings();

        // Production bindings are temporarily disabled for shop testing.
        // Uncomment these once subsystems are characterized and test bindings are no longer needed.
        // configureTurretBindings();
        // configureShooterBindings();
        // configureIndexerBindings();
        // configureClimberBindings();
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
     * Wires A, B, and X buttons on the driver controller to test the dashboard-selected subsystem.
     * <p>
     * A = reverse/min, B = forward/max, X = full SysId sweep. The subsystem under test is chosen via the {@code TriggerBindings/TestSubsystem}
     * SendableChooser on the dashboard. Commands are resolved at button-press time using deferred proxy so the chooser selection is always current.
     * </p>
     */
    private void configureSubsystemTestBindings() {
        // A button: reverse (velocity) or move to minimum setpoint (set-and-seek).
        driverController.a().whileTrue(
                Commands.deferredProxy(this::createSelectedReverseCommand));

        // B button: forward (velocity) or move to maximum setpoint (set-and-seek).
        driverController.b().whileTrue(
                Commands.deferredProxy(this::createSelectedForwardCommand));

        // X button: run full SysId characterization sweep for the selected subsystem.
        // Press X to start (~60 s total); press X again to cancel early.
        driverController.x().whileTrue(
                Commands.deferredProxy(this::createSelectedSysIdCommand));
    }

    /**
     * Creates the reverse/min command for the currently selected test subsystem.
     * <p>
     * For velocity subsystems this spins the motor in reverse at the configured idle RPM. For set-and-seek subsystems this moves to the minimum
     * setpoint (soft limit).
     * </p>
     *
     * @return command for the selected subsystem, or {@link Commands#none()} if nothing is selected
     */
    private Command createSelectedReverseCommand() {
        String selected = testSubsystemChooser.getSelected();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createContinuousVelocityCommand(
                    () -> -shooterCommandFactory.getSubsystem().getConfig().getIdleVelocityRpm());
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createContinuousVelocityCommand(
                    () -> -indexerCommandFactory.getSubsystem().getConfig().getIdleVelocityRpm());
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createContinuousVelocityCommand(
                    () -> -feederCommandFactory.getSubsystem().getConfig().getIdleVelocityRpm());
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createContinuousVelocityCommand(
                    () -> -intakeCommandFactory.getSubsystem().getConfig().getIdleVelocityRpm());
        case TEST_SUBSYSTEM_TURRET:
            return turretCommandFactory.createMoveToAngleCommand(
                    turretCommandFactory.getSubsystem().getConfig()::getMinimumSetpointDegrees);
        case TEST_SUBSYSTEM_HARVESTER:
            return harvesterCommandFactory.createMoveToPositionCommand(
                    harvesterCommandFactory.getSubsystem().getConfig()::getMinimumSetpointDegrees);
        default:
            return Commands.none();
        }
    }

    /**
     * Creates the forward/max command for the currently selected test subsystem.
     * <p>
     * For velocity subsystems this spins the motor forward at the configured idle RPM. For set-and-seek subsystems this moves to the maximum setpoint
     * (soft limit).
     * </p>
     *
     * @return command for the selected subsystem, or {@link Commands#none()} if nothing is selected
     */
    private Command createSelectedForwardCommand() {
        String selected = testSubsystemChooser.getSelected();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createContinuousVelocityCommand(
                    shooterCommandFactory.getSubsystem().getConfig()::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createContinuousVelocityCommand(
                    indexerCommandFactory.getSubsystem().getConfig()::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createContinuousVelocityCommand(
                    feederCommandFactory.getSubsystem().getConfig()::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createContinuousVelocityCommand(
                    intakeCommandFactory.getSubsystem().getConfig()::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_TURRET:
            return turretCommandFactory.createMoveToAngleCommand(
                    turretCommandFactory.getSubsystem().getConfig()::getMaximumSetpointDegrees);
        case TEST_SUBSYSTEM_HARVESTER:
            return harvesterCommandFactory.createMoveToPositionCommand(
                    harvesterCommandFactory.getSubsystem().getConfig()::getMaximumSetpointDegrees);
        default:
            return Commands.none();
        }
    }

    /**
     * Creates a full SysId characterization sweep for the currently selected test subsystem.
     * <p>
     * The sweep runs all four phases (quasistatic forward, quasistatic reverse, dynamic forward, dynamic reverse) with configurable delays and
     * timeouts, totaling approximately 60 seconds.
     * </p>
     *
     * @return SysId sweep command for the selected subsystem, or {@link Commands#none()} if nothing is selected
     */
    private Command createSelectedSysIdCommand() {
        String selected = testSubsystemChooser.getSelected();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        case TEST_SUBSYSTEM_TURRET:
            return turretCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        case TEST_SUBSYSTEM_HARVESTER:
            return harvesterCommandFactory.createSysIdFullSweepCommand(
                    SYSID_DELAY_SECONDS, SYSID_QUASISTATIC_TIMEOUT_SECONDS, SYSID_DYNAMIC_TIMEOUT_SECONDS);
        default:
            return Commands.none();
        }
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

    // ── Production bindings (disabled during shop testing) ──────────────────────
    // Uncomment the configure*Bindings() calls in the constructor and restore these
    // methods once subsystems are characterized and test bindings are no longer needed.

    // private void configureShooterBindings() {
    // driverController.rightBumper().whileTrue(
    // shooterCommandFactory.createContinuousSpinCommand(
    // triggerBindingsConfig::getShooterTestSpeedRpm));
    // }

    // private void configureTurretBindings() {
    // // Hold A/B to spin the turret to the configured angles (for testing and alignment).
    // // driverController.a().whileTrue(
    // // turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_LEFT_DEGREES));
    // // driverController.b().whileTrue(
    // // turretCommandFactory.createMoveToAngleCommand(TURRET_TEST_ANGLE_RIGHT_DEGREES));
    // }

    // private void configureClimberBindings() {
    // // TODO: wire climber commands when motor hardware is selected
    // }

    // private void configureIndexerBindings() {
    // driverController.leftBumper().whileTrue(
    // indexerCommandFactory.createFeedAndHoldCommand());
    // driverController.povDown().whileTrue(
    // indexerCommandFactory.createUnjamCommand());
    // driverController.povLeft().whileTrue(
    // indexerCommandFactory.createReverseCommand());
    // }

}
