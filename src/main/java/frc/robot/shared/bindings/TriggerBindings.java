package frc.robot.shared.bindings;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.commands.GameplayStateCommandFactory;
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
    private static final int                       DEFAULT_DRIVE_CONTROLLER_PORT    = 0;

    /**
     * Default USB port for the operator controller.
     */
    private static final int                       DEFAULT_OPERATOR_CONTROLLER_PORT = 1;

    // Chooser option constants for subsystem test selection.
    private static final String                    TEST_SUBSYSTEM_SHOOTER           = "Shooter";

    private static final String                    TEST_SUBSYSTEM_INDEXER           = "Indexer";

    private static final String                    TEST_SUBSYSTEM_FEEDER            = "Feeder";

    private static final String                    TEST_SUBSYSTEM_INTAKE            = "Intake";

    private static final String                    TEST_SUBSYSTEM_TURRET            = "Turret";

    private static final String                    TEST_SUBSYSTEM_HARVESTER         = "Harvester";

    /**
     * Driver gamepad used for manual driving.
     */
    private final CommandXboxController            driverController;

    /**
     * Operator gamepad used for mechanism control (turret, shooter, etc.).
     */
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

    // TODO: Re-enable climber for post-first-competition
    // /**
    // * Factory that creates climber commands tied to driver buttons.
    // */
    // @SuppressWarnings("unused")
    // private final ClimberSubsystemCommandFactory climberCommandFactory;

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
     * Factory that composes gameplay state transition commands for operator bindings.
     */
    private final GameplayStateCommandFactory      gameplayStateCommandFactory;

    /**
     * Dashboard chooser that selects which subsystem the A/B/X test buttons control. Only initialized when tuning mode is enabled.
     */
    private LoggedDashboardChooser<String>         testSubsystemChooser;

    /**
     * Creates trigger bindings with the default driver controller port.
     *
     * @param driveBaseCommandFactory     factory for creating drive base commands
     * @param triggerBindingsConfig       configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory        factory for creating turret commands
     * @param shooterCommandFactory       factory for creating shooter commands
     * @param indexerCommandFactory       factory for creating indexer commands
     * @param climberCommandFactory       factory for creating climber commands
     * @param feederCommandFactory        factory for creating feeder commands
     * @param intakeCommandFactory        factory for creating intake commands
     * @param harvesterCommandFactory     factory for creating harvester commands
     * @param gameplayStateCommandFactory factory for creating gameplay state transition commands
     */
    public TriggerBindings(
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
    public TriggerBindings(
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
        // TODO: Re-enable climber for post-first-competition
        // this.climberCommandFactory = climberCommandFactory;
        this.feederCommandFactory        = feederCommandFactory;
        this.intakeCommandFactory        = intakeCommandFactory;
        this.harvesterCommandFactory     = harvesterCommandFactory;
        this.gameplayStateCommandFactory = gameplayStateCommandFactory;
        this.driverController            = new CommandXboxController(driverControllerPort);
        this.operatorController          = new CommandXboxController(operatorControllerPort);

        if (triggerBindingsConfig.getTuningEnabled()) {
            configureSubsystemTestBindings();
        } else {
            configureDriveControllerBindings();
            configureOperatorBindings();
        }
    }

    /**
     * Wires the driver controller sticks and triggers to field-relative driving commands.
     * <p>
     * Left stick controls translation (Y = forward/back, X = strafe), right stick X controls rotation rate. Triggers select a speed tier (slow,
     * normal, sprint). Response curve exponents and speed scales are read from tunable config each cycle.
     * </p>
     */
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

        // Right bumper: travel mode (stow harvester and idle all mechanisms).
        driverController.rightBumper().onTrue(
                gameplayStateCommandFactory.createTravelCommand());

        // Left bumper: trench travel mode (deploy harvester for low-profile field traversal).
        driverController.leftBumper().onTrue(
                gameplayStateCommandFactory.createTrenchTravelCommand());

        // X button: spin 180 degrees from current heading. Hold to maintain heading lock;
        // release to return to normal manual rotation control.
        driverController.x().whileTrue(
                driveBaseCommandFactory.createSpin180Command(
                        () -> applyResponseCurve(
                                driverController.getLeftY(),
                                triggerBindingsConfig.getLeftStickYResponseExponent(),
                                computeTranslationSpeedScale()),
                        () -> applyResponseCurve(
                                driverController.getLeftX(),
                                triggerBindingsConfig.getLeftStickXResponseExponent(),
                                computeTranslationSpeedScale())));

        // Y button: snap to nearest field-facing orientation (0 or 180 degrees field-relative).
        // If already at the nearest orientation, snaps to the opposite one instead.
        // Hold to maintain heading lock; release to return to normal manual rotation control.
        driverController.y().whileTrue(
                driveBaseCommandFactory.createSnapToFieldFacingCommand(
                        () -> applyResponseCurve(
                                driverController.getLeftY(),
                                triggerBindingsConfig.getLeftStickYResponseExponent(),
                                computeTranslationSpeedScale()),
                        () -> applyResponseCurve(
                                driverController.getLeftX(),
                                triggerBindingsConfig.getLeftStickXResponseExponent(),
                                computeTranslationSpeedScale())));

        configureDpadPathfindingBindings();
    }

    /**
     * Wires driver controller buttons to test the dashboard-selected subsystem and run drive base characterization.
     * <p>
     * A = reverse/min, B = forward/max, X = full SysId sweep for the chooser-selected subsystem, Y = drive base angle then drive SysId in sequence.
     * The subsystem under test is chosen via the {@code TriggerBindings/TestSubsystem} SendableChooser on the dashboard. Commands are resolved at
     * button-press time using deferred proxy so the chooser selection is always current.
     * </p>
     */
    private void configureSubsystemTestBindings() {
        // Build the subsystem test chooser and publish it to SmartDashboard.
        testSubsystemChooser = new LoggedDashboardChooser<>("TriggerBindings/TestSubsystem");
        testSubsystemChooser.addDefaultOption(TEST_SUBSYSTEM_SHOOTER, TEST_SUBSYSTEM_SHOOTER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INDEXER, TEST_SUBSYSTEM_INDEXER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_FEEDER, TEST_SUBSYSTEM_FEEDER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INTAKE, TEST_SUBSYSTEM_INTAKE);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_TURRET, TEST_SUBSYSTEM_TURRET);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_HARVESTER, TEST_SUBSYSTEM_HARVESTER);

        // A button: reverse (velocity) or move to minimum setpoint (set-and-seek).
        driverController.a().whileTrue(
                Commands.deferredProxy(this::createSelectedReverseCommand));

        // B button: forward (velocity) or move to maximum setpoint (set-and-seek).
        driverController.b().whileTrue(
                Commands.deferredProxy(this::createSelectedForwardCommand));

        // X button: run full SysId sweep for the dashboard-selected subsystem.
        // Press X to start; press X again to cancel early.
        driverController.x().whileTrue(
                Commands.deferredProxy(this::createSelectedSysIdCommand));

        // Y button: run drive base angle and drive SysId characterization in sequence.
        // Press Y to start; press Y again to cancel early.
        driverController.y().whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommand()
                        .andThen(driveBaseCommandFactory.createDriveSysIdCommand()));

        // Operator controller: per-module SysId characterization.
        // Drive motors: A = front-left, B = front-right, X = back-left, Y = back-right.
        operatorController.a().whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_LEFT));
        operatorController.b().whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_RIGHT));
        operatorController.x().whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_LEFT));
        operatorController.y().whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_RIGHT));

        // Angle motors: LB = front-left, RB = front-right, LT = back-left, RT = back-right.
        operatorController.leftBumper().whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_LEFT));
        operatorController.rightBumper().whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_RIGHT));
        operatorController.leftTrigger().whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_LEFT));
        operatorController.rightTrigger().whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_RIGHT));
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
        String selected = testSubsystemChooser.get();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createContinuousVelocityCommand(
                    () -> -shooterCommandFactory.getSubsystem().getConfig().motionProfile.getIdleVelocityRpm());
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createContinuousVelocityCommand(
                    () -> -indexerCommandFactory.getSubsystem().getConfig().motionProfile.getIdleVelocityRpm());
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createContinuousVelocityCommand(
                    () -> -feederCommandFactory.getSubsystem().getConfig().motionProfile.getIdleVelocityRpm());
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createContinuousVelocityCommand(
                    () -> -intakeCommandFactory.getSubsystem().getConfig().motionProfile.getIdleVelocityRpm());
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
        String selected = testSubsystemChooser.get();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createContinuousVelocityCommand(
                    shooterCommandFactory.getSubsystem().getConfig().motionProfile::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createContinuousVelocityCommand(
                    indexerCommandFactory.getSubsystem().getConfig().motionProfile::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createContinuousVelocityCommand(
                    feederCommandFactory.getSubsystem().getConfig().motionProfile::getIdleVelocityRpm);
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createContinuousVelocityCommand(
                    intakeCommandFactory.getSubsystem().getConfig().motionProfile::getIdleVelocityRpm);
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
     * The sweep runs all four phases (quasistatic forward, quasistatic reverse, dynamic forward, dynamic reverse) with delays and timeouts read from
     * each subsystem's config so each mechanism can define its own timing.
     * </p>
     *
     * @return SysId sweep command for the selected subsystem, or {@link Commands#none()} if nothing is selected
     */
    private Command createSelectedSysIdCommand() {
        String selected = testSubsystemChooser.get();
        if (selected == null) {
            return Commands.none();
        }

        switch (selected) {
        case TEST_SUBSYSTEM_SHOOTER:
            return shooterCommandFactory.createSysIdFullSweepCommand();
        case TEST_SUBSYSTEM_INDEXER:
            return indexerCommandFactory.createSysIdFullSweepCommand();
        case TEST_SUBSYSTEM_FEEDER:
            return feederCommandFactory.createSysIdFullSweepCommand();
        case TEST_SUBSYSTEM_INTAKE:
            return intakeCommandFactory.createSysIdFullSweepCommand();
        case TEST_SUBSYSTEM_TURRET:
            return turretCommandFactory.createSysIdFullSweepCommand();
        case TEST_SUBSYSTEM_HARVESTER:
            return harvesterCommandFactory.createSysIdFullSweepCommand();
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

    /**
     * Wires operator controller buttons to gameplay state transition commands.
     * <p>
     * Y enters FIRE_READY, X enters HARVEST_READY, start enters CLIMB_READY, A enters EJECT, B returns to IDLE, right trigger enters TRAVEL, and left
     * trigger enters TRENCH_TRAVEL. Triggers use {@code onTrue} so the state change persists after release.
     * </p>
     */
    private void configureOperatorBindings() {
        // Y button: fire fuel.
        operatorController.y().whileTrue(
                gameplayStateCommandFactory.createFireReadyCommand());

        // X button: harvest fuel.
        operatorController.x().onTrue(
                gameplayStateCommandFactory.createHarvestReadyCommand());

        // A button: eject all fuel.
        operatorController.a().whileTrue(
                gameplayStateCommandFactory.createEjectCommand());

        // B button: return to idle.
        operatorController.b().onTrue(
                gameplayStateCommandFactory.createIdleCommand());

        // Right trigger: travel mode (stow harvester and idle all mechanisms).
        operatorController.rightTrigger().onTrue(
                gameplayStateCommandFactory.createTravelCommand());

        // Left trigger: trench travel mode (deploy harvester for low-profile field traversal).
        operatorController.leftTrigger().onTrue(
                gameplayStateCommandFactory.createTrenchTravelCommand());

        // D-pad up: boost shooter RPM by the configured adjustment amount while held.
        operatorController.povUp().whileTrue(
                shooterCommandFactory.createBoostRpmCommand());

        // D-pad down: cut shooter RPM by the configured adjustment amount while held.
        operatorController.povDown().whileTrue(
                shooterCommandFactory.createCutRpmCommand());

        // Select (back) button: toggle turret lock to 0 degrees.
        // First press locks the turret at 0 and disables field tracking.
        // Second press releases the lock and resumes the default tracking command.
        operatorController.back().toggleOnTrue(
                turretCommandFactory.createLockToZeroCommand());
    }

    /**
     * Wires driver controller d-pad buttons to PathPlanner pathfinding commands.
     * <p>
     * Each d-pad direction pathfinds to a configurable field pose when held. Targets are stored in blue-alliance coordinates and flipped at runtime
     * for red alliance. Only directions with {@code enabled = true} in the config are bound. The target pose and constraints are resolved at
     * button-press time using deferred proxy so tunable values and alliance selection are always current.
     * </p>
     */
    private void configureDpadPathfindingBindings() {
        DriverControllerConfig driverConfig = triggerBindingsConfig.driverControllerConfig;

        bindDpadDirection(driverController.povUp(), driverConfig.dpadUp, driverConfig);
        bindDpadDirection(driverController.povDown(), driverConfig.dpadDown, driverConfig);
        bindDpadDirection(driverController.povLeft(), driverConfig.dpadLeft, driverConfig);
        bindDpadDirection(driverController.povRight(), driverConfig.dpadRight, driverConfig);
    }

    /**
     * Binds a single d-pad direction trigger to a pathfinding command if the target is enabled.
     *
     * @param trigger      the d-pad direction trigger from the driver controller
     * @param targetConfig config holding the target pose for this direction
     * @param driverConfig shared driver controller config holding pathfinding constraints
     */
    private void bindDpadDirection(
            edu.wpi.first.wpilibj2.command.button.Trigger trigger,
            DpadTargetConfig targetConfig,
            DriverControllerConfig driverConfig) {
        if (!targetConfig.enabled) {
            return;
        }

        trigger.whileTrue(Commands.deferredProxy(() -> {
            Pose2d          bluePose    = targetConfig.toPose2d();

            // Flip the target for the red alliance.
            Alliance        alliance    = RobotEnvironment.getAlliance().orElse(Alliance.Blue);
            Pose2d          targetPose  = alliance == Alliance.Red ? FlippingUtil.flipFieldPose(bluePose) : bluePose;

            PathConstraints constraints = new PathConstraints(
                    driverConfig.getDpadMaxVelocityMetersPerSecond(),
                    driverConfig.getDpadMaxAccelerationMetersPerSecondSquared(),
                    Units.degreesToRadians(driverConfig.getDpadMaxAngularVelocityDegreesPerSecond()),
                    Units.degreesToRadians(driverConfig.getDpadMaxAngularAccelerationDegreesPerSecondSquared()));

            return driveBaseCommandFactory.createPathfindToPoseCommand(targetPose, constraints);
        }));
    }

}
