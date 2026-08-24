package frc.robot.shared.bindings;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.commands.GameplayStateCommandFactory;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.robotpose.commands.RobotPoseSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Tuning binding strategy that wires driver and operator controllers to subsystem test and SysId characterization commands. A dashboard chooser
 * selects which subsystem the driver buttons control, and the operator controller provides per-module drive base characterization.
 * <p>
 * Default commands are not registered in this mode so mechanisms stay wherever the test commands leave them.
 * </p>
 */
public class TuningTriggerBindings extends AbstractTriggerBindings {

    // Chooser option constants for subsystem test selection.
    private static final String            TEST_SUBSYSTEM_SHOOTER   = "Shooter";

    private static final String            TEST_SUBSYSTEM_INDEXER   = "Indexer";

    private static final String            TEST_SUBSYSTEM_FEEDER    = "Feeder";

    private static final String            TEST_SUBSYSTEM_INTAKE    = "Intake";

    private static final String            TEST_SUBSYSTEM_TURRET    = "Turret";

    private static final String            TEST_SUBSYSTEM_HARVESTER = "Harvester";

    /**
     * Dashboard chooser that selects which subsystem the A/B/X test buttons control.
     */
    private LoggedDashboardChooser<String> testSubsystemChooser;

    /**
     * Creates tuning trigger bindings with the default controller ports.
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
     * @param robotPoseCommandFactory     factory for creating robot pose commands such as vision-based pose resets
     */
    public TuningTriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            GameplayStateCommandFactory gameplayStateCommandFactory,
            RobotPoseSubsystemCommandFactory robotPoseCommandFactory) {
        super(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                indexerCommandFactory,
                feederCommandFactory,
                intakeCommandFactory,
                harvesterCommandFactory,
                gameplayStateCommandFactory,
                robotPoseCommandFactory);
    }

    /**
     * Wires driver controller buttons to test the dashboard-selected subsystem and run drive base characterization.
     * <p>
     * A = reverse/min, B = forward/max, X = full SysId sweep for the chooser-selected subsystem, Y = drive base angle then drive SysId in sequence.
     * The subsystem under test is chosen via the {@code TriggerBindings/TestSubsystem} SendableChooser on the dashboard. Commands are resolved at
     * button-press time using deferred proxy so the chooser selection is always current.
     * </p>
     */
    @Override
    protected void assignDriverControls() {
        configureSubsystemTestChooser();
        configureSubsystemTestButtons();
        configureDriveBaseSysIdButton();
    }

    /**
     * Wires operator controller buttons to per-module SysId characterization commands for the drive base.
     * <p>
     * Drive motors: A = front-left, B = front-right, X = back-left, Y = back-right. Angle motors: LB = front-left, RB = front-right, LT = back-left,
     * RT = back-right.
     * </p>
     */
    @Override
    protected void assignOperatorControls() {
        configurePerModuleDriveSysId();
        configurePerModuleAngleSysId();
    }

    /**
     * Builds the subsystem test chooser and publishes it to SmartDashboard.
     */
    private void configureSubsystemTestChooser() {
        testSubsystemChooser = new LoggedDashboardChooser<>("TriggerBindings/TestSubsystem");
        testSubsystemChooser.addDefaultOption(TEST_SUBSYSTEM_SHOOTER, TEST_SUBSYSTEM_SHOOTER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INDEXER, TEST_SUBSYSTEM_INDEXER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_FEEDER, TEST_SUBSYSTEM_FEEDER);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_INTAKE, TEST_SUBSYSTEM_INTAKE);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_TURRET, TEST_SUBSYSTEM_TURRET);
        testSubsystemChooser.addOption(TEST_SUBSYSTEM_HARVESTER, TEST_SUBSYSTEM_HARVESTER);
    }

    /**
     * Maps driver A/B/X buttons to deferred subsystem test commands based on the dashboard chooser selection.
     */
    private void configureSubsystemTestButtons() {
        // A button: reverse (velocity) or move to minimum setpoint (set-and-seek).
        debounce(driverController.a()).whileTrue(
                Commands.deferredProxy(this::createSelectedReverseCommand));

        // B button: forward (velocity) or move to maximum setpoint (set-and-seek).
        debounce(driverController.b()).whileTrue(
                Commands.deferredProxy(this::createSelectedForwardCommand));

        // X button: run full SysId sweep for the dashboard-selected subsystem.
        // Press X to start; press X again to cancel early.
        debounce(driverController.x()).onTrue(
                Commands.deferredProxy(this::createSelectedSysIdCommand));
    }

    /**
     * Maps driver Y button to the drive base angle and drive SysId characterization sequence.
     */
    private void configureDriveBaseSysIdButton() {
        // Y button: run drive base angle and drive SysId characterization in sequence.
        // Press Y to start; press Y again to cancel early.
        debounce(driverController.y()).onTrue(
                driveBaseCommandFactory.createAngleSysIdCommand()
                        .andThen(driveBaseCommandFactory.createDriveSysIdCommand()));
    }

    /**
     * Maps operator A/B/X/Y buttons to per-module drive motor SysId commands.
     */
    private void configurePerModuleDriveSysId() {
        // Drive motors: A = front-left, B = front-right, X = back-left, Y = back-right.
        debounce(operatorController.a()).whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_LEFT));
        debounce(operatorController.b()).whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_RIGHT));
        debounce(operatorController.x()).whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_LEFT));
        debounce(operatorController.y()).whileTrue(
                driveBaseCommandFactory.createDriveSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_BACK_RIGHT));
    }

    /**
     * Maps operator bumpers and triggers to per-module angle motor SysId commands.
     */
    private void configurePerModuleAngleSysId() {
        // Angle motors: LB = front-left, RB = front-right, LT = back-left, RT = back-right.
        debounce(operatorController.leftBumper()).whileTrue(
                driveBaseCommandFactory.createAngleSysIdCommandForModule(DriveBaseSubsystemCommandFactory.MODULE_FRONT_LEFT));
        debounce(operatorController.rightBumper()).whileTrue(
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
}
