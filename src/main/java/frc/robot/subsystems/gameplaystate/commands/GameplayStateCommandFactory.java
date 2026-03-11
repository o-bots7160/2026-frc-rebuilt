package frc.robot.subsystems.gameplaystate.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.GameplayState;
import frc.robot.subsystems.gameplaystate.GameplayStateSubsystem;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Composes parallel command groups that coordinate multiple subsystems for each {@link GameplayState}.
 * <p>
 * Each factory method produces a command group that sets the appropriate gameplay state on the {@link GameplayStateSubsystem} and then schedules
 * mechanism commands in parallel so the robot acts as a unified system. The factory delegates to individual subsystem command factories rather than
 * driving motors directly, keeping subsystem ownership intact.
 * </p>
 */
public class GameplayStateCommandFactory extends AbstractSubsystemCommandFactory<GameplayStateSubsystem> {

    private final ShooterSubsystemCommandFactory   shooterCommandFactory;

    private final IndexerSubsystemCommandFactory   indexerCommandFactory;

    private final FeederSubsystemCommandFactory    feederCommandFactory;

    private final IntakeSubsystemCommandFactory    intakeCommandFactory;

    private final TurretSubsystemCommandFactory    turretCommandFactory;

    private final HarvesterSubsystemCommandFactory harvesterCommandFactory;

    private final ClimberSubsystemCommandFactory   climberCommandFactory;

    private final RobotPoseSubsystem               robotPoseSubsystem;

    private final Supplier<Translation2d>          fieldTargetPositionSupplier;

    private final Supplier<Double>                 robotYawRateRadiansPerSecondSupplier;

    /**
     * Creates a factory that composes gameplay state commands from individual subsystem command factories.
     *
     * @param subsystem                            gameplay state subsystem that tracks the current operating mode
     * @param shooterCommandFactory                factory for shooter flywheel commands
     * @param indexerCommandFactory                factory for indexer roller commands
     * @param feederCommandFactory                 factory for feeder belt commands
     * @param intakeCommandFactory                 factory for intake roller commands
     * @param turretCommandFactory                 factory for turret aiming commands
     * @param harvesterCommandFactory              factory for harvester arm commands
     * @param climberCommandFactory                factory for climber commands
     * @param robotPoseSubsystem                   robot pose subsystem for turret tracking
     * @param fieldTargetPositionSupplier          supplier of the active scoring target position in field coordinates (meters)
     * @param robotYawRateRadiansPerSecondSupplier supplier of the robot's yaw rate in radians per second
     */
    public GameplayStateCommandFactory(
            GameplayStateSubsystem subsystem,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            TurretSubsystemCommandFactory turretCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            ClimberSubsystemCommandFactory climberCommandFactory,
            RobotPoseSubsystem robotPoseSubsystem,
            Supplier<Translation2d> fieldTargetPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier) {
        super(subsystem);
        this.shooterCommandFactory                = shooterCommandFactory;
        this.indexerCommandFactory                = indexerCommandFactory;
        this.feederCommandFactory                 = feederCommandFactory;
        this.intakeCommandFactory                 = intakeCommandFactory;
        this.turretCommandFactory                 = turretCommandFactory;
        this.harvesterCommandFactory              = harvesterCommandFactory;
        this.climberCommandFactory                = climberCommandFactory;
        this.robotPoseSubsystem                   = robotPoseSubsystem;
        this.fieldTargetPositionSupplier          = fieldTargetPositionSupplier;
        this.robotYawRateRadiansPerSecondSupplier = robotYawRateRadiansPerSecondSupplier;
    }

    /**
     * Builds a command that transitions the robot to the specified gameplay state.
     * <p>
     * This is the central dispatch method. It records the state on the {@link GameplayStateSubsystem} and returns the appropriate parallel command
     * group for the requested state.
     * </p>
     *
     * @param state  desired gameplay state
     * @param source description of what triggered the transition (e.g., "operator", "auto", "fms")
     * @return command group that coordinates all relevant subsystems for the requested state
     */
    public Command createTransitionCommand(GameplayState state, String source) {
        return Commands.runOnce(() -> subsystem.requestState(state, source))
                .andThen(createCommandGroupForState(state))
                .withName("Transition-" + state.getDisplayName());
    }

    /**
     * Builds the IDLE state command group: all subsystems return to their resting positions.
     * <p>
     * The shooter idles at its configured idle RPM, the indexer and feeder idle, the intake stops, and the harvester stows.
     * </p>
     *
     * @return parallel command group that idles all mechanisms
     */
    public Command createIdleCommand() {
        return Commands.parallel(
                shooterCommandFactory.createIdleCommand(),
                indexerCommandFactory.createIdleCommand(),
                feederCommandFactory.createIdleCommand(),
                intakeCommandFactory.createIdleCommand(),
                harvesterCommandFactory.createStowCommand())
                .withName("GameplayState-Idle");
    }

    /**
     * Builds the HARVEST_READY state command group: deploys the harvester arm and spins the intake and feeder to pull Fuel into the robot while the
     * shooter idles.
     *
     * @return parallel command group that prepares the robot for Fuel collection
     */
    public Command createHarvestReadyCommand() {
        return Commands.parallel(
                harvesterCommandFactory.createDeployCommand(),
                intakeCommandFactory.createIntakeAndHoldCommand()// ,
        // feederCommandFactory.createForwardAndHoldCommand(),
        // indexerCommandFactory.createIdleCommand(),
        // shooterCommandFactory.createIdleCommand()
        )
                .withName("GameplayState-HarvestReady");
    }

    /**
     * Builds the FIRE_READY state command group: spins up the shooter, aims the turret at the field target, and stages Fuel in the indexer while the
     * harvester stows.
     * <p>
     * The indexer holds until the shooter and turret report ready, then feeds automatically via {@code createFireWhenReadyCommand}.
     * </p>
     *
     * @return parallel command group that prepares the robot to score
     */
    public Command createFireReadyCommand() {
        return Commands.parallel(
                shooterCommandFactory.createContinuousVelocityCommand(
                        shooterCommandFactory.getSubsystem().getConfig()::getMaximumShootingRpm),
                turretCommandFactory.createTrackFieldTargetCommand(
                        robotPoseSubsystem,
                        fieldTargetPositionSupplier,
                        robotYawRateRadiansPerSecondSupplier),
                indexerCommandFactory.createFireWhenReadyCommand(
                        shooterCommandFactory.getSubsystem()::isAtTargetVelocity,
                        turretCommandFactory.getSubsystem()::isProfileSettled),
                feederCommandFactory.createIdleCommand()
        // intakeCommandFactory.createIdleCommand(),
        // harvesterCommandFactory.createStowCommand()
        )
                .withName("GameplayState-FireReady");
    }

    /**
     * Builds the AUTO_CYCLE state command group used during autonomous routines.
     * <p>
     * Deploys the harvester and runs the intake to collect Fuel while the shooter spins up and the turret tracks the scoring target. This allows
     * autonomous routines to collect and score without explicit state transitions between harvest and fire.
     * </p>
     *
     * @return parallel command group that simultaneously collects and prepares to score
     */
    public Command createAutoCycleCommand() {
        return Commands.parallel(
                harvesterCommandFactory.createDeployCommand(),
                intakeCommandFactory.createIntakeAndHoldCommand(),
                feederCommandFactory.createForwardAndHoldCommand(),
                shooterCommandFactory.createSpinUpAndHoldCommand(
                        shooterCommandFactory.getSubsystem().getConfig()::getMaximumShootingRpm),
                turretCommandFactory.createTrackFieldTargetCommand(
                        robotPoseSubsystem,
                        fieldTargetPositionSupplier,
                        robotYawRateRadiansPerSecondSupplier),
                indexerCommandFactory.createHoldCommand())
                .withName("GameplayState-AutoCycle");
    }

    /**
     * Builds the CLIMB_READY state command group: stops ball-path mechanisms and prepares the climber.
     * <p>
     * The shooter, indexer, feeder, and intake all idle while the harvester stows to clear the climber's path. The climber is commanded to its deploy
     * position.
     * </p>
     *
     * @return parallel command group that prepares the robot for climbing
     */
    public Command createClimbReadyCommand() {
        return Commands.parallel(
                shooterCommandFactory.createStopCommand(),
                indexerCommandFactory.createIdleCommand(),
                feederCommandFactory.createIdleCommand(),
                intakeCommandFactory.createIdleCommand(),
                harvesterCommandFactory.createStowCommand(),
                climberCommandFactory.createMoveToPositionCommand(
                        climberCommandFactory.getSubsystem().getConfig()::getMaximumSetpointDegrees))
                .withName("GameplayState-ClimbReady");
    }

    /**
     * Builds the EJECT state command group: reverses all ball-path mechanisms to expel Fuel.
     * <p>
     * The shooter stops, and the indexer, feeder, and intake all run in reverse to push Fuel out of the robot. The harvester deploys so ejected Fuel
     * can exit cleanly.
     * </p>
     *
     * @return parallel command group that ejects all Fuel from the robot
     */
    public Command createEjectCommand() {
        return Commands.parallel(
                shooterCommandFactory.createStopCommand(),
                indexerCommandFactory.createReverseCommand(),
                feederCommandFactory.createReverseCommand(),
                intakeCommandFactory.createEjectCommand(),
                harvesterCommandFactory.createDeployCommand())
                .withName("GameplayState-Eject");
    }

    /**
     * Sets the idle command as the default behavior for the gameplay state subsystem.
     *
     * @return the idle command that was set as the default
     */
    public Command setDefaultIdleCommand() {
        Command command = createIdleCommand();
        subsystem.setDefaultCommand(command);
        return command;
    }

    private Command createCommandGroupForState(GameplayState state) {
        switch (state) {
        case IDLE:
            return createIdleCommand();
        case HARVEST_READY:
            return createHarvestReadyCommand();
        case FIRE_READY:
            return createFireReadyCommand();
        case AUTO_CYCLE:
            return createAutoCycleCommand();
        case CLIMB_READY:
            return createClimbReadyCommand();
        case EJECT:
            return createEjectCommand();
        default:
            return createIdleCommand();
        }
    }
}
