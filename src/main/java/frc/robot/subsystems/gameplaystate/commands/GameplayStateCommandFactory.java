package frc.robot.subsystems.gameplaystate.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.GameplayState;
import frc.robot.subsystems.gameplaystate.GameplayStateSubsystem;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
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

    /** Supplies the distance from the robot to the active scoring target in meters. */
    private final Supplier<Double>                 distanceToTargetMetersSupplier;

    /**
     * Creates a factory that composes gameplay state commands from individual subsystem command factories.
     *
     * @param subsystem                      gameplay state subsystem that tracks the current operating mode
     * @param shooterCommandFactory          factory for shooter flywheel commands
     * @param indexerCommandFactory          factory for indexer roller commands
     * @param feederCommandFactory           factory for feeder belt commands
     * @param intakeCommandFactory           factory for intake roller commands
     * @param turretCommandFactory           factory for turret aiming commands
     * @param harvesterCommandFactory        factory for harvester arm commands
     * @param climberCommandFactory          factory for climber commands
     * @param distanceToTargetMetersSupplier supplier returning the distance from the robot to the active scoring target in meters
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
            Supplier<Double> distanceToTargetMetersSupplier) {
        super(subsystem);
        this.shooterCommandFactory          = shooterCommandFactory;
        this.indexerCommandFactory          = indexerCommandFactory;
        this.feederCommandFactory           = feederCommandFactory;
        this.intakeCommandFactory           = intakeCommandFactory;
        this.turretCommandFactory           = turretCommandFactory;
        this.harvesterCommandFactory        = harvesterCommandFactory;
        this.climberCommandFactory          = climberCommandFactory;
        this.distanceToTargetMetersSupplier = distanceToTargetMetersSupplier;
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
     * The shooter idles at its configured idle RPM, the indexer and feeder stop completely, the intake stops, and the harvester stows.
     * </p>
     *
     * @return parallel command group that idles all mechanisms
     */
    public Command createIdleCommand() {
        return Commands.parallel(
                shooterCommandFactory.createIdleCommand(),
                indexerCommandFactory.createStopCommand(),
                feederCommandFactory.createStopCommand(),
                intakeCommandFactory.createIdleCommand())
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
                intakeCommandFactory.createIntakeAndHoldCommand())
                .withName("GameplayState-HarvestReady");
    }

    /**
     * Builds the FIRE_READY state command group: spins up the shooter and stages Fuel in the indexer while the turret continues tracking via its
     * default command.
     * <p>
     * Both the indexer and feeder wait until the shooter and turret report ready before activating. The indexer gates on readiness via
     * {@code createFireWhenReadyCommand}, and the feeder similarly waits before running its reverse-pulse-then-forward sequence. When the command
     * ends or is interrupted (e.g., operator releases the fire trigger), all mechanisms transition back to IDLE so the indexer and feeder stop
     * cleanly. The turret is not explicitly commanded here because it runs its default tracking command at all times.
     * </p>
     *
     * @return parallel command group that prepares the robot to score
     */
    public Command createFireReadyCommand() {
        return Commands.parallel(
                shooterCommandFactory.createDistanceBasedSpinCommand(distanceToTargetMetersSupplier),
                indexerCommandFactory.createFireWhenReadyCommand(
                        shooterCommandFactory.getSubsystem()::isAtShootingVelocity,
                        () -> true
                // turretCommandFactory.getSubsystem()::isProfileSettled
                ),
                feederCommandFactory.createFireWhenReadyCommand(
                        shooterCommandFactory.getSubsystem()::isAtShootingVelocity,
                        () -> true
                // turretCommandFactory.getSubsystem()::isProfileSettled
                ))
                .finallyDo(() -> {
                    subsystem.requestState(GameplayState.IDLE, "fire-end");
                    CommandScheduler.getInstance().schedule(createIdleCommand());
                })
                .withName("GameplayState-FireReady");
    }

    /**
     * Builds the AUTO_CYCLE state command group used during autonomous routines.
     * <p>
     * Deploys the harvester and runs the intake to collect Fuel while the shooter spins up. The turret continues tracking the scoring target via its
     * default command, so it is not explicitly commanded here. This allows autonomous routines to collect and score without explicit state
     * transitions between harvest and fire.
     * </p>
     *
     * @return parallel command group that simultaneously collects and prepares to score
     */
    public Command createAutoCycleCommand() {
        return Commands.parallel(
                harvesterCommandFactory.createDeployCommand(),
                intakeCommandFactory.createIntakeAndHoldCommand(),
                feederCommandFactory.createReversePulseThenForwardCommand(),
                shooterCommandFactory.createDistanceBasedSpinCommand(distanceToTargetMetersSupplier),
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
     * Builds the TRAVEL state command group: stows the harvester and idles all other mechanisms for safe field traversal.
     * <p>
     * This state is useful when driving across the field without actively collecting or scoring. The harvester moves to its stowed position so it
     * stays inside the frame perimeter, while the shooter, indexer, feeder, and intake return to their idle behaviors.
     * </p>
     *
     * @return parallel command group that stows the harvester and idles all other mechanisms
     */
    public Command createTravelCommand() {
        return Commands.parallel(
                harvesterCommandFactory.createStowCommand(),
                shooterCommandFactory.createIdleCommand(),
                indexerCommandFactory.createStopCommand(),
                feederCommandFactory.createStopCommand(),
                intakeCommandFactory.createIdleCommand())
                .withName("GameplayState-Travel");
    }

    /**
     * Builds the TRENCH_TRAVEL state command group: deploys the harvester for low-profile clearance under field obstacles while idling all other
     * mechanisms.
     * <p>
     * This state is identical to TRAVEL except the harvester is deployed instead of stowed, allowing the robot to pass under structures such as the
     * trench run.
     * </p>
     *
     * @return parallel command group that deploys the harvester and idles all other mechanisms
     */
    public Command createTrenchTravelCommand() {
        return Commands.parallel(
                harvesterCommandFactory.createDeployCommand(),
                shooterCommandFactory.createIdleCommand(),
                indexerCommandFactory.createStopCommand(),
                feederCommandFactory.createStopCommand(),
                intakeCommandFactory.createIdleCommand())
                .withName("GameplayState-TrenchTravel");
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
        case TRAVEL:
            return createTravelCommand();
        case TRENCH_TRAVEL:
            return createTrenchTravelCommand();
        default:
            return createIdleCommand();
        }
    }
}
