package frc.robot.subsystems.gameplaystate.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.GameplayState;
import frc.robot.subsystems.gameplaystate.GameplayStateSubsystem;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;

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

    private final HarvesterSubsystemCommandFactory harvesterCommandFactory;

    /**
     * Supplies the distance from the robot to the active scoring target in meters.
     */
    private final Supplier<Double>                 distanceToTargetMetersSupplier;

    /**
     * Supplies whether the turret has settled on its target and is ready to fire.
     */
    private final Supplier<Boolean>                turretReadySupplier;

    /**
     * Creates a factory that composes gameplay state commands from individual subsystem command factories.
     *
     * @param subsystem                      gameplay state subsystem that tracks the current operating mode
     * @param shooterCommandFactory          factory for shooter flywheel commands
     * @param indexerCommandFactory          factory for indexer roller commands
     * @param feederCommandFactory           factory for feeder belt commands
     * @param intakeCommandFactory           factory for intake roller commands
     * @param harvesterCommandFactory        factory for harvester arm commands
     * @param distanceToTargetMetersSupplier supplier returning the distance from the robot to the active scoring target in meters
     * @param turretReadySupplier            supplier returning true when the turret has settled on its target
     */
    public GameplayStateCommandFactory(
            GameplayStateSubsystem subsystem,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            Supplier<Double> distanceToTargetMetersSupplier,
            Supplier<Boolean> turretReadySupplier) {
        super(subsystem);
        this.shooterCommandFactory          = shooterCommandFactory;
        this.indexerCommandFactory          = indexerCommandFactory;
        this.feederCommandFactory           = feederCommandFactory;
        this.intakeCommandFactory           = intakeCommandFactory;
        this.harvesterCommandFactory        = harvesterCommandFactory;
        this.distanceToTargetMetersSupplier = distanceToTargetMetersSupplier;
        this.turretReadySupplier            = turretReadySupplier;
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
        // Publish the IDLE state, then return all ball-path mechanisms to rest in parallel.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.IDLE, "command"))
                .andThen(Commands.parallel(
                        // Spin the shooter down to its configured idle RPM.
                        shooterCommandFactory.createIdleCommand(),
                        // Stop the indexer and feeder rollers.
                        indexerCommandFactory.createStopCommand(),
                        feederCommandFactory.createStopCommand()))
                .withName("GameplayState-Idle");
    }

    /**
     * Builds the HARVEST_READY state command group: deploys the harvester arm and spins the intake and feeder to pull Fuel into the robot while the
     * shooter idles.
     *
     * @return parallel command group that prepares the robot for Fuel collection
     */
    public Command createHarvestReadyCommand() {
        // Publish the HARVEST_READY state, deploy the arm, then start the intake.
        // Sequential (not parallel) so the arm clears the frame before rollers spin.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.HARVEST_READY, "command"))
                // Lower the harvester arm outside the frame perimeter.
                .andThen(harvesterCommandFactory.createDeployCommand())
                // Spin the intake and feeder to pull Fuel into the hopper.
                .andThen(intakeCommandFactory.createIntakeAndHoldCommand())
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
        // Publish the FIRE_READY state, then run four parallel branches:
        //   1. Spin the shooter to a distance-based RPM.
        //   2. Gate the indexer — it waits for shooter + turret ready, then feeds.
        //   3. Gate the feeder — same readiness check, reverse-pulse then forward.
        //   4. After a configurable delay, sweep the harvester arm to push
        //      remaining Fuel from the back of the hopper toward the shooter.
        //
        // The sweep branch uses .asProxy() so the harvester subsystem is NOT
        // locked for the duration of fire ready. During the wait period the
        // harvester's default command keeps running normally. If the operator
        // releases the fire trigger before the delay elapses, the sweep never
        // starts and finallyDo() transitions cleanly back to IDLE.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.FIRE_READY, "command"))
                .andThen(Commands.parallel(
                        // Spin the shooter flywheel based on distance to the scoring target.
                        shooterCommandFactory.createDistanceBasedSpinCommand(distanceToTargetMetersSupplier),
                        // Feed Fuel forward once the shooter and turret are both ready.
                        indexerCommandFactory.createFireWhenReadyCommand(
                                shooterCommandFactory.getSubsystem()::isAtShootingVelocity,
                                turretReadySupplier),
                        feederCommandFactory.createFireWhenReadyCommand(
                                shooterCommandFactory.getSubsystem()::isAtShootingVelocity,
                                turretReadySupplier),
                        // Delayed sweep: wait, then raise the harvester arm to push hopper Fuel forward.
                        Commands.waitSeconds(subsystem.getConfig().getFireReadySweepDelaySeconds())
                                .andThen(harvesterCommandFactory.createSweepCommand().asProxy())
                                .andThen(harvesterCommandFactory.createDeployCommand().asProxy())))
                // Cleanup: transition back to IDLE regardless of how fire ready ends.
                .finallyDo(() -> {
                    subsystem.requestState(GameplayState.IDLE, "fire-end");
                    CommandScheduler.getInstance().schedule(createIdleCommand().asProxy());
                })
                .withName("GameplayState-FireReady");
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
        // Publish the EJECT state, then reverse all ball-path mechanisms in parallel
        // to push Fuel out of the robot. The harvester deploys so ejected Fuel exits cleanly.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.EJECT, "command"))
                .andThen(Commands.parallel(
                        // Stop the shooter so Fuel is not launched.
                        shooterCommandFactory.createStopCommand(),
                        // Reverse the indexer, feeder, and intake to expel Fuel.
                        indexerCommandFactory.createReverseCommand(),
                        feederCommandFactory.createReverseCommand(),
                        intakeCommandFactory.createEjectCommand(),
                        // Deploy the harvester arm so Fuel can exit the frame.
                        harvesterCommandFactory.createDeployCommand()))
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
        // Publish the TRAVEL state, then stow the harvester and stop the intake
        // so the robot can traverse the field safely within the frame perimeter.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.TRAVEL, "command"))
                .andThen(Commands.parallel(
                        // Raise the harvester arm inside the frame perimeter.
                        harvesterCommandFactory.createStowCommand(),
                        // shooterCommandFactory.createIdleCommand(),
                        // indexerCommandFactory.createStopCommand(),
                        // feederCommandFactory.createStopCommand(),
                        // Stop the intake rollers.
                        intakeCommandFactory.createStopCommand()))
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
        // Publish the TRENCH_TRAVEL state, then deploy the harvester for a low
        // profile and stop the intake. Identical to TRAVEL except the arm is
        // lowered so the robot clears trench-height obstacles.
        return Commands.runOnce(() -> subsystem.requestState(GameplayState.TRENCH_TRAVEL, "command"))
                .andThen(Commands.parallel(
                        // Lower the harvester arm for trench clearance.
                        harvesterCommandFactory.createDeployCommand(),
                        // shooterCommandFactory.createIdleCommand(),
                        // indexerCommandFactory.createStopCommand(),
                        // feederCommandFactory.createStopCommand(),
                        // Stop the intake rollers.
                        intakeCommandFactory.createStopCommand()))
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
}
