package frc.robot.subsystems.indexer.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractVelocityCommandFactory;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Factory that creates indexer commands and wires default behaviors.
 */
public class IndexerSubsystemCommandFactory extends AbstractVelocityCommandFactory<IndexerSubsystem> {

    /**
     * Creates a factory for commands that share the given indexer subsystem instance.
     *
     * @param subsystem indexer subsystem instance that commands created by this factory will control
     */
    public IndexerSubsystemCommandFactory(IndexerSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds an idle command that holds the roller at the configured idle RPM (typically 0).
     *
     * @return command that idles the indexer roller
     */
    @Override
    public IdleIndexerCommand createIdleCommand() {
        return new IdleIndexerCommand(subsystem);
    }

    /**
     * Builds a feed command that reads its target RPM from a supplier.
     * <p>
     * Use this when the feed speed depends on runtime conditions evaluated at command start.
     * </p>
     *
     * @param targetRpmSupplier provider for the target feed RPM; evaluated on initialize
     * @return command that feeds Fuel into the shooter at the supplied RPM
     */
    public FeedCommand createFeedCommand(Supplier<Double> targetRpmSupplier) {
        return new FeedCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds a feed command that drives the roller at a fixed RPM.
     *
     * @param targetRpm fixed target feed RPM (positive for forward into shooter)
     * @return command that feeds Fuel into the shooter at the fixed RPM
     */
    public FeedCommand createFeedCommand(double targetRpm) {
        return new FeedCommand(subsystem, targetRpm);
    }

    /**
     * Builds a feed command using the configured default feed velocity.
     *
     * @return command that feeds Fuel into the shooter at the default RPM
     */
    public FeedCommand createFeedCommand() {
        return new FeedCommand(subsystem, subsystem.getConfig().getFeedVelocityRpm());
    }

    /**
     * Builds a hold command that keeps the roller stopped so Fuel stays staged.
     *
     * @return command that holds the indexer at zero RPM until interrupted
     */
    public HoldCommand createHoldCommand() {
        return new HoldCommand(subsystem);
    }

    /**
     * Builds a reverse command that reads its target RPM from a supplier.
     *
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     * @return command that spins the roller in reverse at the supplied RPM
     */
    public ReverseIndexerCommand createReverseCommand(Supplier<Double> targetRpmSupplier) {
        return new ReverseIndexerCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds a reverse command using the configured default reverse velocity.
     * <p>
     * The configured reverse RPM is negated so the roller spins backward toward the hopper.
     * </p>
     *
     * @return command that spins the roller in reverse at the default RPM
     */
    public ReverseIndexerCommand createReverseCommand() {
        return new ReverseIndexerCommand(subsystem, -subsystem.getConfig().getReverseVelocityRpm());
    }

    /**
     * Builds an unjam command that alternates forward and reverse pulses to dislodge stuck Fuel.
     *
     * @return command that pulses the roller back and forth until interrupted
     */
    public UnjamCommand createUnjamCommand() {
        return new UnjamCommand(subsystem, subsystem.getConfig());
    }

    /**
     * Builds a command that feeds Fuel forward at the configured feed velocity and holds that speed indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the roller feeds while a button is held and returns to idle when released.
     * </p>
     *
     * @return command that feeds and holds velocity until interrupted
     */
    public Command createFeedAndHoldCommand() {
        return createFeedCommand()
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a command that continuously reads a target RPM from a supplier and seeks that velocity every cycle.
     * <p>
     * Unlike {@link #createFeedCommand(Supplier)}, which locks the target at command start, this command re-evaluates the supplier each cycle. Use
     * this when the target is backed by a tunable value that operators may adjust while the command is running.
     * </p>
     *
     * @param targetRpmSupplier provider for the target feed RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted
     */
    public Command createContinuousFeedCommand(Supplier<Double> targetRpmSupplier) {
        return createContinuousVelocityCommand(targetRpmSupplier);
    }

    /**
     * Builds a composite command that waits until the shooter and turret are both ready, then feeds Fuel into the shooter.
     * <p>
     * The command holds the indexer stopped while either readiness supplier returns false. Once both report true, it feeds forward at the configured
     * feed velocity. This keeps the indexer decoupled from the shooter and turret subsystems — readiness is checked through suppliers wired in
     * {@code RobotContainer}.
     * </p>
     *
     * @param shooterReadySupplier   supplier that returns true when the shooter flywheel is at the target RPM
     * @param turretOnTargetSupplier supplier that returns true when the turret is aimed at the scoring target
     * @return composite command that waits for readiness then feeds
     */
    public Command createFireWhenReadyCommand(Supplier<Boolean> shooterReadySupplier, Supplier<Boolean> turretOnTargetSupplier) {
        return Commands.waitUntil(() -> shooterReadySupplier.get() && turretOnTargetSupplier.get())
                .andThen(createFeedAndHoldCommand())
                .withName("FireWhenReady");
    }
}
