package frc.robot.subsystems.indexer.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Command that spins the indexer roller forward to feed Fuel into the shooter.
 * <p>
 * The target RPM supplier is evaluated once on initialize. The command finishes when the roller reaches the target velocity and has been stable for
 * the configured settle time, indicating the piece has been launched into the shooter flywheels.
 * </p>
 */
public class FeedCommand extends AbstractVelocityCommand<IndexerSubsystem> {

    /**
     * Creates a feed command that reads its target RPM from a supplier.
     *
     * @param subsystem         indexer subsystem to control
     * @param targetRpmSupplier provider for the target feed RPM; evaluated on initialize
     */
    public FeedCommand(IndexerSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates a feed command that drives the roller to a fixed RPM.
     *
     * @param subsystem indexer subsystem to control
     * @param targetRpm fixed target feed RPM (positive for forward into shooter)
     */
    public FeedCommand(IndexerSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
