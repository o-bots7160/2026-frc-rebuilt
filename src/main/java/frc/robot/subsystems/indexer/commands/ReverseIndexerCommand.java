package frc.robot.subsystems.indexer.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Command that spins the indexer roller in reverse to back Fuel toward the feeder.
 * <p>
 * The target RPM supplier is evaluated once on initialize. Pass a negative RPM to reverse the roller direction. The command finishes when the roller
 * reaches the target velocity and has been stable for the configured settle time.
 * </p>
 */
public class ReverseIndexerCommand extends AbstractVelocityCommand<IndexerSubsystem> {

    /**
     * Creates a reverse command that reads its target RPM from a supplier.
     *
     * @param subsystem         indexer subsystem to control
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     */
    public ReverseIndexerCommand(IndexerSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates a reverse command that drives the roller at a fixed negative RPM.
     *
     * @param subsystem indexer subsystem to control
     * @param targetRpm fixed target reverse RPM (negative value for backward rotation)
     */
    public ReverseIndexerCommand(IndexerSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
