package frc.robot.subsystems.indexer.commands;

import frc.robot.shared.commands.AbstractIdleVelocityCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Default command that maintains the indexer at the configured idle RPM (typically 0 to keep the roller stopped).
 * <p>
 * Set this as the default command so the indexer stays stopped when no feed, hold, or unjam command is running. The command never finishes on its own
 * and stops the motor when interrupted.
 * </p>
 */
public class IdleIndexerCommand extends AbstractIdleVelocityCommand<IndexerSubsystem> {

    /**
     * Creates an idle command for the indexer.
     *
     * @param subsystem indexer subsystem to hold at idle RPM
     */
    public IdleIndexerCommand(IndexerSubsystem subsystem) {
        super(subsystem);
    }
}
