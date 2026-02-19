package frc.robot.subsystems.indexer.commands;

import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Command that holds the indexer stopped so Fuel stays staged and does not advance into the shooter.
 * <p>
 * Use this when the shooter is not up to speed or the turret is not on target. The command sets the roller to 0 RPM on initialize and never finishes
 * on its own — it runs until interrupted by a higher-priority command (such as a feed command when all conditions are met). When ended, it stops the
 * motor.
 * </p>
 */
public class HoldCommand extends AbstractSubsystemCommand<IndexerSubsystem> {

    /**
     * Creates a hold command for the indexer.
     *
     * @param subsystem indexer subsystem to hold at zero RPM
     */
    public HoldCommand(IndexerSubsystem subsystem) {
        super(subsystem);
    }

    @Override
    public void execute() {
        subsystem.seekVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted by another command.
        return false;
    }

    @Override
    protected void onInitialize() {
        subsystem.setTargetVelocityRpm(0.0);
    }
}
