package frc.robot.subsystems.feeder.commands;

import frc.robot.shared.commands.AbstractIdleVelocityCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Default command that maintains the feeder belt at the configured idle RPM.
 * <p>
 * Set this as the default command so the belt keeps running forward at a low RPM when no other command is active. This ensures Fuel continuously
 * moves from the hopper toward the indexer without operator intervention. The command never finishes on its own and stops the motor when interrupted.
 * </p>
 */
public class IdleFeederCommand extends AbstractIdleVelocityCommand<FeederSubsystem> {

    /**
     * Creates an idle command for the feeder.
     *
     * @param subsystem feeder subsystem to hold at idle RPM
     */
    public IdleFeederCommand(FeederSubsystem subsystem) {
        super(subsystem);
    }
}
