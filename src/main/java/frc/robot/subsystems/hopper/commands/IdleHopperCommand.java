package frc.robot.subsystems.hopper.commands;

import frc.robot.shared.commands.AbstractIdleVelocityCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;

/**
 * Default command that maintains the hopper belt at the configured idle RPM.
 * <p>
 * Set this as the default command so the belt keeps running forward at a low RPM when no other command is active. This ensures Fuel continuously
 * moves toward the feeder without operator intervention. The command never finishes on its own and stops the motor when interrupted.
 * </p>
 */
public class IdleHopperCommand extends AbstractIdleVelocityCommand<HopperSubsystem> {

    /**
     * Creates an idle command for the hopper.
     *
     * @param subsystem hopper subsystem to hold at idle RPM
     */
    public IdleHopperCommand(HopperSubsystem subsystem) {
        super(subsystem);
    }
}
