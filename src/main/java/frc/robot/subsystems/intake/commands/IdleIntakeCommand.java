package frc.robot.subsystems.intake.commands;

import frc.robot.shared.commands.AbstractIdleVelocityCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Default command that maintains the intake rollers at the configured idle RPM (typically zero).
 * <p>
 * Set this as the default command so the rollers stay stopped when no other command is active. The intake only spins when the operator explicitly
 * requests it. The command never finishes on its own and stops the motor when interrupted.
 * </p>
 */
public class IdleIntakeCommand extends AbstractIdleVelocityCommand<IntakeSubsystem> {

    /**
     * Creates an idle command for the intake.
     *
     * @param subsystem intake subsystem to hold at idle RPM
     */
    public IdleIntakeCommand(IntakeSubsystem subsystem) {
        super(subsystem);
    }
}
