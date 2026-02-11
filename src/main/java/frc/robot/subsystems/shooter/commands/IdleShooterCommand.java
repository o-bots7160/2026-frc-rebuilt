package frc.robot.subsystems.shooter.commands;

import frc.robot.shared.commands.AbstractIdleVelocityCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Default command that maintains the shooter flywheel at the configured idle RPM.
 * <p>
 * Set this as the default command so the flywheel keeps spinning at a low RPM when no shot is requested. This reduces spin-up time when a shot
 * command arrives. The command never finishes on its own and stops the motor when interrupted.
 * </p>
 */
public class IdleShooterCommand extends AbstractIdleVelocityCommand<ShooterSubsystem> {

    /**
     * Creates an idle command for the shooter.
     *
     * @param subsystem shooter subsystem to hold at idle RPM
     */
    public IdleShooterCommand(ShooterSubsystem subsystem) {
        super(subsystem);
    }
}
