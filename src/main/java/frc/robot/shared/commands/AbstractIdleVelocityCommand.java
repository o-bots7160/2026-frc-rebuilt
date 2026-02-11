package frc.robot.shared.commands;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;

/**
 * Default command that maintains the idle velocity configured for a velocity subsystem.
 * <p>
 * Set this as the default command so the mechanism keeps spinning at the idle RPM when no other command is running. The command never finishes on its
 * own; it runs until interrupted by a higher-priority command (such as a spin-up for shooting). When interrupted or ended, it stops the motor.
 * </p>
 *
 * @param <TSubsystem> concrete velocity subsystem type
 */
public class AbstractIdleVelocityCommand<TSubsystem extends AbstractVelocitySubsystem<?>> extends AbstractSubsystemCommand<TSubsystem> {

    /**
     * Creates an idle command for the given velocity subsystem.
     *
     * @param subsystem subsystem instance to hold at idle RPM
     */
    protected AbstractIdleVelocityCommand(TSubsystem subsystem) {
        super(subsystem);
    }

    @Override
    public void execute() {
        subsystem.seekVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted by another command.
        return false;
    }

    @Override
    protected void onInitialize() {
        subsystem.setTargetVelocityRpm(subsystem.getIdleVelocityRpm());
    }
}
