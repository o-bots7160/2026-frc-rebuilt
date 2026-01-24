package frc.robot.shared.commands;

import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;

/**
 * Retargets a set-and-seek subsystem to its current position and runs the profiled controller until it reports settled, then stops the motor. This
 * reset uses the live position and velocity so the controller decelerates smoothly instead of snapping back to a stale setpoint.
 *
 * @param <TSubsystem> concrete set-and-seek subsystem type
 */
public class SetAndSeekSettleCommand<TSubsystem extends AbstractSetAndSeekSubsystem<?>> extends AbstractSubsystemCommand<TSubsystem> {
    /**
     * Creates a settle command for the given subsystem.
     *
     * @param subsystem set-and-seek subsystem to hold in place
     */
    public SetAndSeekSettleCommand(TSubsystem subsystem) {
        super(subsystem);
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isProfileSettled();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.handleSeekInterrupted();
    }

    @Override
    protected void onInitialize() {
        log.warning("Attempting to settle " + getName() + " after interruption.");
        subsystem.settleAtCurrentPosition();
    }
}
