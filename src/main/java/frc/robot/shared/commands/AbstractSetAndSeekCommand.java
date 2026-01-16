package frc.robot.shared.commands;

import java.util.function.Supplier;

import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;

/**
 * Command base that drives a {@link AbstractSetAndSeekSubsystem} toward a supplied target using its trapezoidal profile.
 * <p>
 * Extend this class to bind operator inputs or autonomous goals without having the subsystem manufacture commands. Provide a target supplier in the
 * constructor; the command will set the goal on initialize, step the profile each cycle, and finish once the subsystem reports
 * {@link AbstractSetAndSeekSubsystem#atTarget()}.
 * </p>
 *
 * @param <TSubsystem> concrete set-and-seek subsystem type
 */
public class AbstractSetAndSeekCommand<TSubsystem extends AbstractSetAndSeekSubsystem<?>> extends AbstractSubsystemCommand<TSubsystem> {
    private final Supplier<Double> targetSupplier;

    /**
     * Builds a profiled seek command for the given subsystem.
     *
     * @param subsystem      subsystem instance to control
     * @param targetSupplier provider for target positions; evaluated on initialize
     */
    protected AbstractSetAndSeekCommand(TSubsystem subsystem, Supplier<Double> targetSupplier) {
        super(subsystem);
        this.targetSupplier = targetSupplier;
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            log.warning("Ending " + getName() + " due to interruption");
            subsystem.handleSeekInterrupted();
        } else {
            log.info("Ending " + getName() + " after reaching target");
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.atTarget();
    }

    @Override
    protected void onInitialize() {
        subsystem.setTarget(targetSupplier.get());
    }
}