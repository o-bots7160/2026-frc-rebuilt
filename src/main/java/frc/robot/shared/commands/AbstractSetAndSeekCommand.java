package frc.robot.shared.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    // TODO: this should be in configuration instead of hardcoded
    private static final double    SETTLE_TIMEOUT_SECONDS = 5.0;

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
            CommandScheduler.getInstance()
                    .schedule(new SetAndSeekSettleCommand<>(subsystem).withTimeout(SETTLE_TIMEOUT_SECONDS));
        } else {
            log.info("Ending " + getName() + " after reaching target");
        }

        // Always stop after completion so the motor does not drift once the command exits
        subsystem.handleSeekInterrupted();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isProfileSettled();
    }

    @Override
    protected void onInitialize() {
        subsystem.setTarget(targetSupplier.get());
    }
}