package frc.robot.shared.commands;

import java.util.function.Supplier;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;

/**
 * Command base that drives an {@link AbstractVelocitySubsystem} toward a supplied target RPM using its velocity controller.
 * <p>
 * Provide a target RPM supplier in the constructor; the command sets the goal on initialize, steps the controller each cycle, and finishes once the
 * subsystem reports {@link AbstractVelocitySubsystem#isAtTargetVelocity()}.
 * </p>
 *
 * @param <TSubsystem> concrete velocity subsystem type
 */
public class AbstractVelocityCommand<TSubsystem extends AbstractVelocitySubsystem<?>> extends AbstractSubsystemCommand<TSubsystem> {

    private final Supplier<Double> targetRpmSupplier;

    /**
     * Builds a velocity command for the given subsystem.
     *
     * @param subsystem         subsystem instance to control
     * @param targetRpmSupplier provider for target velocity in mechanism RPM; evaluated on initialize
     */
    protected AbstractVelocityCommand(TSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem);
        this.targetRpmSupplier = targetRpmSupplier;
    }

    @Override
    public void execute() {
        subsystem.seekVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();

        if (interrupted) {
            log.warning("Ending " + getName() + " due to interruption");
        } else {
            log.info("Ending " + getName() + " after reaching target velocity");
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtTargetVelocity();
    }

    @Override
    protected void onInitialize() {
        subsystem.setTargetVelocityRpm(targetRpmSupplier.get());
    }
}
