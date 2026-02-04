package frc.robot.shared.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shared.logging.Logger;
import frc.robot.shared.subsystems.AbstractSubsystem;

/**
 * Lightweight base command that ties a command to a single subsystem and logs when it starts. Subclasses should override the lifecycle hooks as
 * needed for command behavior.
 *
 * @param <TSubsystem> subsystem type this command operates on
 */
public abstract class AbstractSubsystemCommand<TSubsystem extends AbstractSubsystem<?>> extends Command {
    /**
     * Subsystem instance this command requires and operates on.
     */
    protected final TSubsystem subsystem;

    /**
     * Logger scoped to the concrete command class.
     */
    protected final Logger     log;

    /**
     * Creates a subsystem-scoped command and registers the requirement.
     *
     * @param subsystem subsystem instance this command will control
     */
    protected AbstractSubsystemCommand(TSubsystem subsystem) {
        this.subsystem = subsystem;
        this.log       = Logger.getInstance(getClass());

        addRequirements(subsystem);
    }

    @Override
    public final void initialize() {
        log.info("Starting " + getName());
        onInitialize();
    }

    /**
     * Optional hook for subclasses to run logic during initialize after the start log.
     */
    protected void onInitialize() {
        // no-op by default
    }
}
