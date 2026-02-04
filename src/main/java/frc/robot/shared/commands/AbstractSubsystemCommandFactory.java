package frc.robot.shared.commands;

import frc.robot.shared.subsystems.AbstractSubsystem;

/**
 * Base factory for building commands tied to a single subsystem instance.
 * <p>
 * Extend this in each subsystem package to keep command creation centralized.
 * </p>
 *
 * @param <TSubsystem> subsystem type that commands will operate on
 */
public abstract class AbstractSubsystemCommandFactory<TSubsystem extends AbstractSubsystem<?>> {
    /**
     * Subsystem instance shared by all commands created by this factory.
     */
    protected final TSubsystem subsystem;

    /**
     * Creates a command factory bound to the given subsystem so derived commands can share it. Call from subsystem-specific factory constructors when
     * wiring commands.
     *
     * @param subsystem the subsystem instance that commands produced by this factory will operate on
     */
    protected AbstractSubsystemCommandFactory(TSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Exposes the shared subsystem instance so callers can apply configuration or tuning hooks.
     *
     * @return subsystem that generated commands will operate on
     */
    public TSubsystem getSubsystem() {
        return subsystem;
    }
}
