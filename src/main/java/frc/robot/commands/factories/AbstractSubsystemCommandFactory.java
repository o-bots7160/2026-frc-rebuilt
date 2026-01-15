package frc.robot.commands.factories;

import frc.robot.subsystems.AbstractSubsystem;

public abstract class AbstractSubsystemCommandFactory<TSubsystem extends AbstractSubsystem<?>> {
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
