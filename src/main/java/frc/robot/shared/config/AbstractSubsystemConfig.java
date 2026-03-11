package frc.robot.shared.config;

/**
 * Configuration base class for subsystems that adds enable and verbose flags on top of the tunable infrastructure provided by
 * {@link AbstractConfig}.
 * <p>
 * All subsystem config classes should extend this class to inherit the {@code enabled} and {@code verbose} flags. Non-subsystem configs (motor
 * hardware descriptors, nested config bundles like {@link PidConfig} and {@link FeedforwardConfig}) should extend {@link AbstractConfig} directly
 * since they do not represent independently-toggleable subsystems.
 * </p>
 */
public abstract class AbstractSubsystemConfig extends AbstractConfig {

    /**
     * Enables or disables the subsystem that owns this config.
     * <p>
     * Set this in JSON to fully skip hardware actions when a mechanism is not ready.
     * </p>
     */
    public boolean enabled = true;

    /**
     * Enables verbose logging for the subsystem that owns this config.
     * <p>
     * Use this during tuning to print extra debug information.
     * </p>
     */
    public boolean verbose = false;
}
