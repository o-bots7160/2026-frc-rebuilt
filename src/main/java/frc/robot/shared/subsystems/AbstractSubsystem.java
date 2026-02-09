package frc.robot.shared.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.RobotEnvironment;
import frc.robot.shared.config.AbstractConfig;
import frc.robot.shared.logging.Logger;

/**
 * Base class for all robot subsystems that centralizes configuration flags, logging, and simulation awareness.
 * <p>
 * Extend this class to gain a shared logger, access to the loaded configuration, and convenience helpers for checking whether the subsystem should
 * run on real hardware. The {@code enabled} flag should gate any device actions in concrete subclasses.
 * </p>
 */
public abstract class AbstractSubsystem<TConfig extends AbstractConfig> extends SubsystemBase {
    /**
     * Nominal loop period in seconds for controller updates.
     */
    protected static double kDt = 0.02;

    /**
     * Subsystem configuration bundle loaded from JSON.
     */
    protected TConfig       config;

    /**
     * Cached class name used for logging prefixes.
     */
    protected String        className;

    /**
     * True when verbose logging is enabled for this subsystem.
     */
    protected boolean       verbose;

    /**
     * Logger instance scoped to the subsystem.
     */
    protected Logger        log;

    /**
     * True when the subsystem is allowed to run hardware actions.
     */
    protected boolean       enabled;

    /**
     * Creates a subsystem base with shared configuration and logging support.
     *
     * @param config Configuration object for the subsystem; supplies enable/verbose flags and any hardware identifiers.
     */
    protected AbstractSubsystem(TConfig config) {
        this.config    = config;
        this.enabled   = config.enabled;
        this.verbose   = config.verbose;
        this.className = this.getClass().getSimpleName();
        this.log       = Logger.getInstance(this.getClass(), verbose);
    }

    /**
     * States whether this subsystem is marked as enabled in configuration.
     *
     * @return True when the subsystem should execute its normal behavior.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * States whether this subsystem is marked as disabled in configuration.
     *
     * @return True when the subsystem should not execute any hardware interactions.
     */
    public boolean isSubsystemDisabled() {
        return !enabled;
    }

    /**
     * Reports whether verbose telemetry and debug output are enabled for this subsystem.
     * <p>
     * Use this flag to gate detailed logging so it can be disabled quickly for events.
     * </p>
     *
     * @return True when verbose logging is enabled in the subsystem configuration.
     */
    public boolean isVerbose() {
        return verbose;
    }

    /**
     * Reports whether the code is running in WPILib simulation.
     *
     * @return True when running in simulation rather than on a real robot.
     */
    protected boolean isSimulation() {
        return RobotEnvironment.isSimulation();
    }

    /**
     * Reports whether the code is running on real robot hardware.
     *
     * @return True when running on a real robot.
     */
    protected boolean isReal() {
        return RobotEnvironment.isReal();
    }

    /**
     * Reports whether the robot is attached to the FMS at call time.
     * <p>
     * Uses the per-cycle cached value from {@link RobotEnvironment#refreshCycle()}.
     * </p>
     *
     * @return True when running on a real robot that is currently FMS attached.
     */
    protected boolean isFMSAttached() {
        return RobotEnvironment.isFMSAttached();
    }

    /**
     * Reports a fatal or high-severity error to the Driver Station console.
     * <p>
     * Use this for constructor failures, missing hardware, or anything that should be immediately visible to operators.
     * </p>
     *
     * @param message    error description
     * @param stackTrace stack trace elements to include, or {@code null} to omit
     */
    protected void reportError(String message, StackTraceElement[] stackTrace) {
        RobotEnvironment.reportError(message, stackTrace);
    }

    /**
     * Reports a recoverable warning to the Driver Station console.
     * <p>
     * Use this for situations like a missing starting pose or a non-critical fallback.
     * </p>
     *
     * @param message        warning description
     * @param printStackTrace true to include a stack trace in the output
     */
    protected void reportWarning(String message, boolean printStackTrace) {
        RobotEnvironment.reportWarning(message, printStackTrace);
    }

    /**
     * Logs a standardized message when a call is skipped due to disable state.
     * <p>
     * Call this from public APIs to make skipped actions visible to operators.
     * </p>
     *
     * @param methodName name of the method that was skipped
     */
    protected void logDisabled(String methodName) {
        log.verbose(methodName + " called, but subsystem is disabled.");
    }
}
