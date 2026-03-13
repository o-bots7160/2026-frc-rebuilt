package frc.robot.shared.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.config.AbstractSubsystemConfig;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.logging.Logger;

/**
 * Base class for all robot subsystems that centralizes configuration flags, logging, and simulation awareness.
 * <p>
 * Extend this class to gain a shared logger, access to the loaded configuration, and convenience helpers for checking whether the subsystem should
 * run on real hardware. The {@code enabled} flag should gate any device actions in concrete subclasses.
 * </p>
 */
public abstract class AbstractSubsystem<TConfig extends AbstractSubsystemConfig> extends SubsystemBase {
    /**
     * Nominal loop period in seconds for controller updates.
     */
    protected static double   kDt                   = 0.02;

    /**
     * Subsystem configuration bundle loaded from JSON.
     */
    protected TConfig         config;

    /**
     * Cached class name used for logging prefixes.
     */
    protected String          className;

    /**
     * True when verbose logging is enabled for this subsystem.
     */
    protected boolean         verbose;

    /**
     * Logger instance scoped to the subsystem.
     */
    protected Logger          log;

    /**
     * True when the subsystem is allowed to run hardware actions.
     */
    protected boolean         enabled;

    /**
     * Tracks method names that have already been logged as disabled so each message prints only once.
     */
    private final Set<String> loggedDisabledMethods = new HashSet<>();

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
        this.log       = Logger.getInstance(this.getClass(), config::getVerbose);

        // Eagerly read the tunable so the LoggedNetworkBoolean is created at
        // startup and the Elastic Dashboard toggle can bind immediately.
        config.getVerbose();
    }

    /**
     * Returns the subsystem configuration bundle for command factories and external consumers that need access to subsystem-specific settings.
     *
     * @return subsystem configuration
     */
    public TConfig getConfig() {
        return config;
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
     * The value is read from the tunable config each call so it can be toggled at
     * runtime from SmartDashboard without redeploying.
     * </p>
     *
     * @return True when verbose logging is enabled in the subsystem configuration.
     */
    public boolean isVerbose() {
        verbose = config.getVerbose();
        return verbose;
    }

    /**
     * Reports whether verbose AdvantageKit output should be recorded this cycle.
     * <p>
     * Combines two checks: the robot must not be attached to the FMS, and the
     * subsystem's verbose config flag must be enabled. Use this to guard expensive
     * pre-computation (array allocations, object construction) before calling
     * {@code log.recordVerboseOutput(...)}. The logger's own verbose gate handles
     * the actual suppression, but calling this first avoids unnecessary work.
     * </p>
     *
     * @return true when verbose AdvantageKit output is active for this subsystem
     */
    public boolean isVerboseLoggingEnabled() {
        return !isFMSAttached() && isVerbose();
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
     * Reports a recoverable warning to the Driver Station console.
     * <p>
     * Use this for situations like a missing starting pose or a non-critical fallback.
     * </p>
     *
     * @param message         warning description
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
        if (loggedDisabledMethods.add(methodName)) {
            log.verbose(methodName + " called, but subsystem is disabled.");
        }
    }
}
