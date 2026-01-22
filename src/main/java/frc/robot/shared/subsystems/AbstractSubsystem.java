package frc.robot.shared.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    protected static double kDt = 0.02;

    protected TConfig       config;

    protected String        className;

    protected boolean       verbose;

    protected Logger        log;

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
     * Reports whether the code is running in WPILib simulation.
     *
     * @return True when running in simulation rather than on a real robot.
     */
    protected boolean isSimulation() {
        return !RobotBase.isReal();
    }

    /**
     * Reports whether the code is running on real robot hardware.
     *
     * @return True when running on a real robot.
     */
    protected boolean isReal() {
        return RobotBase.isReal();
    }

    /**
     * Reports whether the robot is attached to the FMS at call time.
     *
     * @return True when running on a real robot that is currently FMS attached.
     */
    protected boolean isFMSAttached() {
        return RobotBase.isReal() && DriverStation.isFMSAttached();
    }

    protected void logDisabled(String methodName) {
        log.verbose(methodName + " called, but subsystem is disabled.");
    }
}
