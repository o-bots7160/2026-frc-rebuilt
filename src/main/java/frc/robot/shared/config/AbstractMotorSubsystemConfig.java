package frc.robot.shared.config;

/**
 * Configuration values shared by any motor-driven subsystem that uses PID control and feedforward estimation.
 * <p>
 * Concrete configs for position-based (set-and-seek) and velocity-based subsystems extend this class to inherit tunable PID and feedforward gains
 * without duplicating the boilerplate. All gains are surfaced to SmartDashboard so they can be tuned live without redeploying.
 * </p>
 */
public abstract class AbstractMotorSubsystemConfig extends AbstractConfig {

    /**
     * Default quasistatic voltage ramp rate used by WPILib SysId when no override is specified.
     */
    private static final double DEFAULT_SYSID_RAMP_RATE_VOLTS_PER_SECOND       = 1.0;

    /**
     * Default step voltage used by WPILib SysId dynamic tests when no override is specified.
     */
    private static final double DEFAULT_SYSID_STEP_VOLTAGE                     = 7.0;

    /**
     * Default delay between SysId phases when no override is specified.
     */
    private static final double DEFAULT_SYSID_DELAY_SECONDS                    = 10.0;

    /**
     * Default quasistatic timeout when no override is specified.
     */
    private static final double DEFAULT_SYSID_QUASISTATIC_TIMEOUT_SECONDS      = 40.0;

    /**
     * Default dynamic timeout when no override is specified.
     */
    private static final double DEFAULT_SYSID_DYNAMIC_TIMEOUT_SECONDS          = 4.0;

    /** Proportional gain for the controller. */
    public double kP;

    /** Integral gain for the controller. */
    public double kI;

    /** Derivative gain for the controller. */
    public double kD;

    /** Static feedforward gain (volts). */
    public double kS;

    /** Velocity feedforward gain (volts per mechanism unit per second). */
    public double kV;

    /** Acceleration feedforward gain (volts per mechanism unit per second squared). */
    public double kA;

    /**
     * Quasistatic voltage ramp rate for SysId characterization in volts per second.
     * <p>
     * Lower values (0.25–0.5) give high-inertia mechanisms more time to reach steady-state at each
     * voltage level, producing more accurate kV fits. The WPILib default is 1.0 V/s. Set to 0 to
     * use the default.
     * </p>
     */
    public double sysIdRampRateVoltsPerSecond;

    /**
     * Step voltage applied during SysId dynamic tests in volts.
     * <p>
     * The WPILib default is 7.0 V. Set to 0 to use the default.
     * </p>
     */
    public double sysIdStepVoltage;

    /**
     * Delay between SysId phases in seconds, giving the mechanism time to coast to a stop so each phase starts from rest.
     * <p>
     * Set to 0 to use the default of 10 seconds.
     * </p>
     */
    public double sysIdDelaySeconds;

    /**
     * Timeout for the quasistatic (slow ramp) portion of SysId in seconds.
     * <p>
     * Must be long enough for the configured ramp rate to sweep through the useful voltage range. At 0.25 V/s, 40 seconds reaches 10 V. Set to 0 to
     * use the default of 40 seconds.
     * </p>
     */
    public double sysIdQuasistaticTimeoutSeconds;

    /**
     * Timeout for the dynamic (step voltage) portion of SysId in seconds.
     * <p>
     * Most mechanisms reach steady-state within 1–2 seconds after a voltage step. Set to 0 to use the default of 4 seconds.
     * </p>
     */
    public double sysIdDynamicTimeoutSeconds;

    /**
     * Returns the proportional gain for the controller.
     *
     * @return kP
     */
    public double getkP() {
        return readTunableNumber("kP", kP);
    }

    /**
     * Returns the integral gain for the controller.
     *
     * @return kI
     */
    public double getkI() {
        return readTunableNumber("kI", kI);
    }

    /**
     * Returns the derivative gain for the controller.
     *
     * @return kD
     */
    public double getkD() {
        return readTunableNumber("kD", kD);
    }

    /**
     * Returns the static feedforward term.
     *
     * @return kS (volts)
     */
    public double getkS() {
        return readTunableNumber("kS", kS);
    }

    /**
     * Returns the velocity feedforward term.
     *
     * @return kV (volts per mechanism unit per second)
     */
    public double getkV() {
        return readTunableNumber("kV", kV);
    }

    /**
     * Returns the acceleration feedforward term.
     *
     * @return kA (volts per mechanism unit per second squared)
     */
    public double getkA() {
        return readTunableNumber("kA", kA);
    }

    /**
     * Returns the quasistatic voltage ramp rate for SysId characterization.
     * <p>
     * When the configured value is zero or negative, the WPILib default of 1.0 V/s is used.
     * </p>
     *
     * @return ramp rate in volts per second
     */
    public double getSysIdRampRateVoltsPerSecond() {
        double value = readTunableNumber("sysIdRampRateVoltsPerSecond", sysIdRampRateVoltsPerSecond);
        return value > 0.0 ? value : DEFAULT_SYSID_RAMP_RATE_VOLTS_PER_SECOND;
    }

    /**
     * Returns the step voltage applied during SysId dynamic tests.
     * <p>
     * When the configured value is zero or negative, the WPILib default of 7.0 V is used.
     * </p>
     *
     * @return step voltage in volts
     */
    public double getSysIdStepVoltage() {
        double value = readTunableNumber("sysIdStepVoltage", sysIdStepVoltage);
        return value > 0.0 ? value : DEFAULT_SYSID_STEP_VOLTAGE;
    }

    /**
     * Returns the delay between SysId phases in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 10 seconds is used.
     * </p>
     *
     * @return delay in seconds
     */
    public double getSysIdDelaySeconds() {
        double value = readTunableNumber("sysIdDelaySeconds", sysIdDelaySeconds);
        return value > 0.0 ? value : DEFAULT_SYSID_DELAY_SECONDS;
    }

    /**
     * Returns the quasistatic SysId timeout in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 40 seconds is used.
     * </p>
     *
     * @return quasistatic timeout in seconds
     */
    public double getSysIdQuasistaticTimeoutSeconds() {
        double value = readTunableNumber("sysIdQuasistaticTimeoutSeconds", sysIdQuasistaticTimeoutSeconds);
        return value > 0.0 ? value : DEFAULT_SYSID_QUASISTATIC_TIMEOUT_SECONDS;
    }

    /**
     * Returns the dynamic SysId timeout in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 4 seconds is used.
     * </p>
     *
     * @return dynamic timeout in seconds
     */
    public double getSysIdDynamicTimeoutSeconds() {
        double value = readTunableNumber("sysIdDynamicTimeoutSeconds", sysIdDynamicTimeoutSeconds);
        return value > 0.0 ? value : DEFAULT_SYSID_DYNAMIC_TIMEOUT_SECONDS;
    }
}
