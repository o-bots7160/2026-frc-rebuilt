package frc.robot.shared.config;

/**
 * Configuration bundle for WPILib SysId characterization routine parameters.
 * <p>
 * These values control how the SysId quasistatic and dynamic tests run: ramp rate, step voltage, inter-phase delay, and timeouts. When a field is
 * zero or negative, the corresponding default is used so subsystems that do not specify SysId settings still get sensible behavior.
 * </p>
 */
public class SysIdRoutineConfig extends AbstractConfig {

    /**
     * Default quasistatic voltage ramp rate used by WPILib SysId when no override is specified.
     */
    private static final double DEFAULT_RAMP_RATE_VOLTS_PER_SECOND       = 1.0;

    /**
     * Default step voltage used by WPILib SysId dynamic tests when no override is specified.
     */
    private static final double DEFAULT_STEP_VOLTAGE                     = 7.0;

    /**
     * Default delay between SysId phases when no override is specified.
     */
    private static final double DEFAULT_DELAY_SECONDS                    = 10.0;

    /**
     * Default quasistatic timeout when no override is specified.
     */
    private static final double DEFAULT_QUASISTATIC_TIMEOUT_SECONDS      = 40.0;

    /**
     * Default dynamic timeout when no override is specified.
     */
    private static final double DEFAULT_DYNAMIC_TIMEOUT_SECONDS          = 4.0;

    /**
     * Quasistatic voltage ramp rate for SysId characterization in volts per second.
     * <p>
     * Lower values (0.25-0.5) give high-inertia mechanisms more time to reach steady-state at each voltage level, producing more accurate kV fits.
     * The WPILib default is 1.0 V/s. Set to 0 to use the default.
     * </p>
     */
    public double rampRateVoltsPerSecond;

    /**
     * Step voltage applied during SysId dynamic tests in volts.
     * <p>
     * The WPILib default is 7.0 V. Set to 0 to use the default.
     * </p>
     */
    public double stepVoltage;

    /**
     * Delay between SysId phases in seconds, giving the mechanism time to coast to a stop so each phase starts from rest.
     * <p>
     * Set to 0 to use the default of 10 seconds.
     * </p>
     */
    public double delaySeconds;

    /**
     * Timeout for the quasistatic (slow ramp) portion of SysId in seconds.
     * <p>
     * Must be long enough for the configured ramp rate to sweep through the useful voltage range. At 0.25 V/s, 40 seconds reaches 10 V. Set to 0 to
     * use the default of 40 seconds.
     * </p>
     */
    public double quasistaticTimeoutSeconds;

    /**
     * Timeout for the dynamic (step voltage) portion of SysId in seconds.
     * <p>
     * Most mechanisms reach steady-state within 1-2 seconds after a voltage step. Set to 0 to use the default of 4 seconds.
     * </p>
     */
    public double dynamicTimeoutSeconds;

    /**
     * Returns the quasistatic voltage ramp rate for SysId characterization.
     * <p>
     * When the configured value is zero or negative, the WPILib default of 1.0 V/s is used.
     * </p>
     *
     * @return ramp rate in volts per second
     */
    public double getRampRateVoltsPerSecond() {
        double value = readTunableNumber("rampRateVoltsPerSecond", rampRateVoltsPerSecond);
        return value > 0.0 ? value : DEFAULT_RAMP_RATE_VOLTS_PER_SECOND;
    }

    /**
     * Returns the step voltage applied during SysId dynamic tests.
     * <p>
     * When the configured value is zero or negative, the WPILib default of 7.0 V is used.
     * </p>
     *
     * @return step voltage in volts
     */
    public double getStepVoltage() {
        double value = readTunableNumber("stepVoltage", stepVoltage);
        return value > 0.0 ? value : DEFAULT_STEP_VOLTAGE;
    }

    /**
     * Returns the delay between SysId phases in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 10 seconds is used.
     * </p>
     *
     * @return delay in seconds
     */
    public double getDelaySeconds() {
        double value = readTunableNumber("delaySeconds", delaySeconds);
        return value > 0.0 ? value : DEFAULT_DELAY_SECONDS;
    }

    /**
     * Returns the quasistatic SysId timeout in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 40 seconds is used.
     * </p>
     *
     * @return quasistatic timeout in seconds
     */
    public double getQuasistaticTimeoutSeconds() {
        double value = readTunableNumber("quasistaticTimeoutSeconds", quasistaticTimeoutSeconds);
        return value > 0.0 ? value : DEFAULT_QUASISTATIC_TIMEOUT_SECONDS;
    }

    /**
     * Returns the dynamic SysId timeout in seconds.
     * <p>
     * When the configured value is zero or negative, the default of 4 seconds is used.
     * </p>
     *
     * @return dynamic timeout in seconds
     */
    public double getDynamicTimeoutSeconds() {
        double value = readTunableNumber("dynamicTimeoutSeconds", dynamicTimeoutSeconds);
        return value > 0.0 ? value : DEFAULT_DYNAMIC_TIMEOUT_SECONDS;
    }
}
