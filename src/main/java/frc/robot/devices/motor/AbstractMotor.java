package frc.robot.devices.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.shared.config.AbstractMotorConfig;
import frc.robot.shared.logging.Logger;

/**
 * Base wrapper around a REV SparkMax that standardizes configuration, unit conversions, and AdvantageKit telemetry.
 * <p>
 * All positions are expressed in radians and velocities in radians per second to align with WPILib math utilities. If a subsystem or config prefers
 * degrees for readability, convert to radians at the call site (e.g., {@code Units.degreesToRadians(...)}).
 * </p>
 */
public abstract class AbstractMotor implements Motor {

    /** Default voltage compensation target applied to the controller. */
    protected static final double DEFAULT_VOLTAGE_COMPENSATION = 12.0;

    /**
     * Converts a gear ratio in motor rotations per mechanism rotation into mechanism radians per motor rotation.
     *
     * @param motorRotationsPerMechanismRotation motor rotations per one mechanism rotation
     * @return mechanism radians per motor rotation (defaults to 1:1 when the ratio is invalid)
     */
    protected static double computeMechanismRadiansPerMotorRotation(double motorRotationsPerMechanismRotation) {
        if (motorRotationsPerMechanismRotation <= 0.0) {
            return Units.rotationsToRadians(1.0);
        }
        return Units.rotationsToRadians(1.0 / motorRotationsPerMechanismRotation);
    }

    /** Friendly name used in log messages and dashboard keys. */
    protected final String       name;

    /** Shared logger instance used for warnings and verbose updates. */
    protected final Logger       log;

    /** Hardware controller that actually drives the motor output. */
    protected final SparkMax     motor;

    /** Base config applied to every SparkMax before subclass customizations. */
    private final SparkMaxConfig baseConfig;

    /** Motion bounds used to clamp position setpoints (in radians). */
    private double               minimumPositionRadians;

    /** Motion bounds used to clamp position setpoints (in radians). */
    private double               maximumPositionRadians;

    /** Conversion factor from motor rotations to mechanism radians. */
    private double               positionRadiansPerMotorRotation;

    /** Conversion factor from motor RPM to mechanism radians per second. */
    private double               velocityRadPerSecPerMotorRpm;

    /** Most recent open-loop command in volts (for telemetry). */
    private double               lastCommandedVolts             = 0.0;

    /** Most recent position setpoint in radians (for telemetry). */
    private double               lastCommandedPositionRads      = Double.NaN;

    /** Most recent velocity setpoint in radians per second (for telemetry). */
    private double               lastCommandedVelocityRadPerSec = Double.NaN;

    /** Tracks whether the hardware config has been pushed to the controller. */
    private boolean              initialized                    = false;

    /** Prevents spammy warnings if the motor is used before init(). */
    private boolean              warnedNotInitialized           = false;

    /** Counts how many times we have re-applied config. */
    private int                  reconfigureCount               = 0;

    /**
     * Creates a motor wrapper backed by a shared motor config using the default brushless SparkMax type.
     *
     * @param name   friendly name for logging and dashboard keys
     * @param config motor configuration bundle providing CAN ID, inversion, and bounds
     */
    protected AbstractMotor(String name, AbstractMotorConfig config) {
        this(name, config, MotorType.kBrushless);
    }

    /**
     * Creates a motor wrapper backed by a shared motor config and explicit motor type.
     *
     * @param name      friendly name for logging and dashboard keys
     * @param config    motor configuration bundle providing CAN ID, inversion, and bounds
     * @param motorType motor type (brushed or brushless)
     */
    protected AbstractMotor(
            String name,
            AbstractMotorConfig config,
            MotorType motorType) {
        this.name = name;
        this.log  = Logger.getInstance(this.getClass());
        // Pull motion bounds from the shared config so all commands clamp consistently.
        if (config.getUseSetpointLimitsSupplier().get()) {
            this.minimumPositionRadians = config.getMinimumSetpointRadiansSupplier().get();
            this.maximumPositionRadians = config.getMaximumSetpointRadiansSupplier().get();
        } else {
            this.minimumPositionRadians = Double.NEGATIVE_INFINITY;
            this.maximumPositionRadians = Double.POSITIVE_INFINITY;
        }

        // Compute unit conversion factors once so telemetry is fast.
        this.positionRadiansPerMotorRotation = computeMechanismRadiansPerMotorRotation(
                config.getMotorRotationsPerMechanismRotationSupplier().get());
        this.velocityRadPerSecPerMotorRpm    = positionRadiansPerMotorRotation / 60.0;

        // Instantiate the controller with the requested CAN ID and motor type.
        int deviceId = config.getMotorCanIdSupplier().get();
        this.motor = new SparkMax(deviceId, motorType);
        log.verbose("Creating SparkMax motor " + name + " (device ID " + deviceId + ")");

        // Create the base config that will be customized by subclasses later.
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        this.baseConfig = sparkMaxConfig;
    }

    /**
     * Applies hardware configuration after the subclass has finished assigning its config.
     * <p>
     * Call this once in the concrete motor constructor, after any config fields are assigned. Subsequent calls are ignored.
     * </p>
     */
    public final void init() {
        if (initialized) {
            return;
        }

        log.verbose("Configuring SparkMax motor " + name + " after initialization");

        // Let the subclass inject additional settings (PID, current limits, soft limits, etc.).
        SparkMaxConfig sparkMaxConfig = configureMotor(baseConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        initialized = true;
        log.recordOutput(name + "/initialized", true);
    }

    /**
     * Reapplies configuration to the controller using the latest config-backed values.
     * <p>
     * Call this after tuning values in Elastic/NetworkTables that affect controller settings (inversion, current limits, encoder scaling, etc.). This
     * does not recreate the SparkMax; it simply reconfigures the existing hardware instance.
     * </p>
     */
    public final void reconfigure() {
        if (!initialized) {
            init();
            return;
        }

        log.verbose("Reconfiguring SparkMax motor " + name + " after config change");

        // Rebuild the config from scratch so tunable values are pulled fresh.
        SparkMaxConfig sparkMaxConfig = configureMotor(baseConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        reconfigureCount++;
        log.recordOutput(name + "/reconfigureCount", reconfigureCount);
    }

    /**
     * Reports whether the motor has been initialized with hardware configuration.
     *
     * @return true when {@link #init()} has completed successfully
     */
    public final boolean isInitialized() {
        return initialized;
    }

    /**
     * Applies an open-loop voltage command to the motor controller.
     *
     * @param volts desired voltage to apply
     */
    @Override
    public void setVoltage(double volts) {
        if (!ensureInitialized("setVoltage")) {
            return;
        }
        // Store the command for logging before sending it to hardware.
        lastCommandedVolts = volts;
        motor.setVoltage(volts);
        log.recordOutput(name + "/commandedVolts", lastCommandedVolts);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(edu.wpi.first.units.Units.Volts));
    }

    /**
     * Applies an open-loop duty cycle (-1 to 1).
     *
     * @param percent duty cycle request, where 1.0 equals full forward
     */
    public void setDutyCycle(double percent) {
        if (!ensureInitialized("setDutyCycle")) {
            return;
        }
        // Clamp the duty cycle because SparkMax expects -1 to 1.
        double clampedPercent = clamp(percent, -1.0, 1.0);
        // Convert the duty cycle into a rough voltage for logging.
        lastCommandedVolts = clampedPercent * DEFAULT_VOLTAGE_COMPENSATION;
        motor.set(clampedPercent);
        log.recordOutput(name + "/commandedDutyCycle", clampedPercent);
        log.recordOutput(name + "/commandedVolts", lastCommandedVolts);
    }

    @Override
    public void setSpeed(double speed) {
        setDutyCycle(speed);
    }

    /**
     * Stops motor output immediately.
     */
    @Override
    public void stop() {
        if (!ensureInitialized("stop")) {
            return;
        }
        motor.stopMotor();
        lastCommandedVolts = 0.0;
        log.recordOutput(name + "/commandedVolts", lastCommandedVolts);
    }

    /**
     * Reports the current measured position in radians.
     *
     * @return mechanism position (radians)
     */
    public double getPositionRadians() {
        if (!ensureInitialized("getPositionRadians")) {
            return 0.0;
        }
        // The SparkMax encoder reports motor rotations; convert to mechanism radians.
        return motor.getEncoder().getPosition() * positionRadiansPerMotorRotation;
    }

    @Override
    public double getEncoderPosition() {
        return getPositionRadians();
    }

    /**
     * Reports the current measured position in degrees (helper for student-facing UI).
     *
     * @return mechanism position (degrees)
     */
    public double getPositionDegrees() {
        return Units.radiansToDegrees(getPositionRadians());
    }

    /**
     * Reports the current measured velocity in radians per second.
     *
     * @return mechanism velocity (radians/second)
     */
    public double getVelocityRadPerSec() {
        if (!ensureInitialized("getVelocityRadPerSec")) {
            return 0.0;
        }
        // The SparkMax encoder reports motor RPM; convert to radians per second.
        return motor.getEncoder().getVelocity() * velocityRadPerSecPerMotorRpm;
    }

    @Override
    public double getEncoderVelocity() {
        return getVelocityRadPerSec();
    }

    /**
     * Reports the current measured velocity in degrees per second (helper for student-facing UI).
     *
     * @return mechanism velocity (degrees/second)
     */
    public double getVelocityDegreesPerSec() {
        return Units.radiansToDegrees(getVelocityRadPerSec());
    }

    /**
     * Returns the last commanded target position for logging convenience.
     *
     * @return target position in radians (may be {@link Double#NaN} if unset)
     */
    public double getTargetPositionRadians() {
        return lastCommandedPositionRads;
    }

    /**
     * Returns the last commanded target velocity for logging convenience.
     *
     * @return target velocity in radians per second (may be {@link Double#NaN} if unset)
     */
    public double getTargetVelocityRadPerSec() {
        return lastCommandedVelocityRadPerSec;
    }

    /**
     * Returns the configured minimum allowed position in radians.
     *
     * @return lower motion bound (radians)
     */
    public double getMinimumPositionRadians() {
        return minimumPositionRadians;
    }

    @Override
    public double getMinimumTargetPosition() {
        return minimumPositionRadians;
    }

    /**
     * Returns the configured maximum allowed position in radians.
     *
     * @return upper motion bound (radians)
     */
    public double getMaximumPositionRadians() {
        return maximumPositionRadians;
    }

    @Override
    public double getMaximumTargetPosition() {
        return maximumPositionRadians;
    }

    /**
     * Reports the applied voltage based on bus voltage and applied output.
     *
     * @return applied voltage in volts
     */
    public double getAppliedVolts() {
        if (!ensureInitialized("getAppliedVolts")) {
            return 0.0;
        }
        // Applied output is a percent; multiply by bus voltage to estimate volts.
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    @Override
    public Voltage getVoltage() {
        return edu.wpi.first.units.Units.Volts.of(getAppliedVolts());
    }

    /**
     * Supplies the underlying SparkMax for advanced configuration or testing.
     *
     * @return backing SparkMax instance
     */
    @Override
    public SparkMax getMotor() {
        if (!ensureInitialized("getMotor")) {
            return motor;
        }
        return motor;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        if (!ensureInitialized("updateInputs")) {
            // Report zeros/NaN so logs show the motor is inactive.
            inputs.positionRads            = 0.0;
            inputs.velocityRadPerSec       = 0.0;
            inputs.appliedVolts            = 0.0;
            inputs.commandedVolts          = 0.0;
            inputs.supplyCurrentAmps       = 0.0;
            inputs.temperatureCelsius      = 0.0;
            inputs.targetPositionRads      = Double.NaN;
            inputs.targetVelocityRadPerSec = Double.NaN;
            return;
        }
        // Pull fresh sensor data for logging and telemetry dashboards.
        inputs.positionRads            = getPositionRadians();
        inputs.velocityRadPerSec       = getVelocityRadPerSec();
        inputs.appliedVolts            = getAppliedVolts();
        inputs.commandedVolts          = lastCommandedVolts;
        inputs.supplyCurrentAmps       = motor.getOutputCurrent();
        inputs.temperatureCelsius      = motor.getMotorTemperature();
        inputs.targetPositionRads      = lastCommandedPositionRads;
        inputs.targetVelocityRadPerSec = lastCommandedVelocityRadPerSec;
    }

    /**
     * Updates the motion bounds and conversion factors used for unit conversions and clamping.
     * <p>
     * Use this when tunable configuration updates change limits or gear ratios so internal conversions stay aligned with the hardware.
     * </p>
     *
     * @param minimumPositionRadians           minimum allowed position in radians
     * @param maximumPositionRadians           maximum allowed position in radians
     * @param mechanismRadiansPerMotorRotation mechanism radians per motor rotation (gear ratio applied)
     */
    protected final void updateMotionConstraints(
            double minimumPositionRadians,
            double maximumPositionRadians,
            double mechanismRadiansPerMotorRotation) {
        // Update motion bounds and conversion factors together to avoid mismatch.
        this.minimumPositionRadians          = minimumPositionRadians;
        this.maximumPositionRadians          = maximumPositionRadians;
        this.positionRadiansPerMotorRotation = mechanismRadiansPerMotorRotation;
        this.velocityRadPerSecPerMotorRpm    = mechanismRadiansPerMotorRotation / 60.0;

        log.recordOutput(name + "/minimumPositionRads", minimumPositionRadians);
        log.recordOutput(name + "/maximumPositionRads", maximumPositionRadians);
        log.recordOutput(name + "/radsPerMotorRotation", mechanismRadiansPerMotorRotation);
    }

    /**
     * Records a position setpoint for telemetry and clamps it to the configured bounds.
     *
     * @param targetPositionRadians desired mechanism position (radians)
     * @return clamped position setpoint in radians
     */
    protected double recordPositionSetpointRadians(double targetPositionRadians) {
        // Clamp the requested position so commands cannot exceed limits.
        double clamped = clamp(targetPositionRadians, minimumPositionRadians, maximumPositionRadians);
        lastCommandedPositionRads = clamped;
        log.recordOutput(name + "/targetRequestedPositionRads", targetPositionRadians);
        log.recordOutput(name + "/targetPositionRads", clamped);
        log.recordOutput(name + "/targetWasClamped", targetPositionRadians != clamped);
        return clamped;
    }

    /**
     * Records a velocity setpoint for telemetry.
     *
     * @param targetVelocityRadPerSec desired mechanism velocity (radians/second)
     */
    protected void recordVelocitySetpointRadians(double targetVelocityRadPerSec) {
        // Store for telemetry (velocity may still be controlled by subclasses).
        lastCommandedVelocityRadPerSec = targetVelocityRadPerSec;
        log.recordOutput(name + "/targetVelocityRadPerSec", targetVelocityRadPerSec);
    }

    /**
     * Allows subclasses to append motor-specific configuration before the object is applied to hardware.
     *
     * @param config initial {@link SparkMaxConfig} with default voltage compensation
     * @return configured {@link SparkMaxConfig} ready for {@link SparkMax#configure}
     */
    protected abstract SparkMaxConfig configureMotor(SparkMaxConfig config);

    private boolean ensureInitialized(String action) {
        if (initialized) {
            return true;
        }

        // Log once so students notice missing initialization without spamming.
        if (!warnedNotInitialized) {
            warnedNotInitialized = true;
            log.warning("Motor " + name + " ignored " + action + " before init(); call init() after assigning config.");
            log.recordOutput(name + "/initialized", false);
        }
        return false;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
