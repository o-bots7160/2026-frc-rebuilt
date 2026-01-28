package frc.robot.devices.motor;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
        // Convert motor rotations per mechanism rotation into mechanism radians per motor rotation.
        return Units.rotationsToRadians(1.0 / motorRotationsPerMechanismRotation);
    }

    /** Friendly name used in log messages and dashboard keys. */
    protected final String          name;

    /** Shared logger instance used for warnings and verbose updates. */
    protected final Logger          log;

    /** Hardware controller that actually drives the motor output. */
    protected final SparkMax        motor;

    /** Base config applied to every SparkMax before subclass customizations. */
    private final SparkMaxConfig    baseConfig;

    /** Supplies whether soft limits should be enforced. */
    private final Supplier<Boolean> useSetpointLimitsSupplier;

    /** Supplies the reverse soft limit position in radians. */
    private final Supplier<Double>  reverseSoftLimitRadiansSupplier;

    /** Supplies the forward soft limit position in radians. */
    private final Supplier<Double>  forwardSoftLimitRadiansSupplier;

    /** Supplies the gear ratio as motor rotations per mechanism rotation. */
    private final Supplier<Double>  motorRotationsPerMechanismRotationSupplier;

    /** Motion bounds used to clamp position setpoints (in radians). */
    private double                  reverseSoftLimitRadians;

    /** Motion bounds used to clamp position setpoints (in radians). */
    private double                  forwardSoftLimitRadians;

    /** Conversion factor from motor rotations to mechanism radians. */
    private double                  positionRadiansPerMotorRotation;

    /** Conversion factor from motor RPM to mechanism radians per second. */
    private double                  velocityRadPerSecPerMotorRpm;

    /** Most recent open-loop command in volts (for telemetry). */
    private double                  lastCommandedVolts             = 0.0;

    /** Most recent position setpoint in radians (for telemetry). */
    private double                  lastCommandedPositionRads      = Double.NaN;

    /** Most recent velocity setpoint in radians per second (for telemetry). */
    private double                  lastCommandedVelocityRadPerSec = Double.NaN;

    /** Tracks whether the hardware config has been pushed to the controller. */
    private boolean                 initialized                    = false;

    /** Prevents spammy warnings if the motor is used before init(). */
    private boolean                 warnedNotInitialized           = false;

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
        this.name                                       = name;
        this.log                                        = Logger.getInstance(this.getClass());
        // Cache suppliers so limits can be refreshed alongside hardware updates.
        this.useSetpointLimitsSupplier                  = config.getUseSetpointLimitsSupplier();
        this.reverseSoftLimitRadiansSupplier            = config.getReverseSoftLimitRadiansSupplier();
        this.forwardSoftLimitRadiansSupplier            = config.getForwardSoftLimitRadiansSupplier();
        this.motorRotationsPerMechanismRotationSupplier = config.getMotorRotationsPerMechanismRotationSupplier();

        // Instantiate the controller with the requested CAN ID and motor type.
        int deviceId = config.getMotorCanIdSupplier().get();
        this.motor = new SparkMax(deviceId, motorType);
        log.verbose("Creating SparkMax motor " + name + " (device ID " + deviceId + ")");

        // Create the base config that will be customized by subclasses later.
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        this.baseConfig = sparkMaxConfig;
        // Compute unit conversion factors once so telemetry is fast.
        applyConversionFactors();
        applyHardwareSoftLimits();
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

        applyHardwareSoftLimits();
        applyConversionFactors();
        // Let the subclass inject additional settings (PID, current limits, soft limits, etc.).
        SparkMaxConfig sparkMaxConfig = configureMotor(baseConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        initialized = true;
        log.recordOutput("initialized", true);
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

        applyHardwareSoftLimits();
        applyConversionFactors();
        // Rebuild the config from scratch so tunable values are pulled fresh.
        SparkMaxConfig sparkMaxConfig = configureMotor(baseConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Reports whether the motor has been initialized with hardware configuration.
     *
     * @return true when {@link #init()} has completed successfully
     */
    public final boolean isInitialized() {
        return initialized;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if (!ensureInitialized("setVoltage")) {
            return;
        }
        double volts = voltage.in(edu.wpi.first.units.Units.Volts);
        // Store the command for logging before sending it to hardware.
        lastCommandedVolts = volts;
        motor.setVoltage(volts);
    }

    /**
     * Applies an open-loop duty cycle (-1 to 1).
     *
     * @param speed duty cycle request, where 1.0 equals full forward
     */
    @Override
    public void setSpeed(double speed) {
        if (!ensureInitialized("setSpeed")) {
            return;
        }
        // Clamp the duty cycle because SparkMax expects -1 to 1.
        double clampedPercent = MathUtil.clamp(speed, -1.0, 1.0);
        // Convert the duty cycle into a rough voltage for logging.
        lastCommandedVolts = clampedPercent * DEFAULT_VOLTAGE_COMPENSATION;
        motor.set(clampedPercent);
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
    }

    /**
     * Reports the current measured position in radians.
     *
     * @return mechanism position (radians)
     */
    @Override
    public double getPositionRadians() {
        if (!ensureInitialized("getPositionRadians")) {
            return 0.0;
        }
        // Encoder conversion factors are configured so SparkMax reports mechanism radians.
        return motor.getEncoder().getPosition();
    }

    /**
     * Reports the current measured velocity in radians per second.
     *
     * @return mechanism velocity (radians/second)
     */
    @Override
    public double getVelocityRadiansPerSecond() {
        if (!ensureInitialized("getVelocityRadiansPerSecond")) {
            return 0.0;
        }
        // Encoder conversion factors are configured so SparkMax reports radians per second.
        return motor.getEncoder().getVelocity();
    }

    @Override
    public Voltage getVoltage() {
        if (!ensureInitialized("getVoltage")) {
            return edu.wpi.first.units.Units.Volts.of(0.0);
        }
        // Applied output is a percent; multiply by bus voltage to estimate volts.
        return edu.wpi.first.units.Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
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
            inputs.positionRadians         = 0.0;
            inputs.velocityRadPerSec       = 0.0;
            inputs.appliedVolts            = edu.wpi.first.units.Units.Volts.of(0.0);
            inputs.busVoltageVolts         = 0.0;
            inputs.commandedVolts          = edu.wpi.first.units.Units.Volts.of(0.0);
            inputs.supplyCurrentAmps       = 0.0;
            inputs.temperatureCelsius      = 0.0;
            inputs.targetPositionRads      = Double.NaN;
            inputs.targetVelocityRadPerSec = Double.NaN;
            return;
        }
        // Pull fresh sensor data for logging and telemetry dashboards.
        inputs.positionRadians         = getPositionRadians();
        inputs.positionMotorRotations  = inputs.positionRadians / positionRadiansPerMotorRotation;
        inputs.positionDegrees         = Math.toDegrees(inputs.positionRadians);
        inputs.velocityRadPerSec       = getVelocityRadiansPerSecond();
        inputs.appliedVolts            = getVoltage();
        inputs.busVoltageVolts         = motor.getBusVoltage();
        inputs.commandedVolts          = edu.wpi.first.units.Units.Volts.of(lastCommandedVolts);
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
     * @param reverseSoftLimitRadians          reverse soft limit position in radians
     * @param forwardSoftLimitRadians          forward soft limit position in radians
     * @param mechanismRadiansPerMotorRotation mechanism radians per motor rotation (gear ratio applied)
     */
    protected final void updateMotionConstraints(
            double reverseSoftLimitRadians,
            double forwardSoftLimitRadians,
            double mechanismRadiansPerMotorRotation) {
        // Update motion bounds and conversion factors together to avoid mismatch.
        this.reverseSoftLimitRadians         = reverseSoftLimitRadians;
        this.forwardSoftLimitRadians         = forwardSoftLimitRadians;
        this.positionRadiansPerMotorRotation = mechanismRadiansPerMotorRotation;
        this.velocityRadPerSecPerMotorRpm    = mechanismRadiansPerMotorRotation / 60.0;
        applyHardwareSoftLimits();
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
            log.recordOutput("initialized", false);
        }

        return false;
    }

    private void applyHardwareSoftLimits() {
        if (useSetpointLimitsSupplier.get()) {
            reverseSoftLimitRadians = reverseSoftLimitRadiansSupplier.get();
            forwardSoftLimitRadians = forwardSoftLimitRadiansSupplier.get();
        } else {
            reverseSoftLimitRadians = Double.NEGATIVE_INFINITY;
            forwardSoftLimitRadians = Double.POSITIVE_INFINITY;
        }

        boolean hasReverseLimit = Double.isFinite(reverseSoftLimitRadians);
        boolean hasForwardLimit = Double.isFinite(forwardSoftLimitRadians);

        baseConfig.softLimit
                .reverseSoftLimitEnabled(hasReverseLimit)
                .forwardSoftLimitEnabled(hasForwardLimit);

        if (hasReverseLimit) {
            baseConfig.softLimit.reverseSoftLimit(reverseSoftLimitRadians);
        }

        if (hasForwardLimit) {
            baseConfig.softLimit.forwardSoftLimit(forwardSoftLimitRadians);
        }
    }

    private void applyConversionFactors() {
        positionRadiansPerMotorRotation = computeMechanismRadiansPerMotorRotation(
                motorRotationsPerMechanismRotationSupplier.get());
        velocityRadPerSecPerMotorRpm    = positionRadiansPerMotorRotation / 60.0;

        baseConfig.encoder
                .positionConversionFactor(positionRadiansPerMotorRotation)
                .velocityConversionFactor(velocityRadPerSecPerMotorRpm);
    }

}
