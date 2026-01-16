package frc.robot.devices.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.logging.Logger;

/**
 * Base wrapper around a REV SparkMax that standardizes configuration, unit conversions, and AdvantageKit telemetry.
 * <p>
 * All positions are expressed in radians and velocities in radians per second to align with WPILib math utilities. If a subsystem or config prefers
 * degrees for readability, convert to radians at the call site (e.g., {@code Units.degreesToRadians(...)}).
 * </p>
 */
public abstract class AbstractMotor implements MotorIO {

    /** Default voltage compensation target applied to the controller. */
    protected static final double DEFAULT_VOLTAGE_COMPENSATION = 12.0;

    protected final String name;

    protected final Logger log;

    protected final SparkMax motor;

    private final double minimumPositionRadians;

    private final double maximumPositionRadians;

    private final double positionRadiansPerMotorRotation;

    private final double velocityRadPerSecPerMotorRpm;

    private double lastCommandedVolts          = 0.0;

    private double lastCommandedPositionRads   = Double.NaN;

    private double lastCommandedVelocityRadPerSec = Double.NaN;

    /**
     * Creates a motor wrapper with motion limits and a position conversion factor.
     *
     * @param name                           friendly name for logging and dashboard keys
     * @param deviceId                       CAN device ID for the SparkMax
     * @param minimumPositionRadians         lowest allowed mechanism position (radians)
     * @param maximumPositionRadians         highest allowed mechanism position (radians)
     * @param mechanismRadiansPerMotorRotation conversion factor from one motor rotation to mechanism radians (includes gear ratio)
     */
    protected AbstractMotor(
            String name,
            int deviceId,
            double minimumPositionRadians,
            double maximumPositionRadians,
            double mechanismRadiansPerMotorRotation) {
        this(name, deviceId, MotorType.kBrushless, minimumPositionRadians, maximumPositionRadians,
                mechanismRadiansPerMotorRotation);
    }

    /**
     * Creates a motor wrapper with motion limits, explicit motor type, and position conversion factor.
     *
     * @param name                           friendly name for logging and dashboard keys
     * @param deviceId                       CAN device ID for the SparkMax
     * @param motorType                      motor type (brushed or brushless)
     * @param minimumPositionRadians         lowest allowed mechanism position (radians)
     * @param maximumPositionRadians         highest allowed mechanism position (radians)
     * @param mechanismRadiansPerMotorRotation conversion factor from one motor rotation to mechanism radians (includes gear ratio)
     */
    protected AbstractMotor(
            String name,
            int deviceId,
            MotorType motorType,
            double minimumPositionRadians,
            double maximumPositionRadians,
            double mechanismRadiansPerMotorRotation) {
        this.name                            = name;
    this.log                             = Logger.getInstance(this.getClass());
        this.minimumPositionRadians          = minimumPositionRadians;
        this.maximumPositionRadians          = maximumPositionRadians;
        this.positionRadiansPerMotorRotation = mechanismRadiansPerMotorRotation;
        this.velocityRadPerSecPerMotorRpm    = mechanismRadiansPerMotorRotation / 60.0;

        this.motor = new SparkMax(deviceId, motorType);
        log.verbose("Configuring SparkMax motor " + name + " (device ID " + deviceId + ")");

        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);

        // Allow subclasses to append hardware-specific settings.
    config = configureMotor(config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Creates a motor wrapper without motion bounds. Positions are still reported in radians.
     *
     * @param name                           friendly name for logging and dashboard keys
     * @param deviceId                       CAN device ID for the SparkMax
     * @param mechanismRadiansPerMotorRotation conversion factor from one motor rotation to mechanism radians (includes gear ratio)
     */
    protected AbstractMotor(String name, int deviceId, double mechanismRadiansPerMotorRotation) {
        this(name, deviceId, MotorType.kBrushless, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
                mechanismRadiansPerMotorRotation);
    }

    /**
     * Applies an open-loop voltage command to the motor controller.
     *
     * @param volts desired voltage to apply
     */
    public void setVoltage(double volts) {
        lastCommandedVolts = volts;
        motor.setVoltage(volts);
    }

    /**
     * Applies an open-loop duty cycle (-1 to 1).
     *
     * @param percent duty cycle request, where 1.0 equals full forward
     */
    public void setDutyCycle(double percent) {
        lastCommandedVolts = percent * DEFAULT_VOLTAGE_COMPENSATION;
        motor.set(percent);
    }

    /**
     * Stops motor output immediately.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Reports the current measured position in radians.
     *
     * @return mechanism position (radians)
     */
    public double getPositionRadians() {
        return motor.getEncoder().getPosition() * positionRadiansPerMotorRotation;
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
        return motor.getEncoder().getVelocity() * velocityRadPerSecPerMotorRpm;
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

    /**
     * Returns the configured maximum allowed position in radians.
     *
     * @return upper motion bound (radians)
     */
    public double getMaximumPositionRadians() {
        return maximumPositionRadians;
    }

    /**
     * Reports the applied voltage based on bus voltage and applied output.
     *
     * @return applied voltage in volts
     */
    public double getAppliedVolts() {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    /**
     * Supplies the underlying SparkMax for advanced configuration or testing.
     *
     * @return backing SparkMax instance
     */
    public SparkMax getMotor() {
        return motor;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
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
     * Records a position setpoint for telemetry and clamps it to the configured bounds.
     *
     * @param targetPositionRadians desired mechanism position (radians)
     * @return clamped position setpoint in radians
     */
    protected double recordPositionSetpointRadians(double targetPositionRadians) {
        double clamped = clamp(targetPositionRadians, minimumPositionRadians, maximumPositionRadians);
        lastCommandedPositionRads = clamped;
        return clamped;
    }

    /**
     * Records a velocity setpoint for telemetry.
     *
     * @param targetVelocityRadPerSec desired mechanism velocity (radians/second)
     */
    protected void recordVelocitySetpointRadians(double targetVelocityRadPerSec) {
        lastCommandedVelocityRadPerSec = targetVelocityRadPerSec;
    }

    /**
     * Allows subclasses to append motor-specific configuration before the object is applied to hardware.
     *
     * @param config initial {@link SparkMaxConfig} with default voltage compensation
     * @return configured {@link SparkMaxConfig} ready for {@link SparkMax#configure}
     */
    protected abstract SparkMaxConfig configureMotor(SparkMaxConfig config);


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
