package frc.robot.devices.motor;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.shared.logging.Logger;

/**
 * Simulation-only motor wrapper that mirrors the {@link Motor} surface without touching CAN hardware.
 * <p>
 * Advances a lightweight first-order motor model each loop so trapezoidal profiles can be exercised in the simulator. Positions are tracked in
 * radians internally; callers can supply converters to expose alternate units (for example, degrees for the turret) while keeping telemetry in
 * radians.
 * </p>
 */
public class SimMotor implements Motor {

    private static final double  kDtSeconds                     = 0.02;

    private final Logger         log;

    private final double         gearRatio;

    private final double         mechanismFreeSpeedRadPerSec;

    private final DoubleSupplier minimumPositionRadiansSupplier;

    private final DoubleSupplier maximumPositionRadiansSupplier;

    private final DoubleSupplier maximumVelocityRadiansPerSecondSupplier;

    private final DoubleSupplier maximumAccelerationRadiansPerSecondSquaredSupplier;

    private double               lastCommandedVolts             = 0.0;

    private double               lastCommandedPositionRads      = Double.NaN;

    private double               lastCommandedVelocityRadPerSec = Double.NaN;

    private double               positionRadians                = 0.0;

    private double               velocityRadPerSec              = 0.0;

    /**
     * Creates a simulated motor with supplier-backed motion bounds. Mechanism units are treated as degrees externally; all internal math runs in
     * radians and is converted for callers.
     */
    public SimMotor(
            String name,
            double motorRotationsPerMechanismRotation,
            Supplier<Double> minimumPositionSupplier,
            Supplier<Double> maximumPositionSupplier,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        DoubleSupplier minimumPositionRadiansSupplier                     = () -> Units.degreesToRadians(minimumPositionSupplier.get());
        DoubleSupplier maximumPositionRadiansSupplier                     = () -> Units.degreesToRadians(maximumPositionSupplier.get());
        DoubleSupplier maximumVelocityRadiansPerSecondSupplier            = () -> Units.degreesToRadians(maximumVelocitySupplier.get());
        DoubleSupplier maximumAccelerationRadiansPerSecondSquaredSupplier = () -> Units.degreesToRadians(maximumAccelerationSupplier.get());

        this.log                                                = Logger.getInstance(name);
        this.gearRatio                                          = motorRotationsPerMechanismRotation;
        this.mechanismFreeSpeedRadPerSec                        = Units.rotationsToRadians((5676.0 / 60.0) / gearRatio);
        this.minimumPositionRadiansSupplier                     = minimumPositionRadiansSupplier;
        this.maximumPositionRadiansSupplier                     = maximumPositionRadiansSupplier;
        this.maximumVelocityRadiansPerSecondSupplier            = maximumVelocityRadiansPerSecondSupplier;
        this.maximumAccelerationRadiansPerSecondSquaredSupplier = maximumAccelerationRadiansPerSecondSquaredSupplier;

        log.verbose("Configuring SimMotor " + name + " (gear ratio " + gearRatio + ")");

        // Seed targets with the initial mechanism position for cleaner telemetry.
        double initialMin = minimumPositionRadiansSupplier.getAsDouble();
        double initialMax = maximumPositionRadiansSupplier.getAsDouble();
        this.lastCommandedPositionRads      = clamp(0.0, initialMin, initialMax);
        this.lastCommandedVelocityRadPerSec = 0.0;
    }

    /**
     * Commands the simulated motor with a voltage request.
     *
     * @param volts desired voltage to apply
     */
    @Override
    public void setVoltage(double volts) {
        lastCommandedVolts = volts;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(edu.wpi.first.units.Units.Volts));
    }

    /**
     * Commands an open-loop duty cycle (-1 to 1) by scaling to 12 V.
     *
     * @param speed duty cycle request from -1.0 to 1.0
     */
    @Override
    public void setSpeed(double speed) {
        double clampedPercent = clamp(speed, -1.0, 1.0);
        setVoltage(clampedPercent * 12.0);
    }

    /**
     * Stops the simulated motor immediately.
     */
    @Override
    public void stop() {
        setVoltage(0.0);
    }

    /**
     * Reports the mechanism position in caller-facing units (default radians).
     */
    @Override
    public double getEncoderPosition() {
        return Units.radiansToDegrees(getPositionRadiansUnconverted());
    }

    /**
     * Reports the mechanism velocity in caller-facing units per second (default radians/second).
     */
    @Override
    public double getEncoderVelocity() {
        return Units.radiansToDegrees(getVelocityRadiansPerSecondUnconverted());
    }

    /**
     * Highest allowed target position in mechanism units.
     */
    @Override
    public double getMaximumTargetPosition() {
        return Units.radiansToDegrees(maximumPositionRadiansSupplier.getAsDouble());
    }

    /**
     * Lowest allowed target position in mechanism units.
     */
    @Override
    public double getMinimumTargetPosition() {
        return Units.radiansToDegrees(minimumPositionRadiansSupplier.getAsDouble());
    }

    /**
     * Reports the applied voltage (mirrors the last commanded voltage in simulation).
     */
    @Override
    public Voltage getVoltage() {
        return edu.wpi.first.units.Units.Volts.of(lastCommandedVolts);
    }

    /**
     * Simulation does not expose a physical SparkMax instance; returns {@code null} to satisfy the interface.
     */
    @Override
    public com.revrobotics.spark.SparkMax getMotor() {
        return null;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        stepSimulation();

        double positionRads      = getPositionRadiansUnconverted();
        double velocityRadPerSec = getVelocityRadiansPerSecondUnconverted();

        inputs.positionRads            = positionRads;
        inputs.velocityRadPerSec       = velocityRadPerSec;
        inputs.appliedVolts            = lastCommandedVolts;
        inputs.commandedVolts          = lastCommandedVolts;
        inputs.supplyCurrentAmps       = Math.abs(lastCommandedVolts) > 1e-3 ? Math.abs(lastCommandedVolts) * 0.5 : 0.0;
        inputs.temperatureCelsius      = 25.0;                                                                          // Simple placeholder; thermal
                                                                                                                        // modeling not required for
                                                                                                                        // sim
        inputs.targetPositionRads      = lastCommandedPositionRads;
        inputs.targetVelocityRadPerSec = lastCommandedVelocityRadPerSec;
    }

    /**
     * Records a position setpoint for telemetry and clamps it to the configured bounds.
     *
     * @param targetPositionRadians desired mechanism position (radians)
     * @return clamped position setpoint in radians
     */
    public double recordPositionSetpointRadians(double targetPositionRadians) {
        double clamped = clamp(targetPositionRadians, minimumPositionRadiansSupplier.getAsDouble(), maximumPositionRadiansSupplier.getAsDouble());
        lastCommandedPositionRads = clamped;
        return clamped;
    }

    /**
     * Records a velocity setpoint for telemetry.
     *
     * @param targetVelocityRadPerSec desired mechanism velocity (radians/second)
     */
    public void recordVelocitySetpointRadians(double targetVelocityRadPerSec) {
        lastCommandedVelocityRadPerSec = targetVelocityRadPerSec;
    }

    private void stepSimulation() {
        double duty                  = clamp(lastCommandedVolts / 12.0, -1.0, 1.0);

        double maxVelocityRadPerSec  = maximumVelocityRadiansPerSecondSupplier.getAsDouble();
        double maxAccelRadPerSecSq   = maximumAccelerationRadiansPerSecondSquaredSupplier.getAsDouble();
        double targetVelocityRadPerS = clamp(duty * mechanismFreeSpeedRadPerSec, -maxVelocityRadPerSec, maxVelocityRadPerSec);

        double maxDeltaV             = maxAccelRadPerSecSq * kDtSeconds;
        double deltaV                = clamp(targetVelocityRadPerS - velocityRadPerSec, -maxDeltaV, maxDeltaV);

        velocityRadPerSec += deltaV;
        positionRadians   += velocityRadPerSec * kDtSeconds;

        // Enforce motion bounds to mirror the hardware wrapper behavior.
        double minPosition = minimumPositionRadiansSupplier.getAsDouble();
        double maxPosition = maximumPositionRadiansSupplier.getAsDouble();

        if (positionRadians < minPosition) {
            positionRadians   = minPosition;
            velocityRadPerSec = 0.0;
        } else if (positionRadians > maxPosition) {
            positionRadians   = maxPosition;
            velocityRadPerSec = 0.0;
        }
    }

    private double getPositionRadiansUnconverted() {
        return clamp(positionRadians, minimumPositionRadiansSupplier.getAsDouble(), maximumPositionRadiansSupplier.getAsDouble());
    }

    private double getVelocityRadiansPerSecondUnconverted() {
        return velocityRadPerSec;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}