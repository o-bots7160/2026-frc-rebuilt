package frc.robot.devices.motor;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.shared.logging.Logger;

/**
 * Simulation-only motor wrapper that mirrors the {@link Motor} surface without touching CAN hardware.
 * <p>
 * Uses a {@link DCMotorSim} model with a simple inertia estimate so open-loop voltage commands behave like a real brushed motor with back-EMF.
 * Positions are tracked in radians internally; callers can expose alternate units (for example, degrees for the turret) while keeping telemetry in
 * radians.
 * </p>
 */
public class SimMotor implements Motor {

    private static final double     kDtSeconds                        = 0.02;

    private static final double     kNominalVoltage                   = 12.0;

    private static final double     kMinimumAccelerationRadPerSecSq   = 1e-3;

    private static final double     kMinimumMomentOfInertiaKgMetersSq = 1e-6;

    private static final double     kDefaultMomentOfInertiaKgMetersSq = 0.001;

    private static final double     kEpsilon                          = 1e-6;

    private final Logger            log;

    private final double            gearRatio;

    private final DCMotor           motorModel;

    private DCMotorSim              motorSim;

    private final double            mechanismFreeSpeedRadPerSec;

    private final DoubleSupplier    minimumPositionRadiansSupplier;

    private final DoubleSupplier    maximumPositionRadiansSupplier;

    private final DoubleSupplier    maximumVelocityRadiansPerSecondSupplier;

    private final DoubleSupplier    maximumAccelerationRadiansPerSecondSquaredSupplier;

    private final Supplier<Boolean> useSetpointLimitsSupplier;

    private double                  lastCommandedVolts                = 0.0;

    private double                  lastCommandedPositionRads         = Double.NaN;

    private double                  lastCommandedVelocityRadPerSec    = Double.NaN;

    private double                  positionRadians                   = 0.0;

    private double                  velocityRadPerSec                 = 0.0;

    private double                  lastMaxAccelerationRadPerSecSq    = Double.NaN;

    private final String            name;

    /**
     * Creates a simulated motor with supplier-backed motion bounds.
     * <p>
     * Mechanism units are treated as degrees externally; all internal math runs in radians and is converted for callers.
     * </p>
     *
     * @param name                               friendly name used for logging
     * @param motorRotationsPerMechanismRotation motor rotations per one mechanism rotation
     * @param minimumPositionSupplier            minimum position in degrees
     * @param maximumPositionSupplier            maximum position in degrees
     * @param maximumVelocitySupplier            maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier        maximum profile acceleration in degrees per second squared
     */
    public SimMotor(
            String name,
            double motorRotationsPerMechanismRotation,
            Supplier<Double> minimumPositionSupplier,
            Supplier<Double> maximumPositionSupplier,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        this(
                name,
                motorRotationsPerMechanismRotation,
                minimumPositionSupplier,
                maximumPositionSupplier,
                maximumVelocitySupplier,
                maximumAccelerationSupplier,
                () -> true,
                DCMotor.getNEO(1));
    }

    /**
     * Creates a simulated motor with an explicit motor model for improved realism.
     * <p>
     * Use this when you know the exact motor type (for example, NEO 550) or the number of motors in the gearbox.
     * </p>
     *
     * @param name                               friendly name used for logging
     * @param motorRotationsPerMechanismRotation motor rotations per one mechanism rotation
     * @param minimumPositionSupplier            minimum position in degrees
     * @param maximumPositionSupplier            maximum position in degrees
     * @param maximumVelocitySupplier            maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier        maximum profile acceleration in degrees per second squared
     * @param motorModel                         motor model (for example, {@link DCMotor#getNEO(int)})
     */
    public SimMotor(
            String name,
            double motorRotationsPerMechanismRotation,
            Supplier<Double> minimumPositionSupplier,
            Supplier<Double> maximumPositionSupplier,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier,
            DCMotor motorModel) {
        this(
                name,
                motorRotationsPerMechanismRotation,
                minimumPositionSupplier,
                maximumPositionSupplier,
                maximumVelocitySupplier,
                maximumAccelerationSupplier,
                () -> true,
                motorModel);
    }

    /**
     * Creates a simulated motor with supplier-backed motion bounds and a toggle for enforcing limits.
     * <p>
     * Disable setpoint limits when running SysId so the characterization sweep is not clipped by the configured bounds.
     * </p>
     *
     * @param name                               friendly name used for logging
     * @param motorRotationsPerMechanismRotation motor rotations per one mechanism rotation
     * @param minimumPositionSupplier            minimum position in degrees
     * @param maximumPositionSupplier            maximum position in degrees
     * @param maximumVelocitySupplier            maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier        maximum profile acceleration in degrees per second squared
     * @param useSetpointLimitsSupplier          supplier that enables or disables position clamping
     */
    public SimMotor(
            String name,
            double motorRotationsPerMechanismRotation,
            Supplier<Double> minimumPositionSupplier,
            Supplier<Double> maximumPositionSupplier,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier,
            Supplier<Boolean> useSetpointLimitsSupplier) {
        this(
                name,
                motorRotationsPerMechanismRotation,
                minimumPositionSupplier,
                maximumPositionSupplier,
                maximumVelocitySupplier,
                maximumAccelerationSupplier,
                useSetpointLimitsSupplier,
                DCMotor.getNEO(1));
    }

    /**
     * Creates a simulated motor with an explicit motor model and optional setpoint limit enforcement.
     * <p>
     * Use this when you know the exact motor type (for example, NEO 550) or the number of motors in the gearbox.
     * </p>
     *
     * @param name                               friendly name used for logging
     * @param motorRotationsPerMechanismRotation motor rotations per one mechanism rotation
     * @param minimumPositionSupplier            minimum position in degrees
     * @param maximumPositionSupplier            maximum position in degrees
     * @param maximumVelocitySupplier            maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier        maximum profile acceleration in degrees per second squared
     * @param useSetpointLimitsSupplier          supplier that enables or disables position clamping
     * @param motorModel                         motor model (for example, {@link DCMotor#getNEO(int)})
     */
    public SimMotor(
            String name,
            double motorRotationsPerMechanismRotation,
            Supplier<Double> minimumPositionSupplier,
            Supplier<Double> maximumPositionSupplier,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier,
            Supplier<Boolean> useSetpointLimitsSupplier,
            DCMotor motorModel) {
        DoubleSupplier minimumPositionRadiansSupplier                     = () -> Units.degreesToRadians(minimumPositionSupplier.get());
        DoubleSupplier maximumPositionRadiansSupplier                     = () -> Units.degreesToRadians(maximumPositionSupplier.get());
        DoubleSupplier maximumVelocityRadiansPerSecondSupplier            = () -> Units.degreesToRadians(maximumVelocitySupplier.get());
        DoubleSupplier maximumAccelerationRadiansPerSecondSquaredSupplier = () -> Units.degreesToRadians(maximumAccelerationSupplier.get());

        this.name                                               = name;
        this.log                                                = Logger.getInstance(name);
        this.gearRatio                                          = motorRotationsPerMechanismRotation > 0.0
                ? motorRotationsPerMechanismRotation
                : 1.0;
        this.motorModel                                         = motorModel;
        this.mechanismFreeSpeedRadPerSec                        = motorModel.freeSpeedRadPerSec / this.gearRatio;
        this.minimumPositionRadiansSupplier                     = minimumPositionRadiansSupplier;
        this.maximumPositionRadiansSupplier                     = maximumPositionRadiansSupplier;
        this.maximumVelocityRadiansPerSecondSupplier            = maximumVelocityRadiansPerSecondSupplier;
        this.maximumAccelerationRadiansPerSecondSquaredSupplier = maximumAccelerationRadiansPerSecondSquaredSupplier;
        this.useSetpointLimitsSupplier                          = useSetpointLimitsSupplier;

        log.verbose("Configuring SimMotor " + name + " (gear ratio " + gearRatio + ")");

        // Seed targets with the initial mechanism position for cleaner telemetry.
        double initialMin = minimumPositionRadiansSupplier.getAsDouble();
        double initialMax = maximumPositionRadiansSupplier.getAsDouble();
        this.lastCommandedPositionRads      = shouldEnforceSetpointLimits()
                ? clamp(0.0, initialMin, initialMax)
                : 0.0;
        this.lastCommandedVelocityRadPerSec = 0.0;
        this.positionRadians                = this.lastCommandedPositionRads;
        this.velocityRadPerSec              = 0.0;

        rebuildMotorSim(maximumAccelerationRadiansPerSecondSquaredSupplier.getAsDouble());
        motorSim.setState(VecBuilder.fill(positionRadians, velocityRadPerSec));

        log.recordOutput("initialized", true);
        log.recordOutput("gearRatio", gearRatio);
        log.recordOutput("freeSpeedRadPerSec", mechanismFreeSpeedRadPerSec);
    }

    /**
     * Commands the simulated motor with a voltage request.
     *
     * @param volts desired voltage to apply
     */
    @Override
    public void setVoltage(double volts) {
        lastCommandedVolts = volts;
        log.recordOutput("commandedVolts", lastCommandedVolts);
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
        // Convert a duty cycle request into an equivalent voltage for the sim model.
        double clampedPercent = clamp(speed, -1.0, 1.0);
        setVoltage(clampedPercent * 12.0);
        log.recordOutput("commandedDutyCycle", clampedPercent);
    }

    /**
     * Stops the simulated motor immediately.
     */
    @Override
    public void stop() {
        setVoltage(0.0);
        log.recordOutput("stopped", true);
    }

    /**
     * Reports the mechanism position in degrees.
     */
    @Override
    public double getEncoderPosition() {
        return Units.radiansToDegrees(getPositionRadiansUnconverted());
    }

    /**
     * Reports the mechanism velocity in degrees per second.
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
        if (!shouldEnforceSetpointLimits()) {
            return Double.POSITIVE_INFINITY;
        }
        return Units.radiansToDegrees(maximumPositionRadiansSupplier.getAsDouble());
    }

    /**
     * Lowest allowed target position in mechanism units.
     */
    @Override
    public double getMinimumTargetPosition() {
        if (!shouldEnforceSetpointLimits()) {
            return Double.NEGATIVE_INFINITY;
        }
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
        // Step the physics model before exporting sensor values.
        stepSimulation();

        double positionRads      = getPositionRadiansUnconverted();
        double velocityRadPerSec = getVelocityRadiansPerSecondUnconverted();

        inputs.positionRads      = positionRads;
        inputs.velocityRadPerSec = velocityRadPerSec;
        double appliedVolts = clamp(lastCommandedVolts, -kNominalVoltage, kNominalVoltage);
        inputs.appliedVolts            = appliedVolts;
        inputs.commandedVolts          = lastCommandedVolts;
        inputs.supplyCurrentAmps       = motorSim.getCurrentDrawAmps();
        inputs.temperatureCelsius      = 25.0;                          // Simple placeholder; thermal
                                                                        // modeling not required for
                                                                        // sim
        inputs.targetPositionRads      = lastCommandedPositionRads;
        inputs.targetVelocityRadPerSec = lastCommandedVelocityRadPerSec;

        log.recordOutput("positionRads", positionRads);
        log.recordOutput("velocityRadPerSec", velocityRadPerSec);
        log.recordOutput("targetPositionRads", lastCommandedPositionRads);
        log.recordOutput("targetVelocityRadPerSec", lastCommandedVelocityRadPerSec);
    }

    /**
     * Records a position setpoint for telemetry and clamps it to the configured bounds.
     *
     * @param targetPositionRadians desired mechanism position (radians)
     * @return clamped position setpoint in radians
     */
    public double recordPositionSetpointRadians(double targetPositionRadians) {
        double clamped = targetPositionRadians;
        if (shouldEnforceSetpointLimits()) {
            clamped = clamp(targetPositionRadians, minimumPositionRadiansSupplier.getAsDouble(), maximumPositionRadiansSupplier.getAsDouble());
        }
        lastCommandedPositionRads = clamped;
        log.recordOutput("targetRequestedPositionRads", targetPositionRadians);
        log.recordOutput("targetPositionRads", clamped);
        log.recordOutput("targetWasClamped", targetPositionRadians != clamped);
        return clamped;
    }

    /**
     * Records a velocity setpoint for telemetry.
     *
     * @param targetVelocityRadPerSec desired mechanism velocity (radians/second)
     */
    public void recordVelocitySetpointRadians(double targetVelocityRadPerSec) {
        lastCommandedVelocityRadPerSec = targetVelocityRadPerSec;
        log.recordOutput("targetVelocityRadPerSec", targetVelocityRadPerSec);
    }

    private void stepSimulation() {
        double maxAccelRadPerSecSq = maximumAccelerationRadiansPerSecondSquaredSupplier.getAsDouble();
        rebuildMotorSimIfNeeded(maxAccelRadPerSecSq);

        double appliedVolts = clamp(lastCommandedVolts, -kNominalVoltage, kNominalVoltage);
        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(kDtSeconds);

        positionRadians   = motorSim.getAngularPositionRad();
        velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();

        double maxVelocityRadPerSec = maximumVelocityRadiansPerSecondSupplier.getAsDouble();
        if (Double.isFinite(maxVelocityRadPerSec) && Math.abs(velocityRadPerSec) > maxVelocityRadPerSec) {
            velocityRadPerSec = Math.copySign(maxVelocityRadPerSec, velocityRadPerSec);
            motorSim.setState(VecBuilder.fill(positionRadians, velocityRadPerSec));
        }

        // Enforce motion bounds to mirror the hardware wrapper behavior.
        if (shouldEnforceSetpointLimits()) {
            double minPosition = minimumPositionRadiansSupplier.getAsDouble();
            double maxPosition = maximumPositionRadiansSupplier.getAsDouble();

            if (positionRadians < minPosition) {
                positionRadians   = minPosition;
                velocityRadPerSec = 0.0;
                motorSim.setState(VecBuilder.fill(positionRadians, velocityRadPerSec));
            } else if (positionRadians > maxPosition) {
                positionRadians   = maxPosition;
                velocityRadPerSec = 0.0;
                motorSim.setState(VecBuilder.fill(positionRadians, velocityRadPerSec));
            }
        }

        log.recordOutput("positionRads", positionRadians);
        log.recordOutput("velocityRadPerSec", velocityRadPerSec);
    }

    private void rebuildMotorSimIfNeeded(double maxAccelerationRadPerSecSq) {
        if (!Double.isFinite(maxAccelerationRadPerSecSq) || maxAccelerationRadPerSecSq <= kMinimumAccelerationRadPerSecSq) {
            maxAccelerationRadPerSecSq = kMinimumAccelerationRadPerSecSq;
        }

        if (Math.abs(maxAccelerationRadPerSecSq - lastMaxAccelerationRadPerSecSq) <= kEpsilon) {
            return;
        }

        rebuildMotorSim(maxAccelerationRadPerSecSq);
        motorSim.setState(VecBuilder.fill(positionRadians, velocityRadPerSec));
    }

    private void rebuildMotorSim(double maxAccelerationRadPerSecSq) {
        double momentOfInertiaKgMetersSq = computeMomentOfInertia(maxAccelerationRadPerSecSq);
        motorSim                       = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, momentOfInertiaKgMetersSq, gearRatio),
                motorModel);
        lastMaxAccelerationRadPerSecSq = maxAccelerationRadPerSecSq;
        log.recordOutput("momentOfInertiaKgMetersSq", momentOfInertiaKgMetersSq);
        log.dashboard(name + "/simMotorModel", motorModel.getClass().getSimpleName());
    }

    private double computeMomentOfInertia(double maxAccelerationRadPerSecSq) {
        if (!Double.isFinite(maxAccelerationRadPerSecSq) || maxAccelerationRadPerSecSq <= kMinimumAccelerationRadPerSecSq) {
            return kDefaultMomentOfInertiaKgMetersSq;
        }

        double outputStallTorque = motorModel.stallTorqueNewtonMeters * gearRatio;
        double moment            = outputStallTorque / maxAccelerationRadPerSecSq;

        if (!Double.isFinite(moment)) {
            return kDefaultMomentOfInertiaKgMetersSq;
        }

        return Math.max(moment, kMinimumMomentOfInertiaKgMetersSq);
    }

    private double getPositionRadiansUnconverted() {
        if (!shouldEnforceSetpointLimits()) {
            return positionRadians;
        }
        return clamp(positionRadians, minimumPositionRadiansSupplier.getAsDouble(), maximumPositionRadiansSupplier.getAsDouble());
    }

    private double getVelocityRadiansPerSecondUnconverted() {
        return velocityRadPerSec;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean shouldEnforceSetpointLimits() {
        if (useSetpointLimitsSupplier == null) {
            return true;
        }
        return Boolean.TRUE.equals(useSetpointLimitsSupplier.get());
    }
}