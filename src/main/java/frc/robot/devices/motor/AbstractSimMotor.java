package frc.robot.devices.motor;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Abstract simulation-only motor wrapper that mirrors {@link AbstractMotor} behavior while using REV simulation helpers.
 * <p>
 * This class pairs a {@link SparkMaxSim} with a {@link DCMotorSim} so the simulated Spark MAX follows the same control modes, soft limits, and
 * conversion factors as the real robot. Motion is modeled in radians and seconds to match the rest of the robot math.
 * </p>
 */
public abstract class AbstractSimMotor extends AbstractMotor {

    private static final double       kDtSeconds                        = 0.02;

    private static final double       kMinimumAccelerationRadPerSecSq   = 1e-3;

    private static final double       kMinimumMomentOfInertiaKgMetersSq = 1e-6;

    private static final double       kDefaultMomentOfInertiaKgMetersSq = 0.001;

    private static final double       kEpsilon                          = 1e-6;

    private final AbstractMotorConfig config;

    private final SparkMaxSim         sparkMaxSim;

    private final DCMotor             motorModel;

    private DCMotorSim                motorSim;

    private final DoubleSupplier      maximumVelocityRadiansPerSecondSupplier;

    private final DoubleSupplier      maximumAccelerationRadiansPerSecondSquaredSupplier;

    private final Supplier<Boolean>   useSetpointLimitsSupplier;

    private final double              gearRatio;

    private double                    lastMaxAccelerationRadPerSecSq    = Double.NaN;

    /**
     * Creates a simulated motor using the default NEO motor model.
     * <p>
     * Use this when your mechanism is driven by a single NEO motor and you want the simulated Spark MAX to follow the same configuration and limits
     * as the real robot.
     * </p>
     *
     * @param name                                          friendly name used for logging
     * @param config                                        motor configuration bundle for limits, gear ratio, and inversion
     * @param maximumVelocityDegreesPerSecondSupplier       supplier yielding the max mechanism velocity (degrees per second)
     * @param maximumAccelerationDegreesPerSecondSqSupplier supplier yielding the max mechanism acceleration (degrees per second squared)
     */
    protected AbstractSimMotor(
            String name,
            AbstractMotorConfig config,
            Supplier<Double> maximumVelocityDegreesPerSecondSupplier,
            Supplier<Double> maximumAccelerationDegreesPerSecondSqSupplier) {
        this(
                name,
                config,
                maximumVelocityDegreesPerSecondSupplier,
                maximumAccelerationDegreesPerSecondSqSupplier,
                DCMotor.getNEO(1));
    }

    /**
     * Creates a simulated motor using an explicit motor model for improved realism.
     * <p>
     * Use this when you know the exact motor type (for example, NEO 550) or the number of motors in the gearbox.
     * </p>
     *
     * @param name                                          friendly name used for logging
     * @param config                                        motor configuration bundle for limits, gear ratio, and inversion
     * @param maximumVelocityDegreesPerSecondSupplier       supplier yielding the max mechanism velocity (degrees per second)
     * @param maximumAccelerationDegreesPerSecondSqSupplier supplier yielding the max mechanism acceleration (degrees per second squared)
     * @param motorModel                                    motor model (for example, {@link DCMotor#getNEO(int)})
     */
    protected AbstractSimMotor(
            String name,
            AbstractMotorConfig config,
            Supplier<Double> maximumVelocityDegreesPerSecondSupplier,
            Supplier<Double> maximumAccelerationDegreesPerSecondSqSupplier,
            DCMotor motorModel) {
        super(name, config);
        this.config                                             = config;
        this.motorModel                                         = motorModel;
        this.maximumVelocityRadiansPerSecondSupplier            = () -> Units.degreesToRadians(maximumVelocityDegreesPerSecondSupplier.get());
        this.maximumAccelerationRadiansPerSecondSquaredSupplier = () -> Units.degreesToRadians(maximumAccelerationDegreesPerSecondSqSupplier.get());
        this.useSetpointLimitsSupplier                          = config.getUseSetpointLimitsSupplier();

        double configuredRatio = config.getMotorRotationsPerMechanismRotationSupplier().get();
        this.gearRatio = configuredRatio > 0.0 ? configuredRatio : 1.0;

        sparkMaxSim    = new SparkMaxSim(motor, motorModel);
        seedInitialState();
        rebuildMotorSim(maximumAccelerationRadiansPerSecondSquaredSupplier.getAsDouble());

        init();
        log.dashboard("simMotorModel", motorModel.getClass().getSimpleName());
        log.recordOutput("gearRatio", gearRatio);
    }

    /**
     * Records a position setpoint for telemetry and clamps it to the configured bounds.
     *
     * @param targetPositionRadians desired mechanism position (radians)
     * @return clamped position setpoint in radians
     */
    @Override
    public double recordPositionSetpointRadians(double targetPositionRadians) {
        return super.recordPositionSetpointRadians(targetPositionRadians);
    }

    /**
     * Records a velocity setpoint for telemetry.
     *
     * @param targetVelocityRadPerSec desired mechanism velocity (radians per second)
     */
    @Override
    public void recordVelocitySetpointRadians(double targetVelocityRadPerSec) {
        super.recordVelocitySetpointRadians(targetVelocityRadPerSec);
    }

    /**
     * Reports the applied voltage request currently sent to the simulated Spark MAX.
     *
     * @return voltage applied to the motor controller
     */
    @Override
    public Voltage getVoltage() {
        return edu.wpi.first.units.Units.Volts.of(getAppliedVolts());
    }

    /**
     * Updates the physics model and then exports sensor values from the simulated Spark MAX.
     *
     * @param inputs mutable inputs container to fill for logging
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        stepSimulation();
        super.updateInputs(inputs);
    }

    /**
     * Applies the base configuration for the simulated Spark MAX.
     * <p>
     * This mirrors the inversion, current limits, and voltage compensation used on the real controller so the simulation follows the same control
     * assumptions.
     * </p>
     *
     * @param sparkConfig base SparkMax config to mutate
     * @return updated SparkMax config ready for {@link com.revrobotics.spark.SparkMax#configure}
     */
    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
                .inverted(config.getMotorInvertedSupplier().get())
                .smartCurrentLimit(config.getSmartCurrentLimitSupplier().get())
                .voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        return sparkConfig;
    }

    private void seedInitialState() {
        double initialPositionRad = 0.0;
        if (Boolean.TRUE.equals(useSetpointLimitsSupplier.get())) {
            double minPosition = config.getMinimumSetpointRadiansSupplier().get();
            double maxPosition = config.getMaximumSetpointRadiansSupplier().get();
            initialPositionRad = MathUtil.clamp(0.0, minPosition, maxPosition);
        }

        sparkMaxSim.setPosition(initialPositionRad);
        sparkMaxSim.setVelocity(0.0);
    }

    private void stepSimulation() {
        double maxAccelRadPerSecSq = maximumAccelerationRadiansPerSecondSquaredSupplier.getAsDouble();
        rebuildMotorSimIfNeeded(maxAccelRadPerSecSq);

        double busVoltage   = RoboRioSim.getVInVoltage();
        double appliedVolts = sparkMaxSim.getAppliedOutput() * busVoltage;

        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(kDtSeconds);

        double mechanismVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        double maxVelocityRadPerSec       = maximumVelocityRadiansPerSecondSupplier.getAsDouble();
        if (Double.isFinite(maxVelocityRadPerSec) && Math.abs(mechanismVelocityRadPerSec) > maxVelocityRadPerSec) {
            mechanismVelocityRadPerSec = Math.copySign(maxVelocityRadPerSec, mechanismVelocityRadPerSec);
            motorSim.setState(VecBuilder.fill(motorSim.getAngularPositionRad(), mechanismVelocityRadPerSec));
        }

        sparkMaxSim.iterate(mechanismVelocityRadPerSec, busVoltage, kDtSeconds);

        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(motorSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(loadedVoltage);

        log.recordOutput("simAppliedVolts", appliedVolts);
        log.recordOutput("simVelocityRadPerSec", mechanismVelocityRadPerSec);
    }

    private void rebuildMotorSimIfNeeded(double maxAccelerationRadPerSecSq) {
        if (!Double.isFinite(maxAccelerationRadPerSecSq) || maxAccelerationRadPerSecSq <= kMinimumAccelerationRadPerSecSq) {
            maxAccelerationRadPerSecSq = kMinimumAccelerationRadPerSecSq;
        }

        if (Math.abs(maxAccelerationRadPerSecSq - lastMaxAccelerationRadPerSecSq) <= kEpsilon) {
            return;
        }

        rebuildMotorSim(maxAccelerationRadPerSecSq);

        double positionRad       = sparkMaxSim.getPosition();
        double velocityRadPerSec = sparkMaxSim.getVelocity();
        motorSim.setState(VecBuilder.fill(positionRad, velocityRadPerSec));
    }

    private void rebuildMotorSim(double maxAccelerationRadPerSecSq) {
        double momentOfInertiaKgMetersSq = computeMomentOfInertia(maxAccelerationRadPerSecSq);
        motorSim                       = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, momentOfInertiaKgMetersSq, gearRatio),
                motorModel);
        lastMaxAccelerationRadPerSecSq = maxAccelerationRadPerSecSq;
        log.recordOutput("momentOfInertiaKgMetersSq", momentOfInertiaKgMetersSq);
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
}