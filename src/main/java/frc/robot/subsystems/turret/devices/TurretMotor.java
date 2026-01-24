package frc.robot.subsystems.turret.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.AbstractMotor;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;

/**
 * SparkMax-backed turret motor with baked-in limits, gearing, and telemetry.
 * <p>
 * Inputs are configured in degrees for approachability; this wrapper converts to radians for WPILib math and AdvantageKit logging via
 * {@link AbstractMotor}. Provide the gear ratio as motor rotations per one turret rotation (e.g., 100:1 reduction = 100.0).
 * </p>
 */
public class TurretMotor extends AbstractMotor {

    private static double computeMechanismRadiansPerMotorRotation(double motorRotationsPerMechanismRotation) {
        if (motorRotationsPerMechanismRotation <= 0.0) {
            return Units.rotationsToRadians(1.0); // Safe fallback: 1:1
        }
        return Units.rotationsToRadians(1.0 / motorRotationsPerMechanismRotation);
    }

    private final TurretSubsystemConfig config;

    /**
     * Builds a turret motor wrapper using values from the turret config.
     *
     * @param config turret configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     */
    public TurretMotor(TurretSubsystemConfig config) {
        super(
                "TurretMotor",
                config.getMotorCanIdSupplier().get(),
                Units.degreesToRadians(config.getMinimumSetpointSupplier().get()),
                Units.degreesToRadians(config.getMaximumSetpointSupplier().get()),
                computeMechanismRadiansPerMotorRotation(config.getMotorRotationsPerMechanismRotationSupplier().get()));
        this.config = config;
    }

    @Override
    public double getEncoderPosition() {
        return Units.radiansToDegrees(super.getEncoderPosition());
    }

    @Override
    public double getEncoderVelocity() {
        return Units.radiansToDegrees(super.getEncoderVelocity());
    }

    @Override
    public double getMaximumTargetPosition() {
        return config.getMaximumSetpointSupplier().get();
    }

    @Override
    public double getMinimumTargetPosition() {
        return config.getMinimumSetpointSupplier().get();
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
                // .inverted(config.getMotorInvertedSupplier().get())
                // .smartCurrentLimit(config.getSmartCurrentLimitSupplier().get())
                .inverted(false)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);

        double positionFactor = Units.rotationsToRadians(1.0)
                / 72;
                // / config.getMotorRotationsPerMechanismRotationSupplier().get();
        double velocityFactor = positionFactor / 60.0;

        // Apply encoder scaling so SparkMax position/velocity already report mechanism radians.
        sparkConfig.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor);

        return sparkConfig;
    }
}
