package frc.robot.subsystems.turret.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.devices.motor.AbstractMotor;
import frc.robot.subsystems.turret.config.TurretMotorConfig;

/**
 * SparkMax-backed turret motor with baked-in limits, gearing, and telemetry.
 * <p>
 * Inputs are configured in degrees for approachability; this wrapper converts to radians for WPILib math and AdvantageKit logging via
 * {@link AbstractMotor}. Provide the gear ratio as motor rotations per one turret rotation (e.g., 100:1 reduction = 100.0).
 * </p>
 */
public class TurretMotor extends AbstractMotor {

    /**
     * Creates a turret motor wrapper using values from the turret motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config turret motor configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     * @return configured turret motor wrapper
     */
    public static TurretMotor create(TurretMotorConfig config) {
        TurretMotor motor = new TurretMotor(config);
        motor.init();
        return motor;
    }

    private final TurretMotorConfig config;

    /**
     * Builds a turret motor wrapper using values from the turret motor config.
     *
     * @param config turret motor configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     */
    private TurretMotor(TurretMotorConfig config) {
        super("TurretMotor", config);
        this.config = config;
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
            .inverted(config.getMotorInverted())
            .smartCurrentLimit(config.getSmartCurrentLimitAmps())
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);

        // Encoder conversion factors are applied in Java (AbstractMotor.getPositionRadians /
        // getVelocityRadiansPerSecond) rather than through SparkMax firmware for reliability
        // across REVLib versions. The encoder reports raw motor rotations and RPM.

        return sparkConfig;
    }

}
