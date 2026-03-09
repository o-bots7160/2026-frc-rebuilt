package frc.robot.subsystems.harvester.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.devices.motor.AbstractMotor;
import frc.robot.subsystems.harvester.config.HarvesterMotorConfig;

/**
 * SparkMax-backed harvester arm motor with baked-in limits, gearing, and telemetry.
 * <p>
 * Inputs are configured in degrees for approachability; this wrapper converts to radians for WPILib math and AdvantageKit logging via
 * {@link AbstractMotor}. Provide the gear ratio as motor rotations per one arm rotation (e.g., 100:1 reduction = 100.0).
 * </p>
 */
public class HarvesterMotor extends AbstractMotor {

    /**
     * Creates a harvester motor wrapper using values from the harvester motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config harvester motor configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     * @return configured harvester motor wrapper
     */
    public static HarvesterMotor create(HarvesterMotorConfig config) {
        HarvesterMotor motor = new HarvesterMotor(config);
        motor.init();
        return motor;
    }

    private final HarvesterMotorConfig config;

    /**
     * Builds a harvester motor wrapper using values from the harvester motor config.
     *
     * @param config harvester motor configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     */
    private HarvesterMotor(HarvesterMotorConfig config) {
        super("HarvesterMotor", config);
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
