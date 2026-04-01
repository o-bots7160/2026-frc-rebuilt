package frc.robot.devices.motor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * SparkMax-backed motor for velocity-controlled mechanisms (flywheels, rollers, belts) with baked-in gearing and coast-mode idle.
 * <p>
 * All positions and velocities are reported in mechanism radians and radians per second after applying the configured gear ratio. The motor idles in
 * coast mode so it does not fight deceleration when the mechanism is stopped. Concrete subclasses only need to supply their specific motor config and
 * a friendly name for logging.
 * </p>
 *
 * @param <TConfig> concrete motor config type
 */
public abstract class AbstractVelocityMotor<TConfig extends AbstractMotorConfig> extends AbstractMotor {

    private final TConfig config;

    /**
     * Builds a velocity motor wrapper using the supplied configuration.
     *
     * @param name   friendly name used for logging and dashboard keys
     * @param config motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    protected AbstractVelocityMotor(String name, TConfig config) {
        super(name, config);
        this.config = config;
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
                .inverted(config.getMotorInverted())
                .smartCurrentLimit(config.getSmartCurrentLimitAmps())
                .idleMode(IdleMode.kCoast);

        // Encoder conversion factors are applied in Java (AbstractMotor.getPositionRadians /
        // getVelocityRadiansPerSecond) rather than through SparkMax firmware for reliability
        // across REVLib versions. The encoder reports raw motor rotations and RPM.

        return sparkConfig;
    }
}
