package frc.robot.devices.motor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * SparkMax-backed motor for velocity-controlled mechanisms (flywheels, rollers, belts) with baked-in gearing, coast-mode idle, and live-tunable
 * configuration.
 * <p>
 * All positions and velocities are reported in mechanism radians and radians per second after applying the configured gear ratio. The motor idles in
 * coast mode so it does not fight deceleration when the mechanism is stopped. Concrete subclasses only need to supply their specific motor config and
 * a friendly name for logging.
 * </p>
 *
 * @param <TConfig> concrete motor config type
 */
public abstract class AbstractVelocityMotor<TConfig extends AbstractMotorConfig> extends AbstractMotor {

    private static final double CONFIG_EPSILON = 1e-6;

    private final TConfig       config;

    private int                 lastMotorCanId;

    private boolean             lastMotorInverted;

    private int                 lastSmartCurrentLimitAmps;

    private double              lastMotorRotationsPerMechanismRotation;

    /**
     * Builds a velocity motor wrapper using the supplied configuration.
     *
     * @param name   friendly name used for logging and dashboard keys
     * @param config motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    protected AbstractVelocityMotor(String name, TConfig config) {
        super(name, config);
        this.config = config;
        cacheConfigSnapshot();
    }

    /**
     * Reapplies SparkMax configuration when tunable values change.
     * <p>
     * Call this periodically from the subsystem so updates to inversion, current limit, or gear ratio take effect without restarting the robot.
     * </p>
     */
    @Override
    public void refreshConfiguration() {
        if (!isInitialized()) {
            return;
        }

        int     motorCanId                         = config.getMotorCanId();
        boolean motorInverted                      = config.getMotorInverted();
        int     smartCurrentLimit                  = config.getSmartCurrentLimitAmps();
        double  motorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotation();

        boolean configChanged                      = motorCanId != lastMotorCanId
                || motorInverted != lastMotorInverted
                || smartCurrentLimit != lastSmartCurrentLimitAmps
                || hasChanged(motorRotationsPerMechanismRotation, lastMotorRotationsPerMechanismRotation);

        if (!configChanged) {
            return;
        }

        if (motorCanId != lastMotorCanId) {
            log.warning(name + " CAN ID changed from " + lastMotorCanId + " to " + motorCanId
                    + "; restart required to recreate the controller.");
        }

        updateMotionConstraints(
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                computeMechanismRadiansPerMotorRotation(motorRotationsPerMechanismRotation));
        reconfigure();
        cacheConfigSnapshot();
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
                .inverted(config.getMotorInverted())
                .smartCurrentLimit(config.getSmartCurrentLimitAmps())
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);

        double motorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotation();
        double positionFactor                     = computeMechanismRadiansPerMotorRotation(motorRotationsPerMechanismRotation);
        double velocityFactor                     = positionFactor / 60.0;

        // Apply encoder scaling so SparkMax position/velocity already report mechanism radians.
        sparkConfig.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor);

        return sparkConfig;
    }

    private void cacheConfigSnapshot() {
        lastMotorCanId                         = config.getMotorCanId();
        lastMotorInverted                      = config.getMotorInverted();
        lastSmartCurrentLimitAmps              = config.getSmartCurrentLimitAmps();
        lastMotorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotation();
    }

    private boolean hasChanged(double currentValue, double lastValue) {
        return Math.abs(currentValue - lastValue) > CONFIG_EPSILON;
    }
}
