package frc.robot.subsystems.hopper.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.devices.motor.AbstractMotor;
import frc.robot.subsystems.hopper.config.HopperMotorConfig;

/**
 * SparkMax-backed hopper belt motor with baked-in limits, gearing, and telemetry.
 * <p>
 * All positions and velocities are reported in mechanism (belt) radians and radians per second after applying the configured gear ratio. The belt
 * uses coast mode so it does not fight deceleration when the hopper is idle.
 * </p>
 */
public class HopperMotor extends AbstractMotor {

    private static final double CONFIG_EPSILON = 1e-6;

    /**
     * Creates a hopper motor wrapper using values from the hopper motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config hopper motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured hopper motor wrapper
     */
    public static HopperMotor create(HopperMotorConfig config) {
        HopperMotor motor = new HopperMotor(config);
        motor.init();
        return motor;
    }

    private final HopperMotorConfig config;

    private int                     lastMotorCanId;

    private boolean                 lastMotorInverted;

    private int                     lastSmartCurrentLimitAmps;

    private double                  lastMotorRotationsPerMechanismRotation;

    /**
     * Builds a hopper motor wrapper using values from the hopper motor config.
     *
     * @param config hopper motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private HopperMotor(HopperMotorConfig config) {
        super("HopperMotor", config);
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
            log.warning("HopperMotor CAN ID changed from " + lastMotorCanId + " to " + motorCanId
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
