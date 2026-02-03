package frc.robot.subsystems.turret.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
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

    private static final double CONFIG_EPSILON = 1e-6;

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

    private int                     lastMotorCanId;

    private boolean                 lastMotorInverted;

    private int                     lastSmartCurrentLimitAmps;

    private double                  lastReverseSoftLimitDegrees;

    private double                  lastForwardSoftLimitDegrees;

    private double                  lastMotorRotationsPerMechanismRotation;

    /**
     * Builds a turret motor wrapper using values from the turret motor config.
     *
     * @param config turret motor configuration containing CAN ID, gear ratio, inversion, and motion bounds (degrees)
     */
    private TurretMotor(TurretMotorConfig config) {
        super("TurretMotor", config);
        this.config = config;
        cacheConfigSnapshot();
    }

    /**
     * Reapplies SparkMax configuration when tunable values change in Elastic.
     * <p>
    * Call this periodically (for example, from the subsystem {@link frc.robot.subsystems.turret.TurretSubsystem#periodic()} hook) so updates to inversion, current limit, gear ratio, or
     * setpoint bounds take effect without restarting the robot. Changes to the CAN ID are logged but require a reboot to take effect.
     * </p>
     */
    public void refreshConfiguration() {
        if (!isInitialized()) {
            return;
        }

        int     motorCanId                         = config.getMotorCanId();
        boolean motorInverted                      = config.getMotorInverted();
        int     smartCurrentLimit                  = config.getSmartCurrentLimitAmps();
        double  reverseSoftLimitDegrees            = config.getReverseSoftLimitDegrees();
        double  forwardSoftLimitDegrees            = config.getForwardSoftLimitDegrees();
        double  motorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotation();

        boolean configChanged                      = motorCanId != lastMotorCanId
                || motorInverted != lastMotorInverted
                || smartCurrentLimit != lastSmartCurrentLimitAmps
                || hasChanged(reverseSoftLimitDegrees, lastReverseSoftLimitDegrees)
                || hasChanged(forwardSoftLimitDegrees, lastForwardSoftLimitDegrees)
                || hasChanged(motorRotationsPerMechanismRotation, lastMotorRotationsPerMechanismRotation);

        if (!configChanged) {
            return;
        }

        if (motorCanId != lastMotorCanId) {
            log.warning("TurretMotor CAN ID changed from " + lastMotorCanId + " to " + motorCanId
                    + "; restart required to recreate the controller.");
        }

        updateMotionConstraints(
            Units.degreesToRadians(reverseSoftLimitDegrees),
            Units.degreesToRadians(forwardSoftLimitDegrees),
                computeMechanismRadiansPerMotorRotation(motorRotationsPerMechanismRotation));
        reconfigure();
        cacheConfigSnapshot();
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig sparkConfig) {
        sparkConfig
            .inverted(config.getMotorInverted())
            .smartCurrentLimit(config.getSmartCurrentLimitAmps())
                .idleMode(IdleMode.kBrake)
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
        lastReverseSoftLimitDegrees            = config.getReverseSoftLimitDegrees();
        lastForwardSoftLimitDegrees            = config.getForwardSoftLimitDegrees();
        lastMotorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotation();
    }

    private boolean hasChanged(double currentValue, double lastValue) {
        return Math.abs(currentValue - lastValue) > CONFIG_EPSILON;
    }
}
