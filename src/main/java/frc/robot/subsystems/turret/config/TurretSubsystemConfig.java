package frc.robot.subsystems.turret.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;
import frc.robot.shared.field.FieldTargetConfig;

/**
 * Configuration bundle for the turret mechanism. Values are stored in degrees for readability but converted to radians at runtime where needed.
 */
public class TurretSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the turret mechanism. */
    public TurretMotorConfig turretMotorConfig = new TurretMotorConfig();

    /** 3D pivot offset for AdvantageScope component visualization. */
    public ComponentPoseConfig componentPoseConfig = new ComponentPoseConfig();

    /** Field target positions and zone boundaries for zone-aware turret tracking. */
    public FieldTargetConfig fieldTargets = new FieldTargetConfig();

    /**
     * Zero offset between the turret's mechanical zero and robot-forward in degrees.
     * <p>
     * A value of 180 means the turret faces the rear of the robot when at its mechanical zero position.
     * </p>
     */
    public double turretZeroOffsetDegrees = 180.0;

    /**
     * Look-ahead time for rotational velocity compensation in seconds.
     * <p>
     * When the robot is spinning, the turret can lead its aim by this many seconds of predicted heading
     * change to stay on target. Set to zero to disable compensation.
     * </p>
     */
    public double rotationalLeadTimeSeconds = 0.1;

    /**
     * Returns the turret zero offset in degrees.
     * <p>
     * Positive values rotate the turret setpoint counter-clockwise relative to robot-forward.
     * </p>
     *
     * @return turret zero offset in degrees
     */
    public double getTurretZeroOffsetDegrees() {
        return readTunableDegrees("turretZeroOffsetDegrees", turretZeroOffsetDegrees);
    }

    /**
     * Returns the rotational lead time used for yaw-rate compensation.
     * <p>
     * The turret multiplies the robot's yaw rate by this value to predict how far the heading will change
     * and pre-rotates accordingly. Tune this higher if the turret lags behind while the robot spins.
     * </p>
     *
     * @return lead time in seconds
     */
    public double getRotationalLeadTimeSeconds() {
        return readTunableNumber("rotationalLeadTimeSeconds", rotationalLeadTimeSeconds);
    }

}
