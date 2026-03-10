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
     * True when the turret barrel faces the rear of the robot.
     * <p>
     * A rear-facing turret mirrors left and right relative to robot-forward, so the robot-relative target
     * angle must be negated before the zero offset is applied. Set to false when the turret faces forward.
     * </p>
     */
    public boolean rearFacingTurret = true;

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
     * This offset is added to the robot-relative target angle (which may be negated when
     * {@link #isRearFacingTurret()} is true). A value of 180 means turret mechanical zero points to the
     * rear of the robot.
     * </p>
     *
     * @return turret zero offset in degrees
     */
    public double getTurretZeroOffsetDegrees() {
        return readTunableDegrees("turretZeroOffsetDegrees", turretZeroOffsetDegrees);
    }

    /**
     * Returns whether the turret barrel faces the rear of the robot.
     * <p>
     * When true, the robot-relative target angle is negated in the tracking calculation to correct the
     * mirrored left-right axis. When false, the standard forward-facing formula is used.
     * </p>
     *
     * @return true when the turret faces backward
     */
    public boolean isRearFacingTurret() {
        return readTunableBoolean("rearFacingTurret", rearFacingTurret);
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
