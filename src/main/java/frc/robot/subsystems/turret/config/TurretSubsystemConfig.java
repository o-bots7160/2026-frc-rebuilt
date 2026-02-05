package frc.robot.subsystems.turret.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Configuration bundle for the turret mechanism. Values are stored in degrees for readability but converted to radians at runtime where needed.
 */
public class TurretSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the turret mechanism. */
    public TurretMotorConfig turretMotorConfig = new TurretMotorConfig();

    /**
     * Zero offset between the turret's mechanical zero and robot-forward in degrees.
     */
    public double turretZeroOffsetDegrees = 0.0;

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

}
