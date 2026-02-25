package frc.robot.subsystems.harvester.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Configuration bundle for the harvester arm mechanism. Values are stored in degrees for readability but converted to radians at runtime where
 * needed.
 * <p>
 * The harvester arm swings the intake rollers between an upright stowed position (inside the robot perimeter) and a lowered deployed position
 * (outside the perimeter to grab Fuel from the floor). Named position fields provide the default stowed and deployed angles so commands do not
 * hard-code magic numbers.
 * </p>
 */
public class HarvesterSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the harvester arm motor. */
    public HarvesterMotorConfig harvesterMotorConfig = new HarvesterMotorConfig();

    /** Arm angle when stowed upright inside the robot perimeter, in degrees. This is the match-start and default position. */
    public double               stowedPositionDegrees;

    /** Arm angle when deployed downward outside the robot perimeter to collect Fuel, in degrees. */
    public double               deployedPositionDegrees;

    /**
     * Returns the stowed arm position, tuned via SmartDashboard.
     *
     * @return stowed position in degrees (arm upright inside the frame)
     */
    public double getStowedPositionDegrees() {
        return readTunableDegrees("stowedPositionDegrees", stowedPositionDegrees);
    }

    /**
     * Returns the deployed arm position, tuned via SmartDashboard.
     *
     * @return deployed position in degrees (arm lowered outside the frame to collect Fuel)
     */
    public double getDeployedPositionDegrees() {
        return readTunableDegrees("deployedPositionDegrees", deployedPositionDegrees);
    }
}
