package frc.robot.config;

/**
 * Root configuration bundle for every subsystem. Individual subsystems can be toggled or tuned via this object after loading JSON from the deploy
 * directory.
 */
public class SubsystemsConfig {

    public DriveBaseSubsystemConfig driveBaseSubsystem = new DriveBaseSubsystemConfig();

    public TurretSubsystemConfig    turretSubsystem    = new TurretSubsystemConfig();
}
