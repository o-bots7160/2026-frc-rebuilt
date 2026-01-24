package frc.robot.shared.config;

import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.vision.config.AprilTagVisionSubsystemConfig;
import frc.robot.subsystems.vision.config.DriverCameraSubsystemConfig;

/**
 * Root configuration bundle for every subsystem. Individual subsystems can be toggled or tuned via this object after loading JSON from the deploy
 * directory.
 */
public class SubsystemsConfig {

    public DriveBaseSubsystemConfig driveBaseSubsystem = new DriveBaseSubsystemConfig();

    public TurretSubsystemConfig    turretSubsystem    = new TurretSubsystemConfig();

    public AprilTagVisionSubsystemConfig aprilTagVisionSubsystem = new AprilTagVisionSubsystemConfig();

    public DriverCameraSubsystemConfig driverCameraSubsystem = new DriverCameraSubsystemConfig();
}
