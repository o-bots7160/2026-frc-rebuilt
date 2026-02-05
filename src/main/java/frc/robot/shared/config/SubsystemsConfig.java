package frc.robot.shared.config;

import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.robotstate.config.RobotStateSubsystemConfig;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.vision.config.AprilTagVisionSubsystemConfig;
import frc.robot.subsystems.vision.config.DriverCameraSubsystemConfig;

/**
 * Root configuration bundle for every subsystem. Individual subsystems can be toggled or tuned via this object after loading JSON from the deploy
 * directory.
 */
public class SubsystemsConfig {

    /**
     * Drive base configuration bundle.
     */
    public DriveBaseSubsystemConfig driveBaseSubsystem = new DriveBaseSubsystemConfig();

    /**
     * Turret configuration bundle.
     */
    public TurretSubsystemConfig    turretSubsystem    = new TurretSubsystemConfig();

    /**
     * AprilTag vision configuration bundle.
     */
    public AprilTagVisionSubsystemConfig aprilTagVisionSubsystem = new AprilTagVisionSubsystemConfig();

    /**
     * Driver camera configuration bundle.
     */
    public DriverCameraSubsystemConfig driverCameraSubsystem = new DriverCameraSubsystemConfig();

    /**
     * Robot state configuration bundle.
     */
    public RobotStateSubsystemConfig robotStateSubsystem = new RobotStateSubsystemConfig();
}
