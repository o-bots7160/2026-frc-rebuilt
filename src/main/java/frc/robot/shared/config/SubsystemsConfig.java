package frc.robot.shared.config;

import frc.robot.shared.bindings.TriggerBindingsConfig;
import frc.robot.subsystems.apriltagvision.config.AprilTagVisionSubsystemConfig;
import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.drivercameravision.config.DriverCameraSubsystemConfig;
import frc.robot.subsystems.robotstate.config.RobotStateSubsystemConfig;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;

/**
 * Root configuration bundle for every subsystem. Individual subsystems can be toggled or tuned via this object after loading JSON from the deploy
 * directory.
 */
public class SubsystemsConfig {

    /**
     * Drive base configuration bundle.
     */
    public DriveBaseSubsystemConfig      driveBaseSubsystem      = new DriveBaseSubsystemConfig();

    /**
     * Turret configuration bundle.
     */
    public TurretSubsystemConfig         turretSubsystem         = new TurretSubsystemConfig();

    /**
     * Shooter configuration bundle.
     */
    public ShooterSubsystemConfig        shooterSubsystem        = new ShooterSubsystemConfig();

    /**
     * AprilTag vision configuration bundle.
     */
    public AprilTagVisionSubsystemConfig aprilTagVisionSubsystem = new AprilTagVisionSubsystemConfig();

    /**
     * Driver camera configuration bundle.
     */
    public DriverCameraSubsystemConfig   driverCameraSubsystem   = new DriverCameraSubsystemConfig();

    /**
     * Robot state configuration bundle.
     */
    public RobotStateSubsystemConfig     robotStateSubsystem     = new RobotStateSubsystemConfig();

    /**
     * Trigger bindings configuration bundle for driver controller sensitivity.
     */
    public TriggerBindingsConfig         triggerBindings         = new TriggerBindingsConfig();
}
