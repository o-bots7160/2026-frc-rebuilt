package frc.robot.shared.config;

import frc.robot.shared.bindings.TriggerBindingsConfig;
import frc.robot.subsystems.apriltagvision.config.AprilTagVisionSubsystemConfig;
import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.feeder.config.FeederSubsystemConfig;
import frc.robot.subsystems.gameplaystate.config.GameplayStateSubsystemConfig;
import frc.robot.subsystems.harvester.config.HarvesterSubsystemConfig;
import frc.robot.subsystems.indexer.config.IndexerSubsystemConfig;
import frc.robot.subsystems.intake.config.IntakeSubsystemConfig;
import frc.robot.subsystems.robotpose.config.RobotPoseSubsystemConfig;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;

/**
 * Root configuration bundle for every subsystem. Individual subsystems can be toggled or tuned via this object after loading JSON from the deploy
 * directory.
 */
public class SubsystemsConfig {

    /**
     * Trigger bindings configuration bundle for driver controller sensitivity and tuning mode.
     */
    public TriggerBindingsConfig         triggerBindings          = new TriggerBindingsConfig();

    /**
     * Drive base configuration bundle.
     */
    public DriveBaseSubsystemConfig      driveBaseSubsystem       = new DriveBaseSubsystemConfig();

    /**
     * Intake configuration bundle.
     */
    public IntakeSubsystemConfig         intakeSubsystem          = new IntakeSubsystemConfig();

    /**
     * Harvester configuration bundle.
     */
    public HarvesterSubsystemConfig      harvesterSubsystem       = new HarvesterSubsystemConfig();

    /**
     * Feeder configuration bundle.
     */
    public FeederSubsystemConfig         feederSubsystem          = new FeederSubsystemConfig();

    /**
     * Indexer configuration bundle.
     */
    public IndexerSubsystemConfig        indexerSubsystem         = new IndexerSubsystemConfig();

    /**
     * Turret configuration bundle.
     */
    public TurretSubsystemConfig         turretSubsystem          = new TurretSubsystemConfig();

    /**
     * Shooter configuration bundle.
     */
    public ShooterSubsystemConfig        shooterSubsystem         = new ShooterSubsystemConfig();

    /**
     * AprilTag vision configuration bundle.
     */
    public AprilTagVisionSubsystemConfig aprilTagVisionSubsystem  = new AprilTagVisionSubsystemConfig();

    /**
     * Robot pose configuration bundle.
     */
    public RobotPoseSubsystemConfig      robotPoseSubsystem       = new RobotPoseSubsystemConfig();

    /**
     * Gameplay state configuration bundle for state management and auto-transitions.
     */
    public GameplayStateSubsystemConfig  gameplayStateSubsystem   = new GameplayStateSubsystemConfig();
}
