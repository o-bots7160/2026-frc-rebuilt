// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.bindings.TriggerBindings;
import frc.robot.shared.config.ConfigurationLoader;
import frc.robot.shared.config.FieldLayoutConfig;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.config.SubsystemsConfig;
import frc.robot.shared.field.FieldTargetSelector;
import frc.robot.subsystems.apriltagvision.AprilTagVisionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.PathPlannerCommandFactory;
import frc.robot.subsystems.drivercameravision.DriverCameraSubsystem;
import frc.robot.subsystems.drivercameravision.commands.DriverCameraSubsystemCommandFactory;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.GameplayState;
import frc.robot.subsystems.gameplaystate.GameplayStateSubsystem;
import frc.robot.subsystems.gameplaystate.commands.GameplayStateCommandFactory;
import frc.robot.subsystems.harvester.HarvesterSubsystem;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Central wiring hub for subsystems, commands, and driver inputs.
 */
public class RobotContainer {

    /** Deserialized subsystem configuration loaded from the environment-specific JSON file. */
    private final SubsystemsConfig                    subsystemsConfig;

    /** Field layout configuration that supplies the current AprilTag positions. */
    private final FieldLayoutConfig                   fieldLayoutConfig;

    /** Lazily supplies the loaded {@link AprilTagFieldLayout} for vision and navigation. */
    private final Supplier<AprilTagFieldLayout>       aprilTagFieldLayoutSupplier;

    /** Swerve drive subsystem responsible for chassis motion and odometry. */
    // Subsystems
    private final DriveBaseSubsystem                  driveBaseSubsystem;

    /** Turret subsystem that rotates the shooter to face field targets. */
    private final TurretSubsystem                     turretSubsystem;

    /** Shooter subsystem that spins flywheels to launch game pieces. */
    private final ShooterSubsystem                    shooterSubsystem;

    /** Indexer subsystem that stages game pieces between the feeder and shooter. */
    private final IndexerSubsystem                    indexerSubsystem;

    /** Robot pose subsystem that fuses odometry and vision into a field-relative estimate. */
    private final RobotPoseSubsystem                  robotPoseSubsystem;

    /** AprilTag vision subsystem that sends pose observations to the robot pose estimator. */
    @SuppressWarnings("unused")
    private final AprilTagVisionSubsystem             aprilTagVisionSubsystem;

    /** Driver camera subsystem that manages camera stream modes for the operator. */
    @SuppressWarnings("unused")
    private final DriverCameraSubsystem               driverCameraSubsystem;

    /** Climber subsystem for end-game climbing. */
    private final ClimberSubsystem                    climberSubsystem;

    /** Feeder subsystem that transports game pieces into the indexer. */
    private final FeederSubsystem                     feederSubsystem;

    /** Intake subsystem that collects game pieces from the ground. */
    private final IntakeSubsystem                     intakeSubsystem;

    /** Harvester subsystem that deploys and stows the intake arm. */
    private final HarvesterSubsystem                  harvesterSubsystem;

    /** Gameplay state machine that orchestrates multi-subsystem transitions. */
    private final GameplayStateSubsystem              gameplayStateSubsystem;

    /** Factory that builds PathPlanner autonomous commands from pre-loaded auto files. */
    private final PathPlannerCommandFactory           pathPlannerCommandFactory;

    /** Factory that builds drive base teleop and utility commands. */
    private final DriveBaseSubsystemCommandFactory    driveBaseCommandFactory;

    /** Factory that builds turret aiming and tracking commands. */
    private final TurretSubsystemCommandFactory       turretCommandFactory;

    /** Factory that builds shooter spin-up and idle commands. */
    private final ShooterSubsystemCommandFactory      shooterCommandFactory;

    /** Factory that builds indexer hold, unjam, and feed commands. */
    private final IndexerSubsystemCommandFactory      indexerCommandFactory;

    /** Factory that builds driver camera stream-toggle commands. */
    @SuppressWarnings("unused")
    private final DriverCameraSubsystemCommandFactory driverCameraCommandFactory;

    /** Factory that builds climber positioning commands. */
    @SuppressWarnings("unused")
    private final ClimberSubsystemCommandFactory      climberCommandFactory;

    /** Factory that builds feeder transport commands. */
    private final FeederSubsystemCommandFactory       feederCommandFactory;

    /** Factory that builds intake collect and eject commands. */
    private final IntakeSubsystemCommandFactory       intakeCommandFactory;

    /** Factory that builds harvester deploy and stow commands. */
    private final HarvesterSubsystemCommandFactory    harvesterCommandFactory;

    /** Factory that builds gameplay state transition commands. */
    private final GameplayStateCommandFactory         gameplayStateCommandFactory;

    /** Selects the nearest field target for turret tracking based on alliance and pose. */
    private final FieldTargetSelector                 fieldTargetSelector;

    /** Binds controller buttons and triggers to commands for both driver and operator. */
    @SuppressWarnings("unused")
    private final TriggerBindings                     triggerBindings;

    /**
     * Builds the robot container and wires subsystems, command factories, and bindings.
     */
    public RobotContainer() {
        try {
            subsystemsConfig            = ConfigurationLoader.load(resolveSubsystemsConfigFileName(), SubsystemsConfig.class);
            fieldLayoutConfig           = ConfigurationLoader.load("field-layout.json", FieldLayoutConfig.class);
            aprilTagFieldLayoutSupplier = fieldLayoutConfig::loadLayout;

            // Subsystems (order matters: drivebase is constructed first so robot state can reference it)
            driveBaseSubsystem          = new DriveBaseSubsystem(subsystemsConfig.driveBaseSubsystem);
            robotPoseSubsystem          = new RobotPoseSubsystem(
                    subsystemsConfig.robotPoseSubsystem,
                    driveBaseSubsystem::getOdometryPose,
                    driveBaseSubsystem::getOdometryOnlyPose,
                    driveBaseSubsystem::addVisionMeasurement,
                    driveBaseSubsystem::resetPose);
            turretSubsystem             = new TurretSubsystem(subsystemsConfig.turretSubsystem);
            shooterSubsystem            = new ShooterSubsystem(subsystemsConfig.shooterSubsystem);
            indexerSubsystem            = new IndexerSubsystem(subsystemsConfig.indexerSubsystem);
            aprilTagVisionSubsystem     = new AprilTagVisionSubsystem(
                    subsystemsConfig.aprilTagVisionSubsystem,
                    aprilTagFieldLayoutSupplier.get(),
                    robotPoseSubsystem::addVisionMeasurement,
                    // This is only for simulation purposes, in real life the vision subsystem will feed directly into the robot state subsystem and
                    // not reset odometry
                    driveBaseSubsystem::getOdometryPose);
            driverCameraSubsystem       = new DriverCameraSubsystem(subsystemsConfig.driverCameraSubsystem);
            climberSubsystem            = new ClimberSubsystem(subsystemsConfig.climberSubsystem);
            feederSubsystem             = new FeederSubsystem(subsystemsConfig.feederSubsystem);
            intakeSubsystem             = new IntakeSubsystem(subsystemsConfig.intakeSubsystem);
            harvesterSubsystem          = new HarvesterSubsystem(subsystemsConfig.harvesterSubsystem);
            gameplayStateSubsystem      = new GameplayStateSubsystem(subsystemsConfig.gameplayStateSubsystem);

            // Cross-subsystem utilities
            fieldTargetSelector         = new FieldTargetSelector(
                    subsystemsConfig.turretSubsystem.fieldTargets,
                    robotPoseSubsystem::getEstimatedPose,
                    RobotEnvironment::getAlliance);

            // Command factories
            driveBaseCommandFactory     = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
            turretCommandFactory        = new TurretSubsystemCommandFactory(turretSubsystem);
            shooterCommandFactory       = new ShooterSubsystemCommandFactory(shooterSubsystem);
            indexerCommandFactory       = new IndexerSubsystemCommandFactory(indexerSubsystem);
            driverCameraCommandFactory  = new DriverCameraSubsystemCommandFactory(driverCameraSubsystem);
            climberCommandFactory       = new ClimberSubsystemCommandFactory(climberSubsystem);
            feederCommandFactory        = new FeederSubsystemCommandFactory(feederSubsystem);
            intakeCommandFactory        = new IntakeSubsystemCommandFactory(intakeSubsystem);
            harvesterCommandFactory     = new HarvesterSubsystemCommandFactory(harvesterSubsystem);
            gameplayStateCommandFactory = new GameplayStateCommandFactory(
                    gameplayStateSubsystem,
                    shooterCommandFactory,
                    indexerCommandFactory,
                    feederCommandFactory,
                    intakeCommandFactory,
                    turretCommandFactory,
                    harvesterCommandFactory,
                    climberCommandFactory,
                    () -> robotPoseSubsystem.getDistanceToPointMeters(fieldTargetSelector.getActiveTargetPosition()));

            // Default commands
            turretCommandFactory.setDefaultTrackFieldTargetCommand(
                    robotPoseSubsystem,
                    fieldTargetSelector::getActiveTargetPosition,
                    driveBaseSubsystem::getYawRateRadiansPerSecond);

            // Register named commands for PathPlanner autos before pre-loading
            // Register named commands that PathPlanner autos reference by string key.
            // These must be registered before pre-loading autos so PathPlanner can resolve them.
            NamedCommands.registerCommand("MoveHarvesterToPositionCommand", harvesterCommandFactory.createDeployCommand());
            NamedCommands.registerCommand("SpinUpShooterCommand",
                    shooterCommandFactory.createSpinUpCommand(subsystemsConfig.shooterSubsystem::getMaximumShootingRpm));
            NamedCommands.registerCommand("SetStateIdle",
                    gameplayStateCommandFactory.createTransitionCommand(GameplayState.IDLE, "auto"));
            NamedCommands.registerCommand("SetStateHarvestReady",
                    gameplayStateCommandFactory.createTransitionCommand(GameplayState.HARVEST_READY, "auto"));
            NamedCommands.registerCommand("SetStateFireReady",
                    gameplayStateCommandFactory.createTransitionCommand(GameplayState.FIRE_READY, "auto"));
            NamedCommands.registerCommand("SetStateAutoCycle",
                    gameplayStateCommandFactory.createTransitionCommand(GameplayState.AUTO_CYCLE, "auto"));

            pathPlannerCommandFactory = new PathPlannerCommandFactory(robotPoseSubsystem::getEstimatedPose);

            // Dashboard commands (clickable buttons in Elastic Dashboard)
            SmartDashboard.putData("TurretSubsystem/ResetEncoder", turretCommandFactory.createResetEncoderCommand());
            SmartDashboard.putData("HarvesterSubsystem/ResetEncoder", harvesterCommandFactory.createResetEncoderCommand());
            SmartDashboard.putData("ClimberSubsystem/ResetEncoder", climberCommandFactory.createResetEncoderCommand());
            SmartDashboard.putData("ResetAllEncoders", Commands.parallel(
                    turretCommandFactory.createResetEncoderCommand(),
                    harvesterCommandFactory.createResetEncoderCommand(),
                    climberCommandFactory.createResetEncoderCommand())
                    .withName("Reset All Encoders"));

            // Input bindings
            triggerBindings = new TriggerBindings(
                    driveBaseCommandFactory,
                    subsystemsConfig.triggerBindings,
                    turretCommandFactory,
                    shooterCommandFactory,
                    indexerCommandFactory,
                    climberCommandFactory,
                    feederCommandFactory,
                    intakeCommandFactory,
                    harvesterCommandFactory,
                    gameplayStateCommandFactory);
        } catch (Exception e) {
            String message = "RobotContainer failed to initialize; robot will shut down.";
            RobotEnvironment.reportError(message, e.getStackTrace());
            throw e instanceof RuntimeException ? (RuntimeException) e : new IllegalStateException(message, e);
        }
    }

    /**
     * Resets the robot pose for both the drivebase and the robot state estimator.
     * <p>
     * Use this in simulation or autonomous init to place the robot at a known field location.
     * </p>
     *
     * @param pose desired starting pose in meters and radians
     */
    public void resetPose(edu.wpi.first.math.geometry.Pose2d pose) {
        robotPoseSubsystem.resetPose(pose);
    }

    /**
     * Resets the robot pose using the latest vision measurement from the cameras.
     * <p>
     * Call this at match start so the pose estimator begins from the camera-derived position rather than a default origin. If no vision measurement
     * is available yet, the reset is safely skipped by the underlying subsystem.
     * </p>
     */
    public void resetPoseFromVision() {
        robotPoseSubsystem.resetPoseFromVision();
    }

    /**
     * Returns the command to run during autonomous mode.
     * <p>
     * Update this method to swap between auto routines.
     * </p>
     *
     * @return command scheduled at the start of autonomous
     */
    public Command getAutonomousCommand() {
        if (!AutoBuilder.isConfigured()) {
            RobotEnvironment.reportWarning("AutoBuilder not configured; returning a no-op autonomous command.", false);
            return Commands.none();
        }

        Command autoCommand = pathPlannerCommandFactory.createAutoCommandForPosition(RobotEnvironment.getAlliance().get(),
                RobotEnvironment.getLocation().getAsInt());

        return autoCommand;
    }

    /**
     * Resolves the subsystem configuration JSON file name based on the current robot environment.
     * <p>
     * Returns {@code "subsystems-sim.json"} in simulation, {@code "subsystems-test.json"} on the test robot, or {@code "subsystems.json"} for the
     * competition robot.
     * </p>
     *
     * @return file name (without path) of the subsystems config to load from the deploy directory
     */
    private String resolveSubsystemsConfigFileName() {
        if (RobotEnvironment.isSimulation()) {
            return "subsystems-sim.json";
        }

        if (isTestRobot()) {
            return "subsystems-test.json";
        }

        return "subsystems.json";
    }

    /**
     * Checks whether the robot is the team's test chassis rather than the competition robot.
     *
     * @return {@code true} if this is the test robot; {@code false} otherwise
     */
    private boolean isTestRobot() {
        // TODO: hardware check to see if we're using the test robot
        return false;
    }

}
