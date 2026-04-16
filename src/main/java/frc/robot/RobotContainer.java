// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.bindings.AbstractTriggerBindings;
import frc.robot.shared.bindings.CompetitionTriggerBindings;
import frc.robot.shared.bindings.TuningTriggerBindings;
import frc.robot.shared.config.ConfigurationLoader;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.config.SubsystemsConfig;
import frc.robot.shared.field.FieldTargetSelector;
import frc.robot.shared.targeting.BallFlightSimulator;
import frc.robot.subsystems.apriltagvision.AprilTagVisionSubsystem;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.PathPlannerCommandFactory;
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
    private final SubsystemsConfig                 subsystemsConfig;

    /** Swerve drive subsystem responsible for chassis motion and odometry. */
    // Subsystems
    private final DriveBaseSubsystem               driveBaseSubsystem;

    /** Turret subsystem that rotates the shooter to face field targets. */
    private final TurretSubsystem                  turretSubsystem;

    /** Shooter subsystem that spins flywheels to launch game pieces. */
    private final ShooterSubsystem                 shooterSubsystem;

    /** Indexer subsystem that stages game pieces between the feeder and shooter. */
    private final IndexerSubsystem                 indexerSubsystem;

    /** Robot pose subsystem that fuses odometry and vision into a field-relative estimate. */
    private final RobotPoseSubsystem               robotPoseSubsystem;

    /** AprilTag vision subsystem that sends pose observations to the robot pose estimator. */
    @SuppressWarnings("unused")
    private final AprilTagVisionSubsystem          aprilTagVisionSubsystem;

    /** Feeder subsystem that transports game pieces into the indexer. */
    private final FeederSubsystem                  feederSubsystem;

    /** Intake subsystem that collects game pieces from the ground. */
    private final IntakeSubsystem                  intakeSubsystem;

    /** Harvester subsystem that deploys and stows the intake arm. */
    private final HarvesterSubsystem               harvesterSubsystem;

    /** Gameplay state machine that orchestrates multi-subsystem transitions. */
    private final GameplayStateSubsystem           gameplayStateSubsystem;

    /** Factory that builds PathPlanner autonomous commands from pre-loaded auto files. */
    private final PathPlannerCommandFactory        pathPlannerCommandFactory;

    /** Factory that builds drive base teleop and utility commands. */
    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    /** Factory that builds turret aiming and tracking commands. */
    private final TurretSubsystemCommandFactory    turretCommandFactory;

    /** Factory that builds shooter spin-up and idle commands. */
    private final ShooterSubsystemCommandFactory   shooterCommandFactory;

    /** Factory that builds indexer hold, unjam, and feed commands. */
    private final IndexerSubsystemCommandFactory   indexerCommandFactory;

    /** Factory that builds feeder transport commands. */
    private final FeederSubsystemCommandFactory    feederCommandFactory;

    /** Factory that builds intake collect and eject commands. */
    private final IntakeSubsystemCommandFactory    intakeCommandFactory;

    /** Factory that builds harvester deploy and stow commands. */
    private final HarvesterSubsystemCommandFactory harvesterCommandFactory;

    /** Factory that builds gameplay state transition commands. */
    private final GameplayStateCommandFactory      gameplayStateCommandFactory;

    /** Selects the nearest field target for turret tracking based on alliance and pose. */
    private final FieldTargetSelector              fieldTargetSelector;

    /** Binds controller buttons and triggers to commands for both driver and operator. */
    private final AbstractTriggerBindings           triggerBindings;

    /** Sim-only ball flight visualizer; null on a real robot. */
    private final BallFlightSimulator              ballFlightSimulator;

    /**
     * Builds the robot container and wires subsystems, command factories, and bindings.
     */
    public RobotContainer() {
        try {
            subsystemsConfig            = ConfigurationLoader.load(resolveSubsystemsConfigFileName(), SubsystemsConfig.class);

            // Subsystems (order matters: drivebase is constructed first so robot state can reference it)
            // The shooting supplier uses a method reference because gameplayStateSubsystem is a
            // final field that is not yet assigned at this point. The method reference binds to
            // 'this', which resolves the field at call time rather than at lambda capture time.
            driveBaseSubsystem          = new DriveBaseSubsystem(
                    subsystemsConfig.driveBaseSubsystem,
                    this::isShootingActive);
            robotPoseSubsystem          = new RobotPoseSubsystem(
                    subsystemsConfig.robotPoseSubsystem,
                    driveBaseSubsystem::getFusedPose,
                    driveBaseSubsystem::getOdometryOnlyPose,
                    driveBaseSubsystem::addVisionMeasurement,
                    driveBaseSubsystem::resetPose);
            turretSubsystem             = new TurretSubsystem(subsystemsConfig.turretSubsystem);
            shooterSubsystem            = new ShooterSubsystem(subsystemsConfig.shooterSubsystem);
            indexerSubsystem            = new IndexerSubsystem(subsystemsConfig.indexerSubsystem);
            aprilTagVisionSubsystem     = new AprilTagVisionSubsystem(
                    subsystemsConfig.aprilTagVisionSubsystem,
                    robotPoseSubsystem::addVisionMeasurement,
                    robotPoseSubsystem::resetPose,
                    // Pose supplier is used for simulation cameras only; use raw odometry to avoid feedback loops.
                    driveBaseSubsystem::getFusedPose);
            feederSubsystem             = new FeederSubsystem(subsystemsConfig.feederSubsystem);
            intakeSubsystem             = new IntakeSubsystem(subsystemsConfig.intakeSubsystem);
            harvesterSubsystem          = new HarvesterSubsystem(subsystemsConfig.harvesterSubsystem);

            // Cross-subsystem utilities (before gameplayState so targeting suppliers are available)
            fieldTargetSelector         = new FieldTargetSelector(
                    subsystemsConfig.turretSubsystem.fieldTargets,
                    robotPoseSubsystem::getEstimatedPose,
                    RobotEnvironment::getAlliance);

            gameplayStateSubsystem      = new GameplayStateSubsystem(
                    subsystemsConfig.gameplayStateSubsystem,
                    () -> robotPoseSubsystem.getDistanceToPointMeters(fieldTargetSelector.getActiveTargetPosition()),
                    fieldTargetSelector::getActiveTargetName);

            // Command factories
            driveBaseCommandFactory     = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
            turretCommandFactory        = new TurretSubsystemCommandFactory(turretSubsystem);
            shooterCommandFactory       = new ShooterSubsystemCommandFactory(shooterSubsystem);
            indexerCommandFactory       = new IndexerSubsystemCommandFactory(indexerSubsystem);
            feederCommandFactory        = new FeederSubsystemCommandFactory(feederSubsystem);
            intakeCommandFactory        = new IntakeSubsystemCommandFactory(intakeSubsystem);
            harvesterCommandFactory     = new HarvesterSubsystemCommandFactory(harvesterSubsystem);
            gameplayStateCommandFactory = new GameplayStateCommandFactory(
                    gameplayStateSubsystem,
                    shooterCommandFactory,
                    indexerCommandFactory,
                    feederCommandFactory,
                    intakeCommandFactory,
                    harvesterCommandFactory,
                    turretCommandFactory::getCompensatedDistanceMeters,
                    turretSubsystem::isOnTarget);

            // Default commands
            turretCommandFactory.setDefaultTrackFieldTargetCommand(
                    robotPoseSubsystem,
                    fieldTargetSelector::getActiveTargetPosition,
                    driveBaseSubsystem::getYawRateRadiansPerSecond,
                    driveBaseSubsystem::getFieldRelativeVelocity,
                    shooterSubsystem::getTimeOfFlightSeconds);
            //harvesterCommandFactory.setDefaultHoldDeployedPositionCommand();

            // Register named commands for PathPlanner autos before pre-loading.
            // These must be registered before pre-loading autos so PathPlanner can resolve them.
            // Each command is wrapped with asProxy() so its subsystem requirements are NOT
            // aggregated into the auto's SequentialCommandGroup at construction time. Without
            // proxying, the auto would claim Shooter/Indexer/Feeder/Intake for its entire
            // duration, conflicting with the GameplayStateSubsystem default idle command and
            // causing the scheduler to cancel the auto on the first cycle.
            NamedCommands.registerCommand("SetStateIdle",
                    gameplayStateCommandFactory.createIdleCommand().asProxy());
            NamedCommands.registerCommand("SetStateHarvestReady",
                    gameplayStateCommandFactory.createHarvestReadyCommand().asProxy());
            NamedCommands.registerCommand("SetStateFireReady",
                    gameplayStateCommandFactory.createFireReadyCommand().asProxy());
            NamedCommands.registerCommand("SetStateEject",
                    gameplayStateCommandFactory.createEjectCommand().asProxy());
            NamedCommands.registerCommand("SetStateTravel",
                    gameplayStateCommandFactory.createTravelCommand().asProxy());
            NamedCommands.registerCommand("SetStateTrenchTravel",
                    gameplayStateCommandFactory.createTrenchTravelCommand().asProxy());

            pathPlannerCommandFactory = new PathPlannerCommandFactory();

            // Dashboard choosers
            RobotEnvironment.initAllianceChooser();

            // Dashboard commands (clickable buttons in Elastic Dashboard)
            SmartDashboard.putData("TurretSubsystem/ResetEncoder", turretCommandFactory.createResetEncoderCommand());
            SmartDashboard.putData("HarvesterSubsystem/ResetEncoder", harvesterCommandFactory.createResetEncoderCommand());
            SmartDashboard.putData("ResetAllEncoders", Commands.parallel(
                    turretCommandFactory.createResetEncoderCommand(),
                    harvesterCommandFactory.createResetEncoderCommand())
                    .withName("Reset All Encoders"));

            // Sim-only ball flight visualization
            if (RobotEnvironment.isSimulation()) {
                // Use lenient readiness checks for sim since sim motors may not reach full
                // target velocity. Check measured RPM > idle instead of isAtShootingVelocity().
                double idleRpm = subsystemsConfig.shooterSubsystem.motionProfile.idleVelocityRpm;
                ballFlightSimulator = new BallFlightSimulator(
                        robotPoseSubsystem::getEstimatedPose,
                        turretSubsystem::getMeasuredPositionDegrees,
                        shooterSubsystem::getMeasuredVelocityRpm,
                        () -> shooterSubsystem.getMeasuredVelocityRpm() > idleRpm,
                        indexerSubsystem::isFeeding,
                        driveBaseSubsystem::getFieldRelativeVelocity,
                        subsystemsConfig.turretSubsystem.getTurretZeroOffsetDegrees(),
                        subsystemsConfig.turretSubsystem.componentPoseConfig.componentPivotX,
                        subsystemsConfig.turretSubsystem.componentPoseConfig.componentPivotY,
                        subsystemsConfig.turretSubsystem.componentPoseConfig.componentPivotZ,
                        subsystemsConfig.turretSubsystem.sotmDragCoefficient,
                        subsystemsConfig.shooterSubsystem.distanceRpmPoints);
            } else {
                ballFlightSimulator = null;
            }

            // Input bindings — competition or tuning mode based on config.
            if (subsystemsConfig.triggerBindings.getTuningEnabled()) {
                triggerBindings = new TuningTriggerBindings(
                        driveBaseCommandFactory,
                        subsystemsConfig.triggerBindings,
                        turretCommandFactory,
                        shooterCommandFactory,
                        indexerCommandFactory,
                        feederCommandFactory,
                        intakeCommandFactory,
                        harvesterCommandFactory,
                        gameplayStateCommandFactory);
            } else {
                triggerBindings = new CompetitionTriggerBindings(
                        driveBaseCommandFactory,
                        subsystemsConfig.triggerBindings,
                        turretCommandFactory,
                        shooterCommandFactory,
                        indexerCommandFactory,
                        feederCommandFactory,
                        intakeCommandFactory,
                        harvesterCommandFactory,
                        gameplayStateCommandFactory);
            }
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
     * Activates initial pose calibration mode for the disabled period.
     * <p>
     * While active, every geometrically valid vision observation is accepted regardless of odometry deviation. The first accepted pose also
     * hard-resets odometry so the Kalman filter tracks the camera-derived position immediately. Call this in {@code disabledInit()} on real hardware
     * so operators can reposition the robot on the field and watch the dashboard pose converge before a match.
     * </p>
     */
    public void beginInitialPoseCalibration() {
        aprilTagVisionSubsystem.beginInitialPoseCalibration();
    }

    /**
     * Deactivates initial pose calibration mode when the robot leaves the disabled state.
     * <p>
     * Normal filtering (including odometry deviation checking) resumes so match-time observations are held to standard confidence thresholds.
     * </p>
     */
    public void endInitialPoseCalibration() {
        aprilTagVisionSubsystem.endInitialPoseCalibration();
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

        Alliance alliance = RobotEnvironment.getAlliance().orElse(Alliance.Blue);
        String   autoName = pathPlannerCommandFactory.getSelectedAutoName();

        return pathPlannerCommandFactory.createAutoCommand(alliance, autoName);
    }

    /**
     * Runs once per robot loop to perform non-subsystem periodic checks.
     * <p>
     * Call from {@code Robot.robotPeriodic()} after the environment snapshot is refreshed.
     * </p>
     */
    public void periodic() {
        triggerBindings.checkControllerHealth();

        if (ballFlightSimulator != null) {
            ballFlightSimulator.periodic();
        }
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

    /**
     * Returns whether the robot is actively shooting (FIRE_READY or AUTO_CYCLE). Used as a supplier for drive base power management so the drivetrain
     * reduces speed while the shooter, indexer, and feeder need full battery current.
     *
     * @return true when the gameplay state is FIRE_READY or AUTO_CYCLE
     */
    private boolean isShootingActive() {
        if (gameplayStateSubsystem == null) {
            return false;
        }

        GameplayState state = gameplayStateSubsystem.getCurrentState();
        return state == GameplayState.FIRE_READY || state == GameplayState.AUTO_CYCLE;
    }

}
