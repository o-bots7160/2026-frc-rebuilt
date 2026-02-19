// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.bindings.TriggerBindings;
import frc.robot.shared.config.ConfigurationLoader;
import frc.robot.shared.config.FieldLayoutConfig;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.config.SubsystemsConfig;
import frc.robot.subsystems.apriltagvision.AprilTagVisionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.ClimberSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.PathPlannerCommandFactory;
import frc.robot.subsystems.drivercameravision.DriverCameraSubsystem;
import frc.robot.subsystems.drivercameravision.commands.DriverCameraSubsystemCommandFactory;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.commands.HopperSubsystemCommandFactory;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.robotstate.RobotStateSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Central wiring hub for subsystems, commands, and driver inputs.
 */
public class RobotContainer {

    // Configuration
    private final SubsystemsConfig                 subsystemsConfig;

    private final FieldLayoutConfig                fieldLayoutConfig;

    private final Supplier<AprilTagFieldLayout>    aprilTagFieldLayoutSupplier;

    // Subsystems
    private final DriveBaseSubsystem               driveBaseSubsystem;

    private final TurretSubsystem                  turretSubsystem;

    private final ShooterSubsystem                 shooterSubsystem;

    private final IndexerSubsystem                 indexerSubsystem;

    private final RobotStateSubsystem              robotStateSubsystem;

    @SuppressWarnings("unused")
    private final AprilTagVisionSubsystem          aprilTagVisionSubsystem;

    @SuppressWarnings("unused")
    private final DriverCameraSubsystem            driverCameraSubsystem;

    private final ClimberSubsystem                 climberSubsystem;

    private final HopperSubsystem                  hopperSubsystem;

    // Command factories

    private final PathPlannerCommandFactory        pathPlannerCommandFactory;

    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    private final TurretSubsystemCommandFactory    turretCommandFactory;

    private final ShooterSubsystemCommandFactory   shooterCommandFactory;

    private final IndexerSubsystemCommandFactory   indexerCommandFactory;

    @SuppressWarnings("unused")
    private final DriverCameraSubsystemCommandFactory driverCameraCommandFactory;

    @SuppressWarnings("unused")
    private final ClimberSubsystemCommandFactory   climberCommandFactory;

    private final HopperSubsystemCommandFactory    hopperCommandFactory;

    // Input bindings
    @SuppressWarnings("unused")
    private final TriggerBindings                  triggerBindings;

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
            robotStateSubsystem         = new RobotStateSubsystem(
                    subsystemsConfig.robotStateSubsystem,
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
                    robotStateSubsystem::addVisionMeasurement,
                    // This is only for simulation purposes, in real life the vision subsystem will feed directly into the robot state subsystem and
                    // not reset odometry
                    driveBaseSubsystem::getOdometryPose);
            driverCameraSubsystem       = new DriverCameraSubsystem(subsystemsConfig.driverCameraSubsystem);
            climberSubsystem            = new ClimberSubsystem(subsystemsConfig.climberSubsystem);
            hopperSubsystem             = new HopperSubsystem(subsystemsConfig.hopperSubsystem);

            // Command factories
            pathPlannerCommandFactory   = new PathPlannerCommandFactory(robotStateSubsystem::getEstimatedPose);
            driveBaseCommandFactory     = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
            turretCommandFactory        = new TurretSubsystemCommandFactory(turretSubsystem);
            shooterCommandFactory       = new ShooterSubsystemCommandFactory(shooterSubsystem);
            indexerCommandFactory       = new IndexerSubsystemCommandFactory(indexerSubsystem);
            driverCameraCommandFactory  = new DriverCameraSubsystemCommandFactory(driverCameraSubsystem);
            climberCommandFactory       = new ClimberSubsystemCommandFactory(climberSubsystem);
            hopperCommandFactory        = new HopperSubsystemCommandFactory(hopperSubsystem);

            // Default Commands
            shooterCommandFactory.setDefaultIdleCommand();
            indexerCommandFactory.setDefaultIdleCommand();
            hopperCommandFactory.setDefaultIdleCommand();
            turretCommandFactory.setDefaultTrackFieldTargetCommand(
                    robotStateSubsystem,
                    () -> {
                        var layout = aprilTagFieldLayoutSupplier.get();
                        return layout.getTagPose(26)
                                .map(pose -> pose.getTranslation().toTranslation2d())
                                .orElseThrow();
                    },
                    driveBaseSubsystem::getYawRateRadiansPerSecond);

            // Input bindings
            triggerBindings = new TriggerBindings(
                    driveBaseCommandFactory,
                    subsystemsConfig.triggerBindings,
                    turretCommandFactory,
                    shooterCommandFactory,
                    indexerCommandFactory,
                    climberCommandFactory);
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
        robotStateSubsystem.resetPose(pose);
    }

    /**
     * Resets the robot pose using the latest vision measurement from the cameras.
     * <p>
     * Call this at match start so the pose estimator begins from the camera-derived position rather than a default origin. If no vision measurement
     * is available yet, the reset is safely skipped by the underlying subsystem.
     * </p>
     */
    public void resetPoseFromVision() {
        robotStateSubsystem.resetPoseFromVision();
    }

    /**
     * Returns the active AprilTag field layout for navigation and vision targeting.
     * <p>
     * Use this to look up tag poses for autonomous destinations or pose correction.
     * </p>
     *
     * @return loaded AprilTag field layout with the configured origin applied
     */
    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayoutSupplier.get();
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

    private String resolveSubsystemsConfigFileName() {
        if (RobotEnvironment.isSimulation()) {
            return "subsystems-sim.json";
        }

        if (isTestRobot()) {
            return "subsystems-test.json";
        }

        return "subsystems.json";
    }

    private boolean isTestRobot() {
        // TODO: hardware check to see if we're using the test robot
        return true;
    }
}
