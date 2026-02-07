// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.bindings.TriggerBindings;
import frc.robot.shared.config.ConfigurationLoader;
import frc.robot.shared.config.FieldLayoutConfig;
import frc.robot.shared.config.SubsystemsConfig;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.commands.PathPlannerCommandFactory;
import frc.robot.subsystems.robotstate.RobotStateSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;
import frc.robot.subsystems.vision.AprilTagVisionSubsystem;
import frc.robot.subsystems.vision.DriverCameraSubsystem;

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

    private final RobotStateSubsystem              robotStateSubsystem;

    @SuppressWarnings("unused")
    private final AprilTagVisionSubsystem          aprilTagVisionSubsystem;

    @SuppressWarnings("unused")
    private final DriverCameraSubsystem            driverCameraSubsystem;

    // Command factories

    private final PathPlannerCommandFactory        pathPlannerCommandFactory;

    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    private final TurretSubsystemCommandFactory    turretCommandFactory;

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

            // Subsystems
            robotStateSubsystem         = new RobotStateSubsystem(subsystemsConfig.robotStateSubsystem);
            driveBaseSubsystem          = new DriveBaseSubsystem(
                    subsystemsConfig.driveBaseSubsystem,
                    robotStateSubsystem::updateOdometryPose);
            robotStateSubsystem.setOdometryResetConsumer(driveBaseSubsystem::resetPose);
            turretSubsystem           = new TurretSubsystem(subsystemsConfig.turretSubsystem);
            aprilTagVisionSubsystem   = new AprilTagVisionSubsystem(
                    subsystemsConfig.aprilTagVisionSubsystem,
                    aprilTagFieldLayoutSupplier.get(),
                    robotStateSubsystem::addVisionMeasurement,
                    driveBaseSubsystem::getOdometryPose);
            driverCameraSubsystem     = new DriverCameraSubsystem(subsystemsConfig.driverCameraSubsystem);

            // Command factories
            pathPlannerCommandFactory = new PathPlannerCommandFactory(robotStateSubsystem::getEstimatedPose);
            driveBaseCommandFactory   = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
            turretCommandFactory      = new TurretSubsystemCommandFactory(turretSubsystem);

            // Default Commands
            turretCommandFactory.setDefaultTrackFieldTargetCommand(
                    robotStateSubsystem,
                    () -> {
                        var layout = aprilTagFieldLayoutSupplier.get();
                        return layout.getTagPose(26)
                                .map(pose -> pose.getTranslation().toTranslation2d())
                                .orElseThrow();
                    });

            // Input bindings
            triggerBindings = new TriggerBindings(
                    driveBaseCommandFactory,
                    subsystemsConfig.driveBaseSubsystem,
                    turretCommandFactory);
        } catch (Exception e) {
            String message = "RobotContainer failed to initialize; robot will shut down.";
            DriverStation.reportError(message, e.getStackTrace());
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
            DriverStation.reportWarning("AutoBuilder not configured; returning a no-op autonomous command.", false);
            return Commands.none();
        }

        Command autoCommand = pathPlannerCommandFactory.createAutoCommandForPosition(DriverStation.getAlliance().get(),
                DriverStation.getLocation().getAsInt());

        return autoCommand;
    }

    private String resolveSubsystemsConfigFileName() {
        if (RobotBase.isSimulation()) {
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
