// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shared.bindings.TriggerBindings;
import frc.robot.shared.config.ConfigurationLoader;
import frc.robot.shared.config.FieldLayoutConfig;
import frc.robot.shared.config.SubsystemsConfig;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

public class RobotContainer {

    // Configuration
    private final SubsystemsConfig                 subsystemsConfig;

    private final FieldLayoutConfig                fieldLayoutConfig;

    private final Supplier<AprilTagFieldLayout>    aprilTagFieldLayoutSupplier;

    // Subsystems
    private final DriveBaseSubsystem               driveBaseSubsystem;

    private final TurretSubsystem                  turretSubsystem;

    // Command factories
    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    private final TurretSubsystemCommandFactory    turretCommandFactory;

    // Input bindings
    @SuppressWarnings("unused")
    private final TriggerBindings                  triggerBindings;

    public RobotContainer() {
        try {
            subsystemsConfig            = ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);
            fieldLayoutConfig           = ConfigurationLoader.load("field-layout.json", FieldLayoutConfig.class);
            aprilTagFieldLayoutSupplier = fieldLayoutConfig::loadLayout;

            // Subsystems
            driveBaseSubsystem          = new DriveBaseSubsystem(subsystemsConfig.driveBaseSubsystem);
            turretSubsystem             = new TurretSubsystem(subsystemsConfig.turretSubsystem);

            // Command factories
            driveBaseCommandFactory     = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
            turretCommandFactory        = new TurretSubsystemCommandFactory(turretSubsystem);

            // Input bindings
            triggerBindings             = new TriggerBindings(
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

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return turretCommandFactory.createMoveToAngleCommand(500)
                .andThen(turretCommandFactory.createMoveToAngleCommand(-500)).repeatedly();
    }
}
