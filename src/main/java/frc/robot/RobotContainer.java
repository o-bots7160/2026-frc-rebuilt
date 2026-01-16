// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.factories.DriveBaseSubsystemCommandFactory;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.TriggerBindings;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {

    // Configuration
    private final SubsystemsConfig                 subsystemsConfig;

    // Subsystems
    private final DriveBaseSubsystem               driveBaseSubsystem;

    private final TurretSubsystem                  turretSubsystem;

    // Command factories
    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    // Input bindings
    private final TriggerBindings                  triggerBindings;

    public RobotContainer() {
        try {
            subsystemsConfig        = ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);

            // Subsystems
            driveBaseSubsystem      = new DriveBaseSubsystem(subsystemsConfig.driveBaseSubsystem);
            turretSubsystem         = new TurretSubsystem(subsystemsConfig.turretSubsystem);

            // Command factories
            driveBaseCommandFactory = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);

            // Input bindings
            triggerBindings         = new TriggerBindings(driveBaseCommandFactory, subsystemsConfig.driveBaseSubsystem);
        } catch (Exception e) {
            String message = "RobotContainer failed to initialize; robot will shut down.";
            DriverStation.reportError(message, e.getStackTrace());
            throw e instanceof RuntimeException ? (RuntimeException) e : new IllegalStateException(message, e);
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
