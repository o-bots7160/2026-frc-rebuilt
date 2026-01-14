// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.ConfigurationException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.factories.DriveBaseSubsystemCommandFactory;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.TriggerBindings;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;

public class RobotContainer {

    private final SubsystemsConfig                 subsystemsConfig;

    private final DriveBaseSubsystem               driveBaseSubsystem;

    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    private final TriggerBindings                  triggerBindings;

    public RobotContainer() {
        subsystemsConfig        = loadSubsystemConfig();
        driveBaseSubsystem      = new DriveBaseSubsystem(subsystemsConfig.driveBaseSubsystem);
        driveBaseCommandFactory = new DriveBaseSubsystemCommandFactory(driveBaseSubsystem);
        triggerBindings         = new TriggerBindings(driveBaseCommandFactory, subsystemsConfig.driveBaseSubsystem);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private SubsystemsConfig loadSubsystemConfig() {
        try {
            return ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);
        } catch (ConfigurationException e) {
            e.printStackTrace();
            return new SubsystemsConfig();
        }
    }
}
