// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command              m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                case 0 -> "All changes committed";
                case 1 -> "Uncommitted changes";
                default -> "Unknown";
                });

        if (isReal()) {
            // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            String replayPath = System.getenv("REPLAY_LOG");

            if (replayPath != null && !replayPath.isBlank()) {
                // Use a supplied replay log (set REPLAY_LOG env var) and save a suffixed copy.
                Logger.setReplaySource(new WPILOGReader(replayPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_sim")));
            } else {
                // No replay provided; publish live telemetry and log to a default sim file.
                Logger.addDataReceiver(new NT4Publisher());
                String basePath = Filesystem.getOperatingDirectory() != null
                        ? Filesystem.getOperatingDirectory().getPath()
                        : System.getProperty("user.dir", ".");
                Path   logDir   = Paths.get(basePath, "logs");
                Path   logFile  = logDir.resolve("sim.wpilog");
                try {
                    Files.createDirectories(logDir);
                } catch (Exception ignored) {
                    // If we cannot create the directory, WPILOGWriter will throw; better to fail later with context.
                }
                Logger.addDataReceiver(new WPILOGWriter(logFile.toString()));
            }
        }

        Logger.start();

        m_robotContainer = new RobotContainer();

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
