// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.shared.RobotEnvironment;

/**
 * Main robot class that owns the command scheduler and logging setup.
 */
public class Robot extends LoggedRobot {

    private static final double  FIELD_LENGTH_METERS   = 16.54;

    private static final double  FIELD_WIDTH_METERS    = 8.21;

    private static final double  START_X_OFFSET_METERS = 1.5;

    private static final double  EDGE_Y_OFFSET_METERS  = 1.1;

    private Command              m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /**
     * Creates the robot and initializes logging, replay, and container wiring.
     */
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
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            String replayPath = System.getenv("REPLAY_LOG");

            if (replayPath != null && !replayPath.isBlank()) {
                setUseTiming(false); // Replay logs as fast as possible.
                // Use a supplied replay log (set REPLAY_LOG env var) and save a suffixed copy.
                Logger.setReplaySource(new WPILOGReader(replayPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_sim")));
            } else {
                setUseTiming(true); // Keep real-time loop timing for live simulation and SysId.
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
            RobotEnvironment.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        // Refresh the cached environment state so all code this cycle uses the same snapshot.
        RobotEnvironment.refreshCycle();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    /** Runs each loop while the robot is disabled; no-op by default. */
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void simulationInit() {
        Pose2d startPose = getSimulationStartPose();
        m_robotContainer.resetPose(startPose);
    }

    /**
     * Seeds the robot pose and schedules the selected autonomous routine.
     * <p>
     * On a real robot the pose is reset from the latest vision measurement so the autonomous path starts at the correct field location. In
     * simulation a computed start pose is used instead since cameras are not present.
     * </p>
     */
    @Override
    public void autonomousInit() {
        if (isSimulation()) {
            Pose2d startPose = getSimulationStartPose();
            m_robotContainer.resetPose(startPose);
        } else {
            m_robotContainer.resetPoseFromVision();
        }

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    /** Runs each loop during autonomous; command scheduler handles execution. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /**
     * Seeds the robot pose and cancels any lingering autonomous command when teleop begins.
     * <p>
     * On a real robot the pose is reset from the latest vision measurement so driver-relative controls start from an accurate field position. In
     * simulation a computed start pose is used instead.
     * </p>
     */

    @Override
    public void teleopInit() {
        if (isSimulation()) {
            Pose2d startPose = getSimulationStartPose();
            m_robotContainer.resetPose(startPose);
        } else {
            m_robotContainer.resetPoseFromVision();
        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** Runs each loop during teleop; command scheduler handles execution. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    /** Cancels all active commands when test mode begins. */

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

    /**
     * Computes a reasonable simulation start pose from the driver station alliance and station.
     * <p>
     * In the simulation app, set the driver station alliance and position so the team side updates and the start pose reflects the selected station.
     * </p>
     *
     * @return pose in meters and radians that matches the simulated alliance wall and station
     */
    private Pose2d getSimulationStartPose() {
        // Prefer the live driver station info, but default to Blue/center when not present in sim.
        Optional<edu.wpi.first.wpilibj.DriverStation.Alliance> alliance        = RobotEnvironment.getAlliance();
        OptionalInt                      stationPosition = RobotEnvironment.getLocation();

        boolean                          isRedAlliance   = alliance.orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        int                              station         = stationPosition.orElse(2);

        // Spread start locations by station: 1 (near edge), 2 (center), 3 (far edge).
        double                           yPositionMeters;
        switch (station) {
        case 1 -> yPositionMeters = EDGE_Y_OFFSET_METERS;
        case 3 -> yPositionMeters = FIELD_WIDTH_METERS - EDGE_Y_OFFSET_METERS;
        default -> yPositionMeters = FIELD_WIDTH_METERS / 2.0;
        }

        // Place the robot just inside the alliance wall, facing the opposing side.
        double     xPositionMeters = isRedAlliance
                ? FIELD_LENGTH_METERS - START_X_OFFSET_METERS
                : START_X_OFFSET_METERS;
        Rotation2d heading         = isRedAlliance ? Rotation2d.fromDegrees(180.0) : new Rotation2d();

        return new Pose2d(xPositionMeters, yPositionMeters, heading);
    }
}
