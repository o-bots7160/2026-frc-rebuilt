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
import frc.robot.shared.config.RobotEnvironment;

/**
 * Main robot class that owns the command scheduler and logging setup.
 */
public class Robot extends LoggedRobot {

    /** Length of the official FRC competition field in meters. */
    private static final double  FIELD_LENGTH_METERS   = 16.54;

    /** Width of the official FRC competition field in meters. */
    private static final double  FIELD_WIDTH_METERS    = 8.21;

    /** Distance in meters from the alliance wall to the default simulation start x-coordinate. */
    private static final double  START_X_OFFSET_METERS = 1.5;

    /** Distance in meters from the field edge to the nearest station start y-coordinate. */
    private static final double  EDGE_Y_OFFSET_METERS  = 1.1;

    /** The autonomous command selected for the current match period, or {@code null} if none. */
    private Command              m_autonomousCommand;

    /** The central wiring hub that owns all subsystems, command factories, and bindings. */
    private final RobotContainer m_robotContainer;

    /** Last simulation start pose applied, used to detect alliance/station changes while disabled. */
    private Pose2d               m_lastSimPose;

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

        // Configure logging data receivers based on robot mode (real hardware, replay, or live simulation).
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

    /**
     * Runs once every scheduler cycle regardless of robot mode.
     * <p>
     * Refreshes the cached environment state so all code this cycle uses the same snapshot, then advances the command scheduler.
     * </p>
     */
    @Override
    public void robotPeriodic() {
        // Refresh the cached environment state so all code this cycle uses the same snapshot.
        RobotEnvironment.refreshCycle();
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();
    }

    /** Runs once when the robot enters the disabled state; no-op by default. */
    @Override
    public void disabledInit() {
    }

    /**
     * Runs each loop while the robot is disabled.
     * <p>
     * In simulation, detects alliance or station changes and resets the robot pose so the simulated robot moves to the correct start location
     * immediately.
     * </p>
     */
    @Override
    public void disabledPeriodic() {
        if (isSimulation()) {
            Pose2d desiredPose = getSimulationStartPose();
            if (!desiredPose.equals(m_lastSimPose)) {
                m_lastSimPose = desiredPose;
                m_robotContainer.resetPose(desiredPose);
            }
        }
    }

    /** Runs once when the robot exits the disabled state; no-op by default. */
    @Override
    public void disabledExit() {
    }

    /**
     * Runs once when the simulator first starts.
     * <p>
     * Seeds the drivebase with a computed start pose so the simulated robot appears at a realistic field location rather than the origin.
     * </p>
     */
    @Override
    public void simulationInit() {
        Pose2d startPose = getSimulationStartPose();
        m_lastSimPose = startPose;
        m_robotContainer.resetPose(startPose);
    }

    /**
     * Seeds the robot pose and schedules the selected autonomous routine.
     * <p>
     * On a real robot the pose is reset from the latest vision measurement so the autonomous path starts at the correct field location. In simulation
     * a computed start pose is used instead since cameras are not present.
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

    /** Runs once when the robot exits autonomous mode; no-op by default. */
    @Override
    public void autonomousExit() {
    }

    /**
     * Seeds the robot pose and cancels any lingering autonomous command when teleop begins.
     * <p>
     * In simulation a computed start pose is used. On real hardware without FMS, the pose is reset from the latest vision measurement so
     * driver-relative controls start from an accurate field position. During competition (FMS attached), the vision subsystem's automatic odometry
     * reset handles pose seeding, so an explicit reset here is skipped to avoid overwriting the already-converged estimate.
     * </p>
     */
    @Override
    public void teleopInit() {
        if (isSimulation()) {
            Pose2d startPose = getSimulationStartPose();
            m_robotContainer.resetPose(startPose);
        } else if (!RobotEnvironment.isFMSAttached()) {
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

    /** Runs once when the robot exits teleop mode; no-op by default. */
    @Override
    public void teleopExit() {
    }

    /** Cancels all active commands when test mode begins. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** Runs each loop during test mode; no-op by default. */
    @Override
    public void testPeriodic() {
    }

    /** Runs once when the robot exits test mode; no-op by default. */
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
        OptionalInt                                            stationPosition = RobotEnvironment.getLocation();

        boolean                                                isRedAlliance   = alliance
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        int                                                    station         = stationPosition.orElse(2);

        // Spread start locations by station: B1/R3 at top (high Y), B3/R1 at bottom (low Y).
        // Blue stations are numbered top-to-bottom; Red stations are mirrored.
        double                                                 yPositionMeters;
        switch (station) {
        case 1 -> yPositionMeters = isRedAlliance
                ? EDGE_Y_OFFSET_METERS
                : FIELD_WIDTH_METERS - EDGE_Y_OFFSET_METERS;
        case 3 -> yPositionMeters = isRedAlliance
                ? FIELD_WIDTH_METERS - EDGE_Y_OFFSET_METERS
                : EDGE_Y_OFFSET_METERS;
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
