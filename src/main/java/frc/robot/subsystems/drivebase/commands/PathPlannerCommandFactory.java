package frc.robot.subsystems.drivebase.commands;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.config.RobotEnvironment;

/**
 * Builds autonomous command sequences for each driver-station position.
 * <p>
 * Before autonomous starts the robot may not be sitting where the pre-planned PathPlanner auto expects. This factory creates a short alignment path
 * on the fly that drives the robot from its current pose to the auto's expected start, then sequences the planned auto after it. All autos are
 * pre-loaded at construction time so there is no file-parsing delay when the match starts.
 * </p>
 *
 * @see <a href="https://pathplanner.dev/pplib-create-a-path-on-the-fly.html">PathPlanner — Create a Path On-the-fly</a>
 * @see <a href="https://pathplanner.dev/pplib-build-an-auto.html#load-an-auto">PathPlanner — Load an Auto</a>
 */
public class PathPlannerCommandFactory {

    /** Maximum linear velocity for the alignment path in meters per second. */
    private static final double                ALIGN_MAX_VELOCITY_METERS_PER_SECOND                      = 3.0;

    /** Maximum linear acceleration for the alignment path in meters per second squared. */
    private static final double                ALIGN_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED          = 3.0;

    /** Maximum angular velocity for the alignment path in radians per second. */
    private static final double                ALIGN_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND             = 2 * Math.PI;

    /** Maximum angular acceleration for the alignment path in radians per second squared. */
    private static final double                ALIGN_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 4 * Math.PI;

    /** Subdirectory under the deploy folder where PathPlanner auto files are stored. */
    private static final String                AUTO_DIRECTORY_PATH                                       = "pathplanner/autos";

    /** File extension used by PathPlanner auto files. */
    private static final String                AUTO_FILE_EXTENSION                                       = ".auto";

    /**
     * Dashboard chooser that lets drivers select which autonomous routine to run.
     * <p>
     * Published to {@code SmartDashboard/Auto Chooser} at construction time.
     * </p>
     */
    private LoggedDashboardChooser<String>     autoChooser;

    /**
     * Pre-loaded PathPlanner autos keyed by auto name.
     * <p>
     * Populated at construction time so there is no file-parsing delay when autonomous starts. Cached instances are wrapped with
     * {@link Command#asProxy()} before being composed into a sequence so that WPILib allows the same underlying command to participate in multiple
     * compositions across autonomous periods.
     * </p>
     */
    private final Map<String, PathPlannerAuto> autoCache                                                 = new HashMap<>();

    /**
     * Creates a new factory and pre-loads all known autos.
     * <p>
     * Call this once during robot initialization in {@code RobotContainer} and reuse the instance every time an autonomous command is needed.
     * </p>
     */
    public PathPlannerCommandFactory() {
        initializeAutos();
    }

    /**
     * Builds the full autonomous command for the given alliance using the auto name selected on the dashboard.
     * <p>
     * The returned command sequences an alignment path (current pose to expected start) followed by the pre-loaded PathPlanner auto. If the auto has
     * no recorded starting pose the alignment step is skipped.
     * </p>
     *
     * @param alliance alliance color used to mirror the starting pose to the correct side of the field
     * @param autoName name of the auto to run, matching a key in the cache
     * @return composite command that first aligns then runs the auto
     */
    public Command createAutoCommand(Alliance alliance, String autoName) {
        // Pull from the cache; fall back to a live load if the cache missed.
        PathPlannerAuto pathPlannerCommand = autoCache.get(autoName);
        if (pathPlannerCommand == null) {
            RobotEnvironment.reportWarning(
                    "Auto '" + autoName + "' was not pre-loaded. Loading now \u2014 expect a short delay.", false);
            pathPlannerCommand = new PathPlannerAuto(autoName);
            autoCache.put(autoName, pathPlannerCommand);
        }

        Pose2d startingPose = resolveStartingPose(alliance, pathPlannerCommand, autoName);

        if (startingPose == null) {
            // Proxy allows the cached command to be re-scheduled across autonomous periods.
            return pathPlannerCommand.asProxy();
        }

        Command alignmentCommand = buildAlignmentCommand(startingPose);
        return Commands.sequence(alignmentCommand, pathPlannerCommand.asProxy());
    }

    /**
     * Returns the auto name currently selected on the dashboard chooser.
     * <p>
     * Falls back to {@code "b1"} if the chooser has not been initialized or returns null.
     * </p>
     *
     * @return selected auto name
     */
    public String getSelectedAutoName() {
        String selected = autoChooser.get();
        return selected != null ? selected : "b1";
    }

    /**
     * Pre-loads every PathPlanner auto into the cache and populates the dashboard chooser.
     * <p>
     * Discovers all {@code .auto} files in the deploy directory so they are ready when autonomous is enabled. The first entry alphabetically is set
     * as the default option in the {@link LoggedDashboardChooser}. The chooser is published to {@code SmartDashboard/Auto Chooser}.
     * </p>
     */
    private void initializeAutos() {
        String[] autoNames = discoverAutoNames();
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        boolean first = true;
        for (String autoName : autoNames) {
            // Populate the dashboard chooser.
            if (first) {
                autoChooser.addDefaultOption(autoName, autoName);
                first = false;
            } else {
                autoChooser.addOption(autoName, autoName);
            }

            // Pre-load into the cache.
            if (autoCache.containsKey(autoName)) {
                continue;
            }

            try {
                autoCache.put(autoName, new PathPlannerAuto(autoName));
            } catch (Exception e) {
                RobotEnvironment.reportError(
                        "Failed to pre-load auto '" + autoName + "': " + e.getMessage(),
                        e.getStackTrace());
            }
        }
    }

    /**
     * Scans the deploy directory for PathPlanner auto files and returns their names.
     * <p>
     * Reads all {@code .auto} files from {@code pathplanner/autos/} under the deploy directory, strips the file extension, and returns the names
     * sorted alphabetically. Reports a warning if no autos are found.
     * </p>
     *
     * @return sorted array of auto names discovered on disk, or an empty array if none are found
     */
    private String[] discoverAutoNames() {
        File autosDirectory = new File(Filesystem.getDeployDirectory(), AUTO_DIRECTORY_PATH);
        if (!autosDirectory.isDirectory()) {
            RobotEnvironment.reportWarning(
                    "Auto directory not found at '" + autosDirectory.getAbsolutePath() + "'. No autos will be available.",
                    false);
            return new String[0];
        }

        File[] autoFiles = autosDirectory.listFiles((dir, name) -> name.endsWith(AUTO_FILE_EXTENSION));
        if (autoFiles == null || autoFiles.length == 0) {
            RobotEnvironment.reportWarning("No .auto files found in '" + autosDirectory.getAbsolutePath() + "'.", false);
            return new String[0];
        }

        String[] names = Arrays.stream(autoFiles)
                .map(file -> file.getName().replace(AUTO_FILE_EXTENSION, ""))
                .sorted()
                .toArray(String[]::new);

        return names;
    }

    /**
     * Creates a pathfinding command that drives from the robot's current pose to the target pose.
     * <p>
     * Uses PathPlanner's AD* pathfinder for obstacle-aware routing. The command ends at zero velocity so the robot is stationary when the planned
     * auto takes over. Coordinates are already alliance-corrected by the caller.
     * </p>
     *
     * @param targetPose field pose the robot should arrive at, already alliance-flipped if on Red
     * @return command that pathfinds to the target pose and stops
     */
    private Command buildAlignmentCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                ALIGN_MAX_VELOCITY_METERS_PER_SECOND,
                ALIGN_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                ALIGN_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                ALIGN_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /**
     * Reads the expected starting pose from a PathPlanner auto and mirrors it for the Red alliance when needed.
     * <p>
     * PathPlanner stores all poses in Blue-alliance coordinates. If we are on Red the pose is flipped across the field center line.
     * </p>
     *
     * @param alliance    current alliance color
     * @param autoCommand loaded PathPlanner auto to read the start pose from
     * @param autoName    human-readable auto name used in warning messages
     * @return starting pose in the correct alliance frame, or {@code null} if the auto has no recorded start pose
     */
    private Pose2d resolveStartingPose(Alliance alliance, PathPlannerAuto autoCommand, String autoName) {
        Pose2d startingPose = autoCommand.getStartingPose();
        if (startingPose == null) {
            RobotEnvironment.reportWarning("Auto start pose missing for '" + autoName + "'. Skipping pose reset.", false);
            return null;
        }

        // Flip for Red so the robot lines up on the correct half of the field.
        if (alliance == Alliance.Red) {
            return FlippingUtil.flipFieldPose(startingPose);
        }

        return startingPose;
    }
}
