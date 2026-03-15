package frc.robot.subsystems.drivebase.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    /** All auto names available for selection, in display order. */
    private static final String[]              ALL_AUTO_NAMES                                            = {
            "b1", "b2tb1", "b2tb3", "b3", "sine", "Default Auto" };

    /**
     * Dashboard chooser that lets drivers select which autonomous routine to run.
     * <p>
     * Published to {@code SmartDashboard/Auto Chooser} at construction time.
     * </p>
     */
    private LoggedDashboardChooser<String>     autoChooser;

    /**
     * Supplier of the robot's current fused pose from odometry and vision.
     * <p>
     * Typically wired to {@code RobotPoseSubsystem::getEstimatedPose}.
     * </p>
     */
    private final Supplier<Pose2d>             currentPoseSupplier;

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
     *
     * @param currentPoseSupplier supplier of the current fused robot pose, typically {@code robotPoseSubsystem::getEstimatedPose}
     */
    public PathPlannerCommandFactory(Supplier<Pose2d> currentPoseSupplier) {
        this.currentPoseSupplier = currentPoseSupplier;
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
     * Builds the full autonomous command for the given alliance and driver-station position.
     * <p>
     * The returned command sequences an alignment path (current pose to expected start) followed by the pre-loaded PathPlanner auto. If the auto has
     * no recorded starting pose the alignment step is skipped.
     * </p>
     *
     * @param alliance         alliance color used to mirror the starting pose to the correct side of the field
     * @param allianceLocation driver-station position number (1, 2, or 3) that selects which auto to load
     * @return composite command that first aligns then runs the auto
     * @deprecated use {@link #createAutoCommand(Alliance, String)} with {@link #getSelectedAutoName()} instead
     */
    @Deprecated
    public Command createAutoCommandForPosition(Alliance alliance, int allianceLocation) {
        String autoName = resolveAutoName(allianceLocation);
        return createAutoCommand(alliance, autoName);
    }

    /**
     * Pre-loads every PathPlanner auto into the cache and populates the dashboard chooser.
     * <p>
     * Loads all autos listed in {@link #ALL_AUTO_NAMES} so they are ready when autonomous is enabled. The first entry is set as the default option in
     * the {@link LoggedDashboardChooser}. The chooser is published to {@code SmartDashboard/Auto Chooser}.
     * </p>
     */
    private void initializeAutos() {
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        boolean first = true;
        for (String autoName : ALL_AUTO_NAMES) {
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
     * Creates an on-the-fly path command that drives from the robot's current pose to the target pose.
     * <p>
     * The path ends at zero velocity with the target's holonomic rotation so the robot is stationary and facing the correct direction when the
     * planned auto takes over. Coordinates are already alliance-corrected so {@code preventFlipping} is set to avoid re-mirroring.
     * </p>
     *
     * @param targetPose field pose the robot should arrive at, already alliance-flipped if on Red
     * @return command that follows the alignment path and stops at the target
     */
    private Command buildAlignmentCommand(Pose2d targetPose) {
        Pose2d          currentPose   = currentPoseSupplier.get();

        // Direction of travel from the current position toward the target.
        Rotation2d      travelHeading = targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

        List<Waypoint>  waypoints     = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), travelHeading),
                new Pose2d(targetPose.getTranslation(), travelHeading));

        PathConstraints constraints   = new PathConstraints(
                ALIGN_MAX_VELOCITY_METERS_PER_SECOND,
                ALIGN_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                ALIGN_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                ALIGN_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, targetPose.getRotation()));

        alignmentPath.preventFlipping = true;

        return AutoBuilder.followPath(alignmentPath);
    }

    /**
     * Maps a driver-station position number to a PathPlanner auto file name.
     * <p>
     * Names must match the {@code .auto} files under {@code src/main/deploy/pathplanner/autos/}.
     * </p>
     *
     * @param allianceLocation driver-station position (1, 2, or 3)
     * @return auto file name without the {@code .auto} extension
     */
    private String resolveAutoName(int allianceLocation) {
        return switch (allianceLocation) {
        case 1 -> "b1";
        case 2 -> "sine";
        case 3 -> "b3";
        default -> "Default Auto";
        };
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
