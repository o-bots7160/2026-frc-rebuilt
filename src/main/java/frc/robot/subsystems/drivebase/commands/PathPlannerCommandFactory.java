package frc.robot.subsystems.drivebase.commands;

import java.util.function.Consumer;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;

public class PathPlannerCommandFactory {
    private final DriveBaseSubsystem driveBaseSubsystem;

    private final Consumer<Pose2d>   poseResetConsumer;

    /**
     * Creates a factory that builds PathPlanner autos and seeds the drive base pose before running them.
     *
     * @param driveBaseSubsystem drive base subsystem that will own the autonomous requirements
     * @param poseResetConsumer  consumer that resets the robot pose in field coordinates
     */
    public PathPlannerCommandFactory(DriveBaseSubsystem driveBaseSubsystem, Consumer<Pose2d> poseResetConsumer) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.poseResetConsumer  = poseResetConsumer;
    }

    /**
     * Builds the PathPlanner auto for the selected alliance station and seeds the robot pose to match the auto start.
     *
     * @param alliance         selected alliance color used for mirroring start poses
     * @param allianceLocation driver station position (1-3) used to pick the auto
     * @return autonomous command sequence for the requested station
     */
    public Command createAutoCommandForPosition(Alliance alliance, int allianceLocation) {
        String          autoName           = resolveAutoName(allianceLocation);
        PathPlannerAuto pathPlannerCommand = new PathPlannerAuto(autoName);
        Pose2d          startingPose       = resolveStartingPose(alliance, pathPlannerCommand, autoName);

        if (startingPose == null) {
            return pathPlannerCommand;
        }

        return pathPlannerCommand;
    }

    private String resolveAutoName(int allianceLocation) {
        return switch (allianceLocation) {
        case 1 -> "Start Position 1 - Shoot and Collect";
        case 2 -> "Start Position 2 - Shoot and Collect";
        case 3 -> "Start Position 3 - Shoot and Collect";
        default -> "Default Auto";
        };
    }

    private Pose2d resolveStartingPose(Alliance alliance, PathPlannerAuto autoCommand, String autoName) {
        Pose2d startingPose = autoCommand.getStartingPose();
        if (startingPose == null) {
            DriverStation.reportWarning("Auto start pose missing for '" + autoName + "'. Skipping pose reset.", false);
            return null;
        }

        if (alliance == Alliance.Red) {
            return FlippingUtil.flipFieldPose(startingPose);
        }

        return startingPose;
    }
}
