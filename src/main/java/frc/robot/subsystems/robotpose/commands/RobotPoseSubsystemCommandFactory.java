package frc.robot.subsystems.robotpose.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;

/**
 * Factory that creates robot pose commands so the pose subsystem stays free of command creation.
 */
public class RobotPoseSubsystemCommandFactory extends AbstractSubsystemCommandFactory<RobotPoseSubsystem> {

    /**
     * Creates a factory that produces commands operating on the provided robot pose subsystem.
     *
     * @param subsystem pose subsystem that generated commands will operate on
     */
    public RobotPoseSubsystemCommandFactory(RobotPoseSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Creates an instant command that snaps the robot pose to the newest camera (AprilTag) measurement.
     * <p>
     * Use this for a driver button or dashboard button when odometry has drifted during a match. The command does not require the pose subsystem, so
     * it can run while driving. If no vision measurement has arrived yet, the underlying subsystem skips the reset and logs a warning.
     * </p>
     *
     * @return command that resets the fused pose estimate to the latest vision pose
     */
    public Command createResetPoseFromVisionCommand() {
        return Commands.runOnce(subsystem::resetPoseFromVision)
                .ignoringDisable(true)
                .withName("Reset Pose From Vision");
    }
}
