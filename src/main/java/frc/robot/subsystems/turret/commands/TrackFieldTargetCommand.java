package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.robotstate.RobotStateSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Continuously aims the turret at a field-relative target position.
 * <p>
 * This command reads the fused robot pose from the Robot State subsystem each loop, computes the turret angle needed to face the target, and drives
 * the turret using the set-and-seek profile. It does not finish on its own and should be interrupted when tracking is no longer required.
 * </p>
 */
public class TrackFieldTargetCommand extends AbstractSubsystemCommand<TurretSubsystem> {

    private final RobotStateSubsystem     robotStateSubsystem;

    private final Supplier<Translation2d> targetFieldPositionSupplier;

    /**
     * Creates a command that tracks a field-relative target position.
     * <p>
     * The target supplier should return the current target location in meters on the field coordinate system.
     * </p>
     *
     * @param turretSubsystem             turret subsystem to control
     * @param robotStateSubsystem         robot state subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier supplier of the field-relative target position in meters
     */
    public TrackFieldTargetCommand(
            TurretSubsystem turretSubsystem,
            RobotStateSubsystem robotStateSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier) {
        super(turretSubsystem);
        this.robotStateSubsystem         = robotStateSubsystem;
        this.targetFieldPositionSupplier = targetFieldPositionSupplier;
    }

    @Override
    public void execute() {
        updateTarget();
        subsystem.seekTarget();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.handleSeekInterrupted();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    protected void onInitialize() {
        updateTarget();
    }

    private void updateTarget() {
        Pose2d        robotPose           = robotStateSubsystem.getEstimatedPose();
        Translation2d targetFieldPosition = targetFieldPositionSupplier.get();
        double        targetDegrees       = subsystem.calculateFieldTargetDegrees(robotPose, targetFieldPosition);
        if (subsystem.isVerbose()) {
            // Log inputs and the computed target so we can verify field-relative math in AdvantageScope.
            log.recordOutput("RobotPose", robotPose);
            log.recordOutput(
                    "TargetFieldPositionMeters",
                    new double[] { targetFieldPosition.getX(), targetFieldPosition.getY() });
            log.recordOutput("TargetDegrees", targetDegrees);
        }
        subsystem.setTarget(targetDegrees);
    }
}
