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

    private final Supplier<Double>        robotYawRateRadiansPerSecondSupplier;

    /**
     * Creates a command that tracks a field-relative target position while compensating for robot rotation.
     * <p>
     * The target supplier should return the current target location in meters on the field coordinate system. The yaw rate supplier provides the
     * robot's current rotational velocity so the turret can lead its aim while the robot spins.
     * </p>
     *
     * @param turretSubsystem                       turret subsystem to control
     * @param robotStateSubsystem                   robot state subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier            supplier of the field-relative target position in meters
     * @param robotYawRateRadiansPerSecondSupplier   supplier of the robot's yaw rate in radians per second (positive is counter-clockwise)
     */
    public TrackFieldTargetCommand(
            TurretSubsystem turretSubsystem,
            RobotStateSubsystem robotStateSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier) {
        super(turretSubsystem);
        this.robotStateSubsystem                     = robotStateSubsystem;
        this.targetFieldPositionSupplier              = targetFieldPositionSupplier;
        this.robotYawRateRadiansPerSecondSupplier     = robotYawRateRadiansPerSecondSupplier;
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
        double        yawRateRadians      = robotYawRateRadiansPerSecondSupplier.get();
        double        targetDegrees       = subsystem.calculateFieldTargetDegrees(robotPose, targetFieldPosition, yawRateRadians);
        if (subsystem.isVerbose()) {
            // Log inputs and the computed target so we can verify field-relative math in AdvantageScope.
            log.recordOutput("RobotPose", robotPose);
            log.recordOutput(
                    "TargetFieldPositionMeters",
                    new double[] { targetFieldPosition.getX(), targetFieldPosition.getY() });
            log.recordOutput("TargetDegrees", targetDegrees);
            log.recordOutput("RobotYawRateRadiansPerSecond", yawRateRadians);
        }
        subsystem.setTarget(targetDegrees);
    }
}
