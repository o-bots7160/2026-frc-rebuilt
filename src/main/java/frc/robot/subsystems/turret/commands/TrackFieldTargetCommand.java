package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Continuously aims the turret at a field-relative target position.
 * <p>
 * This command reads the fused robot pose from the Robot State subsystem each loop, computes the turret angle needed to face the target, and drives
 * the turret using the set-and-seek profile. It does not finish on its own and should be interrupted when tracking is no longer required.
 * </p>
 */
public class TrackFieldTargetCommand extends AbstractSubsystemCommand<TurretSubsystem> {

    private final RobotPoseSubsystem      robotPoseSubsystem;

    private final Supplier<Translation2d> targetFieldPositionSupplier;

    private final Supplier<Double>        robotYawRateRadiansPerSecondSupplier;

    /**
     * Creates a command that tracks a field-relative target position while compensating for robot rotation.
     * <p>
     * The target supplier should return the current target location in meters on the field coordinate system. The yaw rate supplier provides the
     * robot's current rotational velocity so the turret can lead its aim while the robot spins.
     * </p>
     *
     * @param turretSubsystem                      turret subsystem to control
     * @param robotPoseSubsystem                   robot pose subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier          supplier of the field-relative target position in meters
     * @param robotYawRateRadiansPerSecondSupplier supplier of the robot's yaw rate in radians per second (positive is counter-clockwise)
     */
    public TrackFieldTargetCommand(
            TurretSubsystem turretSubsystem,
            RobotPoseSubsystem robotPoseSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier) {
        super(turretSubsystem);
        this.robotPoseSubsystem                   = robotPoseSubsystem;
        this.targetFieldPositionSupplier          = targetFieldPositionSupplier;
        this.robotYawRateRadiansPerSecondSupplier = robotYawRateRadiansPerSecondSupplier;
    }

    /**
     * Recomputes the turret target and drives the profile toward it each cycle.
     */
    @Override
    public void execute() {
        updateTarget();

        // Skip seeking when the turret is already within tolerance of the current target.
        // This prevents the profile from continuously applying small corrections that
        // cause the mechanism to stutter instead of settling cleanly.
        if (!subsystem.isProfileSettled()) {
            subsystem.seekTarget();
        }
    }

    /**
     * Notifies the turret that tracking was interrupted so it can reset its profile state.
     *
     * @param interrupted true when the command was interrupted rather than finishing normally
     */
    @Override
    public void end(boolean interrupted) {
        subsystem.handleSeekInterrupted();
    }

    /**
     * Returns false so the command tracks continuously until interrupted.
     *
     * @return always {@code false}
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Seeds the turret target with the current field-relative angle on first run. */
    @Override
    protected void onInitialize() {
        updateTarget();
    }

    /**
     * Reads the current robot pose and target position, computes the turret angle, logs telemetry,
     * and pushes the new setpoint to the turret subsystem.
     */
    private void updateTarget() {
        Pose2d        robotPose           = robotPoseSubsystem.getEstimatedPose();
        Translation2d targetFieldPosition = targetFieldPositionSupplier.get();
        double        yawRateRadians      = robotYawRateRadiansPerSecondSupplier.get();
        double        targetDegrees       = subsystem.calculateFieldTargetDegrees(robotPose, targetFieldPosition, yawRateRadians);

        log.recordOutput("TargetPose", new Pose2d(targetFieldPosition, new Rotation2d()));

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
