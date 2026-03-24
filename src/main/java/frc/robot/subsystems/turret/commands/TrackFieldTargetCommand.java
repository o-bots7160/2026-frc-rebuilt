package frc.robot.subsystems.turret.commands;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.shared.targeting.ShootOnTheMoveCalculator;
import frc.robot.subsystems.robotpose.RobotPoseSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Continuously aims the turret at a field-relative target position with shoot-on-the-move compensation.
 * <p>
 * This command reads the fused robot pose each loop, runs the SOTM solver to compute where the turret should aim (accounting for robot velocity and
 * ball flight time), and drives the turret using the set-and-seek profile. It does not finish on its own and should be interrupted when tracking is
 * no longer required.
 * </p>
 */
public class TrackFieldTargetCommand extends AbstractSubsystemCommand<TurretSubsystem> {

    private final RobotPoseSubsystem      robotPoseSubsystem;

    private final Supplier<Translation2d> targetFieldPositionSupplier;

    private final Supplier<Double>        robotYawRateRadiansPerSecondSupplier;

    private final Supplier<ChassisSpeeds> fieldVelocitySupplier;

    private final DoubleUnaryOperator     tofLookup;

    private final ShootOnTheMoveCalculator sotmCalculator;

    /** Most recent compensated distance from the SOTM solver, exposed for the shooter RPM pipeline. */
    private volatile double lastCompensatedDistanceMeters;

    /**
     * Creates a command that tracks a field-relative target position with shoot-on-the-move compensation.
     * <p>
     * The target supplier should return the current target location in meters on the field coordinate system. The yaw rate supplier provides the
     * robot's current rotational velocity so the turret can lead its aim while the robot spins. The field velocity supplier and TOF lookup feed the
     * SOTM solver so the turret compensates for ball drift during translational motion.
     * </p>
     *
     * @param turretSubsystem                      turret subsystem to control
     * @param robotPoseSubsystem                   robot pose subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier          supplier of the field-relative target position in meters
     * @param robotYawRateRadiansPerSecondSupplier supplier of the robot's yaw rate in radians per second (positive is counter-clockwise)
     * @param fieldVelocitySupplier                supplier of the robot's field-relative velocity for SOTM compensation
     * @param tofLookup                            function returning estimated time of flight given a distance in meters
     * @param sotmCalculator                       shoot-on-the-move solver instance
     */
    public TrackFieldTargetCommand(
            TurretSubsystem turretSubsystem,
            RobotPoseSubsystem robotPoseSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier,
            Supplier<Double> robotYawRateRadiansPerSecondSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            DoubleUnaryOperator tofLookup,
            ShootOnTheMoveCalculator sotmCalculator) {
        super(turretSubsystem);
        this.robotPoseSubsystem                   = robotPoseSubsystem;
        this.targetFieldPositionSupplier          = targetFieldPositionSupplier;
        this.robotYawRateRadiansPerSecondSupplier = robotYawRateRadiansPerSecondSupplier;
        this.fieldVelocitySupplier                = fieldVelocitySupplier;
        this.tofLookup                            = tofLookup;
        this.sotmCalculator                       = sotmCalculator;
        this.lastCompensatedDistanceMeters        = 0.0;
    }

    /**
     * Returns the most recent compensated distance computed by the SOTM solver.
     * <p>
     * Use this as the distance supplier for the shooter RPM lookup during fire-ready so the flywheel speed matches the effective shot distance
     * rather than the raw straight-line distance.
     * </p>
     *
     * @return compensated distance in meters from the launcher to the virtual target
     */
    public double getCompensatedDistanceMeters() {
        return lastCompensatedDistanceMeters;
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
     * Reads the current robot pose, runs the SOTM solver, computes the turret angle, logs telemetry, and pushes the new setpoint to the turret
     * subsystem.
     */
    private void updateTarget() {
        Pose2d        robotPose           = robotPoseSubsystem.getEstimatedPose();
        Translation2d targetFieldPosition = targetFieldPositionSupplier.get();
        double        yawRateRadians      = robotYawRateRadiansPerSecondSupplier.get();
        ChassisSpeeds fieldVelocity       = fieldVelocitySupplier.get();

        // Run the SOTM solver to compute the compensated aim point.
        Translation2d launcherFieldPos = subsystem.getTurretFieldPosition(robotPose);
        ShootOnTheMoveCalculator.ShotSolution solution = sotmCalculator.solve(
                launcherFieldPos,
                fieldVelocity,
                subsystem.getConfig().componentPoseConfig.componentPivotX,
                subsystem.getConfig().componentPoseConfig.componentPivotY,
                robotPose.getRotation().getRadians(),
                targetFieldPosition,
                tofLookup);

        lastCompensatedDistanceMeters = solution.compensatedDistanceMeters();

        // Aim the turret at the SOTM-compensated target instead of the raw target.
        Translation2d aimTarget     = solution.compensatedTargetPosition();
        double        targetDegrees = subsystem.calculateFieldTargetDegrees(robotPose, aimTarget, yawRateRadians);

        // Telemetry
        log.recordVerboseOutput("TargetPose", new Pose2d(targetFieldPosition, new Rotation2d()));
        log.recordVerboseOutput("RobotPose", robotPose);
        log.recordVerboseOutput("TargetDegrees", targetDegrees);
        log.recordVerboseOutput("RobotYawRateRadiansPerSecond", yawRateRadians);
        log.recordOutput("SOTM/CompensatedTargetX", aimTarget.getX());
        log.recordOutput("SOTM/CompensatedTargetY", aimTarget.getY());
        log.recordOutput("SOTM/CompensatedDistanceMeters", solution.compensatedDistanceMeters());
        log.recordOutput("SOTM/RawDistanceMeters", launcherFieldPos.getDistance(targetFieldPosition));
        log.recordOutput("SOTM/TimeOfFlightSeconds", solution.timeOfFlightSeconds());
        log.recordOutput("SOTM/RobotSpeedMetersPerSecond", Math.hypot(
                fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));
        log.recordOutput("SOTM/ConvergenceIterations", solution.convergenceIterations());
        log.recordOutput("SOTM/Active", solution.sotmActive());

        subsystem.setTarget(targetDegrees);
    }
}
