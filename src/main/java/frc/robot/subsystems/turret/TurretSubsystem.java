package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.turret.devices.TurretMotor;
import frc.robot.subsystems.turret.devices.TurretSimMotor;

/**
 * Turret subsystem with a single profiled motor. Exposes the set-and-seek API so callers can drive to angles in degrees while the superclass handles
 * motion profiling, limits, and logging.
 */
public class TurretSubsystem extends AbstractSetAndSeekSubsystem<TurretSubsystemConfig> {
    /**
     * Selects the correct motor implementation (real or sim) based on the runtime environment.
     *
     * @param config turret configuration bundle used to construct the motor
     * @return configured motor instance, or {@code null} when the subsystem is disabled
     */
    private static Motor buildMotor(TurretSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? TurretMotor.create(config.turretMotorConfig)
                : TurretSimMotor.create(
                        config.turretMotorConfig,
                        config.motionProfile::getMaximumVelocityDegreesPerSecond,
                        config.motionProfile::getMaximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Builds the turret subsystem with a single SparkMax-driven motor and default motion profile values.
     *
     * @param config turret configuration bundle loaded from JSON; angles are expressed in degrees
     */
    public TurretSubsystem(TurretSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    /**
     * Creates the turret subsystem with an explicit motor instance.
     *
     * @param config turret configuration bundle
     * @param motor  motor to drive, or {@code null} when the subsystem is disabled
     */
    private TurretSubsystem(TurretSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Publishes the simulated turret's 3D pose for AdvantageScope animation.
     * <p>
     * The pose combines the configured pivot offset with a Z-axis (yaw) rotation matching the measured turret angle, adjusted by the turret zero
     * offset so 0° points backward in simulation. This ensures the simulated model matches the real robot's orientation when the turret is
     * rear-facing.
     * </p>
     */
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        double poseDegrees         = getMeasuredPositionDegrees();
        Pose3d turretComponentPose = new Pose3d(
                config.componentPoseConfig.toTranslation3d(),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(poseDegrees)));
        log.recordOutput("componentPoses", new Pose3d[] { turretComponentPose });
    }

    /**
     * Computes the turret pivot position on the field by transforming the configured component offset from robot-relative to field-relative
     * coordinates.
     * <p>
     * The offset comes from {@code componentPoseConfig} (the same values used for AdvantageScope rendering), so the sim model and the aim
     * calculations always agree on where the turret sits. This method is used by {@link #calculateFieldTargetDegrees} and by the shoot-on-the-move
     * solver for distance and velocity calculations.
     * </p>
     *
     * @param robotPose current robot pose in meters and radians
     * @return turret pivot position on the field in meters
     */
    public Translation2d getTurretFieldPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        double cosH    = Math.cos(heading);
        double sinH    = Math.sin(heading);
        double pivotX  = config.componentPoseConfig.componentPivotX;
        double pivotY  = config.componentPoseConfig.componentPivotY;
        return new Translation2d(
                robotPose.getX() + pivotX * cosH - pivotY * sinH,
                robotPose.getY() + pivotX * sinH + pivotY * cosH);
    }

    /**
     * Computes the turret target angle needed to face a field-relative target while compensating for robot rotation.
     * <p>
     * The returned angle is relative to the turret's own zero direction, which is defined by {@code turretZeroOffsetDegrees}. For a rear-facing
     * turret (offset = 180°), turret 0° points straight backward, positive angles rotate counter-clockwise when viewed from above, and the configured
     * setpoint limits bound how far the turret can swing from its zero. The formula is:
     * </p>
     * <p>
     * {@code turretAngle = fieldAngleToTarget − robotHeading − turretZeroOffset}
     * </p>
     * <p>
     * The angle is measured from the turret's physical pivot point (defined by {@code componentPoseConfig}) rather than from the robot center. This
     * eliminates a small aim error that would otherwise grow at close range.
     * </p>
     * <p>
     * Rotational lead-time compensation predicts where the robot heading will be after a short look-ahead period and subtracts that predicted change
     * so the turret pre-rotates instead of lagging behind. The result is clamped to the configured turret limits.
     * </p>
     *
     * @param robotPose                    current robot pose in meters and radians
     * @param targetFieldPositionMeters    target position on the field in meters
     * @param robotYawRateRadiansPerSecond current robot rotational velocity in radians per second (positive is counter-clockwise)
     * @return turret target angle in degrees, relative to turret zero
     */
    public double calculateFieldTargetDegrees(
            Pose2d robotPose,
            Translation2d targetFieldPositionMeters,
            double robotYawRateRadiansPerSecond) {

        // Compute the vector from the turret pivot to the target in field coordinates.
        // The pivot position accounts for the turret's physical offset from robot center
        // (componentPoseConfig), ensuring the aim angle is measured from where the ball
        // actually launches rather than from the robot's geometric center.
        Translation2d turretFieldPos             = getTurretFieldPosition(robotPose);
        double deltaX                            = targetFieldPositionMeters.getX() - turretFieldPos.getX();
        double deltaY                            = targetFieldPositionMeters.getY() - turretFieldPos.getY();

        // Compute the field-relative angle from the robot to the target using atan2.
        // atan2 returns an angle in radians measured counter-clockwise from the +X axis,
        // matching the WPILib field coordinate convention.
        //
        // Forward-facing example: deltaX = 2, deltaY = 2 → atan2(2, 2) = π/4 (45°), target is northeast.
        // Rear-facing example:    deltaX = -2, deltaY = 0 → atan2(0, -2) = π (180°), target is due west.
        double fieldAngleRadians             = Math.atan2(deltaY, deltaX);

        // Read the robot's current heading in radians. Heading 0 means the robot's front faces the +X
        // direction (toward the opposing alliance wall). Positive rotation is counter-clockwise (CCW).
        double robotHeadingRadians           = robotPose.getRotation().getRadians();

        // Convert the turret zero offset from degrees to radians. This offset defines where the turret's
        // 0° position points relative to the front of the robot.
        //   Forward-facing turret: offset =   0° → turret 0° aligns with robot front.
        //   Rear-facing turret:    offset = 180° → turret 0° aligns with robot rear.
        double zeroOffsetRadians             = Units.degreesToRadians(config.getTurretZeroOffsetDegrees());

        // Core formula: convert the field-relative angle into a turret-relative angle.
        //   turretAngle = fieldAngleToTarget − robotHeading − turretZeroOffset
        //
        // Subtracting robotHeading rotates from the field frame into the robot frame (making the angle
        // relative to the robot's front). Subtracting zeroOffset then rotates from the robot frame into
        // the turret frame (making the angle relative to wherever the turret's mechanical zero points).
        //
        // Forward-facing turret example (offset = 0°, heading = 0°):
        //   Robot at (1, 1) facing east, target at (3, 3). fieldAngle = π/4 (45°).
        //   rawTarget = π/4 − 0 − 0 = π/4 (45°)
        //   Meaning: turret turns 45° CCW from robot-forward to face the target. Correct.
        //
        // Rear-facing turret example (offset = 180°, heading = 0°):
        //   Robot at (1, 1) facing east, target at (-1, 1) (due west, directly behind the robot).
        //   fieldAngle = π (180°).
        //   rawTarget = π − 0 − π = 0
        //   Meaning: turret stays at 0° (pointing straight back). The target is directly behind. Correct.
        //
        // Rear-facing turret, target to the side (offset = 180°, heading = 0°):
        //   Robot at (1, 1) facing east, target at (1, 3) (due north).
        //   fieldAngle = π/2 (90°).
        //   rawTarget = π/2 − 0 − π = −π/2 (−90°)
        //   Meaning: turret swings −90° from its backward-facing zero, which points north. Correct.
        //
        // Forward-facing turret with rotated robot (offset = 0°, heading = π/2 = 90° CCW):
        //   Robot at (1, 1) facing north, target at (3, 1) (due east, to the robot's right).
        //   fieldAngle = atan2(0, 2) = 0.
        //   rawTarget = 0 − π/2 − 0 = −π/2 (−90°)
        //   Meaning: turret turns 90° clockwise from forward (to the robot's right). Correct.
        double rawTargetRadians              = fieldAngleRadians - robotHeadingRadians - zeroOffsetRadians;

        // Compensate for robot rotation so the turret leads its aim instead of lagging behind.
        // When the robot spins, the heading changes every control cycle. Without compensation the turret
        // aims at where the target was relative to the robot, not where it will be by the time the
        // turret finishes moving.
        //
        // Lead time (seconds) multiplied by yaw rate (rad/s) gives the predicted heading change.
        // We subtract it because a positive yaw rate (CCW spin) increases the heading, and since the
        // formula is (fieldAngle − heading − offset), a larger heading produces a smaller turret angle.
        // Subtracting the predicted change now makes the turret pre-rotate in the opposite direction,
        // anticipating the robot's spin.
        //
        // Example: robot spinning CCW at 1 rad/s, lead time = 0.1 s.
        //   compensation = 1.0 × 0.1 = 0.1 rad (≈ 5.7°).
        //   If rawTarget was π/4 (45°), compensated = π/4 − 0.1 ≈ 0.685 rad (≈ 39.3°).
        //   The turret aims about 5.7° ahead of the raw target, anticipating that the robot will spin
        //   5.7° CCW during the next 0.1 s, which will bring the target back to the turret's bore line.
        //
        // Rear-facing turret at limit, robot spinning CW at −2 rad/s, lead time = 0.1 s:
        //   compensation = −2.0 × 0.1 = −0.2 rad (≈ −11.5°).
        //   If rawTarget was −π/2 (−90°), compensated = −π/2 − (−0.2) = −π/2 + 0.2 ≈ −1.37 rad (≈ −78.5°).
        //   The turret eases away from its limit because the CW spin will push the target further
        //   into the reachable range.
        //
        // Set rotationalLeadTimeSeconds to 0 in config to disable compensation entirely.
        double leadTimeSeconds               = config.getRotationalLeadTimeSeconds();
        double rotationalCompensationRadians = robotYawRateRadiansPerSecond * leadTimeSeconds;
        double compensatedTargetRadians      = rawTargetRadians - rotationalCompensationRadians;

        // Clamp the compensated angle to the turret's physical swing limits, then convert to degrees.
        // This method returns a pure math-frame result (positive = CCW). If the real motor's positive
        // direction is CW, handle that via motorInverted = true in the motor config rather than
        // negating here. Motor inversion flips both the SparkMax output and built-in encoder, keeping
        // the PID frame consistent without polluting the targeting math.
        //
        // Forward-facing turret example (limits −90° to +90°):
        //   compensatedTarget = π/4 (45°) → after clamp = π/4 (within limits)
        //   Return: 45° → motor drives to +45° encoder (CCW from zero). Correct.
        //
        // Rear-facing turret, unreachable target (limits −90° to +90°):
        //   Robot facing east, target due east (directly in front). fieldAngle = 0, offset = π.
        //   rawTarget = 0 − 0 − π = −π → normalized to −π by angleModulus.
        //   Return: −180° — the angle is outside the turret's reachable range. setTarget will
        //   clamp it to the nearest limit and flag targetWasClamped so isOnTarget returns false.
        return Units.radiansToDegrees(normalizeAngleRadians(compensatedTargetRadians));
    }

    /**
     * Sets a new turret goal after normalizing the angle into the [-180, 180] range.
     * <p>
     * The superclass clamp then detects whether the normalized angle falls outside the turret's
     * setpoint limits and flags it so {@link #isOnTarget()} correctly returns false for unreachable targets.
     * </p>
     *
     * @param targetPositionDegrees desired turret position in degrees, relative to turret zero
     */
    @Override
    public void setTarget(double targetPositionDegrees) {
        double normalizedDegrees = Units.radiansToDegrees(
                MathUtil.angleModulus(Units.degreesToRadians(targetPositionDegrees)));
        super.setTarget(normalizedDegrees);
    }

    /**
     * Checks whether the turret is aimed close enough to shoot, ignoring velocity.
     * <p>
     * Returns false when the requested target was clamped (target is outside the turret's reachable arc).
     * Otherwise compares the position error against {@code onTargetPositionToleranceDegrees} from the turret config.
     * </p>
     *
     * @return true when the turret is within the on-target tolerance of its goal
     */
    public boolean isOnTarget() {
        if (targetWasClamped) {
            log.recordOutput("onTarget", false);
            return false;
        }

        double positionErrorRadians = Math.abs(goalState.position - getMeasuredPosition());
        double toleranceRadians     = Units.degreesToRadians(config.getOnTargetPositionToleranceDegrees());
        boolean onTarget            = positionErrorRadians <= toleranceRadians;
        log.recordOutput("onTarget", onTarget);
        return onTarget;
    }

    /**
     * Normalizes a turret target angle into the [-pi, pi] range.
     * <p>
     * Wrapping ensures equivalent angles are recognized before the superclass clamps to the
     * configured setpoint limits. Without normalization, angles like 3pi/2 (270 degrees) would
     * not be recognized as equivalent to -pi/2 (-90 degrees).
     * </p>
     *
     * @param targetRadians requested turret angle in radians, relative to turret zero
     * @return normalized angle in the [-pi, pi] range in radians
     */
    private double normalizeAngleRadians(double targetRadians) {
        return MathUtil.angleModulus(targetRadians);
    }

}
