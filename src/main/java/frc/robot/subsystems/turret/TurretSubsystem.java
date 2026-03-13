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

        // Compute the vector from the robot's position to the target in field coordinates.
        // deltaX points along field +X (toward the opposing alliance wall).
        // deltaY points along field +Y (to the left when facing the opposing wall).
        //
        // Forward-facing turret example (offset = 0°):
        //   Robot at (1, 1), target at (3, 3) → deltaX = 2, deltaY = 2
        //
        // Rear-facing turret example (offset = 180°):
        //   Robot at (1, 1), target at (-1, 1) → deltaX = -2, deltaY = 0
        double deltaX                        = targetFieldPositionMeters.getX() - robotPose.getX();
        double deltaY                        = targetFieldPositionMeters.getY() - robotPose.getY();

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
        //   rawTarget = 0 − 0 − π = −π → normalized to −π by angleModulus, clamped to −π/2 (−90°).
        //   Return: −90° — turret swings to its physical limit, getting as close to forward as it
        //   can. The target is unreachable because the turret faces backward, so it does the best it
        //   can by parking at the nearest limit.
        return Units.radiansToDegrees(clampToTurretLimitsRadians(compensatedTargetRadians));
    }

    /**
     * Normalizes and clamps a turret target angle to the configured setpoint limits.
     * <p>
     * The angle is first wrapped into the [-pi, pi] range with {@link MathUtil#angleModulus(double)}, then clamped to the configured min/max setpoint
     * radians. When the target is outside the turret's reachable range (e.g., a target in front of a rear-facing turret), the turret moves to the
     * nearest limit, which keeps the barrel as close to the target direction as possible.
     * </p>
     *
     * @param targetRadians requested turret angle in radians, relative to turret zero
     * @return closest reachable angle within the turret limits in radians
     */
    private double clampToTurretLimitsRadians(double targetRadians) {
        // Read the configured turret swing limits in radians (converted from degrees in the config).
        // For a turret with minimumSetpointDegrees = −90 and maximumSetpointDegrees = +90:
        //   minRadians = −π/2 (−1.571 rad), maxRadians = +π/2 (+1.571 rad)
        // This means the turret can swing 90° in either direction from its zero.
        double minRadians = config.getMinimumSetpointRadians();
        double maxRadians = config.getMaximumSetpointRadians();

        // Wrap the raw angle into the [−π, +π] range so equivalent angles are recognized.
        // Without normalization, angles like 3π/2 (270°) would not be recognized as equivalent
        // to −π/2 (−90°), and the subsequent clamp would produce incorrect results.
        //
        // Example: raw angle = 5π/4 (225°) → normalized to −3π/4 (−135°).
        //   Now the clamp correctly sees the angle is past the −90° limit.
        double normalized = MathUtil.angleModulus(targetRadians);

        // Clamp the normalized angle into [minRadians, maxRadians]. If the target falls outside the
        // turret's reachable arc, the angle snaps to whichever limit is closer.
        //
        // Forward-facing turret example (limits −π/2 to +π/2):
        //   normalized = π/4 (45°) → within limits, returned as-is.
        //
        // Rear-facing turret, target behind and to the far left:
        //   normalized = −3π/4 (−135°) → clamped to −π/2 (−90°).
        //   The turret parks at its left-most swing, the closest it can get to the target.
        return MathUtil.clamp(normalized, minRadians, maxRadians);
    }

}
