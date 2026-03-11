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
     * so the turret pre-rotates instead of lagging behind. The returned angle is clamped to the configured turret limits.
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
        double deltaX                        = targetFieldPositionMeters.getX() - robotPose.getX();
        double deltaY                        = targetFieldPositionMeters.getY() - robotPose.getY();

        double fieldAngleRadians             = Math.atan2(deltaY, deltaX);
        double robotHeadingRadians           = robotPose.getRotation().getRadians();
        double zeroOffsetRadians             = Units.degreesToRadians(config.getTurretZeroOffsetDegrees());

        // Turret angle = direction-to-target in field frame, minus robot heading, minus turret zero offset.
        // This gives the angle relative to the turret's own zero direction.
        double rawTargetRadians              = fieldAngleRadians - robotHeadingRadians - zeroOffsetRadians;

        // Compensate for robot rotation by predicting where the heading will be after the configured lead
        // time. A positive yaw rate (CCW) increases the heading, which decreases the turret-relative angle,
        // so we subtract the predicted heading change.
        double leadTimeSeconds               = config.getRotationalLeadTimeSeconds();
        double rotationalCompensationRadians = robotYawRateRadiansPerSecond * leadTimeSeconds;
        double compensatedTargetRadians      = rawTargetRadians - rotationalCompensationRadians;

        return Units.radiansToDegrees(clampToTurretLimitsRadians(compensatedTargetRadians));
    }

    @Override
    public boolean isProfileSettled() {
        // The turret is settled when the profiled motor is at its target and the velocity is below the configured threshold.
        return true;
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
        double minRadians = config.getMinimumSetpointRadians();
        double maxRadians = config.getMaximumSetpointRadians();
        double normalized = MathUtil.angleModulus(targetRadians);
        return MathUtil.clamp(normalized, minRadians, maxRadians);
    }

}
