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
    private static Motor buildMotor(TurretSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? TurretMotor.create(config.turretMotorConfig)
                : TurretSimMotor.create(
                        config.turretMotorConfig,
                        config::getMaximumVelocityDegreesPerSecond,
                        config::getMaximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Builds the turret subsystem with a single SparkMax-driven motor and default motion profile values.
     *
     * @param config turret configuration bundle loaded from JSON; angles are expressed in degrees
     */
    public TurretSubsystem(TurretSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private TurretSubsystem(TurretSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Publishes the turret's 3D component pose each cycle so AdvantageScope can animate the turret model.
     * <p>
     * The component pose combines the configured pivot offset with a Z-axis (yaw) rotation matching the measured turret angle.
     * </p>
     */
    @Override
    public void periodic() {
        super.periodic();

        Pose3d turretComponentPose = new Pose3d(
                config.componentPoseConfig.toTranslation3d(),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(getMeasuredPositionDegrees())));

        log.recordOutput("componentPoses", new Pose3d[] { turretComponentPose });
    }

    /**
     * Computes the turret target angle needed to face a field-relative target while compensating for robot rotation.
     * <p>
     * When the turret faces the rear of the robot ({@code rearFacingTurret} is true), the robot-relative direction to the target is negated before the
     * zero offset is applied. Negation is necessary because a backward-facing turret mirrors left and right relative to robot-forward: what is to the
     * robot's left appears on the turret's right when looking along the barrel. When the turret faces forward the standard formula is used instead. The
     * rotational lead-time compensation predicts how far the heading will change and pre-rotates the turret so it stays on target instead of lagging
     * behind. The returned angle is clamped to the configured turret limits.
     * </p>
     *
     * @param robotPose                   current robot pose in meters and radians
     * @param targetFieldPositionMeters   target position on the field in meters
     * @param robotYawRateRadiansPerSecond current robot rotational velocity in radians per second (positive is counter-clockwise)
     * @return turret target angle in degrees
     */
    public double calculateFieldTargetDegrees(
            Pose2d robotPose,
            Translation2d targetFieldPositionMeters,
            double robotYawRateRadiansPerSecond) {
        double deltaX              = targetFieldPositionMeters.getX() - robotPose.getX();
        double deltaY              = targetFieldPositionMeters.getY() - robotPose.getY();

        double fieldAngleRadians   = Math.atan2(deltaY, deltaX);
        double robotHeadingRadians = robotPose.getRotation().getRadians();
        double zeroOffsetRadians   = Units.degreesToRadians(config.getTurretZeroOffsetDegrees());

        double robotRelativeRadians = fieldAngleRadians - robotHeadingRadians;
        if (config.isRearFacingTurret()) {
            robotRelativeRadians = -robotRelativeRadians;
        }
        double rawTargetRadians = robotRelativeRadians + zeroOffsetRadians;

        // Compensate for robot rotation by predicting where the heading will be after the configured lead time.
        double leadTimeSeconds             = config.getRotationalLeadTimeSeconds();
        double rotationalCompensationRadians = robotYawRateRadiansPerSecond * leadTimeSeconds;
        double compensatedTargetRadians    = rawTargetRadians + rotationalCompensationRadians;

        return Units.radiansToDegrees(clampToTurretLimitsRadians(compensatedTargetRadians));
    }

    /**
     * Clamps a turret target angle to the configured setpoint limits.
     * <p>
     * Uses {@link MathUtil#inputModulus(double, double, double)} to wrap the target into the turret's travel range, then snaps to the nearest limit
     * if the range is smaller than a full rotation and the wrapped result is still outside the allowed band.
     * </p>
     *
     * @param targetRadians requested turret angle in radians
     * @return closest equivalent angle within the turret limits in radians
     */
    private double clampToTurretLimitsRadians(double targetRadians) {
        double minRadians = config.getMinimumSetpointRadians();
        double maxRadians = config.getMaximumSetpointRadians();

        if (minRadians > maxRadians) {
            double swap = minRadians;
            minRadians = maxRadians;
            maxRadians = swap;
        }

        if (maxRadians - minRadians <= 0.0) {
            return minRadians;
        }

        double wrapped = MathUtil.inputModulus(targetRadians, minRadians, maxRadians);
        return MathUtil.clamp(wrapped, minRadians, maxRadians);
    }

}
