package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.RobotEnvironment;
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
     * Computes the turret target angle needed to face a field-relative target.
     * <p>
     * The result is expressed in turret degrees, where 0 degrees aligns with robot-forward plus the configured zero offset. The returned angle is
     * clamped to the configured turret limits and will pick the closest equivalent angle within that range.
     * </p>
     *
     * @param robotPose                 current robot pose in meters and radians
     * @param targetFieldPositionMeters target position on the field in meters
     * @return turret target angle in degrees
     */
    public double calculateFieldTargetDegrees(Pose2d robotPose, Translation2d targetFieldPositionMeters) {
        double deltaX              = targetFieldPositionMeters.getX() - robotPose.getX();
        double deltaY              = targetFieldPositionMeters.getY() - robotPose.getY();

        double fieldAngleRadians   = Math.atan2(deltaY, deltaX);
        double robotHeadingRadians = robotPose.getRotation().getRadians();
        double zeroOffsetRadians   = Math.toRadians(config.getTurretZeroOffsetDegrees());
        double rawTargetRadians    = fieldAngleRadians - robotHeadingRadians + zeroOffsetRadians;
        return Math.toDegrees(clampToTurretLimitsRadians(rawTargetRadians));
    }

    /**
     * Clamps a turret target angle to the configured setpoint limits.
     * <p>
     * This method shifts the target by whole rotations to find the closest equivalent angle inside the allowed range.
     * </p>
     *
     * @param targetDegrees requested turret angle in degrees
     * @return closest equivalent angle within the turret limits
     */
    private double clampToTurretLimitsRadians(double targetRadians) {
        double minRadians = config.getMinimumSetpointRadians();
        double maxRadians = config.getMaximumSetpointRadians();

        if (minRadians > maxRadians) {
            double swap = minRadians;
            minRadians = maxRadians;
            maxRadians = swap;
        }

        double clampedTarget = targetRadians;
        double rangeRadians  = maxRadians - minRadians;
        if (rangeRadians <= 0.0) {
            return minRadians;
        }

        double fullRotation = Math.PI * 2.0;
        while (clampedTarget < minRadians) {
            clampedTarget += fullRotation;
        }

        while (clampedTarget > maxRadians) {
            clampedTarget -= fullRotation;
        }

        if (clampedTarget < minRadians || clampedTarget > maxRadians) {
            double minOffset = Math.abs(clampedTarget - minRadians);
            double maxOffset = Math.abs(maxRadians - clampedTarget);
            return minOffset <= maxOffset ? minRadians : maxRadians;
        }

        return clampedTarget;
    }

}
