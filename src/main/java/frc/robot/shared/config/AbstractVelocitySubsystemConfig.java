package frc.robot.shared.config;

import edu.wpi.first.math.util.Units;

/**
 * Configuration values for subsystems that maintain a target velocity using feedforward and PID control.
 * <p>
 * Velocity motion profile parameters (max velocity, acceleration, tolerances, settle time, idle velocity) are organized into a nested
 * {@link VelocityMotionConfig} bundle. All RPM values represent mechanism (flywheel) speed after gear reduction, not motor shaft speed. PID and
 * feedforward gains are inherited from {@link AbstractMotorSubsystemConfig}.
 * </p>
 */
public abstract class AbstractVelocitySubsystemConfig extends AbstractMotorSubsystemConfig {

    /**
     * Converts an RPM value to radians per second.
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in radians per second
     * @deprecated Use {@link VelocityMotionConfig#rpmToRadiansPerSecond(double)} instead.
     */
    @Deprecated
    public static double rpmToRadiansPerSecond(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    /**
     * Converts an RPM value to degrees per second.
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in degrees per second
     * @deprecated Use {@link VelocityMotionConfig#rpmToDegreesPerSecond(double)} instead.
     */
    @Deprecated
    public static double rpmToDegreesPerSecond(double rpm) {
        return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /** Velocity motion profile parameters for this velocity subsystem. */
    public VelocityMotionConfig motionProfile = new VelocityMotionConfig();

    /**
     * Returns the maximum mechanism velocity by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return max velocity in RPM
     * @deprecated Use {@code motionProfile.getMaximumVelocityRpm()} instead.
     */
    @Deprecated
    public double getMaximumVelocityRpm() {
        return motionProfile.getMaximumVelocityRpm();
    }

    /**
     * Returns the maximum mechanism velocity in radians per second by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return max velocity in radians per second
     * @deprecated Use {@code motionProfile.getMaximumVelocityRadiansPerSecond()} instead.
     */
    @Deprecated
    public double getMaximumVelocityRadiansPerSecond() {
        return motionProfile.getMaximumVelocityRadiansPerSecond();
    }

    /**
     * Returns the maximum acceleration for the velocity ramp by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return max acceleration in RPM per second
     * @deprecated Use {@code motionProfile.getMaximumAccelerationRpmPerSecond()} instead.
     */
    @Deprecated
    public double getMaximumAccelerationRpmPerSecond() {
        return motionProfile.getMaximumAccelerationRpmPerSecond();
    }

    /**
     * Returns the maximum acceleration in radians per second squared by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return max acceleration in radians per second squared
     * @deprecated Use {@code motionProfile.getMaximumAccelerationRadiansPerSecondSquared()} instead.
     */
    @Deprecated
    public double getMaximumAccelerationRadiansPerSecondSquared() {
        return motionProfile.getMaximumAccelerationRadiansPerSecondSquared();
    }

    /**
     * Returns the acceptable velocity error for the at-target check by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return velocity tolerance in RPM
     * @deprecated Use {@code motionProfile.getVelocityToleranceRpm()} instead.
     */
    @Deprecated
    public double getVelocityToleranceRpm() {
        return motionProfile.getVelocityToleranceRpm();
    }

    /**
     * Returns the acceptable velocity error in radians per second by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return velocity tolerance in radians per second
     * @deprecated Use {@code motionProfile.getVelocityToleranceRadiansPerSecond()} instead.
     */
    @Deprecated
    public double getVelocityToleranceRadiansPerSecond() {
        return motionProfile.getVelocityToleranceRadiansPerSecond();
    }

    /**
     * Returns how long the velocity must remain within tolerance before reporting ready by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return settle time in seconds
     * @deprecated Use {@code motionProfile.getSettleTimeSeconds()} instead.
     */
    @Deprecated
    public double getSettleTimeSeconds() {
        return motionProfile.getSettleTimeSeconds();
    }

    /**
     * Returns the default idle velocity by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return idle velocity in RPM
     * @deprecated Use {@code motionProfile.getIdleVelocityRpm()} instead.
     */
    @Deprecated
    public double getIdleVelocityRpm() {
        return motionProfile.getIdleVelocityRpm();
    }

    /**
     * Returns the default idle velocity in radians per second by delegating to the nested {@link VelocityMotionConfig}.
     *
     * @return idle velocity in radians per second
     * @deprecated Use {@code motionProfile.getIdleVelocityRadiansPerSecond()} instead.
     */
    @Deprecated
    public double getIdleVelocityRadiansPerSecond() {
        return motionProfile.getIdleVelocityRadiansPerSecond();
    }
}
