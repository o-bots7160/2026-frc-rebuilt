package frc.robot.shared.config;

/**
 * Configuration values for subsystems that follow a trapezoidal motion profile.
 * <p>
 * Motion profile parameters (velocity, acceleration, tolerances, initial state) are organized into a nested {@link SetAndSeekMotionConfig} bundle.
 * Setpoint limits are read from the motor configuration's soft limits so there is a single source of truth for the mechanism's safe travel range.
 * PID and feedforward gains are inherited from {@link AbstractMotorSubsystemConfig}.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystemConfig extends AbstractMotorSubsystemConfig {

    /** Trapezoidal motion profile parameters for this set-and-seek subsystem. */
    public SetAndSeekMotionConfig motionProfile = new SetAndSeekMotionConfig();

    /**
     * Returns the minimum setpoint derived from the motor's reverse soft limit.
     *
     * @return minimum allowed setpoint in degrees
     */
    public double getMinimumSetpointDegrees() {
        return motorConfig.getReverseSoftLimitDegrees();
    }

    /**
     * Returns the minimum setpoint derived from the motor's reverse soft limit.
     *
     * @return minimum allowed setpoint in radians
     */
    public double getMinimumSetpointRadians() {
        return motorConfig.getReverseSoftLimitRadians();
    }

    /**
     * Returns the maximum setpoint derived from the motor's forward soft limit.
     *
     * @return maximum allowed setpoint in degrees
     */
    public double getMaximumSetpointDegrees() {
        return motorConfig.getForwardSoftLimitDegrees();
    }

    /**
     * Returns the maximum setpoint derived from the motor's forward soft limit.
     *
     * @return maximum allowed setpoint in radians
     */
    public double getMaximumSetpointRadians() {
        return motorConfig.getForwardSoftLimitRadians();
    }
}