package frc.robot.shared.config;

/**
 * Configuration values for subsystems that follow a trapezoidal motion profile.
 * <p>
 * Motion profile parameters (velocity, acceleration, tolerances, initial state) are organized into a nested {@link SetAndSeekMotionConfig} bundle.
 * Setpoint limits remain at the subsystem level because they set the clamping range for incoming targets. PID and feedforward gains are inherited
 * from {@link AbstractMotorSubsystemConfig}.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystemConfig extends AbstractMotorSubsystemConfig {

    /** Trapezoidal motion profile parameters for this set-and-seek subsystem. */
    public SetAndSeekMotionConfig motionProfile = new SetAndSeekMotionConfig();

    /** Minimum allowed setpoint for the profile, in degrees. */
    public double                 minimumSetpointDegrees;

    /** Maximum allowed setpoint for the profile, in degrees. */
    public double                 maximumSetpointDegrees;

    /**
     * Returns the minimum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return minimum allowed setpoint (degrees)
     */
    public double getMinimumSetpointDegrees() {
        return readTunableDegrees("minimumSetpointDegrees", minimumSetpointDegrees);
    }

    /**
     * Returns the minimum setpoint in radians.
     *
     * @return minimum allowed setpoint (radians)
     */
    public double getMinimumSetpointRadians() {
        return readTunableDegreesAsRadians("minimumSetpointDegrees", minimumSetpointDegrees);
    }

    /**
     * Returns the maximum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return maximum allowed setpoint (degrees)
     */
    public double getMaximumSetpointDegrees() {
        return readTunableDegrees("maximumSetpointDegrees", maximumSetpointDegrees);
    }

    /**
     * Returns the maximum setpoint in radians.
     *
     * @return maximum allowed setpoint (radians)
     */
    public double getMaximumSetpointRadians() {
        return readTunableDegreesAsRadians("maximumSetpointDegrees", maximumSetpointDegrees);
    }
}