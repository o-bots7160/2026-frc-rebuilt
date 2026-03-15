package frc.robot.subsystems.shooter.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the shooter subsystem. All RPM values represent flywheel (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Add shooter-specific
 * tuning fields here as the design evolves (e.g., reverse velocity for clearing jams).
 * </p>
 */
public class ShooterSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the primary shooter flywheel motor. */
    public ShooterMotorConfig shooterMotorConfig         = new ShooterMotorConfig();

    /**
     * Motor configuration bundle for the follower shooter flywheel motor.
     * <p>
     * The follower can have independent inversion, current limits, and CAN ID. Enable or disable the follower via the {@link #followerEnabled} flag.
     * </p>
     */
    public ShooterMotorConfig shooterFollowerMotorConfig = new ShooterMotorConfig();

    /**
     * Enables or disables the follower motor.
     * <p>
     * Set to {@code false} in JSON when the robot has only one shooter motor (e.g., the test robot). The subsystem operates with a single motor
     * transparently.
     * </p>
     */
    public boolean            followerEnabled            = true;

    /**
     * Lookup table mapping distance in meters to target RPM for distance-based shooting.
     * <p>
     * The subsystem linearly interpolates between these points at runtime. Add more points for finer control over the distance-to-RPM curve. Points
     * do not need to be sorted; the interpolation table handles ordering.
     * </p>
     */
    public DistanceRpmPoint[] distanceRpmPoints          = new DistanceRpmPoint[0];

    /** Scale factor applied to the interpolated RPM for quick field adjustments. */
    public double             distanceRpmMultiplier      = 1.0;

    /**
     * Returns the scale factor applied to interpolated distance-based RPM, tuned via SmartDashboard.
     * <p>
     * Use this to quickly adjust all distance-based RPM values up or down without changing individual lookup table points. A value of 1.0 applies no
     * scaling.
     * </p>
     *
     * @return RPM multiplier (1.0 = no change)
     */
    public double getDistanceRpmMultiplier() {
        return readTunableNumber("distanceRpmMultiplier", distanceRpmMultiplier);
    }
}
