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

    /** Reverse velocity used for clearing stuck pieces, in RPM. Set to 0 to disable reverse mode. */
    public double             reverseVelocityRpm;

    /**
     * Lookup table mapping distance in meters to target RPM for distance-based shooting.
     * <p>
     * The subsystem linearly interpolates between these points at runtime. Add more points for finer control over the distance-to-RPM curve. Points
     * do not need to be sorted; the interpolation table handles ordering.
     * </p>
     */
    public DistanceRpmPoint[] distanceRpmPoints          = new DistanceRpmPoint[0];

    /** Minimum RPM floor for distance-based shooting. Values below this are clamped up. */
    public double             minimumShootingRpm         = 1500.0;

    /** Maximum RPM ceiling for distance-based shooting. Values above this are clamped down. */
    public double             maximumShootingRpm         = 5000.0;

    /** Scale factor applied to the interpolated RPM for quick field adjustments. */
    public double             distanceRpmMultiplier      = 1.0;

    /**
     * Returns the reverse velocity for clearing jams, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }

    /**
     * Returns the minimum RPM floor for distance-based shooting, tuned via SmartDashboard.
     *
     * @return minimum shooting RPM
     */
    public double getMinimumShootingRpm() {
        return readTunableNumber("minimumShootingRpm", minimumShootingRpm);
    }

    /**
     * Returns the maximum RPM ceiling for distance-based shooting, tuned via SmartDashboard.
     *
     * @return maximum shooting RPM
     */
    public double getMaximumShootingRpm() {
        return readTunableNumber("maximumShootingRpm", maximumShootingRpm);
    }

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
