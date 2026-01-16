package frc.robot.shared.config;

import java.util.function.Supplier;

/**
 * Configuration values for subsystems that follow a trapezoidal motion profile.
 * <p>
 * Values are stored in native units for the mechanism (e.g., rotations, meters, degrees) and should be kept consistent across commands that target
 * the subsystem. Every field is mirrored to the dashboard so it can be tuned live without redeploying.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystemConfig extends AbstractSubsystemConfig {
    /** Minimum allowed setpoint for the profile, in mechanism units. */
    public double minimumSetpoint;

    /** Maximum allowed setpoint for the profile, in mechanism units. */
    public double maximumSetpoint;

    /** Maximum velocity for the trapezoidal profile, in mechanism units per second. */
    public double maximumVelocity;

    /** Maximum acceleration for the trapezoidal profile, in mechanism units per second squared. */
    public double maximumAcceleration;

    /** Acceptable position error when deciding if the mechanism is at its goal, in mechanism units. */
    public double positionTolerance;

    /** Starting position used to seed the initial profile state, in mechanism units. */
    public double initialPosition;

    /** Starting velocity used to seed the initial profile state, in mechanism units per second. */
    public double initialVelocity;

    /** Optional prefix override for dashboard keys (defaults to the config class name without the Config suffix). */
    public String dashboardPrefix;

    /**
     * Supplies the minimum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return supplier yielding the minimum allowed setpoint (mechanism units)
     */
    public Supplier<Double> getMinimumSetpointSupplier() {
        return () -> readTunableNumber("minimumSetpoint", minimumSetpoint);
    }

    /**
     * Supplies the maximum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return supplier yielding the maximum allowed setpoint (mechanism units)
     */
    public Supplier<Double> getMaximumSetpointSupplier() {
        return () -> readTunableNumber("maximumSetpoint", maximumSetpoint);
    }

    /**
     * Supplies the maximum profile velocity, tuned via SmartDashboard.
     *
     * @return supplier yielding the max velocity (mechanism units per second)
     */
    public Supplier<Double> getMaximumVelocitySupplier() {
        return () -> readTunableNumber("maximumVelocity", maximumVelocity);
    }

    /**
     * Supplies the maximum profile acceleration, tuned via SmartDashboard.
     *
     * @return supplier yielding the max acceleration (mechanism units per second squared)
     */
    public Supplier<Double> getMaximumAccelerationSupplier() {
        return () -> readTunableNumber("maximumAcceleration", maximumAcceleration);
    }

    /**
     * Supplies the allowed position error used to decide when the mechanism is at its goal.
     *
     * @return supplier yielding the position tolerance (mechanism units)
     */
    public Supplier<Double> getPositionToleranceSupplier() {
        return () -> readTunableNumber("positionTolerance", positionTolerance);
    }

    /**
     * Supplies the initial position that seeds the profile state on startup.
     *
     * @return supplier yielding the starting position (mechanism units)
     */
    public Supplier<Double> getInitialPositionSupplier() {
        return () -> readTunableNumber("initialPosition", initialPosition);
    }

    /**
     * Supplies the initial velocity that seeds the profile state on startup.
     *
     * @return supplier yielding the starting velocity (mechanism units per second)
     */
    public Supplier<Double> getInitialVelocitySupplier() {
        return () -> readTunableNumber("initialVelocity", initialVelocity);
    }
}