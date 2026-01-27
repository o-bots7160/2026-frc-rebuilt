package frc.robot.shared.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;

/**
 * Configuration values for subsystems that follow a trapezoidal motion profile.
 * <p>
 * Values are stored in degrees (and degrees per second) for readability and should be kept consistent across commands that target the subsystem.
 * Radian-based helpers are provided to keep math and motor wrappers aligned with WPILib conventions. Every field is mirrored to the dashboard so it
 * can be tuned live without redeploying.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystemConfig extends AbstractConfig {
    /** Minimum allowed setpoint for the profile, in degrees. */
    public double minimumSetpointDegrees;

    /** Maximum allowed setpoint for the profile, in degrees. */
    public double maximumSetpointDegrees;

    /** Maximum velocity for the trapezoidal profile, in degrees per second. */
    public double maximumVelocityDegreesPerSecond;

    /** Maximum acceleration for the trapezoidal profile, in degrees per second squared. */
    public double maximumAccelerationDegreesPerSecondSquared;

    /** Acceptable position error when deciding if the mechanism is at its goal, in degrees. */
    public double positionToleranceDegrees;

    /** Starting position used to seed the initial profile state, in degrees. */
    public double initialPositionDegrees;

    /** Starting velocity used to seed the initial profile state, in degrees per second. */
    public double initialVelocityDegreesPerSecond;

    /** Proportional gain for the profiled position controller. */
    public double kP;

    /** Integral gain for the profiled position controller. */
    public double kI;

    /** Derivative gain for the profiled position controller. */
    public double kD;

    /** Static feedforward gain (volts). */
    public double kS;

    /** Velocity feedforward gain (volts per mechanism unit per second). */
    public double kV;

    /** Acceleration feedforward gain (volts per mechanism unit per second squared). */
    public double kA;

    /** Optional prefix override for dashboard keys (defaults to the config class name without the Config suffix). */
    public String dashboardPrefix;

    /**
     * Supplies the minimum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return supplier yielding the minimum allowed setpoint (degrees)
     */
    public Supplier<Double> getMinimumSetpointDegreesSupplier() {
        return () -> readTunableNumber("minimumSetpointDegrees", minimumSetpointDegrees);
    }

    /**
     * Supplies the minimum setpoint in radians.
     *
     * @return supplier yielding the minimum allowed setpoint (radians)
     */
    public Supplier<Double> getMinimumSetpointRadiansSupplier() {
        return () -> Units.degreesToRadians(getMinimumSetpointDegreesSupplier().get());
    }

    /**
     * Supplies the maximum setpoint, tuned via SmartDashboard, to clamp incoming targets.
     *
     * @return supplier yielding the maximum allowed setpoint (degrees)
     */
    public Supplier<Double> getMaximumSetpointDegreesSupplier() {
        return () -> readTunableNumber("maximumSetpointDegrees", maximumSetpointDegrees);
    }

    /**
     * Supplies the maximum setpoint in radians.
     *
     * @return supplier yielding the maximum allowed setpoint (radians)
     */
    public Supplier<Double> getMaximumSetpointRadiansSupplier() {
        return () -> Units.degreesToRadians(getMaximumSetpointDegreesSupplier().get());
    }

    /**
     * Supplies the maximum profile velocity, tuned via SmartDashboard.
     *
     * @return supplier yielding the max velocity (degrees per second)
     */
    public Supplier<Double> getMaximumVelocityDegreesPerSecondSupplier() {
        return () -> readTunableNumber("maximumVelocityDegreesPerSecond", maximumVelocityDegreesPerSecond);
    }

    /**
     * Supplies the maximum profile velocity in radians per second.
     *
     * @return supplier yielding the max velocity (radians per second)
     */
    public Supplier<Double> getMaximumVelocityRadiansPerSecondSupplier() {
        return () -> Units.degreesToRadians(getMaximumVelocityDegreesPerSecondSupplier().get());
    }

    /**
     * Supplies the maximum profile acceleration, tuned via SmartDashboard.
     *
     * @return supplier yielding the max acceleration (degrees per second squared)
     */
    public Supplier<Double> getMaximumAccelerationDegreesPerSecondSquaredSupplier() {
        return () -> readTunableNumber("maximumAccelerationDegreesPerSecondSquared",
                maximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Supplies the maximum profile acceleration in radians per second squared.
     *
     * @return supplier yielding the max acceleration (radians per second squared)
     */
    public Supplier<Double> getMaximumAccelerationRadiansPerSecondSquaredSupplier() {
        return () -> Units.degreesToRadians(getMaximumAccelerationDegreesPerSecondSquaredSupplier().get());
    }

    /**
     * Supplies the allowed position error used to decide when the mechanism is at its goal.
     *
     * @return supplier yielding the position tolerance (degrees)
     */
    public Supplier<Double> getPositionToleranceDegreesSupplier() {
        return () -> readTunableNumber("positionToleranceDegrees", positionToleranceDegrees);
    }

    /**
     * Supplies the allowed position error in radians.
     *
     * @return supplier yielding the position tolerance (radians)
     */
    public Supplier<Double> getPositionToleranceRadiansSupplier() {
        return () -> Units.degreesToRadians(getPositionToleranceDegreesSupplier().get());
    }

    /**
     * Supplies the initial position that seeds the profile state on startup.
     *
     * @return supplier yielding the starting position (degrees)
     */
    public Supplier<Double> getInitialPositionDegreesSupplier() {
        return () -> readTunableNumber("initialPositionDegrees", initialPositionDegrees);
    }

    /**
     * Supplies the initial position in radians.
     *
     * @return supplier yielding the starting position (radians)
     */
    public Supplier<Double> getInitialPositionRadiansSupplier() {
        return () -> Units.degreesToRadians(getInitialPositionDegreesSupplier().get());
    }

    /**
     * Supplies the initial velocity that seeds the profile state on startup.
     *
     * @return supplier yielding the starting velocity (degrees per second)
     */
    public Supplier<Double> getInitialVelocityDegreesPerSecondSupplier() {
        return () -> readTunableNumber("initialVelocityDegreesPerSecond", initialVelocityDegreesPerSecond);
    }

    /**
     * Supplies the initial velocity in radians per second.
     *
     * @return supplier yielding the starting velocity (radians per second)
     */
    public Supplier<Double> getInitialVelocityRadiansPerSecondSupplier() {
        return () -> Units.degreesToRadians(getInitialVelocityDegreesPerSecondSupplier().get());
    }

    /**
     * Supplies the proportional gain for the profiled controller.
     *
     * @return supplier yielding kP
     */
    public Supplier<Double> getkPSupplier() {
        return () -> readTunableNumber("kP", kP);
    }

    /**
     * Supplies the integral gain for the profiled controller.
     *
     * @return supplier yielding kI
     */
    public Supplier<Double> getkISupplier() {
        return () -> readTunableNumber("kI", kI);
    }

    /**
     * Supplies the derivative gain for the profiled controller.
     *
     * @return supplier yielding kD
     */
    public Supplier<Double> getkDSupplier() {
        return () -> readTunableNumber("kD", kD);
    }

    /**
     * Supplies the static feedforward term.
     *
     * @return supplier yielding kS (volts)
     */
    public Supplier<Double> getkSSupplier() {
        return () -> readTunableNumber("kS", kS);
    }

    /**
     * Supplies the velocity feedforward term.
     *
     * @return supplier yielding kV (volts per mechanism unit per second)
     */
    public Supplier<Double> getkVSupplier() {
        return () -> readTunableNumber("kV", kV);
    }

    /**
     * Supplies the acceleration feedforward term.
     *
     * @return supplier yielding kA (volts per mechanism unit per second squared)
     */
    public Supplier<Double> getkASupplier() {
        return () -> readTunableNumber("kA", kA);
    }
}