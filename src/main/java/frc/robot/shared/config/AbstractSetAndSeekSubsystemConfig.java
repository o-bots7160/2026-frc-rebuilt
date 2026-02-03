package frc.robot.shared.config;

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

    /** Acceptable velocity error when deciding if the mechanism is at its goal, in degrees per second. */
    public double velocityToleranceDegreesPerSecond;

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

    /**
     * Returns the maximum profile velocity, tuned via SmartDashboard.
     *
     * @return max velocity (degrees per second)
     */
    public double getMaximumVelocityDegreesPerSecond() {
        return readTunableDegrees("maximumVelocityDegreesPerSecond", maximumVelocityDegreesPerSecond);
    }

    /**
     * Returns the maximum profile velocity in radians per second.
     *
     * @return max velocity (radians per second)
     */
    public double getMaximumVelocityRadiansPerSecond() {
        return readTunableDegreesAsRadians("maximumVelocityDegreesPerSecond", maximumVelocityDegreesPerSecond);
    }

    /**
     * Returns the maximum profile acceleration, tuned via SmartDashboard.
     *
     * @return max acceleration (degrees per second squared)
     */
    public double getMaximumAccelerationDegreesPerSecondSquared() {
        return readTunableDegrees(
                "maximumAccelerationDegreesPerSecondSquared",
                maximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Returns the maximum profile acceleration in radians per second squared.
     *
     * @return max acceleration (radians per second squared)
     */
    public double getMaximumAccelerationRadiansPerSecondSquared() {
        return readTunableDegreesAsRadians(
                "maximumAccelerationDegreesPerSecondSquared",
                maximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Returns the allowed position error used to decide when the mechanism is at its goal.
     *
     * @return position tolerance (degrees)
     */
    public double getPositionToleranceDegrees() {
        return readTunableDegrees("positionToleranceDegrees", positionToleranceDegrees);
    }

    /**
     * Returns the allowed position error in radians.
     *
     * @return position tolerance (radians)
     */
    public double getPositionToleranceRadians() {
        return readTunableDegreesAsRadians("positionToleranceDegrees", positionToleranceDegrees);
    }

    /**
     * Returns the allowed velocity error used to decide when the mechanism is at its goal.
     *
     * @return velocity tolerance (degrees per second)
     */
    public double getVelocityToleranceDegreesPerSecond() {
        return readTunableDegrees("velocityToleranceDegreesPerSecond", velocityToleranceDegreesPerSecond);
    }

    /**
     * Returns the allowed velocity error in radians per second.
     *
     * @return velocity tolerance (radians per second)
     */
    public double getVelocityToleranceRadiansPerSecond() {
        return readTunableDegreesAsRadians("velocityToleranceDegreesPerSecond", velocityToleranceDegreesPerSecond);
    }

    /**
     * Returns the initial position that seeds the profile state on startup.
     *
     * @return starting position (degrees)
     */
    public double getInitialPositionDegrees() {
        return readTunableDegrees("initialPositionDegrees", initialPositionDegrees);
    }

    /**
     * Returns the initial position in radians.
     *
     * @return starting position (radians)
     */
    public double getInitialPositionRadians() {
        return readTunableDegreesAsRadians("initialPositionDegrees", initialPositionDegrees);
    }

    /**
     * Returns the initial velocity that seeds the profile state on startup.
     *
     * @return starting velocity (degrees per second)
     */
    public double getInitialVelocityDegreesPerSecond() {
        return readTunableDegrees("initialVelocityDegreesPerSecond", initialVelocityDegreesPerSecond);
    }

    /**
     * Returns the initial velocity in radians per second.
     *
     * @return starting velocity (radians per second)
     */
    public double getInitialVelocityRadiansPerSecond() {
        return readTunableDegreesAsRadians("initialVelocityDegreesPerSecond", initialVelocityDegreesPerSecond);
    }

    /**
     * Returns the proportional gain for the profiled controller.
     *
     * @return kP
     */
    public double getkP() {
        return readTunableNumber("kP", kP);
    }

    /**
     * Returns the integral gain for the profiled controller.
     *
     * @return kI
     */
    public double getkI() {
        return readTunableNumber("kI", kI);
    }

    /**
     * Returns the derivative gain for the profiled controller.
     *
     * @return kD
     */
    public double getkD() {
        return readTunableNumber("kD", kD);
    }

    /**
     * Returns the static feedforward term.
     *
     * @return kS (volts)
     */
    public double getkS() {
        return readTunableNumber("kS", kS);
    }

    /**
     * Returns the velocity feedforward term.
     *
     * @return kV (volts per mechanism unit per second)
     */
    public double getkV() {
        return readTunableNumber("kV", kV);
    }

    /**
     * Returns the acceleration feedforward term.
     *
     * @return kA (volts per mechanism unit per second squared)
     */
    public double getkA() {
        return readTunableNumber("kA", kA);
    }
}