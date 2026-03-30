package frc.robot.shared.config;

/**
 * Configuration bundle for trapezoidal motion profile parameters used by set-and-seek subsystems.
 * <p>
 * Values are stored in degrees (and degrees per second) for human readability. Radian-based getters are provided for WPILib controllers that expect
 * radians. Every field is surfaced to SmartDashboard so it can be tuned live without redeploying.
 * </p>
 */
public class SetAndSeekMotionConfig extends AbstractConfig {

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

    /**
     * Returns the maximum profile velocity, tuned via SmartDashboard.
     *
     * @return max velocity in degrees per second
     */
    public double getMaximumVelocityDegreesPerSecond() {
        return readTunableDegrees("maximumVelocityDegreesPerSecond", maximumVelocityDegreesPerSecond);
    }

    /**
     * Returns the maximum profile velocity in radians per second.
     *
     * @return max velocity in radians per second
     */
    public double getMaximumVelocityRadiansPerSecond() {
        return readTunableDegreesAsRadians("maximumVelocityDegreesPerSecond", maximumVelocityDegreesPerSecond);
    }

    /**
     * Returns the maximum profile acceleration, tuned via SmartDashboard.
     *
     * @return max acceleration in degrees per second squared
     */
    public double getMaximumAccelerationDegreesPerSecondSquared() {
        return readTunableDegrees(
                "maximumAccelerationDegreesPerSecondSquared",
                maximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Returns the maximum profile acceleration in radians per second squared.
     *
     * @return max acceleration in radians per second squared
     */
    public double getMaximumAccelerationRadiansPerSecondSquared() {
        return readTunableDegreesAsRadians(
                "maximumAccelerationDegreesPerSecondSquared",
                maximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Returns the allowed position error in radians.
     *
     * @return position tolerance in radians
     */
    public double getPositionToleranceRadians() {
        return readTunableDegreesAsRadians("positionToleranceDegrees", positionToleranceDegrees);
    }

    /**
     * Returns the allowed velocity error in radians per second.
     *
     * @return velocity tolerance in radians per second
     */
    public double getVelocityToleranceRadiansPerSecond() {
        return readTunableDegreesAsRadians("velocityToleranceDegreesPerSecond", velocityToleranceDegreesPerSecond);
    }

    /**
     * Returns the initial position in radians.
     *
     * @return starting position in radians
     */
    public double getInitialPositionRadians() {
        return readTunableDegreesAsRadians("initialPositionDegrees", initialPositionDegrees);
    }

    /**
     * Returns the initial velocity in radians per second.
     *
     * @return starting velocity in radians per second
     */
    public double getInitialVelocityRadiansPerSecond() {
        return readTunableDegreesAsRadians("initialVelocityDegreesPerSecond", initialVelocityDegreesPerSecond);
    }
}
