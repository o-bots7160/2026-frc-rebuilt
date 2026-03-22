package frc.robot.shared.bindings;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration for the driver controller's d-pad pathfinding targets and shared pathfinding constraints.
 * <p>
 * Each d-pad direction has its own {@link DpadTargetConfig} holding the target field pose. All four directions share one set of PathPlanner
 * pathfinding constraints (velocity and acceleration limits). Targets are stored in blue-alliance coordinates and flipped at runtime for red.
 * </p>
 */
public class DriverControllerConfig extends AbstractConfig {

    /**
     * Target pose for the d-pad up button.
     */
    public DpadTargetConfig dpadUp    = new DpadTargetConfig();

    /**
     * Target pose for the d-pad down button.
     */
    public DpadTargetConfig dpadDown  = new DpadTargetConfig();

    /**
     * Target pose for the d-pad left button.
     */
    public DpadTargetConfig dpadLeft  = new DpadTargetConfig();

    /**
     * Target pose for the d-pad right button.
     */
    public DpadTargetConfig dpadRight = new DpadTargetConfig();

    /**
     * Maximum linear velocity for d-pad pathfinding in meters per second.
     * <p>
     * Applies to all four d-pad directions. A conservative value (e.g., 3.0 m/s) keeps the robot under control during teleop pathfinding.
     * </p>
     */
    public double dpadMaxVelocityMetersPerSecond               = 3.0;

    /**
     * Maximum linear acceleration for d-pad pathfinding in meters per second squared.
     */
    public double dpadMaxAccelerationMetersPerSecondSquared     = 3.0;

    /**
     * Maximum angular velocity for d-pad pathfinding in degrees per second.
     * <p>
     * Stored in degrees for consistency with other config values. Converted to radians at the call site.
     * </p>
     */
    public double dpadMaxAngularVelocityDegreesPerSecond        = 360.0;

    /**
     * Maximum angular acceleration for d-pad pathfinding in degrees per second squared.
     * <p>
     * Stored in degrees for consistency with other config values. Converted to radians at the call site.
     * </p>
     */
    public double dpadMaxAngularAccelerationDegreesPerSecondSquared = 720.0;

    /**
     * Reads the tunable maximum linear velocity for d-pad pathfinding.
     *
     * @return maximum velocity in meters per second
     */
    public double getDpadMaxVelocityMetersPerSecond() {
        return readTunableNumber("dpadMaxVelocityMetersPerSecond", dpadMaxVelocityMetersPerSecond);
    }

    /**
     * Reads the tunable maximum linear acceleration for d-pad pathfinding.
     *
     * @return maximum acceleration in meters per second squared
     */
    public double getDpadMaxAccelerationMetersPerSecondSquared() {
        return readTunableNumber("dpadMaxAccelerationMetersPerSecondSquared", dpadMaxAccelerationMetersPerSecondSquared);
    }

    /**
     * Reads the tunable maximum angular velocity for d-pad pathfinding in degrees per second.
     *
     * @return maximum angular velocity in degrees per second
     */
    public double getDpadMaxAngularVelocityDegreesPerSecond() {
        return readTunableDegrees("dpadMaxAngularVelocityDegreesPerSecond", dpadMaxAngularVelocityDegreesPerSecond);
    }

    /**
     * Reads the tunable maximum angular acceleration for d-pad pathfinding in degrees per second squared.
     *
     * @return maximum angular acceleration in degrees per second squared
     */
    public double getDpadMaxAngularAccelerationDegreesPerSecondSquared() {
        return readTunableDegrees("dpadMaxAngularAccelerationDegreesPerSecondSquared", dpadMaxAngularAccelerationDegreesPerSecondSquared);
    }
}
