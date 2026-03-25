package frc.robot.shared.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration for the driver controller's d-pad pathfinding targets, shared pathfinding constraints, and trench zone definitions.
 * <p>
 * Each d-pad direction has its own {@link DpadTargetConfig} holding the target field pose. All four directions share one set of PathPlanner
 * pathfinding constraints (velocity and acceleration limits). Targets are stored in blue-alliance coordinates and flipped at runtime for red.
 * </p>
 * <p>
 * Trench zones define rectangular areas on the field where the robot must maintain a specific heading while passing through. When a d-pad
 * pathfinding route crosses a trench zone, intermediate waypoints are inserted at the zone entry and exit so the robot passes through at the
 * correct heading.
 * </p>
 */
public class DriverControllerConfig extends AbstractConfig {

    /**
     * Target pose for the d-pad up button.
     */
    public DpadTargetConfig  dpadUp       = new DpadTargetConfig();

    /**
     * Target pose for the d-pad down button.
     */
    public DpadTargetConfig  dpadDown     = new DpadTargetConfig();

    /**
     * Target pose for the d-pad left button.
     */
    public DpadTargetConfig  dpadLeft     = new DpadTargetConfig();

    /**
     * Target pose for the d-pad right button.
     */
    public DpadTargetConfig  dpadRight    = new DpadTargetConfig();

    /**
     * Trench zones that require heading alignment when the robot pathfinds through them.
     * <p>
     * All zones are defined in field (blue-alliance) coordinates. Trenches are physical field features on both halves of the field, so zones should
     * cover both the blue-side and red-side trenches. When a d-pad pathfinding route crosses any enabled zone, entry and exit waypoints are
     * inserted so the robot arrives at the trench with the correct heading.
     * </p>
     */
    public TrenchZoneConfig[] trenchZones = new TrenchZoneConfig[0];

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

    /**
     * Finds the first enabled trench zone that a straight-line path from the current pose to the target crosses.
     * <p>
     * Both poses must be in field (blue-alliance) coordinates. This method checks all configured trench zones regardless of alliance color, since
     * trenches are physical field features on both halves of the field.
     * </p>
     *
     * @param currentPose robot's current odometry pose in field coordinates
     * @param targetPose  target pose in field coordinates (already alliance-flipped)
     * @return the first intersecting {@link TrenchZoneConfig}, or {@code null} if no zone is crossed
     */
    public TrenchZoneConfig findIntersectingTrenchZone(Pose2d currentPose, Pose2d targetPose) {
        for (TrenchZoneConfig zone : trenchZones) {
            if (zone.intersectsLineSegment(currentPose, targetPose)) {
                return zone;
            }
        }
        return null;
    }
}
