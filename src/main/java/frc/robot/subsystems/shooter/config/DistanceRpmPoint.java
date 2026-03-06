package frc.robot.subsystems.shooter.config;

/**
 * Single data point mapping a distance in meters to a target flywheel RPM for the distance-based shooter interpolation table.
 * <p>
 * Define an array of these in the shooter config JSON. At runtime, the subsystem builds an interpolating lookup table from these points and linearly
 * interpolates between them to compute the target RPM for any given distance.
 * </p>
 */
public class DistanceRpmPoint {

    /** Distance from the robot to the target in meters. */
    public double distanceMeters;

    /** Target flywheel RPM at this distance. */
    public double rpm;
}
