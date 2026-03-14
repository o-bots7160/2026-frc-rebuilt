package frc.robot.subsystems.drivebase.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractSubsystemConfig;
import frc.robot.shared.config.PidConfig;

/**
 * Configuration bundle for the drive base subsystem. The values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class DriveBaseSubsystemConfig extends AbstractSubsystemConfig {

    /**
     * Maximum linear speed in feet per second.
     */
    public double    maximumLinearSpeedFeetPerSecond;

    /**
     * Maximum angular speed in degrees per second.
     */
    public double    maximumAngularSpeedDegreesPerSecond;

    /**
     * PID gains for the heading hold controller.
     */
    public PidConfig heading                  = new PidConfig();

    /**
     * Allowed heading error in degrees.
     */
    public double    rotationToleranceDegrees;

    /**
     * Margin in degrees around each field-facing orientation (0 and 180 degrees) within which the robot is considered to already be facing that
     * direction. Used by the snap-to-field-facing command to decide whether to flip to the opposite heading.
     */
    public double    fieldFacingMarginDegrees = 15.0;

    /**
     * PID gains for the path following translation controller.
     */
    public PidConfig pathTranslation          = new PidConfig();

    /**
     * PID gains for the path following rotation controller.
     */
    public PidConfig pathRotation             = new PidConfig();

    /**
     * Additional translation scale applied in simulation.
     */
    public double    simulationTranslationScale;

    /**
     * Additional rotation scale applied in simulation.
     */
    public double    simulationOmegaScale;

    /**
     * Name of the swerve configuration directory under the deploy folder (e.g., "swerve" or "swerve-test").
     */
    public String    swerveConfigDirectory    = "swerve";

    /**
     * Returns the swerve configuration directory name relative to the deploy folder.
     *
     * @return directory name such as "swerve" or "swerve-test"
     */
    public String getSwerveConfigDirectory() {
        return swerveConfigDirectory;
    }

    /**
     * Supplies the maximum linear speed in meters per second.
     *
     * @return supplier yielding the current max linear speed (m/s)
     */
    public Supplier<Double> getMaximumLinearSpeedMetersPerSecond() {
        return () -> Units.feetToMeters(
                readTunableNumber("maximumLinearSpeedFeetPerSecond", maximumLinearSpeedFeetPerSecond));
    }

    /**
     * Supplies the maximum angular speed in radians per second.
     *
     * @return supplier yielding the current max angular speed (rad/s)
     */
    public Supplier<Double> getMaximumAngularSpeedRadiansPerSecond() {
        return () -> Units.degreesToRadians(
                readTunableNumber("maximumAngularSpeedDegreesPerSecond", maximumAngularSpeedDegreesPerSecond));
    }

    /**
     * Returns the heading PID config.
     *
     * @return heading PID config
     */
    public PidConfig getHeading() {
        return heading;
    }

    /**
     * Supplies the additional joystick translation scale applied during simulation.
     *
     * @return supplier yielding the current simulation translation scale (0–1)
     */
    public Supplier<Double> getSimulationTranslationScale() {
        return () -> readTunableNumber("simulationTranslationScale", simulationTranslationScale);
    }

    /**
     * Supplies the additional joystick rotation scale applied during simulation.
     *
     * @return supplier yielding the current simulation rotation scale (0–1)
     */
    public Supplier<Double> getSimulationOmegaScale() {
        return () -> readTunableNumber("simulationOmegaScale", simulationOmegaScale);
    }

    /**
     * Supplies the rotation tolerance in radians.
     *
     * @return supplier yielding the current rotation tolerance (rad)
     */
    public Supplier<Double> getRotationToleranceRadians() {
        return () -> Units.degreesToRadians(
                readTunableNumber("rotationToleranceDegrees", rotationToleranceDegrees));
    }

    /**
     * Supplies the field-facing margin in radians. The robot is considered "facing" a field-aligned heading when it is within this many radians of
     * that heading.
     *
     * @return supplier yielding the current field-facing margin (rad)
     */
    public Supplier<Double> getFieldFacingMarginRadians() {
        return () -> Units.degreesToRadians(
                readTunableNumber("fieldFacingMarginDegrees", fieldFacingMarginDegrees));
    }

    /**
     * Returns the path following translation PID config.
     *
     * @return path translation PID config
     */
    public PidConfig getPathTranslation() {
        return pathTranslation;
    }

    /**
     * Returns the path following rotation PID config.
     *
     * @return path rotation PID config
     */
    public PidConfig getPathRotation() {
        return pathRotation;
    }
}
