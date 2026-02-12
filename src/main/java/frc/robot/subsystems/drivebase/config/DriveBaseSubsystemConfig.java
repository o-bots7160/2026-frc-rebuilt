package frc.robot.subsystems.drivebase.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the drive base subsystem. The values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class DriveBaseSubsystemConfig extends AbstractConfig {

    /**
     * Maximum linear speed in feet per second.
     */
    public double maximumLinearSpeedFeetPerSecond;

    /**
     * Maximum angular speed in degrees per second.
     */
    public double maximumAngularSpeedDegreesPerSecond;

    /**
     * Heading controller proportional gain.
     */
    public double headingKp;

    /**
     * Heading controller integral gain.
     */
    public double headingKi;

    /**
     * Heading controller derivative gain.
     */
    public double headingKd;

    /**
     * Allowed heading error in degrees.
     */
    public double rotationToleranceDegrees;

    /**
     * Path following translation proportional gain.
     */
    public double pathTranslationKp;

    /**
     * Path following translation integral gain.
     */
    public double pathTranslationKi;

    /**
     * Path following translation derivative gain.
     */
    public double pathTranslationKd;

    /**
     * Path following rotation proportional gain.
     */
    public double pathRotationKp;

    /**
     * Path following rotation integral gain.
     */
    public double pathRotationKi;

    /**
     * Path following rotation derivative gain.
     */
    public double pathRotationKd;

    /**
     * Additional translation scale applied in simulation.
     */
    public double simulationTranslationScale;

    /**
     * Additional rotation scale applied in simulation.
     */
    public double simulationOmegaScale;

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
     * Supplies the heading hold proportional gain.
     *
     * @return supplier yielding the current heading Kp
     */
    public Supplier<Double> getHeadingKp() {
        return () -> readTunableNumber("headingKp", headingKp);
    }

    /**
     * Supplies the heading hold integral gain.
     *
     * @return supplier yielding the current heading Ki
     */
    public Supplier<Double> getHeadingKi() {
        return () -> readTunableNumber("headingKi", headingKi);
    }

    /**
     * Supplies the heading hold derivative gain.
     *
     * @return supplier yielding the current heading Kd
     */
    public Supplier<Double> getHeadingKd() {
        return () -> readTunableNumber("headingKd", headingKd);
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
     * Supplies the path following translation proportional gain.
     *
     * @return supplier yielding the current translation Kp
     */
    public Supplier<Double> getPathTranslationKp() {
        return () -> readTunableNumber("pathTranslationKp", pathTranslationKp);
    }

    /**
     * Supplies the path following translation integral gain.
     *
     * @return supplier yielding the current translation Ki
     */
    public Supplier<Double> getPathTranslationKi() {
        return () -> readTunableNumber("pathTranslationKi", pathTranslationKi);
    }

    /**
     * Supplies the path following translation derivative gain.
     *
     * @return supplier yielding the current translation Kd
     */
    public Supplier<Double> getPathTranslationKd() {
        return () -> readTunableNumber("pathTranslationKd", pathTranslationKd);
    }

    /**
     * Supplies the path following rotation proportional gain.
     *
     * @return supplier yielding the current rotation Kp
     */
    public Supplier<Double> getPathRotationKp() {
        return () -> readTunableNumber("pathRotationKp", pathRotationKp);
    }

    /**
     * Supplies the path following rotation integral gain.
     *
     * @return supplier yielding the current rotation Ki
     */
    public Supplier<Double> getPathRotationKi() {
        return () -> readTunableNumber("pathRotationKi", pathRotationKi);
    }

    /**
     * Supplies the path following rotation derivative gain.
     *
     * @return supplier yielding the current rotation Kd
     */
    public Supplier<Double> getPathRotationKd() {
        return () -> readTunableNumber("pathRotationKd", pathRotationKd);
    }
}
