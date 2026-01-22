package frc.robot.subsystems.drivebase.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the drive base subsystem. The values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class DriveBaseSubsystemConfig extends AbstractConfig {

    public double maximumLinearSpeedFeetPerSecond;

    public double maximumAngularSpeedDegreesPerSecond;

    public double headingKp;

    public double headingKi;

    public double headingKd;

    public double rotationToleranceDegrees;

    public double translationScale;

    public double simulationTranslationScale;

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
     * Supplies the joystick translation scale.
     *
     * @return supplier yielding the current translation scale (0–1)
     */
    public Supplier<Double> getTranslationScale() {
        return () -> readTunableNumber("translationScale", translationScale);
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
}
