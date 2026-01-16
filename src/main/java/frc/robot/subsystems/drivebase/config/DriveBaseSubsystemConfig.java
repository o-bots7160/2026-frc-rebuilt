package frc.robot.subsystems.drivebase.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractSubsystemConfig;

/**
 * Configuration bundle for the drive base subsystem. The values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class DriveBaseSubsystemConfig extends AbstractSubsystemConfig {

    public double maximumLinearSpeedFeetPerSecond;

    public double maximumAngularSpeedDegreesPerSecond;

    public double maximumAngularAccelerationDegreesPerSecondSquared;

    public double translationKp;

    public double translationKi;

    public double translationKd;

    public double rotationKp;

    public double rotationKi;

    public double rotationKd;

    public double headingKp;

    public double headingKi;

    public double headingKd;

    public double translationToleranceInches;

    public double rotationToleranceDegrees;

    public double translationScale;

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
     * Supplies the maximum angular acceleration in radians per second squared.
     *
     * @return supplier yielding the current max angular acceleration (rad/s^2)
     */
    public Supplier<Double> getMaximumAngularAccelerationRadiansPerSecondSquared() {
        return () -> Units.degreesToRadians(
                readTunableNumber(
                        "maximumAngularAccelerationDegreesPerSecondSquared",
                        maximumAngularAccelerationDegreesPerSecondSquared));
    }

    /**
     * Supplies the translation proportional gain.
     *
     * @return supplier yielding the current translation Kp
     */
    public Supplier<Double> getTranslationKp() {
        return () -> readTunableNumber("translationKp", translationKp);
    }

    /**
     * Supplies the translation integral gain.
     *
     * @return supplier yielding the current translation Ki
     */
    public Supplier<Double> getTranslationKi() {
        return () -> readTunableNumber("translationKi", translationKi);
    }

    /**
     * Supplies the translation derivative gain.
     *
     * @return supplier yielding the current translation Kd
     */
    public Supplier<Double> getTranslationKd() {
        return () -> readTunableNumber("translationKd", translationKd);
    }

    /**
     * Supplies the rotation proportional gain.
     *
     * @return supplier yielding the current rotation Kp
     */
    public Supplier<Double> getRotationKp() {
        return () -> readTunableNumber("rotationKp", rotationKp);
    }

    /**
     * Supplies the rotation integral gain.
     *
     * @return supplier yielding the current rotation Ki
     */
    public Supplier<Double> getRotationKi() {
        return () -> readTunableNumber("rotationKi", rotationKi);
    }

    /**
     * Supplies the rotation derivative gain.
     *
     * @return supplier yielding the current rotation Kd
     */
    public Supplier<Double> getRotationKd() {
        return () -> readTunableNumber("rotationKd", rotationKd);
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
     * Supplies the translation tolerance in meters.
     *
     * @return supplier yielding the current translation tolerance (m)
     */
    public Supplier<Double> getTranslationToleranceMeters() {
        return () -> Units.inchesToMeters(
                readTunableNumber("translationToleranceInches", translationToleranceInches));
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
