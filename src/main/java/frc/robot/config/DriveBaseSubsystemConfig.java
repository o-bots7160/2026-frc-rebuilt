package frc.robot.config;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Configuration bundle for the drive base subsystem. The values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class DriveBaseSubsystemConfig extends AbstractSubsystemConfig {

    public String swerveDirectory                                   = "swerve";

    public double maximumLinearSpeedFeetPerSecond                   = 15.0;

    public double maximumLinearAccelerationFeetPerSecondSquared     = 10.0;

    public double maximumAngularSpeedDegreesPerSecond               = 450.0;

    public double maximumAngularAccelerationDegreesPerSecondSquared = 720.0;

    public double translationKp                                     = 4.0;

    public double translationKi                                     = 0.0;

    public double translationKd                                     = 0.0;

    public double rotationKp                                        = 6.0;

    public double rotationKi                                        = 0.0;

    public double rotationKd                                        = 0.0;

    public double headingKp                                         = 5.0;

    public double headingKi                                         = 0.0;

    public double headingKd                                         = 0.0;

    public double translationToleranceInches                        = 1.0;

    public double rotationToleranceDegrees                          = 2.0;

    public double translationScale                                  = 0.8;

    public String getSwerveDirectory() {
        return SmartDashboard.getString("DriveBaseSubsystemConfig/swerveDirectory", swerveDirectory);
    }

    public double getMaximumLinearSpeedMetersPerSecond() {
        return Units.feetToMeters(
                SmartDashboard.getNumber("DriveBaseSubsystemConfig/maximumLinearSpeedFeetPerSecond", maximumLinearSpeedFeetPerSecond));
    }

    public double getMaximumLinearAccelerationMetersPerSecondSquared() {
        return Units.feetToMeters(
                SmartDashboard.getNumber(
                        "DriveBaseSubsystemConfig/maximumLinearAccelerationFeetPerSecondSquared",
                        maximumLinearAccelerationFeetPerSecondSquared));
    }

    public double getMaximumAngularSpeedRadiansPerSecond() {
        return Units.degreesToRadians(
                SmartDashboard.getNumber("DriveBaseSubsystemConfig/maximumAngularSpeedDegreesPerSecond", maximumAngularSpeedDegreesPerSecond));
    }

    public double getMaximumAngularAccelerationRadiansPerSecondSquared() {
        return Units.degreesToRadians(
                SmartDashboard.getNumber(
                        "DriveBaseSubsystemConfig/maximumAngularAccelerationDegreesPerSecondSquared",
                        maximumAngularAccelerationDegreesPerSecondSquared));
    }

    public double getTranslationKp() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/translationKp", translationKp);
    }

    public double getTranslationKi() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/translationKi", translationKi);
    }

    public double getTranslationKd() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/translationKd", translationKd);
    }

    public double getRotationKp() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/rotationKp", rotationKp);
    }

    public double getRotationKi() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/rotationKi", rotationKi);
    }

    public double getRotationKd() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/rotationKd", rotationKd);
    }

    public double getHeadingKp() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/headingKp", headingKp);
    }

    public double getHeadingKi() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/headingKi", headingKi);
    }

    public double getHeadingKd() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/headingKd", headingKd);
    }

    public double getTranslationScale() {
        return SmartDashboard.getNumber("DriveBaseSubsystemConfig/translationScale", translationScale);
    }

    public double getTranslationToleranceMeters() {
        return Units.inchesToMeters(
                SmartDashboard.getNumber("DriveBaseSubsystemConfig/translationToleranceInches", translationToleranceInches));
    }

    public double getRotationToleranceRadians() {
        return Units.degreesToRadians(
                SmartDashboard.getNumber("DriveBaseSubsystemConfig/rotationToleranceDegrees", rotationToleranceDegrees));
    }
}
