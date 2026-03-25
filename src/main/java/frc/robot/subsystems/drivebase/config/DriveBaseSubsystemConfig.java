package frc.robot.subsystems.drivebase.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractSubsystemConfig;
import frc.robot.shared.config.PidConfig;
import frc.robot.shared.config.SysIdRoutineConfig;

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
     * Allowed heading error in degrees.
     */
    public double    rotationToleranceDegrees;

    /**
     * Margin in degrees around each field-facing orientation (0 and 180 degrees) within which the robot is considered to already be facing that
     * direction. Used by the snap-to-field-facing command to decide whether to flip to the opposite heading.
     */
    public double    fieldFacingMarginDegrees = 15.0;

    /**
     * SysId routine parameters shared by all drive base motors (drive and angle).
     */
    public SysIdRoutineConfig sysId            = new SysIdRoutineConfig();

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
     * Whether YAGSL heading correction is enabled. Heading correction passively counteracts gyro drift during straight-line driving.
     * <p>
     * Disable in simulation where the correction assumes real-world friction and inertia.
     * </p>
     */
    public boolean   headingCorrectionEnabled = true;

    /**
     * Percentage of maximum drive speed allowed when the robot is actively shooting (FIRE_READY or AUTO_CYCLE). A value of 70 means the drive base
     * runs at 70% of its configured max speed while the shooter, indexer, and feeder are powered, reducing current draw so the shooter maintains
     * full accuracy.
     */
    public double    shootingSpeedScalePercent              = 70.0;

    /**
     * Battery voltage threshold below which the drive base begins scaling speed to conserve power. When the smoothed battery voltage drops below
     * this value, the drive base applies {@link #lowVoltageSpeedScalePercent} as an additional multiplier.
     */
    public double    lowVoltageThresholdVolts               = 12.0;

    /**
     * Percentage of maximum drive speed applied when battery voltage is below {@link #lowVoltageThresholdVolts} but above
     * {@link #criticalVoltageThresholdVolts}.
     */
    public double    lowVoltageSpeedScalePercent            = 80.0;

    /**
     * Battery voltage threshold indicating severe sag. When the smoothed battery voltage drops below this value, the drive base applies the more
     * aggressive {@link #criticalVoltageSpeedScalePercent} multiplier.
     */
    public double    criticalVoltageThresholdVolts          = 11.0;

    /**
     * Percentage of maximum drive speed applied when battery voltage is below {@link #criticalVoltageThresholdVolts}.
     */
    public double    criticalVoltageSpeedScalePercent       = 50.0;

    /**
     * Whether state-based power management (shooting speed scale) applies during autonomous mode. When false, only voltage-based scaling is active
     * during auto; the shooting speed scale is skipped so PathPlanner path-following accuracy is preserved.
     */
    public boolean   applyPowerManagementInAuto            = false;

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
     * Returns whether YAGSL heading correction is enabled.
     *
     * @return true when heading correction should be active
     */
    public boolean isHeadingCorrectionEnabled() {
        return headingCorrectionEnabled;
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

    /**
     * Reads the shooting speed scale as a fraction (0.0–1.0). A config value of 70 means the drive runs at 70% of max speed while shooting.
     *
     * @return shooting speed scale factor between 0.0 and 1.0
     */
    public double getShootingSpeedScale() {
        return readTunableNumber("shootingSpeedScalePercent", shootingSpeedScalePercent) / 100.0;
    }

    /**
     * Reads the battery voltage threshold below which low-voltage speed scaling activates.
     *
     * @return low voltage threshold in volts
     */
    public double getLowVoltageThresholdVolts() {
        return readTunableNumber("lowVoltageThresholdVolts", lowVoltageThresholdVolts);
    }

    /**
     * Reads the low-voltage speed scale as a fraction (0.0–1.0).
     *
     * @return low-voltage speed scale factor between 0.0 and 1.0
     */
    public double getLowVoltageSpeedScale() {
        return readTunableNumber("lowVoltageSpeedScalePercent", lowVoltageSpeedScalePercent) / 100.0;
    }

    /**
     * Reads the battery voltage threshold below which critical-voltage speed scaling activates.
     *
     * @return critical voltage threshold in volts
     */
    public double getCriticalVoltageThresholdVolts() {
        return readTunableNumber("criticalVoltageThresholdVolts", criticalVoltageThresholdVolts);
    }

    /**
     * Reads the critical-voltage speed scale as a fraction (0.0–1.0).
     *
     * @return critical-voltage speed scale factor between 0.0 and 1.0
     */
    public double getCriticalVoltageSpeedScale() {
        return readTunableNumber("criticalVoltageSpeedScalePercent", criticalVoltageSpeedScalePercent) / 100.0;
    }

    /**
     * Returns whether state-based power management applies during autonomous mode.
     *
     * @return true when shooting speed scale should also apply during auto
     */
    public boolean isApplyPowerManagementInAuto() {
        return readTunableBoolean("applyPowerManagementInAuto", applyPowerManagementInAuto);
    }
}
