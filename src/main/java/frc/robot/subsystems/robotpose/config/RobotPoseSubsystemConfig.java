package frc.robot.subsystems.robotpose.config;

import frc.robot.shared.config.AbstractSubsystemConfig;

/**
 * Configuration bundle for the Robot Pose subsystem.
 * <p>
 * Pose fusion is delegated to the drivebase's internal YAGSL pose estimator, so these settings control only high-level gating. The
 * {@code enableVisionFusion} toggle lets operators disable all vision input for debugging without a redeploy.
 * </p>
 */
public class RobotPoseSubsystemConfig extends AbstractSubsystemConfig {

    /**
     * Enables or disables vision fusion in the Robot Pose subsystem.
     */
    public boolean enableVisionFusion = true;

    /**
     * Maximum age of a vision sample that is allowed to seed the robot pose.
     */
    public double  maximumVisionResetAgeSeconds           = 0.5;

    /**
     * Maximum translational standard deviation allowed when seeding from vision.
     */
    public double  maximumVisionResetLinearStdDevMeters   = 0.35;

    /**
     * Maximum rotational standard deviation, in degrees, allowed when seeding from vision.
     */
    public double  maximumVisionResetAngularStdDevDegrees = 15.0;

    /**
     * Returns whether vision fusion should be enabled.
     * <p>
     * Disable this when running without cameras or while diagnosing vision issues. When disabled, vision measurements are still recorded for logging
     * but not forwarded to the pose estimator.
     * </p>
     *
     * @return true when vision fusion is enabled
     */
    public boolean getEnableVisionFusion() {
        return readTunableBoolean("enableVisionFusion", enableVisionFusion);
    }

    /**
     * Returns the maximum age of a vision sample that may be used for a pose reset.
     *
     * @return age threshold in seconds
     */
    public double getMaximumVisionResetAgeSeconds() {
        return readTunableNumber("maximumVisionResetAgeSeconds", maximumVisionResetAgeSeconds);
    }

    /**
     * Returns the maximum translational uncertainty allowed for a vision-based reset.
     *
     * @return translational standard deviation threshold in meters
     */
    public double getMaximumVisionResetLinearStdDevMeters() {
        return readTunableNumber("maximumVisionResetLinearStdDevMeters", maximumVisionResetLinearStdDevMeters);
    }

    /**
     * Returns the maximum rotational uncertainty allowed for a vision-based reset.
     *
     * @return rotational standard deviation threshold in radians
     */
    public double getMaximumVisionResetAngularStdDevRadians() {
        return readTunableDegreesAsRadians("maximumVisionResetAngularStdDevDegrees", maximumVisionResetAngularStdDevDegrees);
    }
}
