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
}
