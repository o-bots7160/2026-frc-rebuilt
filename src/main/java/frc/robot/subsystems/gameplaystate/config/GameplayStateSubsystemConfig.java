package frc.robot.subsystems.gameplaystate.config;

import frc.robot.shared.config.AbstractSubsystemConfig;

/**
 * Configuration bundle for the Gameplay State subsystem.
 * <p>
 * Controls automatic state transitions triggered by FMS match phase changes and match time thresholds. All threshold values are tunable at runtime
 * through AdvantageKit-backed SmartDashboard entries when not attached to the FMS.
 * </p>
 */
public class GameplayStateSubsystemConfig extends AbstractSubsystemConfig {

    /**
     * Match time threshold in seconds below which the subsystem sets the endgame suggestion flag for dashboard alerts.
     * <p>
     * Only evaluated during teleop. A value of 30.0 means the suggestion fires when approximately 30 seconds remain in the teleop period.
     * </p>
     */
    public double  endgameThresholdSeconds        = 30.0;

    /**
     * Enables automatic state transitions based on FMS match phase (autonomous, teleop, disabled).
     * <p>
     * When true, the subsystem automatically transitions to {@code AUTO_CYCLE} at the start of autonomous, reverts to {@code IDLE} at the start of
     * teleop, and forces {@code IDLE} when the robot is disabled.
     * </p>
     */
    public boolean autoTransitionEnabled          = true;

    /**
     * Returns the endgame match time threshold in seconds.
     *
     * @return seconds remaining in teleop below which the endgame suggestion flag is set
     */
    public double getEndgameThresholdSeconds() {
        return readTunableNumber("endgameThresholdSeconds", endgameThresholdSeconds);
    }

    /**
     * Returns whether FMS-based automatic state transitions are enabled.
     *
     * @return true when the subsystem should automatically transition states based on match phase
     */
    public boolean getAutoTransitionEnabled() {
        return readTunableBoolean("autoTransitionEnabled", autoTransitionEnabled);
    }
}
