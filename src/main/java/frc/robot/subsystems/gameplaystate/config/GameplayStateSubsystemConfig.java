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
     * Match time threshold in seconds below which the subsystem suggests transitioning to {@code CLIMB_READY}.
     * <p>
     * Only evaluated during teleop when {@link #endgameAutoTransitionEnabled} is true. A value of 30.0 means the suggestion fires when approximately
     * 30 seconds remain in the teleop period.
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
     * Enables automatic transition to {@code CLIMB_READY} when match time drops below the configured threshold.
     * <p>
     * Defaults to false to avoid surprising the operator. When false, the subsystem logs a telemetry suggestion but does not force the transition.
     * </p>
     */
    public boolean endgameAutoTransitionEnabled   = false;

    /**
     * Returns the endgame match time threshold in seconds.
     *
     * @return seconds remaining in teleop below which the subsystem may transition to CLIMB_READY
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

    /**
     * Returns whether automatic endgame transition to CLIMB_READY is enabled.
     *
     * @return true when the subsystem should auto-transition to CLIMB_READY at the endgame threshold
     */
    public boolean getEndgameAutoTransitionEnabled() {
        return readTunableBoolean("endgameAutoTransitionEnabled", endgameAutoTransitionEnabled);
    }
}
