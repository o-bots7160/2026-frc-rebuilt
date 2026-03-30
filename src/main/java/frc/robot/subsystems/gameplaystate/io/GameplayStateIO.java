package frc.robot.subsystems.gameplaystate.io;

import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the contract for gameplay state telemetry inputs that AdvantageKit will log.
 */
public interface GameplayStateIO {

    /**
     * Container of gameplay state telemetry fields that AdvantageKit will persist.
     */
    @AutoLog
    public static class GameplayStateIOInputs {
        /**
         * Name of the current gameplay state (e.g., "Idle", "Fire Ready").
         */
        public String  currentState                  = "Idle";

        /**
         * Name of the most recently requested state before it was applied.
         */
        public String  requestedState                = "Idle";

        /**
         * Source that triggered the most recent state transition (e.g., "operator", "fms", "auto", "timer").
         */
        public String  transitionSource              = "init";

        /**
         * True when the robot is in the autonomous match phase.
         */
        public boolean isAutonomous                  = false;

        /**
         * True when the robot is in the teleop match phase.
         */
        public boolean isTeleop                      = false;

        /**
         * True when the robot is disabled.
         */
        public boolean isDisabled                    = true;

        /**
         * Match time remaining in seconds, or -1.0 when unavailable.
         */
        public double  matchTimeSeconds              = -1.0;

        /**
         * True when auto-transition based on FMS match phase is enabled.
         */
        public boolean autoTransitionEnabled         = true;

        /**
         * True when the endgame threshold has been reached.
         */
        public boolean endgameSuggested              = false;

        /**
         * Straight-line distance from the robot to the active field target in meters.
         */
        public double  distanceToTargetMeters        = 0.0;

        /**
         * Human-readable name of the active field target (e.g., "Hub", "Left Rally", "Right Rally").
         */
        public String  activeTargetName              = "Unknown";
    }

    /**
     * Refreshes the inputs structure with the latest gameplay state telemetry.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(GameplayStateIOInputs inputs);
}
