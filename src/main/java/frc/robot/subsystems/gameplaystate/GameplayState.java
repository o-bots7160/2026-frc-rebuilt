package frc.robot.subsystems.gameplaystate;

/**
 * Enumerates the high-level operating modes the robot can be in during a match.
 * <p>
 * Each state describes a coordinated configuration of multiple subsystems. When the robot transitions into a state, the
 * {@link GameplayStateSubsystem} publishes the new value and the command factory schedules the appropriate parallel command group so every mechanism
 * moves into position together.
 * </p>
 */
public enum GameplayState {

    /**
     * All mechanisms at rest. Harvester stowed, turret centered, shooter at idle RPM, ball-path subsystems idle.
     * <p>
     * This is the default state when no specific action is requested.
     * </p>
     */
    IDLE("Idle"),

    /**
     * Robot is configured to collect Fuel from the field. Harvester deployed, intake and feeder running forward, indexer in feed mode, shooter at
     * idle RPM, turret tracking the field target.
     */
    HARVEST_READY("Harvest Ready"),

    /**
     * Robot is primed to shoot Fuel. Shooter spun up to distance-based RPM, turret tracking the field target, indexer and feeder primed to feed on
     * command. Harvester stowed.
     */
    FIRE_READY("Fire Ready"),

    /**
     * Combined harvest and fire mode for maximum autonomous throughput. Turret tracks the target while the intake collects Fuel simultaneously. All
     * ball-path subsystems run forward and the shooter stays spun up.
     */
    AUTO_CYCLE("Auto Cycle"),

    /**
     * End-game climbing configuration. Climber extends, non-climb mechanisms stow or stop to reduce power draw and avoid interference.
     */
    CLIMB_READY("Climb Ready"),

    /**
     * Reverses the ball path to clear a jam. Intake, feeder, and indexer run in reverse. Shooter stops. Harvester stays in its current position.
     */
    EJECT("Eject"),

    /**
     * Safe transit mode for moving across the field. The harvester stows to stay inside the frame perimeter while all other mechanisms idle.
     */
    TRAVEL("Travel");

    private final String displayName;

    GameplayState(String displayName) {
        this.displayName = displayName;
    }

    /**
     * Returns a human-readable name for dashboard display and telemetry logging.
     *
     * @return display-friendly state name
     */
    public String getDisplayName() {
        return displayName;
    }
}
