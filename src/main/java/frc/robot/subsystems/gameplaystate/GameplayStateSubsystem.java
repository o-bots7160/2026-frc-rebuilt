package frc.robot.subsystems.gameplaystate;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.gameplaystate.config.GameplayStateSubsystemConfig;
import frc.robot.subsystems.gameplaystate.io.GameplayStateIO;
import frc.robot.subsystems.gameplaystate.io.GameplayStateIOInputsAutoLogged;

/**
 * Tracks the robot's high-level operating mode and publishes the current gameplay state for command factories and telemetry.
 * <p>
 * This subsystem does not own hardware or directly schedule commands on other subsystems. It maintains the current {@link GameplayState} and exposes
 * it so the companion {@code GameplayStateCommandFactory} can compose the appropriate parallel command groups. Automatic transitions based on FMS
 * match phase and endgame timer are evaluated each cycle when enabled in configuration.
 * </p>
 */
public class GameplayStateSubsystem extends AbstractSubsystem<GameplayStateSubsystemConfig> {

    /**
     * Internal enum representing the FMS match phase for transition detection.
     */
    private enum MatchPhase {
        DISABLED,
        AUTONOMOUS,
        TELEOP
    }

    private final GameplayStateIO                 io;

    private final GameplayStateIOInputsAutoLogged inputs           = new GameplayStateIOInputsAutoLogged();

    private final Supplier<Double>                distanceToTargetMetersSupplier;

    private final Supplier<String>                activeTargetNameSupplier;

    private GameplayState                         currentState     = GameplayState.IDLE;

    private GameplayState                         requestedState   = GameplayState.IDLE;

    private String                                transitionSource = "init";

    private boolean                               endgameSuggested = false;

    /**
     * Tracks the previous match phase so we can detect transitions between autonomous, teleop, and disabled.
     */
    private MatchPhase                            previousPhase    = MatchPhase.DISABLED;

    /**
     * Creates the Gameplay State subsystem with targeting telemetry suppliers.
     *
     * @param config                          configuration values for auto-transition behavior and endgame thresholds
     * @param distanceToTargetMetersSupplier  supplier returning the distance from the robot to the active field target in meters
     * @param activeTargetNameSupplier        supplier returning the human-readable name of the active field target
     */
    public GameplayStateSubsystem(
            GameplayStateSubsystemConfig config,
            Supplier<Double> distanceToTargetMetersSupplier,
            Supplier<String> activeTargetNameSupplier) {
        super(config);

        this.io                             = this::updateInputs;
        this.distanceToTargetMetersSupplier  = distanceToTargetMetersSupplier != null ? distanceToTargetMetersSupplier : () -> 0.0;
        this.activeTargetNameSupplier        = activeTargetNameSupplier != null ? activeTargetNameSupplier : () -> "Unknown";

        // Publish initial state so the NetworkTables topic exists before the first periodic cycle.
        SmartDashboard.putString("GameplayStateSubsystem/CurrentState", currentState.getDisplayName());
        SmartDashboard.putNumber("GameplayStateSubsystem/DistanceToTargetMeters", 0.0);
        SmartDashboard.putString("GameplayStateSubsystem/ActiveTarget", "Unknown");

        if (isSubsystemDisabled()) {
            log.verbose("GameplayStateSubsystem disabled; skipping initialization.");
        }
    }

    /**
     * Evaluates automatic state transitions and logs telemetry each robot loop.
     */
    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            return;
        }

        if (!isFMSAttached()) {
            refreshTunables();
        }

        evaluateAutoTransitions();

        io.updateInputs(inputs);
        log.processInputs("GameplayState", inputs);

        // Publish operator-critical state to SmartDashboard for the Elastic Dashboard.
        SmartDashboard.putString("GameplayStateSubsystem/CurrentState", currentState.getDisplayName());
        SmartDashboard.putNumber("GameplayStateSubsystem/DistanceToTargetMeters", distanceToTargetMetersSupplier.get());
        SmartDashboard.putString("GameplayStateSubsystem/ActiveTarget", activeTargetNameSupplier.get());
    }

    /**
     * Requests a transition to the specified gameplay state.
     * <p>
     * The state change takes effect immediately. The transition source is recorded for telemetry so operators can see what triggered each change.
     * </p>
     *
     * @param state  desired gameplay state
     * @param source description of what triggered this transition (e.g., "operator", "fms", "auto", "timer")
     */
    public void requestState(GameplayState state, String source) {
        if (isSubsystemDisabled()) {
            logDisabled("requestState");
            return;
        }

        this.requestedState   = state;
        this.currentState     = state;
        this.transitionSource = source;

        if (isVerbose()) {
            log.verbose("State transition: " + state.getDisplayName() + " (source: " + source + ")");
        }
    }

    /**
     * Returns the current gameplay state.
     *
     * @return active gameplay state
     */
    public GameplayState getCurrentState() {
        return currentState;
    }

    /**
     * Returns true when the endgame threshold has been reached.
     * <p>
     * Use this as a dashboard indicator or driver alert.
     * </p>
     *
     * @return true when match time is below the configured endgame threshold during teleop
     */
    public boolean isEndgameSuggested() {
        return endgameSuggested;
    }

    private void evaluateAutoTransitions() {
        if (!config.getAutoTransitionEnabled()) {
            evaluateEndgameSuggestion();
            return;
        }

        MatchPhase currentPhase = detectMatchPhase();

        // Detect phase transitions.
        if (currentPhase != previousPhase) {
            handlePhaseTransition(currentPhase);
            previousPhase = currentPhase;
        }

        evaluateEndgameSuggestion();
    }

    private void handlePhaseTransition(MatchPhase newPhase) {
        switch (newPhase) {
        case AUTONOMOUS:
            requestState(GameplayState.AUTO_CYCLE, "fms");
            break;
        case TELEOP:
            requestState(GameplayState.IDLE, "fms");
            break;
        case DISABLED:
            requestState(GameplayState.IDLE, "fms");
            break;
        }
    }

    private void evaluateEndgameSuggestion() {
        double matchTime = RobotEnvironment.getMatchTimeSeconds();
        boolean isTeleop = RobotEnvironment.isTeleop();

        // Match time of -1.0 means no match time is available.
        if (!isTeleop || matchTime < 0) {
            endgameSuggested = false;
            return;
        }

        double threshold = config.getEndgameThresholdSeconds();
        if (matchTime <= threshold) {
            endgameSuggested = true;
        } else {
            endgameSuggested = false;
        }
    }

    private MatchPhase detectMatchPhase() {
        if (RobotEnvironment.isDisabled()) {
            return MatchPhase.DISABLED;
        }
        if (RobotEnvironment.isAutonomous()) {
            return MatchPhase.AUTONOMOUS;
        }
        if (RobotEnvironment.isTeleop()) {
            return MatchPhase.TELEOP;
        }
        return MatchPhase.DISABLED;
    }

    private void updateInputs(GameplayStateIO.GameplayStateIOInputs inputs) {
        inputs.currentState                 = currentState.getDisplayName();
        inputs.requestedState               = requestedState.getDisplayName();
        inputs.transitionSource             = transitionSource;
        inputs.isAutonomous                 = RobotEnvironment.isAutonomous();
        inputs.isTeleop                     = RobotEnvironment.isTeleop();
        inputs.isDisabled                   = RobotEnvironment.isDisabled();
        inputs.matchTimeSeconds             = RobotEnvironment.getMatchTimeSeconds();
        inputs.autoTransitionEnabled        = config.getAutoTransitionEnabled();
        inputs.endgameSuggested             = endgameSuggested;
        inputs.distanceToTargetMeters       = distanceToTargetMetersSupplier.get();
        inputs.activeTargetName             = activeTargetNameSupplier.get();
    }

    private void refreshTunables() {
        // All config values are read via their getters using readTunableNumber/readTunableBoolean,
        // so tunables refresh automatically when accessed. No cached fields to update here.
    }
}
