package frc.robot.shared.config;

import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Single source of truth for robot environment state: FMS attachment, simulation detection, alliance info, and Driver Station reporting.
 * <p>
 * All code outside of {@code Robot.java} should call these methods instead of reaching directly into {@link DriverStation} or {@link RobotBase}. This
 * keeps external static dependencies in one place and allows future gating or caching logic to be applied uniformly.
 * </p>
 */
public final class RobotEnvironment {

    private static final String                   ALLIANCE_CHOOSER_AUTO   = "Auto (FMS)";

    private static final String                   ALLIANCE_CHOOSER_BLUE_1 = "Blue 1";

    private static final String                   ALLIANCE_CHOOSER_BLUE_2 = "Blue 2";

    private static final String                   ALLIANCE_CHOOSER_BLUE_3 = "Blue 3";

    private static final String                   ALLIANCE_CHOOSER_RED_1  = "Red 1";

    private static final String                   ALLIANCE_CHOOSER_RED_2  = "Red 2";

    private static final String                   ALLIANCE_CHOOSER_RED_3  = "Red 3";

    private static LoggedDashboardChooser<String> allianceChooser;

    private static Optional<Alliance>             allianceCached          = Optional.empty();

    private static OptionalInt                    locationCached          = OptionalInt.empty();

    /** True when running an AdvantageKit log replay (REPLAY_LOG env var is set). */
    private static final boolean                  REPLAY_MODE             = System.getenv("REPLAY_LOG") != null
            && !System.getenv("REPLAY_LOG").isBlank();

    private static boolean                        fmsAttachedCached       = false;

    private static boolean                        autonomousCached        = false;

    private static boolean                        teleopCached            = false;

    private static boolean                        disabledCached          = true;

    private static double                         matchTimeCached         = -1.0;

    /**
     * Refreshes cached environment state once per robot loop.
     * <p>
     * Call this at the top of {@code robotPeriodic()} so every subsequent call in the same cycle uses the same snapshot without re-querying
     * {@link DriverStation}.
     * </p>
     */
    public static void refreshCycle() {
        fmsAttachedCached = DriverStation.isFMSAttached();
        autonomousCached  = DriverStation.isAutonomous();
        teleopCached      = DriverStation.isTeleop();
        disabledCached    = DriverStation.isDisabled();
        matchTimeCached   = DriverStation.getMatchTime();
        allianceCached    = resolveAlliance();
        locationCached    = resolveLocation();
    }

    /**
     * Reports whether the robot is currently attached to the FMS.
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}.
     * </p>
     *
     * @return true when connected to the official field management system
     */
    public static boolean isFMSAttached() {
        if (REPLAY_MODE) {
            return false;
        }
        return fmsAttachedCached;
    }

    /**
     * Reports whether the code is running in WPILib simulation.
     *
     * @return true when running in simulation rather than on a real robot
     */
    public static boolean isSimulation() {
        return !RobotBase.isReal();
    }

    /**
     * Reports whether the code is running on real robot hardware.
     *
     * @return true when running on a real robot
     */
    public static boolean isReal() {
        return RobotBase.isReal();
    }

    /**
     * Reports whether the code is running an AdvantageKit log replay.
     * <p>
     * During replay the HAL simulation extensions are not loaded, so any code that creates real or simulated hardware
     * (e.g., YAGSL swerve modules with vendor motor controllers) must be skipped to avoid native crashes.
     * </p>
     *
     * @return true when the {@code REPLAY_LOG} environment variable is set
     */
    public static boolean isReplay() {
        return REPLAY_MODE;
    }

    /**
     * Returns the current alliance color, resolved from the FMS or dashboard override.
     * <p>
     * When the FMS is attached its alliance always wins. When the FMS is absent the dashboard {@code Alliance Chooser} is consulted: selecting "Blue"
     * or "Red" forces that alliance, while "Auto (FMS)" falls through to {@link DriverStation#getAlliance()}. The resolved value is cached per cycle
     * by {@link #refreshCycle()} so all consumers in the same loop see a consistent answer.
     * </p>
     *
     * @return optional alliance color
     */
    public static Optional<Alliance> getAlliance() {
        return allianceCached;
    }

    /**
     * Initializes the alliance dashboard chooser with Auto and alliance/station options.
     * <p>
     * Call this once during {@code RobotContainer} construction. The chooser is automatically published to {@code SmartDashboard/Alliance} by the
     * {@link LoggedDashboardChooser} constructor.
     * </p>
     */
    public static void initAllianceChooser() {
        allianceChooser = new LoggedDashboardChooser<>("Alliance");
        allianceChooser.addDefaultOption(ALLIANCE_CHOOSER_AUTO, ALLIANCE_CHOOSER_AUTO);
        allianceChooser.addOption(ALLIANCE_CHOOSER_BLUE_1, ALLIANCE_CHOOSER_BLUE_1);
        allianceChooser.addOption(ALLIANCE_CHOOSER_BLUE_2, ALLIANCE_CHOOSER_BLUE_2);
        allianceChooser.addOption(ALLIANCE_CHOOSER_BLUE_3, ALLIANCE_CHOOSER_BLUE_3);
        allianceChooser.addOption(ALLIANCE_CHOOSER_RED_1, ALLIANCE_CHOOSER_RED_1);
        allianceChooser.addOption(ALLIANCE_CHOOSER_RED_2, ALLIANCE_CHOOSER_RED_2);
        allianceChooser.addOption(ALLIANCE_CHOOSER_RED_3, ALLIANCE_CHOOSER_RED_3);
    }

    /**
     * Returns the driver station position number (1, 2, or 3).
     * <p>
     * When the FMS is attached the value comes directly from the Driver Station. When the FMS is absent the dashboard alliance chooser is consulted.
     * The resolved value is cached per cycle by {@link #refreshCycle()}.
     * </p>
     *
     * @return optional station number
     */
    public static OptionalInt getLocation() {
        return locationCached;
    }

    /**
     * Reports whether the robot is currently in autonomous mode.
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}.
     * </p>
     *
     * @return true when the robot is running an autonomous period
     */
    public static boolean isAutonomous() {
        return autonomousCached;
    }

    /**
     * Reports whether the robot is currently in teleop mode.
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}.
     * </p>
     *
     * @return true when the robot is in operator-controlled teleop
     */
    public static boolean isTeleop() {
        return teleopCached;
    }

    /**
     * Reports whether the robot is currently disabled.
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}.
     * </p>
     *
     * @return true when the robot is disabled
     */
    public static boolean isDisabled() {
        return disabledCached;
    }

    /**
     * Returns the approximate match time remaining in the current period in seconds.
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}. The value is -1.0 when no match time is available (e.g., during practice or
     * when the FMS is not connected). During a real match, this counts down from the period length toward zero.
     * </p>
     *
     * @return match time remaining in seconds, or -1.0 when unavailable
     */
    public static double getMatchTimeSeconds() {
        return matchTimeCached;
    }

    /**
     * Reports a fatal or high-severity error to the Driver Station console.
     * <p>
     * Use this for constructor failures, missing hardware, or anything that should be immediately visible to operators.
     * </p>
     *
     * @param message    error description
     * @param stackTrace stack trace to include, or {@code null} to omit
     */
    public static void reportError(String message, StackTraceElement[] stackTrace) {
        DriverStation.reportError(message, stackTrace);
    }

    /**
     * Reports a recoverable warning to the Driver Station console.
     * <p>
     * Use this for situations like a missing starting pose or an auto file that was not pre-loaded.
     * </p>
     *
     * @param message         warning description
     * @param printStackTrace true to include a stack trace in the output
     */
    public static void reportWarning(String message, boolean printStackTrace) {
        DriverStation.reportWarning(message, printStackTrace);
    }

    /**
     * Silences the joystick connection warning that fires in simulation when no controllers are plugged in.
     *
     * @param silence true to suppress the warning
     */
    public static void silenceJoystickConnectionWarning(boolean silence) {
        DriverStation.silenceJoystickConnectionWarning(silence);
    }

    /**
     * Resolves the alliance from the FMS or dashboard override.
     * <p>
     * FMS takes priority when attached. Otherwise the dashboard chooser is consulted.
     * </p>
     *
     * @return resolved alliance, or empty if undetermined
     */
    private static Optional<Alliance> resolveAlliance() {
        // FMS always wins when attached.
        if (fmsAttachedCached) {
            return DriverStation.getAlliance();
        }

        // Consult the dashboard override when FMS is absent.
        if (allianceChooser != null) {
            String selected = allianceChooser.get();
            if (selected != null && selected.startsWith("Blue")) {
                return Optional.of(Alliance.Blue);
            }
            if (selected != null && selected.startsWith("Red")) {
                return Optional.of(Alliance.Red);
            }
        }

        // "Auto (FMS)" or no chooser — fall through to DriverStation.
        return DriverStation.getAlliance();
    }

    /**
     * Resolves the station position from the FMS or dashboard override.
     * <p>
     * FMS takes priority when attached. Otherwise the trailing digit from the dashboard chooser selection is parsed.
     * </p>
     *
     * @return resolved station number, or empty if undetermined
     */
    private static OptionalInt resolveLocation() {
        // FMS always wins when attached.
        if (fmsAttachedCached) {
            return DriverStation.getLocation();
        }

        // Parse the station number from the chooser selection (e.g., "Blue 2" → 2).
        if (allianceChooser != null) {
            String selected = allianceChooser.get();
            if (selected != null && selected.length() >= 2) {
                char lastChar = selected.charAt(selected.length() - 1);
                if (lastChar >= '1' && lastChar <= '3') {
                    return OptionalInt.of(lastChar - '0');
                }
            }
        }

        // "Auto (FMS)" or no chooser — fall through to DriverStation.
        return DriverStation.getLocation();
    }

    private RobotEnvironment() {
        // Utility class — no instances.
    }
}
