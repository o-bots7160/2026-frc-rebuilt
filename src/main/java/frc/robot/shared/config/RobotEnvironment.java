package frc.robot.shared.config;

import java.util.Optional;
import java.util.OptionalInt;

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

    private static boolean fmsAttachedCached  = false;

    private static boolean autonomousCached   = false;

    private static boolean teleopCached       = false;

    private static boolean disabledCached     = true;

    private static boolean enabledCached      = false;

    private static double  matchTimeCached    = -1.0;

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
        enabledCached     = DriverStation.isEnabled();
        matchTimeCached   = DriverStation.getMatchTime();
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
     * Returns the current alliance color from the Driver Station.
     * <p>
     * The value may be empty early in the match lifecycle or in simulation before an alliance is selected.
     * </p>
     *
     * @return optional alliance color
     */
    public static Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

    /**
     * Returns the driver station position number (1, 2, or 3).
     * <p>
     * The value may be empty if the driver station has not assigned a position yet.
     * </p>
     *
     * @return optional station number
     */
    public static OptionalInt getLocation() {
        return DriverStation.getLocation();
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
     * Reports whether the robot is currently enabled (autonomous or teleop).
     * <p>
     * Returns the per-cycle cached value set by {@link #refreshCycle()}.
     * </p>
     *
     * @return true when the robot is enabled
     */
    public static boolean isEnabled() {
        return enabledCached;
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

    private RobotEnvironment() {
        // Utility class — no instances.
    }
}
