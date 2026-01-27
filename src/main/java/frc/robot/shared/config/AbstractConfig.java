package frc.robot.shared.config;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Base configuration bundle that mirrors values to AdvantageKit-backed SmartDashboard entries so they can be tuned without redeploying. Concrete
 * configs inherit the tunable readers provided here.
 */
public abstract class AbstractConfig {
    private static final String SMART_DASHBOARD_PREFIX = "SmartDashboard/";

    private static String computeDefaultDashboardPrefix(Class<?> clazz) {
        String classPrefix = clazz.getSimpleName();
        if (classPrefix.endsWith("Config")) {
            classPrefix = classPrefix.substring(0, classPrefix.length() - "Config".length());
        }
        return classPrefix + "/";
    }

    private final String                            defaultDashboardPrefix;

    public boolean                                  enabled         = true;

    public boolean                                  verbose         = true;

    private final Map<String, LoggedNetworkBoolean> tunableBooleans = new HashMap<>();

    private final Map<String, LoggedNetworkNumber>  tunableNumbers  = new HashMap<>();

    private final Map<String, LoggedNetworkString>  tunableStrings  = new HashMap<>();

    protected AbstractConfig() {
        this.defaultDashboardPrefix = computeDefaultDashboardPrefix(getClass());
    }

    /**
     * Reads a tunable number backed by AdvantageKit's logged network inputs so tweaks are captured in logs and respected during replay, but still
     * falls back to the default when attached to FMS to avoid match-time latency. Warning: when FMS is attached, this short-circuits to the default
     * and does not create or read any dashboard entry.
     *
     * @param key          dashboard key suffix to read (class prefix is applied automatically)
     * @param defaultValue fallback value used when FMS is attached or no entry exists
     * @return latest tunable number or the provided default when FMS is attached
     */
    protected double readTunableNumber(String key, double defaultValue) {
        if (DriverStation.isFMSAttached()) {
            return defaultValue;
        }
        String              resolvedKey     = dashboardKey(key);
        LoggedNetworkNumber dashboardNumber = tunableNumbers.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkNumber(k, defaultValue));
        return dashboardNumber.get();
    }

    /**
     * Reads a tunable boolean backed by AdvantageKit's logged network inputs so tweaks are captured in logs and respected during replay, but still
     * falls back to the default when attached to FMS to avoid match-time latency. Warning: when FMS is attached, this short-circuits to the default
     * and does not create or read any dashboard entry.
     *
     * @param key          dashboard key suffix to read (class prefix is applied automatically)
     * @param defaultValue fallback value used when FMS is attached or no entry exists
     * @return latest tunable boolean or the provided default when FMS is attached
     */
    protected boolean readTunableBoolean(String key, boolean defaultValue) {
        if (DriverStation.isFMSAttached()) {
            return defaultValue;
        }
        String               resolvedKey      = dashboardKey(key);
        LoggedNetworkBoolean dashboardBoolean = tunableBooleans.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkBoolean(k, defaultValue));
        return dashboardBoolean.get();
    }

    /**
     * Reads a tunable string backed by AdvantageKit's logged network inputs so tweaks are captured in logs and respected during replay, but still
     * falls back to the default when attached to FMS to avoid match-time latency. Warning: when FMS is attached, this short-circuits to the default
     * and does not create or read any dashboard entry.
     *
     * @param key          dashboard key suffix to read (class prefix is applied automatically)
     * @param defaultValue fallback value used when FMS is attached or no entry exists
     * @return latest tunable string or the provided default when FMS is attached
     */
    protected String readTunableString(String key, String defaultValue) {
        if (DriverStation.isFMSAttached()) {
            return defaultValue;
        }
        String              resolvedKey     = dashboardKey(key);
        LoggedNetworkString dashboardString = tunableStrings.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkString(k, defaultValue));
        return dashboardString.get();
    }

    /**
     * Computes the default dashboard prefix as the simple class name without a trailing "Config" suffix.
     */
    private String dashboardKey(String key) {
        return SMART_DASHBOARD_PREFIX + defaultDashboardPrefix + key;
    }
}