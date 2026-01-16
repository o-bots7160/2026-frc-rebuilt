package frc.robot.shared.config;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class AbstractSubsystemConfig {
    private static final String SMART_DASHBOARD_PREFIX = "SmartDashboard/";

    private static String computeDefaultDashboardPrefix(Class<?> clazz) {
        String classPrefix = clazz.getSimpleName();
        if (classPrefix.endsWith("Config")) {
            classPrefix = classPrefix.substring(0, classPrefix.length() - "Config".length());
        }
        return classPrefix + "/";
    }

    private final String                           defaultDashboardPrefix;

    public boolean                                 enabled        = true;

    public boolean                                 verbose        = true;

    private final Map<String, LoggedNetworkNumber> tunableNumbers = new HashMap<>();

    private final Map<String, LoggedNetworkString> tunableStrings = new HashMap<>();

    protected AbstractSubsystemConfig() {
        this.defaultDashboardPrefix = computeDefaultDashboardPrefix(getClass());
    }

    /**
     * Reads a tunable number backed by AdvantageKit's logged network inputs so tweaks are captured in logs and respected during replay, but still
     * falls back to the default when attached to FMS to avoid match-time latency.
     */
    protected double readTunableNumber(String key, double defaultValue) {
        String              resolvedKey     = dashboardKey(key);
        LoggedNetworkNumber dashboardNumber = tunableNumbers.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkNumber(k, defaultValue));
        return DriverStation.isFMSAttached() ? defaultValue : dashboardNumber.get();
    }

    /**
     * Reads a tunable string backed by AdvantageKit's logged network inputs so tweaks are captured in logs and respected during replay, but still
     * falls back to the default when attached to FMS to avoid match-time latency.
     */
    protected String readTunableString(String key, String defaultValue) {
        String              resolvedKey     = dashboardKey(key);
        LoggedNetworkString dashboardString = tunableStrings.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkString(k, defaultValue));
        return DriverStation.isFMSAttached() ? defaultValue : dashboardString.get();
    }

    /**
     * Computes the default dashboard prefix as the simple class name without a trailing "Config" suffix.
     */
    private String dashboardKey(String key) {
        return SMART_DASHBOARD_PREFIX + defaultDashboardPrefix + key;
    }
}
