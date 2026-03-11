package frc.robot.shared.config;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Base configuration bundle that mirrors values to AdvantageKit-backed SmartDashboard entries so they can be tuned without redeploying. Concrete
 * configs inherit the tunable readers provided here.
 */
public abstract class AbstractConfig {
    /**
     * Describes the 3D pivot offset for an articulated component in AdvantageScope.
     * <p>
     * Embed an instance in any mechanism config that has a corresponding model component in the robot's AdvantageScope asset. The pivot values
     * represent the component's origin relative to the robot's floor-level origin in meters.
     * </p>
     */
    public static class ComponentPoseConfig {

        /** Forward/backward offset in meters (positive = forward). */
        public double componentPivotX = 0.0;

        /** Left/right offset in meters (positive = left). */
        public double componentPivotY = 0.0;

        /** Vertical offset in meters (positive = up). */
        public double componentPivotZ = 0.0;

        /**
         * Converts the configured pivot offsets to a WPILib Translation3d.
         *
         * @return translation representing the component pivot in meters
         */
        public Translation3d toTranslation3d() {
            return new Translation3d(componentPivotX, componentPivotY, componentPivotZ);
        }
    }

    private static final String SMART_DASHBOARD_PREFIX = "SmartDashboard/";

    private static String computeDefaultDashboardPrefix(Class<?> clazz) {
        String classPrefix = clazz.getSimpleName();
        if (classPrefix.endsWith("Config")) {
            classPrefix = classPrefix.substring(0, classPrefix.length() - "Config".length());
        }
        return classPrefix + "/";
    }

    private String                                  dashboardPrefix;

    private final Map<String, LoggedNetworkBoolean> tunableBooleans = new HashMap<>();

    private final Map<String, LoggedNetworkNumber>  tunableNumbers  = new HashMap<>();

    private final Map<String, LoggedNetworkString>  tunableStrings  = new HashMap<>();

    /**
     * Creates a config base and derives the default SmartDashboard prefix from the class name.
     * <p>
     * Subclasses should call this implicitly when deserialized from JSON.
     * </p>
     */
    protected AbstractConfig() {
        this.dashboardPrefix = computeDefaultDashboardPrefix(getClass());
    }

    /**
     * Overrides the SmartDashboard prefix used for tunable key resolution.
     * <p>
     * Call this after Jackson deserialization to propagate a parent config's prefix into nested config objects so their tunable keys appear under the
     * parent's namespace (e.g., {@code FeederSubsystem/pid/kP} instead of {@code Pid/kP}).
     * </p>
     *
     * @param prefix the new dashboard prefix including a trailing slash (e.g., {@code "FeederSubsystem/pid/"})
     */
    public void setDashboardPrefix(String prefix) {
        this.dashboardPrefix = prefix;
    }

    /**
     * Recursively sets the SmartDashboard prefix on nested {@link AbstractConfig} fields so their tunable keys appear under the parent's namespace.
     * <p>
     * Call this after Jackson deserialization. For each public field of type {@link AbstractConfig} (that is not an
     * {@link AbstractSubsystemConfig}), the method sets the field's prefix to {@code parentPrefix + fieldName + "/"} and then recurses into that
     * field. Subsystem configs are skipped because they own their own prefix based on their class name.
     * </p>
     */
    public void initializeNestedDashboardPrefixes() {
        String parentPrefix = getDashboardPrefix();
        for (var field : getClass().getFields()) {
            try {
                Object value = field.get(this);
                if (value instanceof AbstractConfig nested && !(value instanceof AbstractSubsystemConfig)) {
                    nested.setDashboardPrefix(parentPrefix + field.getName() + "/");
                    nested.initializeNestedDashboardPrefixes();
                }
            } catch (IllegalAccessException e) {
                // Skip inaccessible fields
            }
        }
    }

    /**
     * Returns the current SmartDashboard prefix used for tunable key resolution.
     *
     * @return the dashboard prefix (includes trailing slash)
     */
    protected String getDashboardPrefix() {
        return dashboardPrefix;
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
        if (RobotEnvironment.isFMSAttached()) {
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
        if (RobotEnvironment.isFMSAttached()) {
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
        if (RobotEnvironment.isFMSAttached()) {
            return defaultValue;
        }
        String              resolvedKey     = dashboardKey(key);
        LoggedNetworkString dashboardString = tunableStrings.computeIfAbsent(resolvedKey,
                k -> new LoggedNetworkString(k, defaultValue));
        return dashboardString.get();
    }

    /**
     * Reads a tunable number that represents degrees.
     * <p>
     * Use this helper when the stored value is in degrees so all configs share the same conversion pattern.
     * </p>
     *
     * @param key             dashboard key suffix to read (class prefix is applied automatically)
     * @param fallbackDegrees fallback value in degrees used when FMS is attached or no entry exists
     * @return latest tunable value in degrees, or the provided fallback when FMS is attached
     */
    protected double readTunableDegrees(String key, double fallbackDegrees) {
        return readTunableNumber(key, fallbackDegrees);
    }

    /**
     * Reads a tunable number stored in degrees and returns the value in radians.
     * <p>
     * Call this when your config stores angles in degrees but your subsystem expects radians.
     * </p>
     *
     * @param key             dashboard key suffix to read (class prefix is applied automatically)
     * @param fallbackDegrees fallback value in degrees used when FMS is attached or no entry exists
     * @return latest tunable value in radians, or the provided fallback when FMS is attached
     */
    protected double readTunableDegreesAsRadians(String key, double fallbackDegrees) {
        return Units.degreesToRadians(readTunableDegrees(key, fallbackDegrees));
    }

    /**
     * Computes the dashboard key for the given suffix.
     */
    private String dashboardKey(String key) {
        return SMART_DASHBOARD_PREFIX + dashboardPrefix + key;
    }
}