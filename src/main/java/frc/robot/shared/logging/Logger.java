package frc.robot.shared.logging;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.shared.config.RobotEnvironment;

/**
 * Logger is a utility class for logging messages with different levels of severity. It supports verbose, debug, info, warning, and error messages.
 * The output is color-coded for better readability in the console and automatically includes the class name to help diagnose where the log
 * originated. AdvantageKit is the preferred telemetry path; SmartDashboard helpers remain for operator-critical values only.
 * <p>
 * The {@code recordVerboseOutput} family of methods gates on both FMS attachment and the per-instance
 * verbose flag. Verbose output is suppressed when the robot is connected to the FMS or when the
 * owning subsystem has verbose disabled in its configuration. Use {@code recordOutput} for data
 * that must always be logged for match replay.
 * </p>
 * <p>
 * For robot mode detection, FMS state, and Driver Station reporting, see {@link RobotEnvironment}.
 * </p>
 */
public class Logger {

    /**
     * Returns an instance of Logger for the specified class
     *
     * @param <T> the type of the class for which the Logger instance is being created
     * @param c   the Class object for which the Logger instance is being created
     * @return a new Logger instance for the specified class with the given verbosity setting
     */
    public static <T> Logger getInstance(Class<T> c) {
        return getInstance(c.getSimpleName());
    }

    /**
     * Returns an instance of Logger for the specified class with a fixed verbosity setting.
     *
     * @param <T>     the type of the class for which the Logger instance is being created
     * @param c       the Class object for which the Logger instance is being created
     * @param verbose the verbosity setting for the logger
     * @return a new Logger instance for the specified class with the given verbosity setting
     */
    public static <T> Logger getInstance(Class<T> c, boolean verbose) {
        return getInstance(c.getSimpleName(), verbose);
    }

    /**
     * Returns an instance of Logger for the specified class with a dynamic verbosity supplier.
     * <p>
     * The supplier is evaluated each time the logger checks verbosity, allowing live
     * toggling from dashboards without calling {@code setVerbose}.
     * </p>
     *
     * @param <T>             the type of the class for which the Logger instance is being created
     * @param c               the Class object for which the Logger instance is being created
     * @param verboseSupplier supplier that returns true when verbose output should be enabled
     * @return a new Logger instance for the specified class with the given verbosity supplier
     */
    public static <T> Logger getInstance(Class<T> c, BooleanSupplier verboseSupplier) {
        return getInstance(c.getSimpleName(), verboseSupplier);
    }

    /**
     * Returns an instance of Logger for the specified name
     *
     * @param className the name to scope the logger instance to
     * @return a new Logger instance for the specified class with the given verbosity setting
     */
    public static Logger getInstance(String className) {
        return new Logger(className, () -> false);
    }

    /**
     * Returns an instance of Logger for the specified name with a fixed verbosity setting.
     *
     * @param className the name to scope the logger instance to
     * @param verbose   the verbosity setting for the logger
     * @return a new Logger instance for the specified class with the given verbosity setting
     */
    public static Logger getInstance(String className, boolean verbose) {
        return new Logger(className, () -> verbose);
    }

    /**
     * Returns an instance of Logger for the specified name with a dynamic verbosity supplier.
     * <p>
     * The supplier is evaluated each time the logger checks verbosity, allowing live
     * toggling from dashboards without calling {@code setVerbose}.
     * </p>
     *
     * @param className       the name to scope the logger instance to
     * @param verboseSupplier supplier that returns true when verbose output should be enabled
     * @return a new Logger instance for the specified class with the given verbosity supplier
     */
    public static Logger getInstance(String className, BooleanSupplier verboseSupplier) {
        return new Logger(className, verboseSupplier);
    }

    private String          className;

    private BooleanSupplier verboseSupplier;

    /** Persistent dashboard alert for the most recent error message. */
    private final Alert errorAlert;

    /** Persistent dashboard alert for the most recent warning message. */
    private final Alert warningAlert;

    /**
     * Creates a logger scoped to the provided class name.
     * <p>
     * Two persistent {@link Alert} instances are created using the class name as the alert group so dashboard
     * alerts are organized per-subsystem. Alerts remain active until explicitly cleared via {@link #clearAlerts()}.
     * </p>
     *
     * @param className       name to prefix all log messages with
     * @param verboseSupplier supplier evaluated each cycle to determine whether verbose output is enabled
     */
    protected Logger(String className, BooleanSupplier verboseSupplier) {
        this.className       = className;
        this.verboseSupplier = verboseSupplier;
        this.errorAlert      = new Alert(className, "", AlertType.kError);
        this.warningAlert    = new Alert(className, "", AlertType.kWarning);
    }

    /**
     * Logs a verbose message to the console if verbose output is enabled. The message is prefixed with "VERBOSE:" and the class name, and is
     * displayed in gray color.
     *
     * @param message The message to be logged.
     */
    public void verbose(String message) {
        if (verboseSupplier.getAsBoolean()) {
            System.out.println("\u001B[90mVERBOSE: " + className + ": " + message + "\u001B[0m");
        }
    }

    /**
     * Logs a debug message to the console if verbose output is enabled. The message is prefixed with "DEBUG:" and the class name, and is displayed in
     * white color.
     *
     * @param message The debug message to be logged.
     */
    public void debug(String message) {
        if (verboseSupplier.getAsBoolean()) {
            System.out.println("\u001B[37mDEBUG: " + className + ": " + message + "\u001B[0m");
        }
    }

    /**
     * Logs an informational message to the console. The message is prefixed with "INFO:" and the class name, and is displayed in white color.
     *
     * @param message The informational message to be logged.
     */
    public void info(String message) {
        System.out.println("\u001B[37mINFO: " + className + ": " + message + "\u001B[0m");
    }

    /**
     * Logs a warning message to the console and activates a persistent dashboard {@link Alert}.
     * <p>
     * The alert remains visible on supported dashboards until dismissed via {@link #clearAlerts()}. Each call
     * replaces the previous warning text, so only the most recent warning is displayed.
     * </p>
     *
     * @param message The warning message to be logged.
     */
    public void warning(String message) {
        System.out.println("\u001B[33mWARN: " + className + ": " + message + "\u001B[0m");
        warningAlert.setText(message);
        warningAlert.set(true);
    }

    /**
     * Logs an error message to the standard error console and activates a persistent dashboard {@link Alert}.
     * <p>
     * The alert remains visible on supported dashboards until dismissed via {@link #clearAlerts()}. Each call
     * replaces the previous error text, so only the most recent error is displayed.
     * </p>
     *
     * @param message The error message to be logged.
     */
    public void error(String message) {
        System.err.println("\u001B[31mERROR: " + className + ": " + message + "\u001B[0m");
        errorAlert.setText(message);
        errorAlert.set(true);
    }

    /**
     * Dismisses both the error and warning dashboard alerts.
     * <p>
     * Call this when a previously reported condition has recovered so stale alerts do not remain visible to
     * operators. Safe to call even if no alerts are currently active.
     * </p>
     */
    public void clearAlerts() {
        errorAlert.set(false);
        warningAlert.set(false);
    }

    /**
     * Records a boolean to AdvantageKit using the class name as a prefix. Prefer this for telemetry instead of SmartDashboard to reduce NetworkTables
     * noise.
     *
     * @param key   telemetry key suffix
     * @param value value to record
     */
    public void recordOutput(String key, boolean value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
        // debug("ak/" + key + ": " + value);
    }

    /**
     * Records a numeric value to AdvantageKit using the class name as a prefix. Prefer this for telemetry instead of SmartDashboard to reduce
     * NetworkTables noise.
     *
     * @param key   telemetry key suffix
     * @param value value to record
     */
    public void recordOutput(String key, double value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
        // debug("ak/" + key + ": " + value);
    }

    /**
     * Records an array of numeric values to AdvantageKit using the class name as a prefix.
     *
     * @param key    telemetry key suffix
     * @param values values to record
     */
    public void recordOutput(String key, double[] values) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, values);
    }

    /**
     * Records a string value to AdvantageKit using the class name as a prefix.
     *
     * @param key   telemetry key suffix
     * @param value string value to record
     */
    public void recordOutput(String key, String value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
    }

    /**
     * Records a struct-serializable value to AdvantageKit using the class name as a prefix.
     * <p>
     * Use this for WPILib types that implement {@link StructSerializable} (poses, rotations, chassis speeds, module states, etc.).
     * </p>
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable value to record
     * @param <T>   value type that supports struct serialization
     */
    public <T extends StructSerializable> void recordOutput(String key, T value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
    }

    /**
     * Records an array of struct-serializable values to AdvantageKit using the class name as a prefix.
     * <p>
     * This matches AdvantageKit's struct array logging for types like module states or pose lists.
     * </p>
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable values to record
     * @param <T>   value type that supports struct serialization
     */
    @SuppressWarnings("unchecked")
    public <T extends StructSerializable> void recordOutput(String key, T... value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
    }

    /**
     * Records a 2D array of struct-serializable values to AdvantageKit using the class name as a prefix.
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable values to record
     * @param <T>   value type that supports struct serialization
     */
    public <T extends StructSerializable> void recordOutput(String key, T[][] value) {
        org.littletonrobotics.junction.Logger.recordOutput(className + '/' + key, value);
    }

    /**
     * Records an auto-logged input snapshot with the logger prefix applied.
     *
     * @param key    input key suffix for AdvantageKit
     * @param inputs auto-logged inputs bundle to record
     */
    public void processInputs(String key, LoggableInputs inputs) {
        org.littletonrobotics.junction.Logger.processInputs(className + '/' + key, inputs);
    }

    /**
     * Records a string value to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value string value to record
     */
    public void recordVerboseOutput(String key, String value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

    /**
     * Records a boolean to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value value to record
     */
    public void recordVerboseOutput(String key, boolean value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

    /**
     * Records a numeric value to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value value to record
     */
    public void recordVerboseOutput(String key, double value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

    /**
     * Records an array of numeric values to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key    telemetry key suffix
     * @param values values to record
     */
    public void recordVerboseOutput(String key, double[] values) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, values);
        }
    }

    /**
     * Records a struct-serializable value to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable value to record
     * @param <T>   value type that supports struct serialization
     */
    public <T extends StructSerializable> void recordVerboseOutput(String key, T value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

    /**
     * Records an array of struct-serializable values to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable values to record
     * @param <T>   value type that supports struct serialization
     */
    @SuppressWarnings("unchecked")
    public <T extends StructSerializable> void recordVerboseOutput(String key, T... value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

    /**
     * Records a 2D array of struct-serializable values to AdvantageKit only when not attached to the FMS and verbose is enabled.
     *
     * @param key   telemetry key suffix
     * @param value struct-serializable values to record
     * @param <T>   value type that supports struct serialization
     */
    public <T extends StructSerializable> void recordVerboseOutput(String key, T[][] value) {
        if (!RobotEnvironment.isFMSAttached() && verboseSupplier.getAsBoolean()) {
            recordOutput(key, value);
        }
    }

}
