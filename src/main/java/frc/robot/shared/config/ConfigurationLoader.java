package frc.robot.shared.config;

import java.io.File;

import javax.naming.ConfigurationException;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.type.TypeFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Utility for loading subsystem configuration JSON from the deploy directory and recording a read-only snapshot of every field to AdvantageKit.
 * <p>
 * Tunable values are managed separately by {@link AbstractConfig} through {@code LoggedNetworkNumber/Boolean/String}. This loader only produces a
 * one-time, read-only snapshot under {@code /AdvantageKit/RealOutputs/Config/...} so operators can inspect the loaded configuration without
 * confusing it with editable tunable entries.
 * </p>
 */
public class ConfigurationLoader {

    private static final String CONFIG_OUTPUT_PREFIX = "Config";

    /**
     * Loads a configuration file from the deploy directory and maps it to a type.
     * <p>
     * After deserialization, every public primitive and String field is recorded as a read-only AdvantageKit output under
     * {@code Config/<className>/...}. Nested {@link AbstractConfig} objects are recursed into automatically.
     * </p>
     *
     * @param <TConfig> Java type to bind the configuration to
     * @param fileName  JSON filename relative to {@code src/main/deploy}
     * @param classOfT  class token for the configuration type
     * @return loaded configuration instance with AK snapshot recorded
     * @throws ConfigurationException when the file cannot be read or parsed
     */
    public static <TConfig> TConfig load(String fileName, Class<TConfig> classOfT) throws ConfigurationException {
        try {
            // Generic and mapping setup
            JavaType     type   = TypeFactory.defaultInstance().constructType(classOfT);
            ObjectMapper om     = new ObjectMapper();
            om.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

            // Custom deserializer setup
            SimpleModule module = new SimpleModule();
            module.addDeserializer(Pose2d.class, new Pose2dDeserializer());
            om.registerModule(module);

            // File setup
            File    deployDirectory = Filesystem.getDeployDirectory();
            File    configFile      = new File(deployDirectory, fileName);

            // Map the config to the class type and return
            TConfig config          = om.readValue(configFile, type);

            // Propagate dashboard prefixes into nested config objects
            if (config instanceof AbstractConfig abstractConfig) {
                abstractConfig.initializeNestedDashboardPrefixes();
            } else {
                // Container types like SubsystemsConfig are not AbstractConfig,
                // so iterate their fields and initialize any AbstractConfig children
                for (var field : classOfT.getFields()) {
                    try {
                        Object value = field.get(config);
                        if (value instanceof AbstractConfig nestedConfig) {
                            nestedConfig.initializeNestedDashboardPrefixes();
                        }
                    } catch (IllegalAccessException e) {
                        // Skip inaccessible fields
                    }
                }
            }

            // Record a read-only AK snapshot of every config field
            logConfigSnapshot(classOfT, config, CONFIG_OUTPUT_PREFIX);

            return config;
        } catch (Exception e) {
            e.printStackTrace();

            throw new ConfigurationException("Failed to load configuration file: " + fileName);
        }
    }

    /**
     * Recursively records every public field value as a read-only AdvantageKit output.
     */
    private static <TConfig> void logConfigSnapshot(Class<?> classOfT, TConfig config, String parentKey)
            throws IllegalAccessException {
        for (var field : classOfT.getFields()) {
            field.setAccessible(true);
            String key   = parentKey + "/" + field.getName();
            Object value = field.get(config);

            if (value instanceof Double) {
                org.littletonrobotics.junction.Logger.recordOutput(key, (Double) value);
            } else if (value instanceof Boolean) {
                org.littletonrobotics.junction.Logger.recordOutput(key, (Boolean) value);
            } else if (value instanceof Integer) {
                org.littletonrobotics.junction.Logger.recordOutput(key, (double) (Integer) value);
            } else if (value instanceof String) {
                org.littletonrobotics.junction.Logger.recordOutput(key, (String) value);
            } else if (value instanceof AbstractConfig) {
                logConfigSnapshot(value.getClass(), value, key);
            } else if (value instanceof AbstractConfig[] array) {
                for (int i = 0; i < array.length; i++) {
                    if (array[i] != null) {
                        logConfigSnapshot(array[i].getClass(), array[i], key + "/" + i);
                    }
                }
            }
        }
    }
}
