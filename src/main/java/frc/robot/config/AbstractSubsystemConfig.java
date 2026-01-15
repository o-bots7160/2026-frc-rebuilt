package frc.robot.config;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AbstractSubsystemConfig {
    public boolean enabled = true;

    public boolean verbose = true;

    /**
     * Reads a tunable number from SmartDashboard, but returns the default when attached to FMS to avoid match-time latency.
     */
    protected double readTunableNumber(String key, double defaultValue) {
        if (DriverStation.isFMSAttached()) {
            return defaultValue;
        }
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /**
     * Reads a tunable string from SmartDashboard, but returns the default when attached to FMS to avoid match-time latency.
     */
    protected String readTunableString(String key, String defaultValue) {
        if (DriverStation.isFMSAttached()) {
            return defaultValue;
        }
        return SmartDashboard.getString(key, defaultValue);
    }
}
