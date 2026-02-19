package frc.robot.subsystems.indexer.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the indexer subsystem. All RPM values represent roller (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Indexer-specific fields
 * cover the default feed speed, reverse speed for clearing, and unjam pulse timing.
 * </p>
 */
public class IndexerSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the indexer roller motor. */
    public IndexerMotorConfig indexerMotorConfig = new IndexerMotorConfig();

    /** Default forward velocity used for feeding Fuel into the shooter, in RPM. */
    public double             feedVelocityRpm;

    /** Reverse velocity used for clearing jams or backing Fuel toward the hopper, in RPM. Stored as a positive value; the subsystem negates it. */
    public double             reverseVelocityRpm;

    /** Duration of each forward pulse during unjam cycling, in seconds. */
    public double             unjamForwardDurationSeconds;

    /** Duration of each reverse pulse during unjam cycling, in seconds. */
    public double             unjamReverseDurationSeconds;

    /**
     * Returns the default feed velocity, tuned via SmartDashboard.
     *
     * @return feed velocity in RPM (positive value; forward into the shooter)
     */
    public double getFeedVelocityRpm() {
        return readTunableNumber("feedVelocityRpm", feedVelocityRpm);
    }

    /**
     * Returns the reverse velocity for clearing jams, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }

    /**
     * Returns the forward pulse duration for the unjam cycle, tuned via SmartDashboard.
     *
     * @return forward pulse duration in seconds
     */
    public double getUnjamForwardDurationSeconds() {
        return readTunableNumber("unjamForwardDurationSeconds", unjamForwardDurationSeconds);
    }

    /**
     * Returns the reverse pulse duration for the unjam cycle, tuned via SmartDashboard.
     *
     * @return reverse pulse duration in seconds
     */
    public double getUnjamReverseDurationSeconds() {
        return readTunableNumber("unjamReverseDurationSeconds", unjamReverseDurationSeconds);
    }
}
