package frc.robot.subsystems.feeder.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the feeder subsystem. All RPM values represent belt (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Feeder-specific fields
 * cover the default forward transport speed and the reverse speed used for clearing Fuel back toward the intake.
 * </p>
 */
public class FeederSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the feeder belt motor. */
    public FeederMotorConfig feederMotorConfig = new FeederMotorConfig();

    /** Default forward velocity used for transporting Fuel toward the indexer, in RPM. */
    public double            forwardVelocityRpm;

    /** Reverse velocity used for clearing Fuel back toward the intake, in RPM. Stored as a positive value; the subsystem negates it. */
    public double            reverseVelocityRpm;

    /**
     * Returns the default forward transport velocity, tuned via SmartDashboard.
     *
     * @return forward velocity in RPM (positive value; belt moves toward the indexer)
     */
    public double getForwardVelocityRpm() {
        return readTunableNumber("forwardVelocityRpm", forwardVelocityRpm);
    }

    /**
     * Returns the reverse velocity for clearing Fuel, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }
}
