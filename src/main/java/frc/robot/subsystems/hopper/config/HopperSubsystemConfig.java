package frc.robot.subsystems.hopper.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the hopper subsystem. All RPM values represent belt (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Hopper-specific fields
 * cover the default forward transport speed and the reverse speed used for purging Fuel back through the intake.
 * </p>
 */
public class HopperSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the hopper belt motor. */
    public HopperMotorConfig hopperMotorConfig = new HopperMotorConfig();

    /** Default forward velocity used for transporting Fuel toward the feeder, in RPM. */
    public double            forwardVelocityRpm;

    /** Reverse velocity used for purging Fuel back through the intake, in RPM. Stored as a positive value; the subsystem negates it. */
    public double            reverseVelocityRpm;

    /**
     * Returns the default forward transport velocity, tuned via SmartDashboard.
     *
     * @return forward velocity in RPM (positive value; belt moves toward the feeder)
     */
    public double getForwardVelocityRpm() {
        return readTunableNumber("forwardVelocityRpm", forwardVelocityRpm);
    }

    /**
     * Returns the reverse velocity for purging Fuel, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }
}
