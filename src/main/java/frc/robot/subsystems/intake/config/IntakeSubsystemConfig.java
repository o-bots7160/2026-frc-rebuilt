package frc.robot.subsystems.intake.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the intake subsystem. All RPM values represent roller (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Intake-specific fields
 * cover the default forward intake speed and the reverse speed used for ejecting Fuel back onto the field.
 * </p>
 */
public class IntakeSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the intake roller motor. */
    public IntakeMotorConfig intakeMotorConfig = new IntakeMotorConfig();

    /** Default forward velocity used for pulling Fuel from the field into the feeder, in RPM. */
    public double            forwardVelocityRpm;

    /** Reverse velocity used for ejecting Fuel back onto the field, in RPM. Stored as a positive value; the subsystem negates it. */
    public double            reverseVelocityRpm;

    /**
     * Returns the default forward intake velocity, tuned via SmartDashboard.
     *
     * @return forward velocity in RPM (positive value; rollers pull Fuel inward)
     */
    public double getForwardVelocityRpm() {
        return readTunableNumber("forwardVelocityRpm", forwardVelocityRpm);
    }

    /**
     * Returns the reverse velocity for ejecting Fuel, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }
}
