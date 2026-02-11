package frc.robot.subsystems.shooter.config;

import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

/**
 * Configuration bundle for the shooter subsystem. All RPM values represent flywheel (mechanism) speed after gear reduction, not motor shaft speed.
 * <p>
 * Velocity limits, PID gains, feedforward gains, and settle time are inherited from {@link AbstractVelocitySubsystemConfig}. Add shooter-specific
 * tuning fields here as the design evolves (e.g., reverse velocity for clearing jams).
 * </p>
 */
public class ShooterSubsystemConfig extends AbstractVelocitySubsystemConfig {

    /** Motor configuration bundle for the shooter flywheel motor. */
    public ShooterMotorConfig shooterMotorConfig = new ShooterMotorConfig();

    /** Reverse velocity used for clearing stuck pieces, in RPM. Set to 0 to disable reverse mode. */
    public double             reverseVelocityRpm;

    /**
     * Returns the reverse velocity for clearing jams, tuned via SmartDashboard.
     *
     * @return reverse velocity in RPM (positive value; the subsystem applies the sign)
     */
    public double getReverseVelocityRpm() {
        return readTunableNumber("reverseVelocityRpm", reverseVelocityRpm);
    }
}
