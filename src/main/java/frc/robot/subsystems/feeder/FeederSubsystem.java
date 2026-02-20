package frc.robot.subsystems.feeder;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.feeder.config.FeederSubsystemConfig;
import frc.robot.subsystems.feeder.devices.FeederMotor;
import frc.robot.subsystems.feeder.devices.FeederSimMotor;

/**
 * Feeder subsystem that drives a belt to transport Fuel from the hopper toward the indexer.
 * <p>
 * The feeder sits between the hopper and the indexer. Its belt moves forward (positive RPM) to carry Fuel inward toward the indexer, or reverses
 * (negative RPM) to clear Fuel back toward the hopper when jams occur or the operator requests a purge.
 * </p>
 * <p>
 * All RPM values in the public API represent belt (mechanism) speed after gear reduction, not motor shaft speed. The subsystem extends
 * {@link AbstractVelocitySubsystem} which provides bidirectional velocity control with feedforward and PID.
 * </p>
 */
public class FeederSubsystem extends AbstractVelocitySubsystem<FeederSubsystemConfig> {

    /**
     * Builds the feeder subsystem with a single motor-driven belt.
     *
     * @param config feeder configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public FeederSubsystem(FeederSubsystemConfig config) {
        super(config, buildVelocityMotor(config, config.feederMotorConfig, FeederMotor::create, FeederSimMotor::create));
    }

    /**
     * Convenience method that sets the belt to the configured forward transport velocity.
     * <p>
     * Use this to move Fuel from the hopper side toward the indexer.
     * </p>
     */
    public void setForwardVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setForwardVelocity");
            return;
        }
        setTargetVelocityRpm(config.getForwardVelocityRpm());
    }

    /**
     * Convenience method that sets the belt to reverse at the configured clearing velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the belt spins backward toward the hopper.
     * </p>
     */
    public void setReverseVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setReverseVelocity");
            return;
        }
        setTargetVelocityRpm(-config.getReverseVelocityRpm());
    }
}
