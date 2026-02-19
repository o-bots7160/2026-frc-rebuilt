package frc.robot.subsystems.hopper;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.hopper.config.HopperSubsystemConfig;
import frc.robot.subsystems.hopper.devices.HopperMotor;
import frc.robot.subsystems.hopper.devices.HopperSimMotor;

/**
 * Hopper subsystem that drives a belt to transport Fuel from the intake toward the feeder.
 * <p>
 * The hopper sits between the intake and the feeder. Its belt moves forward (positive RPM) to carry Fuel inward toward the feeder, or reverses
 * (negative RPM) to purge Fuel back through the intake when clearing jams or ejecting unwanted pieces.
 * </p>
 * <p>
 * All RPM values in the public API represent belt (mechanism) speed after gear reduction, not motor shaft speed. The subsystem extends
 * {@link AbstractVelocitySubsystem} which provides bidirectional velocity control with feedforward and PID.
 * </p>
 */
public class HopperSubsystem extends AbstractVelocitySubsystem<HopperSubsystemConfig> {

    /**
     * Builds the hopper subsystem with a single motor-driven belt.
     *
     * @param config hopper configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public HopperSubsystem(HopperSubsystemConfig config) {
        super(config, buildVelocityMotor(config, config.hopperMotorConfig, HopperMotor::create, HopperSimMotor::create));
    }

    /**
     * Convenience method that sets the belt to the configured forward transport velocity.
     * <p>
     * Use this to move Fuel from the intake side toward the feeder.
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
     * Convenience method that sets the belt to reverse at the configured purge velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the belt spins backward toward the intake.
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
