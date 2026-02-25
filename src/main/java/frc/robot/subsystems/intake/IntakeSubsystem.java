package frc.robot.subsystems.intake;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.intake.config.IntakeSubsystemConfig;
import frc.robot.subsystems.intake.devices.IntakeMotor;
import frc.robot.subsystems.intake.devices.IntakeSimMotor;

/**
 * Intake subsystem that spins rollers to pull Fuel from the field into the hopper.
 * <p>
 * The intake is the first mechanism a game piece touches after leaving the field. Its rollers spin forward (positive RPM) to grab Fuel off the
 * carpet and push it into the hopper, or reverse (negative RPM) to eject unwanted pieces back onto the field.
 * </p>
 * <p>
 * All RPM values in the public API represent roller (mechanism) speed after gear reduction, not motor shaft speed. The subsystem extends
 * {@link AbstractVelocitySubsystem} which provides bidirectional velocity control with feedforward and PID.
 * </p>
 */
public class IntakeSubsystem extends AbstractVelocitySubsystem<IntakeSubsystemConfig> {

    /**
     * Builds the intake subsystem with a single motor-driven roller.
     *
     * @param config intake configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public IntakeSubsystem(IntakeSubsystemConfig config) {
        super(config, buildVelocityMotor(config, config.intakeMotorConfig, IntakeMotor::create, IntakeSimMotor::create));
    }

    /**
     * Convenience method that sets the rollers to the configured forward intake velocity.
     * <p>
     * Use this to pull Fuel from the field into the hopper.
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
     * Convenience method that sets the rollers to reverse at the configured eject velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the rollers spin backward to push Fuel out.
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
