package frc.robot.shared.config;

/**
 * Configuration values for subsystems that maintain a target velocity using feedforward and PID control.
 * <p>
 * Velocity motion profile parameters (max velocity, acceleration, tolerances, settle time, idle velocity) are organized into a nested
 * {@link VelocityMotionConfig} bundle. All RPM values represent mechanism (flywheel) speed after gear reduction, not motor shaft speed. PID and
 * feedforward gains are inherited from {@link AbstractMotorSubsystemConfig}.
 * </p>
 */
public abstract class AbstractVelocitySubsystemConfig extends AbstractMotorSubsystemConfig {

    /** Velocity motion profile parameters for this velocity subsystem. */
    public VelocityMotionConfig motionProfile = new VelocityMotionConfig();
}
