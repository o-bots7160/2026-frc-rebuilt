package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.shooter.devices.ShooterMotor;
import frc.robot.subsystems.shooter.devices.ShooterSimMotor;

/**
 * Shooter subsystem that spins a flywheel to launch Fuel into the scoring hub.
 * <p>
 * All RPM values in the public API represent flywheel (mechanism) speed after gear reduction, not motor shaft speed. The gear ratio is applied at the
 * motor encoder conversion layer, so callers never deal with motor-side rotations.
 * </p>
 * <p>
 * The subsystem uses a feedforward model to estimate the voltage needed for a target velocity and a PID controller to correct for disturbances (such
 * as when a piece enters the shooter and momentarily slows the flywheel). An optional trapezoidal velocity ramp provides smooth spin-up when
 * configured.
 * </p>
 */
public class ShooterSubsystem extends AbstractVelocitySubsystem<ShooterSubsystemConfig> {

    /**
     * Builds the shooter subsystem with a single SparkMax-driven flywheel motor.
     *
     * @param config shooter configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public ShooterSubsystem(ShooterSubsystemConfig config) {
        super(config, buildVelocityMotor(config, config.shooterMotorConfig, ShooterMotor::create, ShooterSimMotor::create));
    }

    /**
     * Sets a new target velocity, clamped to forward-only rotation.
     * <p>
     * Flywheels should never reverse through the velocity controller. Negative values are clamped to zero before delegating to the base class.
     * </p>
     *
     * @param targetRpm desired flywheel velocity in RPM (0 to stop, positive to spin forward)
     */
    @Override
    public void setTargetVelocityRpm(double targetRpm) {
        double forwardOnlyRpm = MathUtil.clamp(targetRpm, 0.0, config.getMaximumVelocityRpm());
        super.setTargetVelocityRpm(forwardOnlyRpm);
    }
}
