package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.devices.motor.CompositeMotor;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.shooter.devices.ShooterMotor;
import frc.robot.subsystems.shooter.devices.ShooterSimMotor;

/**
 * Shooter subsystem that spins a flywheel to launch Fuel into the scoring hub.
 * <p>
 * The competition robot uses two mechanically coupled motors: a primary and a follower. The follower can be
 * independently inverted and current-limited via its own config block. On robots with only one shooter motor
 * (e.g., the test robot), set {@code shooterFollowerMotorConfig.enabled = false} and the subsystem operates
 * with a single motor transparently.
 * </p>
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
     * Builds the correct motor configuration for the shooter, wrapping two motors in a {@link CompositeMotor}
     * when the follower is enabled, or returning a single motor when it is not.
     *
     * @param config shooter subsystem config containing both motor config bundles
     * @return configured motor (composite or single), or null when the subsystem is disabled
     */
    private static Motor buildShooterMotor(ShooterSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        Motor primary = buildVelocityMotor(
                config,
                config.shooterMotorConfig,
                ShooterMotor::create,
                ShooterSimMotor::create);

        // When the follower config is disabled, operate with the primary motor only.
        if (!config.shooterFollowerMotorConfig.enabled) {
            return primary;
        }

        Motor follower = buildVelocityMotor(
                config,
                config.shooterFollowerMotorConfig,
                ShooterMotor::create,
                ShooterSimMotor::create);

        return new CompositeMotor(primary, follower);
    }

    /**
     * Builds the shooter subsystem with a primary motor and an optional follower motor.
     * <p>
     * When the follower motor config is enabled, both motors are wrapped in a {@link CompositeMotor} so the
     * subsystem hierarchy sees a single {@link Motor}. When the follower is disabled, only the primary motor
     * is used.
     * </p>
     *
     * @param config shooter configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public ShooterSubsystem(ShooterSubsystemConfig config) {
        super(config, buildShooterMotor(config));
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
