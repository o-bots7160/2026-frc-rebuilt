package frc.robot.subsystems.shooter.devices;

import frc.robot.devices.motor.AbstractVelocityMotor;
import frc.robot.subsystems.shooter.config.ShooterMotorConfig;

/**
 * SparkMax-backed shooter flywheel motor configured from the shooter subsystem config.
 * <p>
 * Inherits coast-mode idle, gearing, live-tunable configuration, and telemetry from {@link AbstractVelocityMotor}. The flywheel spins continuously,
 * so soft limits are typically not used.
 * </p>
 */
public class ShooterMotor extends AbstractVelocityMotor<ShooterMotorConfig> {

    /**
     * Creates a shooter motor wrapper using values from the shooter motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config shooter motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured shooter motor wrapper
     */
    public static ShooterMotor create(ShooterMotorConfig config) {
        ShooterMotor motor = new ShooterMotor(config);
        motor.init();
        return motor;
    }

    /**
     * Builds a shooter motor wrapper using values from the shooter motor config.
     *
     * @param config shooter motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private ShooterMotor(ShooterMotorConfig config) {
        super("ShooterMotor", config);
    }
}
