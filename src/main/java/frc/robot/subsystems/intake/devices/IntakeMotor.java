package frc.robot.subsystems.intake.devices;

import frc.robot.devices.motor.AbstractVelocityMotor;
import frc.robot.shared.config.MotorConfig;

/**
 * SparkMax-backed intake roller motor configured from the intake subsystem config.
 * <p>
 * Inherits coast-mode idle, gearing, live-tunable configuration, and telemetry from {@link AbstractVelocityMotor}. The rollers spin continuously in
 * both directions, so soft limits are typically not used.
 * </p>
 */
public class IntakeMotor extends AbstractVelocityMotor<MotorConfig> {

    /**
     * Creates an intake motor wrapper using values from the intake motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config intake motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured intake motor wrapper
     */
    public static IntakeMotor create(MotorConfig config) {
        IntakeMotor motor = new IntakeMotor(config);
        motor.init();
        return motor;
    }

    /**
     * Builds an intake motor wrapper using values from the intake motor config.
     *
     * @param config intake motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private IntakeMotor(MotorConfig config) {
        super("IntakeMotor", config);
    }
}
