package frc.robot.subsystems.feeder.devices;

import frc.robot.devices.motor.AbstractVelocityMotor;
import frc.robot.shared.config.MotorConfig;

/**
 * SparkMax-backed feeder belt motor configured from the feeder subsystem config.
 * <p>
 * Inherits coast-mode idle, gearing, live-tunable configuration, and telemetry from {@link AbstractVelocityMotor}. The belt spins continuously in
 * both directions, so soft limits are typically not used.
 * </p>
 */
public class FeederMotor extends AbstractVelocityMotor<MotorConfig> {

    /**
     * Creates a feeder motor wrapper using values from the feeder motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config feeder motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured feeder motor wrapper
     */
    public static FeederMotor create(MotorConfig config) {
        FeederMotor motor = new FeederMotor(config);
        motor.init();
        return motor;
    }

    /**
     * Builds a feeder motor wrapper using values from the feeder motor config.
     *
     * @param config feeder motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private FeederMotor(MotorConfig config) {
        super("FeederMotor", config);
    }
}
