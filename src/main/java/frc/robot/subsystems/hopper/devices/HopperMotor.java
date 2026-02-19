package frc.robot.subsystems.hopper.devices;

import frc.robot.devices.motor.AbstractVelocityMotor;
import frc.robot.subsystems.hopper.config.HopperMotorConfig;

/**
 * SparkMax-backed hopper belt motor configured from the hopper subsystem config.
 * <p>
 * Inherits coast-mode idle, gearing, live-tunable configuration, and telemetry from {@link AbstractVelocityMotor}. The belt spins continuously in
 * both directions, so soft limits are typically not used.
 * </p>
 */
public class HopperMotor extends AbstractVelocityMotor<HopperMotorConfig> {

    /**
     * Creates a hopper motor wrapper using values from the hopper motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config hopper motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured hopper motor wrapper
     */
    public static HopperMotor create(HopperMotorConfig config) {
        HopperMotor motor = new HopperMotor(config);
        motor.init();
        return motor;
    }

    /**
     * Builds a hopper motor wrapper using values from the hopper motor config.
     *
     * @param config hopper motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private HopperMotor(HopperMotorConfig config) {
        super("HopperMotor", config);
    }
}
