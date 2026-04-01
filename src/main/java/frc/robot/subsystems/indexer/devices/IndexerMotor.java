package frc.robot.subsystems.indexer.devices;

import frc.robot.devices.motor.AbstractVelocityMotor;
import frc.robot.shared.config.MotorConfig;

/**
 * SparkMax-backed indexer motor configured from the indexer subsystem config.
 * <p>
 * Inherits coast-mode idle, gearing, live-tunable configuration, and telemetry from {@link AbstractVelocityMotor}. The roller spins continuously in
 * both directions, so soft limits are typically not used.
 * </p>
 */
public class IndexerMotor extends AbstractVelocityMotor<MotorConfig> {

    /**
     * Creates an indexer motor wrapper using values from the indexer motor config.
     * <p>
     * Use this factory so {@link #init()} is always called after config assignment.
     * </p>
     *
     * @param config indexer motor configuration containing CAN ID, gear ratio, inversion, and current limits
     * @return configured indexer motor wrapper
     */
    public static IndexerMotor create(MotorConfig config) {
        IndexerMotor motor = new IndexerMotor(config);
        motor.init();
        return motor;
    }

    /**
     * Builds an indexer motor wrapper using values from the indexer motor config.
     *
     * @param config indexer motor configuration containing CAN ID, gear ratio, inversion, and current limits
     */
    private IndexerMotor(MotorConfig config) {
        super("IndexerMotor", config);
    }
}
