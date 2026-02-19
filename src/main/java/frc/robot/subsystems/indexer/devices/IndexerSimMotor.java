package frc.robot.subsystems.indexer.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractVelocitySimMotor;
import frc.robot.subsystems.indexer.config.IndexerMotorConfig;

/**
 * Simulation-only indexer motor wrapper configured from the indexer subsystem config.
 * <p>
 * Use this class when running in simulation so the roller velocity behavior can be exercised without hardware. Construct it with the same config used
 * by the subsystem to ensure gearing matches the real device.
 * </p>
 */
public class IndexerSimMotor extends AbstractVelocitySimMotor<IndexerMotorConfig> {

    /**
     * Creates a simulated indexer motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 indexer motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     * @return configured simulated indexer motor wrapper
     */
    public static IndexerSimMotor create(
            IndexerMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new IndexerSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated indexer motor using the supplied configuration values.
     *
     * @param motorConfig                 indexer motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    private IndexerSimMotor(
            IndexerMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "IndexerMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
