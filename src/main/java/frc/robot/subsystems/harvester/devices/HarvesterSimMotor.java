package frc.robot.subsystems.harvester.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractSimMotor;
import frc.robot.subsystems.harvester.config.HarvesterMotorConfig;

/**
 * Simulation-only harvester arm motor wrapper configured from the harvester subsystem config.
 * <p>
 * Use this class when running in simulation so the arm profile behavior can be exercised without hardware. Construct it with the same config used by
 * the subsystem to ensure limits and gearing match the real device.
 * </p>
 */
public class HarvesterSimMotor extends AbstractSimMotor {

    /**
     * Creates a simulated harvester motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 harvester motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum profile acceleration in degrees per second squared
     * @return configured simulated harvester motor wrapper
     */
    public static HarvesterSimMotor create(
            HarvesterMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new HarvesterSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated harvester motor using the supplied configuration values.
     * <p>
     * The profile limits remain owned by the subsystem config, so supply those values separately to keep the sim wrapper focused on motor
     * configuration.
     * </p>
     *
     * @param motorConfig                 harvester motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum profile acceleration in degrees per second squared
     */
    private HarvesterSimMotor(
            HarvesterMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "HarvesterMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
