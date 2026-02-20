package frc.robot.subsystems.feeder.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractVelocitySimMotor;
import frc.robot.subsystems.feeder.config.FeederMotorConfig;

/**
 * Simulation-only feeder motor wrapper configured from the feeder subsystem config.
 * <p>
 * Use this class when running in simulation so the belt velocity behavior can be exercised without hardware. Construct it with the same config used
 * by the subsystem to ensure gearing matches the real device.
 * </p>
 */
public class FeederSimMotor extends AbstractVelocitySimMotor<FeederMotorConfig> {

    /**
     * Creates a simulated feeder motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 feeder motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     * @return configured simulated feeder motor wrapper
     */
    public static FeederSimMotor create(
            FeederMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new FeederSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated feeder motor using the supplied configuration values.
     *
     * @param motorConfig                 feeder motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    private FeederSimMotor(
            FeederMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "FeederMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
