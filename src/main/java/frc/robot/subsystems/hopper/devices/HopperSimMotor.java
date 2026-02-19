package frc.robot.subsystems.hopper.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractSimMotor;
import frc.robot.subsystems.hopper.config.HopperMotorConfig;

/**
 * Simulation-only hopper motor wrapper configured from the hopper subsystem config.
 * <p>
 * Use this class when running in simulation so the belt velocity behavior can be exercised without hardware. Construct it with the same config used
 * by the subsystem to ensure gearing matches the real device.
 * </p>
 */
public class HopperSimMotor extends AbstractSimMotor {

    /**
     * Creates a simulated hopper motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 hopper motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     * @return configured simulated hopper motor wrapper
     */
    public static HopperSimMotor create(
            HopperMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new HopperSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated hopper motor using the supplied configuration values.
     *
     * @param motorConfig                 hopper motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    private HopperSimMotor(
            HopperMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "HopperMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
