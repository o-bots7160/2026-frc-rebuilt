package frc.robot.subsystems.shooter.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractSimMotor;
import frc.robot.subsystems.shooter.config.ShooterMotorConfig;

/**
 * Simulation-only shooter motor wrapper configured from the shooter subsystem config.
 * <p>
 * Use this class when running in simulation so the flywheel velocity behavior can be exercised without hardware. Construct it with the same config
 * used by the subsystem to ensure gearing matches the real device.
 * </p>
 */
public class ShooterSimMotor extends AbstractSimMotor {

    /**
     * Creates a simulated shooter motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 shooter motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     * @return configured simulated shooter motor wrapper
     */
    public static ShooterSimMotor create(
            ShooterMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new ShooterSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated shooter motor using the supplied configuration values.
     *
     * @param motorConfig                 shooter motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    private ShooterSimMotor(
            ShooterMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "ShooterMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
