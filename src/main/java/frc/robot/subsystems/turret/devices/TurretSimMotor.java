package frc.robot.subsystems.turret.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractSimMotor;
import frc.robot.subsystems.turret.config.TurretMotorConfig;

/**
 * Simulation-only turret motor wrapper configured from the turret subsystem config.
 * <p>
 * Use this class when running in simulation so the turret profile behavior can be exercised without hardware. Construct it with the same config used
 * by the subsystem to ensure limits and gearing match the real device.
 * </p>
 */
public class TurretSimMotor extends AbstractSimMotor {

    /**
     * Creates a simulated turret motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 turret motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum profile acceleration in degrees per second squared
     * @return configured simulated turret motor wrapper
     */
    public static TurretSimMotor create(
            TurretMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new TurretSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated turret motor using the supplied configuration values.
     * <p>
     * The profile limits remain owned by the subsystem config, so supply those values separately to keep the sim wrapper focused on motor
     * configuration.
     * </p>
     *
     * @param motorConfig                 turret motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum profile velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum profile acceleration in degrees per second squared
     */
    private TurretSimMotor(
            TurretMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "TurretMotorSim",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
