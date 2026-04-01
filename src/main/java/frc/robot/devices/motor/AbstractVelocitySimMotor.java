package frc.robot.devices.motor;

import java.util.function.Supplier;

import frc.robot.shared.config.MotorConfig;

/**
 * Simulation-only motor wrapper for velocity-controlled mechanisms.
 * <p>
 * Concrete subclasses supply their specific motor config and friendly name so the simulation motor matches the gearing and limits of the real
 * device.
 * </p>
 *
 * @param <TConfig> concrete motor config type
 */
public abstract class AbstractVelocitySimMotor<TConfig extends MotorConfig> extends AbstractSimMotor {

    /**
     * Creates a simulated velocity motor using the supplied configuration values.
     *
     * @param name                              friendly name used for logging
     * @param motorConfig                       motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier           supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier       supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    protected AbstractVelocitySimMotor(
            String name,
            TConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                name,
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
