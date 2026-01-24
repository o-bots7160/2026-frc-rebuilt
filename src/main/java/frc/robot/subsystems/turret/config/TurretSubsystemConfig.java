package frc.robot.subsystems.turret.config;

import java.util.function.Supplier;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Configuration bundle for the turret mechanism. Values are stored in degrees for readability but converted to radians at runtime where needed.
 */
public class TurretSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the turret mechanism. */
    public TurretMotorConfig turretMotorConfig = new TurretMotorConfig();

    /**
     * Supplies the motor configuration bundle for the turret.
     * <p>
     * Use this when constructing hardware wrappers so they only depend on motor-specific settings.
     * </p>
     *
     * @return turret motor configuration bundle
     */
    public TurretMotorConfig getTurretMotorConfig() {
        return turretMotorConfig;
    }

    /**
     * Supplies the minimum turret angle in degrees.
     * <p>
     * Delegates to {@link TurretMotorConfig} so motion bounds live alongside motor settings.
     * </p>
     *
     * @return supplier that yields the minimum turret setpoint in degrees
     */
    @Override
    public Supplier<Double> getMinimumSetpointSupplier() {
        return turretMotorConfig.getMinimumSetpointSupplier();
    }

    /**
     * Supplies the maximum turret angle in degrees.
     * <p>
     * Delegates to {@link TurretMotorConfig} so motion bounds live alongside motor settings.
     * </p>
     *
     * @return supplier that yields the maximum turret setpoint in degrees
     */
    @Override
    public Supplier<Double> getMaximumSetpointSupplier() {
        return turretMotorConfig.getMaximumSetpointSupplier();
    }
}
