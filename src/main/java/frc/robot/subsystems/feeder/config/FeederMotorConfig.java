package frc.robot.subsystems.feeder.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the feeder belt motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. The feeder belt spins continuously
 * in both directions, so soft limits are typically not used.
 * </p>
 */
public class FeederMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
